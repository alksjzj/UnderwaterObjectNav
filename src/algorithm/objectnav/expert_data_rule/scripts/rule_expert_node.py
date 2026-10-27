#!/usr/bin/env python3
"""Continuous rule-expert action publisher and data-collection orchestrator."""

import json
import math
import threading
import time

import message_filters
import numpy as np
import rospkg
import rospy
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse

from data_generate.srv import EndEpisode, StartEpisode
from robot_control.msg import ActionRequest, ActionStatus
from stonefish_ros.srv import Respawn

from expert_data_rule.config import load_config
from expert_data_rule.policy import RuleBasedExpertPolicy
from expert_data_rule.runtime.episode_manager import EpisodeManager, Lifecycle
from expert_data_rule.runtime.inference_client import InferenceClient
from expert_data_rule.runtime.spawn_sampler import SceneSpawnSampler
from expert_data_rule.types import Action, Detection, PerceptionOutput, TargetObservation


def rpy_from_quaternion(q):
    sinr = 2.0*(q.w*q.x+q.y*q.z)
    cosr = 1.0-2.0*(q.x*q.x+q.y*q.y)
    roll = math.atan2(sinr, cosr)
    sinp = 2.0*(q.w*q.y-q.z*q.x)
    pitch = math.copysign(math.pi/2, sinp) if abs(sinp) >= 1 else math.asin(sinp)
    yaw = math.atan2(2.0*(q.w*q.z+q.x*q.y),
                     1.0-2.0*(q.y*q.y+q.z*q.z))
    return roll, pitch, yaw


class RuleExpertNode:
    TERMINAL_STATES = ("SUCCEEDED", "TIMEOUT", "FAILED", "STOPPED")

    def __init__(self):
        config_path = rospy.get_param("~config")
        self.config = load_config(config_path)
        scenario = rospy.get_param("~scenario_file", "") or \
            self.config["spawn"]["scenario_file"]
        self.config["spawn"]["scenario_file"] = scenario
        self.config["spawn"]["fixed_z_m"] = float(rospy.get_param(
            "~spawn_z_m", self.config["spawn"]["fixed_z_m"]))
        self.config["episode"]["scene_id"] = rospy.get_param(
            "~scene_id", self.config["episode"]["scene_id"])
        ros = self.config["ros"]
        timing = self.config["timing"]
        self.lock = threading.RLock()
        self.condition = threading.Condition(self.lock)
        self.bridge = CvBridge()
        self.latest_observation = None
        self.current_action_id = None
        self.current_status = None
        self.next_action_id = 1
        self.request_id = 1
        self.shutdown_requested = False

        self.lifecycle = EpisodeManager(
            self.config["episode"]["auto_start"],
            self.config["episode"].get("max_episodes", 0))
        self.policy = RuleBasedExpertPolicy(self.config)
        resolver = rospkg.RosPack().get_path
        self.sampler = SceneSpawnSampler(
            scenario, self.config["spawn"],
            self.config["episode"]["random_seed"], resolver)
        self.client = InferenceClient(
            rospy.get_param("~worker_host", "127.0.0.1"),
            rospy.get_param("~worker_port", 29510),
            timing["worker_connect_timeout_s"])

        self.action_pub = rospy.Publisher(
            ros["action_request_topic"], ActionRequest, queue_size=10)
        self.state_pub = rospy.Publisher(
            "~collection_state", String, queue_size=1, latch=True)
        rospy.Subscriber(ros["action_status_topic"], ActionStatus,
                         self.action_status_cb, queue_size=20)
        rgb_sub = message_filters.Subscriber(ros["rgb_topic"], Image)
        depth_sub = message_filters.Subscriber(ros["depth_topic"], Image)
        pose_sub = message_filters.Subscriber(ros["pose_topic"], Odometry)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [rgb_sub, depth_sub, pose_sub], int(timing["sync_queue_size"]),
            float(timing["sync_slop_s"]))
        self.sync.registerCallback(self.observation_cb)
        rospy.Service("~start_collection", Trigger, self.start_collection)
        rospy.Service("~pause_collection", Trigger, self.pause_collection)

        wait = float(timing["service_wait_timeout_s"])
        for service in (ros["respawn_service"], ros["collector_start_service"],
                        ros["collector_end_service"]):
            rospy.wait_for_service(service, timeout=wait)
        self.respawn = rospy.ServiceProxy(ros["respawn_service"], Respawn)
        self.start_episode_service = rospy.ServiceProxy(
            ros["collector_start_service"], StartEpisode)
        self.end_episode_service = rospy.ServiceProxy(
            ros["collector_end_service"], EndEpisode)
        self.connect_worker()
        rospy.on_shutdown(self.shutdown)
        self.publish_state("ready")

    def connect_worker(self):
        delay = float(self.config["timing"]["worker_connect_retry_s"])
        while not rospy.is_shutdown():
            try:
                self.client.connect()
                rospy.loginfo("connected to rule expert perception worker")
                return
            except (OSError, ConnectionError) as exc:
                rospy.logwarn_throttle(5.0, "waiting for perception worker: %s", exc)
                rospy.sleep(delay)
        raise rospy.ROSInterruptException("shutdown while waiting for worker")

    def observation_cb(self, rgb, depth, odom):
        stamp = max(rgb.header.stamp, depth.header.stamp, odom.header.stamp)
        with self.condition:
            self.latest_observation = (rgb, depth, odom, stamp)
            self.condition.notify_all()

    def action_status_cb(self, message):
        with self.condition:
            if message.action_id != self.current_action_id:
                return
            self.current_status = message
            self.condition.notify_all()

    def start_collection(self, _request):
        with self.condition:
            self.lifecycle.start()
            self.condition.notify_all()
        self.publish_state("collection enabled")
        return TriggerResponse(True, "continuous collection enabled")

    def pause_collection(self, _request):
        with self.condition:
            self.lifecycle.pause()
            self.condition.notify_all()
        self.publish_state("pause requested")
        return TriggerResponse(
            True, "pause requested; the current action and episode will close safely")

    def publish_state(self, detail, **extra):
        document = {"lifecycle": self.lifecycle.state.value,
                    "enabled": self.lifecycle.enabled,
                    "episode_active": self.lifecycle.active,
                    "completed_episodes": self.lifecycle.completed,
                    "detail": detail}
        document.update(extra)
        self.state_pub.publish(String(data=json.dumps(document)))

    def wait_observation_after(self, minimum_stamp):
        timeout = float(self.config["timing"]["service_wait_timeout_s"])
        deadline = time.monotonic()+timeout
        with self.condition:
            while not rospy.is_shutdown():
                if (self.latest_observation is not None and
                        self.latest_observation[3] > minimum_stamp):
                    stamp = self.latest_observation[3]
                    max_age = float(
                        self.config["timing"]["max_observation_age_s"])
                    age = ((rospy.Time.now()-stamp).to_sec()
                           if stamp != rospy.Time() else 0.0)
                    if max_age <= 0.0 or age <= max_age:
                        return self.latest_observation
                remaining = deadline-time.monotonic()
                if remaining <= 0:
                    raise TimeoutError("fresh synchronized observation timeout")
                self.condition.wait(min(remaining, 0.25))
        raise rospy.ROSInterruptException("shutdown")

    def make_spawn_pose(self, sample):
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = sample.x, sample.y, sample.z
        pose.orientation.z = math.sin(sample.yaw/2.0)
        pose.orientation.w = math.cos(sample.yaw/2.0)
        return pose

    def prepare_episode(self):
        self.lifecycle.begin_preparing()
        sample = self.sampler.sample()
        response = self.respawn(
            self.config["spawn"]["robot_name"], self.make_spawn_pose(sample))
        if not response.success:
            raise RuntimeError("robot respawn failed: %s" % response.message)
        respawn_time = rospy.Time.now()
        # action_executor owns its idle hold target. After teleportation, send
        # an unrecorded STOP as soon as fresh odometry arrives so that the old
        # episode's hold target cannot pull the robot back across the scene.
        self.wait_observation_after(respawn_time)
        hold_status = self.execute_action(Action.STOP)
        if hold_status.state != "STOPPED":
            raise RuntimeError("could not anchor action executor after respawn")
        settle_start = rospy.Time.now()
        rospy.sleep(float(self.config["timing"]["respawn_settle_s"]))
        self.wait_observation_after(settle_start)
        self.client.request({"type": "RESET"}, timeout_s=float(
            self.config["timing"]["inference_timeout_s"]))
        self.policy.reset()
        episode = self.config["episode"]
        result = self.start_episode_service(
            str(episode["scene_id"]), str(episode["goal_category"]))
        if not result.accepted:
            raise RuntimeError("collector rejected episode: %s" % result.message)
        self.lifecycle.began()
        self.publish_state("episode started", episode_id=result.episode_id,
                           spawn={"x": sample.x, "y": sample.y,
                                  "z": sample.z, "yaw": sample.yaw})
        rospy.loginfo("expert episode %d spawn=(%.2f, %.2f, %.2f, %.2f)",
                      result.episode_id, sample.x, sample.y, sample.z, sample.yaw)

    def observation_arrays(self, observation):
        rgb_msg, depth_msg, odom, stamp = observation
        rgb = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
        depth = np.asarray(self.bridge.imgmsg_to_cv2(
            depth_msg, desired_encoding="passthrough"), np.float32)
        pose = odom.pose.pose
        roll, pitch, yaw = rpy_from_quaternion(pose.orientation)
        return np.asarray(rgb, np.uint8), depth, {
            "x": float(pose.position.x), "y": float(pose.position.y),
            "z": float(pose.position.z), "roll": roll, "pitch": pitch,
            "yaw": yaw}, stamp

    def infer(self, observation):
        rgb, depth, pose, stamp = self.observation_arrays(observation)
        request_id, self.request_id = self.request_id, self.request_id+1
        response, arrays = self.client.request(
            {"type": "INFER", "request_id": request_id, "pose": pose},
            {"rgb": rgb, "depth": depth},
            timeout_s=float(self.config["timing"]["inference_timeout_s"]))
        detections = tuple(Detection(tuple(item["bbox"]), float(item["score"]),
                                     str(item["class_name"]))
                           for item in response["detections"])
        targets = tuple(TargetObservation(
            bool(item["visible"]), float(item["bearing_rad"]),
            float(item["distance_m"]), tuple(item["bbox"]), float(item["score"]))
            for item in response["targets"])
        perception = PerceptionOutput(
            detections, targets, np.asarray(arrays["global_map"], np.float32),
            np.asarray(arrays["local_map"], np.float32),
            float(response["sensor_world_z_m"]))
        return perception, pose, stamp

    def execute_action(self, action):
        message = ActionRequest()
        message.action_id = self.next_action_id
        self.next_action_id += 1
        message.action = action.value
        message.request_time = rospy.Time.now()
        with self.condition:
            self.current_action_id = message.action_id
            self.current_status = None
        self.action_pub.publish(message)
        deadline = time.monotonic()+float(
            self.config["timing"]["action_status_timeout_s"])
        with self.condition:
            while not rospy.is_shutdown():
                status = self.current_status
                if status is not None and status.state in self.TERMINAL_STATES:
                    self.current_action_id = None
                    self.current_status = None
                    return status
                remaining = deadline-time.monotonic()
                if remaining <= 0:
                    self.current_action_id = None
                    self.lifecycle.pause()
                    raise TimeoutError("action status timeout; collection paused")
                self.condition.wait(min(remaining, 0.25))
        raise rospy.ROSInterruptException("shutdown")

    def end_episode(self, success, reason):
        if not self.lifecycle.active:
            return
        self.lifecycle.state = Lifecycle.ENDING
        try:
            result = self.end_episode_service(bool(success), "" if success else str(reason))
            if not result.accepted:
                rospy.logerr("collector rejected episode end: %s", result.message)
        finally:
            self.lifecycle.ended()
        self.publish_state("episode ended", success=bool(success), reason=reason)

    def run_episode(self):
        last_stamp = rospy.Time(0)
        while not rospy.is_shutdown() and self.lifecycle.active:
            if not self.lifecycle.enabled:
                self.end_episode(False, "operator_paused")
                return
            observation = self.wait_observation_after(last_stamp)
            perception, pose, stamp = self.infer(observation)
            if not self.lifecycle.enabled:
                self.end_episode(False, "operator_paused")
                return
            decision = self.policy.decide(perception, pose["yaw"])
            if decision.failed:
                self.end_episode(False, decision.failure_reason)
                return
            if decision.action is None:
                self.publish_state(decision.reason,
                                   step_count=self.policy.state.step_count)
                last_stamp = stamp
                continue
            self.publish_state("action selected", action=decision.action.value,
                               reason=decision.reason,
                               step_count=self.policy.state.step_count)
            status = self.execute_action(decision.action)
            if status.state not in ("SUCCEEDED", "STOPPED"):
                self.end_episode(False, status.failure_reason or status.state.lower())
                return
            if decision.action.value == "STOP":
                self.end_episode(True, "")
                return
            self.policy.record_completed_action(decision.action)
            rospy.sleep(float(self.config["timing"]["post_action_settle_s"]))
            last_stamp = max(stamp, status.finish_time)

    def run(self):
        while not rospy.is_shutdown():
            with self.condition:
                if not self.lifecycle.may_start():
                    self.publish_state("paused")
                    self.condition.wait(0.5)
                    continue
            try:
                self.prepare_episode()
                self.run_episode()
            except (CvBridgeError, OSError, RuntimeError, TimeoutError,
                    ValueError, rospy.ServiceException) as exc:
                rospy.logerr("expert episode failed: %s", exc)
                if self.lifecycle.active:
                    self.end_episode(False, "runtime_error:%s" % exc)
                else:
                    self.lifecycle.state = Lifecycle.PAUSED
                if self.lifecycle.enabled and not self.client.connected:
                    self.connect_worker()
                # Repeated infrastructure failures must not create a tight loop.
                rospy.sleep(float(self.config["timing"]["inter_episode_delay_s"]))
                continue
            if self.lifecycle.enabled:
                rospy.sleep(float(self.config["timing"]["inter_episode_delay_s"]))

    def shutdown(self):
        self.shutdown_requested = True
        self.lifecycle.pause()
        try:
            if self.lifecycle.active:
                self.end_episode(False, "node_shutdown")
        except Exception as exc:
            rospy.logerr("could not close episode during shutdown: %s", exc)
        self.client.close()


def main():
    rospy.init_node("rule_based_expert")
    RuleExpertNode().run()


if __name__ == "__main__":
    main()
