"""Conda-side frozen detector and semantic mapper; contains no ROS imports."""

import argparse
import socket
import time
import traceback

import numpy as np

from expert_data_rule.config import load_config, resolve_path
from expert_data_rule.mapping import SemanticMappingPipeline
from expert_data_rule.perception import MMDetSemanticDetector
from .inference_protocol import receive_message, send_message


class PerceptionEngine:
    def __init__(self, config_path, checkpoint, threshold_file, device):
        self.config = load_config(config_path)
        detector = self.config["detector"]
        model_config = resolve_path(self.config, detector["config_path"])
        checkpoint = checkpoint or detector["checkpoint_path"]
        checkpoint = resolve_path(self.config, checkpoint)
        threshold_file = threshold_file or detector.get("threshold_file", "")
        threshold_file = (resolve_path(self.config, threshold_file)
                          if threshold_file else None)
        self.detector = MMDetSemanticDetector(
            model_config, checkpoint, tuple(detector["classes"]),
            detector.get("score_thresholds"), threshold_file,
            detector.get("candidate_score_threshold", 0.001), device)
        self.mapper = SemanticMappingPipeline(self.config)

    def reset(self):
        self.mapper.reset()

    def infer(self, payload, arrays):
        start = time.perf_counter()
        rgb = np.asarray(arrays["rgb"], np.uint8)
        depth = np.asarray(arrays["depth"], np.float32)
        detections = self.detector.detect(rgb)
        output = self.mapper.update(rgb, depth, payload["pose"], detections)
        response = {
            "type": "INFER_RESULT", "ok": True,
            "request_id": int(payload["request_id"]),
            "inference_time_s": time.perf_counter()-start,
            "sensor_world_z_m": output.sensor_world_z_m,
            "detections": [
                {"bbox": list(item.bbox), "score": item.score,
                 "class_name": item.class_name}
                for item in output.detections],
            "targets": [
                {"visible": item.visible, "bearing_rad": item.bearing_rad,
                 "distance_m": item.distance_m, "bbox": list(item.bbox),
                 "score": item.score}
                for item in output.targets],
        }
        return response, {"global_map": output.global_map.astype(np.float16),
                          "local_map": output.local_map.astype(np.float16)}


def serve(connection, engine):
    while True:
        payload, arrays = receive_message(connection)
        try:
            message_type = payload.get("type")
            if message_type == "HEALTH":
                send_message(connection, {"type": "READY", "ok": True})
            elif message_type == "RESET":
                engine.reset()
                send_message(connection, {"type": "RESET_DONE", "ok": True})
            elif message_type == "INFER":
                response, outputs = engine.infer(payload, arrays)
                send_message(connection, response, outputs)
            elif message_type == "SHUTDOWN":
                send_message(connection, {"type": "SHUTDOWN_DONE", "ok": True})
                return
            else:
                raise ValueError("unknown request %r" % message_type)
        except Exception as exc:
            traceback.print_exc()
            send_message(connection, {"type": "ERROR", "ok": False,
                                      "error": "%s: %s" %
                                      (type(exc).__name__, exc)})


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    parser.add_argument("--checkpoint", default="")
    parser.add_argument("--threshold-file", default="")
    parser.add_argument("--device", default="cuda:0")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=29510)
    args = parser.parse_args()
    engine = PerceptionEngine(args.config, args.checkpoint,
                              args.threshold_file, args.device)
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((args.host, args.port))
    server.listen(1)
    print("rule expert perception READY on %s:%d" % (args.host, args.port),
          flush=True)
    try:
        connection, _ = server.accept()
        with connection:
            serve(connection, engine)
    finally:
        server.close()


if __name__ == "__main__":
    main()
