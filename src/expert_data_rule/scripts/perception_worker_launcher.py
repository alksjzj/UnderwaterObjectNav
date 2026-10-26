#!/usr/bin/env python3
"""Replace this ROS Python process with the configured Conda worker."""

import argparse
import os
import sys


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--conda-executable", required=True)
    parser.add_argument("--conda-env", required=True)
    parser.add_argument("--package-root", required=True)
    parser.add_argument("--config", required=True)
    parser.add_argument("--checkpoint", default="")
    parser.add_argument("--threshold-file", default="")
    parser.add_argument("--device", default="cuda:0")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=29510)
    args = parser.parse_args([item for item in sys.argv[1:] if ":=" not in item])
    command = [args.conda_executable, "run", "--no-capture-output", "-n",
               args.conda_env, "python", "-u", "-m",
               "expert_data_rule.runtime.inference_worker", "--config",
               args.config, "--device", args.device, "--host", args.host,
               "--port", str(args.port)]
    if args.checkpoint:
        command.extend(("--checkpoint", args.checkpoint))
    if args.threshold_file:
        command.extend(("--threshold-file", args.threshold_file))
    environment = os.environ.copy()
    environment["PYTHONPATH"] = os.path.abspath(args.package_root)
    os.execve(args.conda_executable, command, environment)


if __name__ == "__main__":
    main()
