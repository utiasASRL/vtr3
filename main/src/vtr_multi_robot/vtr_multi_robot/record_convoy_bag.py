#!/usr/bin/env python3

import argparse
from datetime import datetime
import os
import shutil
import subprocess
import sys
import yaml
import ast


def parse_args():
    parser = argparse.ArgumentParser(
        description="Record ROS 2 bags with dynamic naming and file post-processing."
    )

    # Positional argument for the YAML file path
    parser.add_argument(
        "setup_params_path",
        type=str,
        help="Path to the configuration YAML file.",
    )

    # Flag accepting one or more space-separated namespace strings
    parser.add_argument(
        "-n",
        "--namespaces",
        type=str,
        required=True,
        metavar="NS",
        help="List of robot namespaces to record (e.g., -n ['robot1','robot2'])",
    )

    # Optional flags for files to copy
    parser.add_argument(
        "--configs",
        nargs="+",
        default=[],
        metavar=("FILES"),
        help="Two file paths to copy into the bag directory after recording.",
    )

    return parser.parse_args()

def parse_namespace_string(ns_str: str) -> list[str]:
    """Safely converts a string formatted like \"['ns1','ns2']\" into a Python list."""
    try:
        parsed = ast.literal_eval(ns_str)
        if not isinstance(parsed, list):
            raise ValueError("Parsed result is not a list.")
        return [str(item) for item in parsed]
    except (SyntaxError, ValueError) as e:
        print(
            f"Error: Invalid namespace list format '{ns_str}'. Expected format like \"['ns1','ns2']\".",
            file=sys.stderr,
        )
        sys.exit(1)

def main():
    args = parse_args()
    namespaces = parse_namespace_string(args.namespaces)

    # 1. Read value from the YAML file
    if not os.path.isfile(args.setup_params_path):
        print(f"Error: Config file '{args.setup_params_path}' not found.", file=sys.stderr)
        sys.exit(1)

    prefix = 'rosbag2'
    try:
        with open(args.setup_params_path, "r") as f:
            config = yaml.safe_load(f)
            print(config)
            prefix = os.path.basename(config["/**"]["ros__parameters"]["data_dir"])
    except yaml.YAMLError as e:
        print(f"Error parsing YAML file: {e}", file=sys.stderr)
        sys.exit(1)

    # 2. Build dynamic bag name
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    bag_dir = f"{prefix}_{timestamp}"

    # 3. Construct topic list using the provided namespaces
    topics_to_record = ['/tf', '/tf_static']
    for ns in namespaces:
        clean_ns = ns.strip("/")
        # Example topics per namespace (adjust as needed)
        topics_to_record.extend([
            f"/{clean_ns}/cmd_vel",
            f"/{clean_ns}/vtr/mpc_prediction",
            f"/{clean_ns}/vtr/odometry",
            f"/{clean_ns}/vtr/leader_mpc_prediction",
            f"/{clean_ns}/vtr/stamped_reference_poses",
            f"/{clean_ns}/vtr/leader_distance",
            f"/{clean_ns}/vtr/estimated_leader_distance",
        ])

    # 4. Build ros2 bag record command
    cmd = ["ros2", "bag", "record", "-o", bag_dir] + topics_to_record

    print(f"Bag Directory : {bag_dir}")
    print(f"Namespaces    : {args.namespaces}")
    print("Starting ROS 2 bag record. Press Ctrl+C to stop...\n")

    # 5. Execute recording process
    proc = subprocess.Popen(cmd)

    try:
        proc.wait()
    except KeyboardInterrupt:
        print("\nCtrl+C received. Waiting for bag writer to close safely...")
        proc.wait()
    finally:
        # 6. Copy files into bag directory
        if os.path.exists(bag_dir):
            for file_path in args.configs:
                if os.path.exists(file_path):
                    shutil.copy(file_path, bag_dir)
                    print(f"Copied '{file_path}' -> '{bag_dir}/'")
                else:
                    print(f"Warning: '{file_path}' not found, skipping copy.")
            print("Finished.")
        else:
            print(f"Error: Bag directory '{bag_dir}' was not created.")


if __name__ == "__main__":
    main()