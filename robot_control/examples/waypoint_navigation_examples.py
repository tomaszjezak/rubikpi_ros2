#!/usr/bin/env python3
"""
Example usage script for waypoint navigation system
This demonstrates how to use the waypoint navigation nodes
"""

import subprocess
import time
import sys


def run_command(cmd, description):
    """Run a command and print description"""
    print(f"\n{'='*60}")
    print(f"🚀 {description}")
    print(f"{'='*60}")
    print(f"Command: {cmd}")
    print("-" * 60)
    
    try:
        result = subprocess.run(cmd, shell=True, check=True, capture_output=True, text=True)
        print("✅ Command completed successfully")
        if result.stdout:
            print("Output:", result.stdout)
    except subprocess.CalledProcessError as e:
        print(f"❌ Command failed with exit code {e.returncode}")
        if e.stderr:
            print("Error:", e.stderr)
        return False
    return True


def main():
    print("""
🤖 Waypoint Navigation System - Usage Examples
==============================================

This script demonstrates various ways to use the waypoint navigation system.

Prerequisites:
1. Make sure ROS2 is sourced: source /opt/ros/humble/setup.bash
2. Build the workspace: colcon build
3. Source the workspace: source install/setup.bash
4. Connect your robot to /dev/ttyUSB0

Available Commands:
""")
    
    examples = [
        # Basic waypoint navigation
        {
            "cmd": "ros2 run robot_control waypoint_navigation --x 1.0 --y 0.0 --theta 0",
            "desc": "Navigate to waypoint (1.0m, 0.0m, 0°) - Simple forward motion"
        },
        {
            "cmd": "ros2 run robot_control waypoint_navigation --x 0.0 --y 1.0 --theta 90",
            "desc": "Navigate to waypoint (0.0m, 1.0m, 90°) - Turn and move"
        },
        {
            "cmd": "ros2 run robot_control waypoint_navigation --x 1.0 --y 1.0 --theta 45",
            "desc": "Navigate to waypoint (1.0m, 1.0m, 45°) - Diagonal movement"
        },
        
        # Advanced waypoint navigation with interpolation
        {
            "cmd": "ros2 run robot_control advanced_waypoint_navigation --x 1.0 --y 0.0 --theta 0",
            "desc": "Advanced navigation to (1.0m, 0.0m, 0°) with trajectory interpolation"
        },
        {
            "cmd": "ros2 run robot_control advanced_waypoint_navigation --waypoints '0.5,0,0;1.0,0,0;1.0,0.5,90'",
            "desc": "Multi-waypoint navigation: (0.5,0,0°) → (1.0,0,0°) → (1.0,0.5,90°)"
        },
        
        # Utility commands
        {
            "cmd": "ros2 run robot_control waypoint_navigation --status",
            "desc": "Check current robot pose"
        },
        {
            "cmd": "ros2 run robot_control waypoint_navigation --reset",
            "desc": "Reset robot pose to origin (0,0,0)"
        },
        {
            "cmd": "ros2 run robot_control waypoint_navigation --stop",
            "desc": "Stop current navigation"
        },
        
        # Launch files
        {
            "cmd": "ros2 launch robot_control waypoint_navigation_launch.py",
            "desc": "Launch waypoint navigation system (basic)"
        },
        {
            "cmd": "ros2 launch robot_control advanced_waypoint_navigation_launch.py",
            "desc": "Launch advanced waypoint navigation system with trajectory generation"
        }
    ]
    
    print("Choose an example to run:")
    for i, example in enumerate(examples, 1):
        print(f"{i:2d}. {example['desc']}")
    
    print("\n0. Run all examples sequentially")
    print("q. Quit")
    
    while True:
        try:
            choice = input("\nEnter your choice (0-{} or q): ".format(len(examples)))
            
            if choice.lower() == 'q':
                print("👋 Goodbye!")
                break
            elif choice == '0':
                print("\n🔄 Running all examples sequentially...")
                for example in examples:
                    if not run_command(example['cmd'], example['desc']):
                        print("❌ Stopping due to error")
                        break
                    time.sleep(2)  # Wait between examples
                break
            else:
                try:
                    idx = int(choice) - 1
                    if 0 <= idx < len(examples):
                        example = examples[idx]
                        run_command(example['cmd'], example['desc'])
                        break
                    else:
                        print("❌ Invalid choice. Please try again.")
                except ValueError:
                    print("❌ Invalid input. Please enter a number or 'q'.")
                    
        except KeyboardInterrupt:
            print("\n👋 Interrupted by user. Goodbye!")
            break


if __name__ == '__main__':
    main()
