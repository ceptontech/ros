#!/usr/bin/env python3
"""
ROS2 PCAP Replay Test Script for Cepton Driver

This script tests the Cepton ROS2 driver's ability to replay PCAP files
and verifies that data is received on all relevant topics.

Usage:
    python3 test_pcap_replay.py <pcap_file> [--timeout SECONDS] [--loop]

Example:
    python3 test_pcap_replay.py indoor.pcap --timeout 30 --loop

"""

import argparse
import subprocess
import threading
import time
import signal
import sys
import os
from collections import deque
from datetime import datetime
import tempfile


class TopicMonitor:
    """Monitor ROS2 topics for message activity and collect statistics"""

    def __init__(self, topic_name, message_count=100):
        self.topic_name = topic_name
        self.message_count = message_count
        self.messages_received = 0
        self.timestamps = deque(maxlen=message_count)
        self.first_message_time = None
        self.last_message_time = None
        self.active = True
        self.process = None

    def start_monitoring(self):
        """Start monitoring the topic in a separate process"""
        cmd = [
            "ros2",
            "topic",
            "echo",
            "--once",
            "--times",
            str(self.message_count),
            self.topic_name,
        ]

        def monitor():
            try:
                self.process = subprocess.Popen(
                    cmd,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    text=True,
                )

                while self.active:
                    # Try to read a line (this will block)
                    if self.process.poll() is not None:
                        break

                    # Check if process is still running
                    try:
                        line = self.process.stdout.readline()
                        if line:
                            current_time = time.time()
                            self.messages_received += 1
                            self.timestamps.append(current_time)

                            if self.first_message_time is None:
                                self.first_message_time = current_time
                            self.last_message_time = current_time
                    except:
                        break

            except Exception as e:
                print(f"Error monitoring {self.topic_name}: {e}")

        self.monitor_thread = threading.Thread(target=monitor)
        self.monitor_thread.daemon = True
        self.monitor_thread.start()

    def stop_monitoring(self):
        """Stop monitoring the topic"""
        self.active = False
        if self.process:
            self.process.terminate()
            self.process.wait()

    def get_stats(self):
        """Calculate statistics from collected timestamps"""
        if self.messages_received == 0:
            return {
                "messages_received": 0,
                "frame_rate": 0.0,
                "duration": 0.0,
                "first_message": None,
                "last_message": None,
            }

        duration = 0.0
        frame_rate = 0.0

        if len(self.timestamps) > 1:
            duration = self.timestamps[-1] - self.timestamps[0]
            if duration > 0:
                frame_rate = len(self.timestamps) / duration

        return {
            "messages_received": self.messages_received,
            "frame_rate": round(frame_rate, 2),
            "duration": round(duration, 2),
            "first_message": (
                datetime.fromtimestamp(self.first_message_time).strftime("%H:%M:%S.%f")[
                    :-3
                ]
                if self.first_message_time
                else None
            ),
            "last_message": (
                datetime.fromtimestamp(self.last_message_time).strftime("%H:%M:%S.%f")[
                    :-3
                ]
                if self.last_message_time
                else None
            ),
        }


class CeptonTestRunner:
    """Main test runner for Cepton PCAP replay testing"""

    def __init__(self):
        self.topics_to_monitor = [
            "/cepton_pcl2",  # Main point cloud topic
            "/cepton_info",  # Sensor information
            # "/cepton_sensor_status",  # Sensor status
        ]
        self.monitors = {}
        self.publisher_process = None
        self.build_success = False

    def build_packages(self):
        """Build the ROS2 packages"""
        print("🔨 Building ROS2 packages...")

        os.chdir("/home/wlauer/ros")

        try:
            # Build cepton_messages first
            result = subprocess.run(
                ["colcon", "build", "--packages-select", "cepton_messages"],
                cwd="ros2",
                capture_output=True,
                text=True,
                timeout=120,
            )

            if result.returncode != 0:
                print("❌ Failed to build cepton_messages:")
                print(result.stderr)
                return False

            # Build cepton_publisher
            result = subprocess.run(
                ["colcon", "build", "--packages-select", "cepton_publisher"],
                cwd="ros2",
                capture_output=True,
                text=True,
                timeout=120,
            )

            if result.returncode != 0:
                print("❌ Failed to build cepton_publisher:")
                print(result.stderr)
                return False

            print("✅ Build completed successfully")
            self.build_success = True
            return True

        except subprocess.TimeoutExpired:
            print("❌ Build timed out after 2 minutes")
            return False
        except Exception as e:
            print(f"❌ Build failed with exception: {e}")
            return False

    def source_workspace(self):
        """Source the ROS2 workspace. Assumes that the current directory is in ros2"""
        # We need to source the workspace before running ros2 commands
        # This is handled by setting the environment properly
        setup_path = "install/setup.bash"
        return f"source {setup_path} && "

    def start_publisher(self, pcap_file, loop=False):
        """Start the cepton publisher with the specified PCAP file"""
        print(f"🚀 Starting publisher with PCAP file: {pcap_file}")

        # Create a temporary parameters file
        params_content = f"""
/cepton_publisher:
  ros__parameters:
    capture_file: "{pcap_file}"
    capture_loop: {str(loop).lower()}
    sensor_port: 8808
    cepx_output_type: BOTH
    pcl2_output_type: BOTH
    include_saturated_points: true
    include_second_return_points: false
    include_invalid_points: false
    include_blocked_points: true
    include_ambient_points: true
    min_altitude: -90.0
    max_altitude: 90.0
    min_azimuth: -90.0
    max_azimuth: 90.0
    max_distance: 1000.0
    min_distance: 0.0
"""

        with tempfile.NamedTemporaryFile(mode="w", suffix=".yaml", delete=False) as f:
            f.write(params_content)
            temp_params_file = f.name

        # Prepare the command
        source_cmd = self.source_workspace()
        cmd = f"{source_cmd}ros2 run cepton_publisher cepton_publisher_node --ros-args --params-file {temp_params_file}"
        try:
            self.publisher_process = subprocess.Popen(
                cmd,
                shell=True,
                cwd="ros2",
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                executable="/bin/bash",
                preexec_fn=os.setsid,  # Create new process group
            )

            # Give it a moment to start
            time.sleep(2)

            if self.publisher_process.poll() is not None:
                stdout, stderr = self.publisher_process.communicate()
                print(f"❌ Publisher failed to start:")
                print(f"STDOUT: {stdout}")
                print(f"STDERR: {stderr}")
                return False

            print("✅ Publisher started successfully")
            return True

        except Exception as e:
            print(f"❌ Failed to start publisher: {e}")
            return False
        finally:
            # Clean up temp file
            try:
                os.unlink(temp_params_file)
            except:
                pass

    def setup_monitors(self):
        """Set up topic monitors for all topics"""
        print("📡 Setting up topic monitors...")

        source_cmd = self.source_workspace()

        # First check which topics are available
        try:
            result = subprocess.run(
                f"{source_cmd}ros2 topic list",
                shell=True,
                capture_output=True,
                text=True,
                executable="/bin/bash",
                timeout=10,
            )

            available_topics = result.stdout.strip().split("\n")
            print(f"Available topics: {available_topics}")

        except Exception as e:
            print(f"Warning: Could not list topics: {e}")
            available_topics = self.topics_to_monitor  # Fallback to expected topics

        for topic in self.topics_to_monitor:
            self.monitors[topic] = TopicMonitor(topic)

    def start_monitoring(self):
        """Start monitoring all topics"""
        print("🔍 Starting topic monitoring...")
        source_cmd = self.source_workspace()

        for topic_name, monitor in self.monitors.items():
            print(f"  Monitoring {topic_name}...")

            def create_monitor_func(topic, monitor_obj):
                def monitor():
                    try:
                        cmd = f"{source_cmd}timeout 60 ros2 topic echo {topic}"
                        process = subprocess.Popen(
                            cmd,
                            shell=True,
                            cwd="ros2",
                            stdout=subprocess.PIPE,
                            stderr=subprocess.PIPE,
                            executable="/bin/bash",
                            text=True,
                        )

                        start_time = time.time()
                        while time.time() - start_time < 60 and monitor_obj.active:
                            if process.poll() is not None:
                                break

                            # Simple way to detect messages - just count non-empty lines
                            line = process.stdout.readline()
                            if line.strip():
                                current_time = time.time()
                                monitor_obj.messages_received += 1
                                monitor_obj.timestamps.append(current_time)

                                if monitor_obj.first_message_time is None:
                                    monitor_obj.first_message_time = current_time
                                monitor_obj.last_message_time = current_time

                                # Print first message received
                                if monitor_obj.messages_received == 1:
                                    print(f"  ✅ First message received on {topic}")

                        if process.poll() is None:
                            process.terminate()

                    except Exception as e:
                        print(f"  ❌ Error monitoring {topic}: {e}")

                return monitor

            monitor_thread = threading.Thread(
                target=create_monitor_func(topic_name, monitor)
            )
            monitor_thread.daemon = True
            monitor_thread.start()

    def wait_for_data(self, timeout_seconds):
        """Wait for data on topics with timeout"""
        print(f"⏳ Waiting up to {timeout_seconds} seconds for data...")

        start_time = time.time()

        while time.time() - start_time < timeout_seconds:
            # Check if we've received data on the main topic
            main_topic_monitor = self.monitors.get("/cepton_pcl2")
            if main_topic_monitor and main_topic_monitor.messages_received > 0:
                print("✅ Data received on main topic!")
                break

            time.sleep(1)

            # Print periodic status
            if (
                int(time.time() - start_time) % 10 == 0
                and int(time.time() - start_time) > 0
            ):
                elapsed = int(time.time() - start_time)
                print(f"  ⏱️  {elapsed}s elapsed, still waiting for data...")

        # Give a bit more time for additional messages
        print("📊 Collecting additional data for statistics...")
        time.sleep(5)

    def stop_monitoring(self):
        """Stop all topic monitoring"""
        print("🛑 Stopping monitors...")
        for monitor in self.monitors.values():
            monitor.active = False

    def stop_publisher(self):
        """Stop the publisher process"""
        if self.publisher_process:
            print("🛑 Stopping publisher...")
            try:
                # Kill the process group to ensure all child processes are terminated
                os.killpg(os.getpgid(self.publisher_process.pid), signal.SIGTERM)
                self.publisher_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(os.getpgid(self.publisher_process.pid), signal.SIGKILL)
                self.publisher_process.wait()
            except Exception as e:
                print(f"Warning: Error stopping publisher: {e}")

    def generate_report(self):
        """Generate test summary report"""
        print("\n" + "=" * 60)
        print("📋 TEST SUMMARY REPORT")
        print("=" * 60)

        # Build status
        print(f"🔨 Build Status: {'✅ SUCCESS' if self.build_success else '❌ FAILED'}")

        # Topic statistics
        print("\n📊 TOPIC DATA RECEPTION:")
        topics_with_data = 0
        total_messages = 0

        for topic_name, monitor in self.monitors.items():
            stats = monitor.get_stats()
            status = "✅ RECEIVED" if stats["messages_received"] > 0 else "❌ NO DATA"

            print(f"\n  Topic: {topic_name}")
            print(f"    Status: {status}")
            print(f"    Messages: {stats['messages_received']}")

            if stats["messages_received"] > 0:
                topics_with_data += 1
                total_messages += stats["messages_received"]
                print(f"    Frame Rate: {stats['frame_rate']} Hz")
                print(f"    Duration: {stats['duration']} seconds")
                print(f"    First Message: {stats['first_message']}")
                print(f"    Last Message: {stats['last_message']}")

        # Overall summary
        print(f"\n🎯 OVERALL RESULTS:")
        print(f"    Topics with data: {topics_with_data}/{len(self.monitors)}")
        print(f"    Total messages received: {total_messages}")

        # Test verdict
        main_topic_success = (
            self.monitors["/cepton_pcl2"].get_stats()["messages_received"] > 0
        )
        overall_success = self.build_success and main_topic_success

        print(f"\n🏆 TEST RESULT: {'✅ PASS' if overall_success else '❌ FAIL'}")

        if not overall_success:
            print("\n❌ Test failed because:")
            if not self.build_success:
                print("    - Build failed")
            if not main_topic_success:
                print("    - No data received on main topic /cepton_pcl2")

        print("=" * 60)

        return overall_success


def signal_handler(sig, frame):
    """Handle Ctrl+C gracefully"""
    print("\n\n🛑 Test interrupted by user")
    sys.exit(1)


def main():
    parser = argparse.ArgumentParser(description="Test Cepton ROS2 driver PCAP replay")
    parser.add_argument("pcap_file", help="Path to PCAP file to replay")
    parser.add_argument(
        "--timeout", type=int, default=30, help="Timeout in seconds (default: 30)"
    )
    parser.add_argument("--loop", action="store_true", help="Loop the PCAP replay")

    args = parser.parse_args()

    # Validate pcap file
    if not os.path.exists(args.pcap_file):
        print(f"❌ PCAP file not found: {args.pcap_file}")
        return 1

    # Make pcap_file path absolute
    pcap_file = os.path.abspath(args.pcap_file)

    print("🧪 Cepton ROS2 PCAP Replay Test")
    print("=" * 40)
    print(f"PCAP File: {pcap_file}")
    print(f"Timeout: {args.timeout} seconds")
    print(f"Loop: {args.loop}")
    print("=" * 40)

    # Set up signal handler
    signal.signal(signal.SIGINT, signal_handler)

    # Create test runner
    runner = CeptonTestRunner()

    try:
        # Step 1: Build packages
        if not runner.build_packages():
            return 1

        # Step 2: Set up monitoring
        runner.setup_monitors()

        # Step 3: Start publisher
        if not runner.start_publisher(pcap_file, args.loop):
            return 1

        # Step 4: Start monitoring topics
        runner.start_monitoring()

        # Step 5: Wait for data
        runner.wait_for_data(args.timeout)

        # Step 6: Generate report
        success = runner.generate_report()

        return 0 if success else 1

    except KeyboardInterrupt:
        print("\n\n🛑 Test interrupted by user")
        return 1
    except Exception as e:
        print(f"\n❌ Test failed with exception: {e}")
        return 1
    finally:
        # Clean up
        runner.stop_monitoring()
        runner.stop_publisher()


if __name__ == "__main__":
    exit(main())
