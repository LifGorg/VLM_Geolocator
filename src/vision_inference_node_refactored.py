"""
Refactored Vision Inference Node

Features:
- GStreamer video reception
- Button/topic triggered capture
- Target detection inference
- GPS coordinate estimation
- ROS2 topic publishing
"""
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

# Import refactored modules
from vlm_geolocator.core import ConfigManager
from vlm_geolocator.sensors import SensorManager
from vlm_geolocator.gps import GPSCalculator
from vlm_geolocator.vision import VideoFrameReceiver, IsaacDetectorWrapper
from vlm_geolocator.ros_interface import ROSPublisherManager, ROSSubscriberManager


class VisionInferenceNode(Node):
    """Vision Inference Node"""
    
    def __init__(self, config_dir: str = None):
        super().__init__('vision_inference_node')
        
        # Record startup time
        self.node_start_time = time.time()
        
        # Load configuration
        self.get_logger().info('Loading configuration...')
        self.config = ConfigManager(config_dir)
        
        # Initialize sensor manager
        self.get_logger().info('Initializing sensor manager...')
        self.sensor_manager = SensorManager(
            timeout=self.config.system.sensor_timeout,
            stale_warning=self.config.system.sensor_stale_warning
        )
        
        # Initialize GPS calculator
        self.get_logger().info('Initializing GPS calculator...')
        self.gps_calculator = GPSCalculator(
            intrinsic_matrix=self.config.camera.intrinsic_matrix,
            earth_radius_lat=self.config.system.gps_earth_radius_lat
        )
        
        # Initialize video receiver
        self.get_logger().info('Initializing video receiver...')
        pipeline_str = self.config.gstreamer.get_pipeline_string()
        self.video_receiver = VideoFrameReceiver(
            pipeline_str=pipeline_str,
            frame_callback=self._on_frame_received
        )
        
        # Initialize detector
        self.get_logger().info('Loading detection model...')
        self.detector = IsaacDetectorWrapper(
            model_path=self.config.system.model_path,
            device=self.config.system.model_device,
            save_dir=self.config.system.log_dir
        )
        
        # Initialize ROS2 publishers
        self.get_logger().info('Initializing ROS2 publishers...')
        self.publisher_manager = ROSPublisherManager(self, self.config.ros2.qos)
        self.publisher_manager.create_drone_gps_publisher(
            self.config.ros2.output_topics['drone_gps']
        )
        self.publisher_manager.create_casualties_gps_publisher(
            self.config.ros2.output_topics['casualties_gps']
        )
        
        # Initialize ROS2 subscribers
        self.get_logger().info('Initializing ROS2 subscribers...')
        qos_profile = self.publisher_manager._create_qos_profile(self.config.ros2.qos['mavros'])
        self.subscriber_manager = ROSSubscriberManager(self, qos_profile)
        
        # Subscribe to sensor topics
        self.subscriber_manager.subscribe_gps(
            self.config.ros2.input_topics['gps'],
            self._gps_callback
        )
        self.subscriber_manager.subscribe_compass(
            self.config.ros2.input_topics['compass'],
            self._compass_callback
        )
        self.subscriber_manager.subscribe_altitude(
            self.config.ros2.input_topics['altitude'],
            self._altitude_callback
        )
        self.subscriber_manager.subscribe_gimbal(
            self.config.ros2.input_topics['gimbal'],
            self._gimbal_callback
        )
        self.subscriber_manager.subscribe_trigger(
            '/trigger_capture',
            self._trigger_callback
        )
        
        # Create capture service
        self.capture_service = self.create_service(
            Trigger,
            self.config.ros2.services['trigger_capture'],
            self._handle_capture_service
        )
        
        # Thread pool
        self.thread_pool = ThreadPoolExecutor(
            max_workers=self.config.system.thread_pool_max_workers
        )
        
        # Timers
        # Sensor health check
        self.health_timer = self.create_timer(1.0, self._check_sensor_health)
        # Drone GPS relay
        self.gps_relay_timer = self.create_timer(1.0, self._relay_drone_gps)
        
        # Store latest GPS data for relay
        self._latest_gps_data = None
        
        self.get_logger().info('✅ Vision inference node initialization complete')
        self._print_system_info()
    
    def _print_system_info(self):
        """Print system information"""
        self.get_logger().info('='*60)
        self.get_logger().info('System Configuration:')
        self.get_logger().info(f'  Camera: {self.config.camera.name}')
        self.get_logger().info(f'  Resolution: {self.config.camera.width}x{self.config.camera.height}')
        self.get_logger().info(f'  Model path: {self.config.system.model_path}')
        self.get_logger().info(f'  Device: {self.config.system.model_device}')
        self.get_logger().info(f'  Log directory: {self.config.system.log_dir}')
        self.get_logger().info('='*60)
    
    def _on_frame_received(self, frame, timestamp):
        """Video frame reception callback"""
        frame_count = self.video_receiver.get_frame_count()
        if frame_count % 1000 == 0:
            self.get_logger().info(f'✓ Received {frame_count} frames')
    
    def _gps_callback(self, msg):
        """GPS callback"""
        self.sensor_manager.update_gps(msg.latitude, msg.longitude)
        self._latest_gps_data = msg
    
    def _compass_callback(self, msg):
        """Compass callback"""
        self.sensor_manager.update_heading(msg.data)
    
    def _altitude_callback(self, msg):
        """Altitude callback"""
        self.sensor_manager.update_altitude(msg.data)
    
    def _gimbal_callback(self, msg):
        """Gimbal attitude callback"""
        self.sensor_manager.update_gimbal(msg.y, msg.x, msg.z)  # roll, pitch, yaw
    
    def _trigger_callback(self, msg):
        """Trigger topic callback"""
        if msg.data:
            self.get_logger().info('📸 Capture triggered from Foxglove')
            self._capture_and_process()
    
    def _handle_capture_service(self, request, response):
        """Capture service callback"""
        self.get_logger().info('📸 Capture triggered via service')
        
        frame_data = self.video_receiver.get_latest_frame()
        if frame_data is None:
            response.success = False
            response.message = 'No video frame available'
            self.get_logger().warn('⚠️  No video frame available')
            return response
        
        frame, timestamp, frame_number = frame_data
        frame_age = time.time() - timestamp
        
        response.success = True
        response.message = f'Capture successful, frame#{frame_number}, size: {frame.shape}, age: {frame_age:.3f}s'
        
        # Async processing
        self._capture_and_process()
        
        return response
    
    def _capture_and_process(self):
        """Capture and process"""
        # Get latest frame
        frame_data = self.video_receiver.get_latest_frame()
        if frame_data is None:
            self.get_logger().warn('⚠️  No video frame available')
            return
        
        frame, timestamp, frame_number = frame_data
        frame_age = time.time() - timestamp
        
        self.get_logger().info(
            f'📷 Captured frame #{frame_number}, size: {frame.shape}, age: {frame_age:.3f}s'
        )
        
        # Get sensor snapshot
        capture_time = time.time()
        sensor_snapshot = self.sensor_manager.get_snapshot(capture_time)
        
        # Print sensor data
        snapshot_str = self.sensor_manager.format_snapshot(sensor_snapshot)
        for line in snapshot_str.split('\n'):
            self.get_logger().info(line)
        
        # Async processing
        self.thread_pool.submit(
            self._process_frame,
            frame, frame_number, sensor_snapshot
        )
    
    def _process_frame(self, frame, frame_number, sensor_snapshot):
        """Process frame (inference + GPS estimation + publishing)"""
        try:
            # 1. Run detection
            self.get_logger().info(f'🔄 Running inference (frame #{frame_number})...')
            detections = self.detector.detect(frame)
            
            self.get_logger().info(f'✅ Detected {len(detections)} targets')
            for i, (cx, cy) in enumerate(detections, 1):
                self.get_logger().info(f'  Target {i}: ({cx:.1f}, {cy:.1f})')
            
            # 2. GPS estimation
            successful_count = 0
            for i, (cx, cy) in enumerate(detections, 1):
                try:
                    gps_result = self.gps_calculator.estimate_from_snapshot(
                        cx, cy, sensor_snapshot
                    )
                    
                    lat = gps_result['estimated_latitude']
                    lon = gps_result['estimated_longitude']
                    dist = gps_result['lateral_distance']
                    
                    self.get_logger().info(
                        f'  Target {i}: GPS ({lat:.6f}, {lon:.6f}), distance: {dist:.1f}m'
                    )
                    
                    # 3. Publish results
                    self.publisher_manager.publish_casualties_gps(lat, lon, frame)
                    successful_count += 1
                    
                except Exception as e:
                    self.get_logger().error(f'  Target {i} GPS estimation failed: {e}')
            
            self.get_logger().info(f'✅ Successfully processed {successful_count}/{len(detections)} targets')
            
        except Exception as e:
            self.get_logger().error(f'Frame processing failed: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())
    
    def _check_sensor_health(self):
        """Check sensor health status"""
        uptime = time.time() - self.node_start_time
        warnings = self.sensor_manager.check_health(
            uptime,
            self.config.system.warning_interval
        )
        
        for sensor, message in warnings.items():
            self.get_logger().warn(f'⚠️  {message}')
    
    def _relay_drone_gps(self):
        """Relay drone GPS"""
        if self._latest_gps_data is not None:
            self.publisher_manager.publish_drone_gps(self._latest_gps_data)
            self.get_logger().debug(
                f'Relay drone GPS: lat={self._latest_gps_data.latitude:.6f}, '
                f'lon={self._latest_gps_data.longitude:.6f}'
            )
    
    def destroy_node(self):
        """Clean up resources"""
        self.get_logger().info('🛑 Shutting down node...')
        self.video_receiver.stop()
        self.thread_pool.shutdown(wait=True)
        super().destroy_node()


def main(args=None):
    """Main function"""
    rclpy.init(args=args)
    
    # Optional: get config directory from command line args
    config_dir = None
    if args and '--config-dir' in args:
        idx = args.index('--config-dir')
        if idx + 1 < len(args):
            config_dir = args[idx + 1]
    
    node = VisionInferenceNode(config_dir)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
