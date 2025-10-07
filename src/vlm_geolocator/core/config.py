"""Configuration Management Module"""
import yaml
from pathlib import Path
from typing import Dict, Any
from dataclasses import dataclass
import numpy as np


@dataclass
class CameraConfig:
    """Camera Configuration"""
    name: str
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    
    @property
    def intrinsic_matrix(self) -> np.ndarray:
        """Return camera intrinsic matrix"""
        return np.array([
            [self.fx, 0, self.cx],
            [0, self.fy, self.cy],
            [0, 0, 1]
        ])


@dataclass
class GStreamerConfig:
    """GStreamer Configuration"""
    udp_port: int
    buffer_size: int
    latency: int
    encoding: str
    max_buffers: int
    drop: bool
    
    def get_pipeline_string(self) -> str:
        """Generate GStreamer pipeline string"""
        return f"""
            udpsrc port={self.udp_port} buffer-size={self.buffer_size}
            ! application/x-rtp,media=video,clock-rate=90000,encoding-name={self.encoding},payload=96
            ! rtpjitterbuffer latency={self.latency}
            ! rtph265depay
            ! h265parse
            ! avdec_h265
            ! videoconvert
            ! video/x-raw,format=RGB
            ! appsink name=sink emit-signals=true max-buffers={self.max_buffers} drop={str(self.drop).lower()}
        """


@dataclass
class ROS2Config:
    """ROS2 Configuration"""
    input_topics: Dict[str, str]
    output_topics: Dict[str, str]
    services: Dict[str, str]
    qos: Dict[str, Any]


@dataclass
class SystemConfig:
    """System Configuration"""
    sensor_timeout: float
    sensor_stale_warning: float
    warning_interval: float
    startup_grace_period: float
    log_dir: str
    model_path: str
    model_device: str
    model_dtype: str
    gps_earth_radius_lat: float
    thread_pool_max_workers: int


class ConfigManager:
    """Configuration Manager"""
    
    def __init__(self, config_dir: str = None):
        if config_dir is None:
            # Default config directory in project root
            config_dir = Path(__file__).parent.parent.parent.parent / "config"
        else:
            config_dir = Path(config_dir)
        
        self.config_dir = config_dir
        self._load_all_configs()
    
    def _load_yaml(self, filename: str) -> Dict[str, Any]:
        """Load YAML file"""
        filepath = self.config_dir / filename
        if not filepath.exists():
            raise FileNotFoundError(f"Configuration file does not exist: {filepath}")
        
        with open(filepath, 'r', encoding='utf-8') as f:
            return yaml.safe_load(f)
    
    def _load_all_configs(self):
        """Load all configurations"""
        # Load camera configuration
        camera_data = self._load_yaml("camera_config.yaml")
        self.camera = CameraConfig(
            name=camera_data['camera']['name'],
            width=camera_data['camera']['resolution']['width'],
            height=camera_data['camera']['resolution']['height'],
            fx=camera_data['camera']['intrinsics']['fx'],
            fy=camera_data['camera']['intrinsics']['fy'],
            cx=camera_data['camera']['intrinsics']['cx'],
            cy=camera_data['camera']['intrinsics']['cy']
        )
        
        # Load GStreamer configuration
        gst_data = self._load_yaml("camera_config.yaml")['gstreamer']
        self.gstreamer = GStreamerConfig(
            udp_port=gst_data['udp_port'],
            buffer_size=gst_data['buffer_size'],
            latency=gst_data['latency'],
            encoding=gst_data['encoding'],
            max_buffers=gst_data['max_buffers'],
            drop=gst_data['drop']
        )
        
        # Load ROS2 configuration
        ros_data = self._load_yaml("ros_config.yaml")['ros2']
        self.ros2 = ROS2Config(
            input_topics=ros_data['input_topics'],
            output_topics=ros_data['output_topics'],
            services=ros_data['services'],
            qos=ros_data['qos']
        )
        
        # Load system configuration
        system_data = self._load_yaml("system_config.yaml")['system']
        self.system = SystemConfig(
            sensor_timeout=system_data['sensor_timeout'],
            sensor_stale_warning=system_data['sensor_stale_warning'],
            warning_interval=system_data['warning_interval'],
            startup_grace_period=system_data['startup_grace_period'],
            log_dir=system_data['log_dir'],
            model_path=system_data['model']['path'],
            model_device=system_data['model']['device'],
            model_dtype=system_data['model']['dtype'],
            gps_earth_radius_lat=system_data['gps']['earth_radius_lat'],
            thread_pool_max_workers=system_data['thread_pool']['max_workers']
        )
    
    def get_camera_config(self) -> CameraConfig:
        """Get camera configuration"""
        return self.camera
    
    def get_gstreamer_config(self) -> GStreamerConfig:
        """Get GStreamer configuration"""
        return self.gstreamer
    
    def get_ros2_config(self) -> ROS2Config:
        """Get ROS2 configuration"""
        return self.ros2
    
    def get_system_config(self) -> SystemConfig:
        """Get system configuration"""
        return self.system
