"""
VLM Geolocator - 视觉语言模型地理定位系统

一个模块化的系统，用于从无人机视频流中检测目标并估算其GPS坐标。
"""
__version__ = '2.0.0'
__author__ = 'VLM Geolocator Team'

from . import core
from . import sensors
from . import gps
from . import vision
from . import ros_interface

__all__ = [
    'core',
    'sensors',
    'gps',
    'vision',
    'ros_interface'
]

