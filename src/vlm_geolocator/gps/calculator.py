"""GPS Coordinate Calculation Module"""
import numpy as np
from typing import Dict, Any, Tuple
from scipy.spatial.transform import Rotation


class GPSCalculator:
    """GPS Coordinate Estimator"""
    
    def __init__(self, intrinsic_matrix: np.ndarray, earth_radius_lat: float = 111111.0):
        """
        Args:
            intrinsic_matrix: Camera intrinsic matrix
            earth_radius_lat: Earth radius in latitude direction (meters/degree)
        """
        self.intrinsic = intrinsic_matrix
        self.intrinsic_inv = np.linalg.inv(intrinsic_matrix)
        self.earth_radius_lat = earth_radius_lat
    
    def estimate_target_gps(
        self,
        pixel_x: float,
        pixel_y: float,
        drone_gps: Tuple[float, float],
        drone_heading: float,
        drone_altitude: float,
        gimbal_attitude: Tuple[float, float, float]
    ) -> Dict[str, float]:
        """
        Estimate target GPS coordinates from pixel coordinates
        
        Args:
            pixel_x: Pixel X coordinate
            pixel_y: Pixel Y coordinate
            drone_gps: Drone GPS (latitude, longitude)
            drone_heading: Drone heading (radians)
            drone_altitude: Drone altitude (meters)
            gimbal_attitude: Gimbal attitude (roll, pitch, yaw radians)
            
        Returns:
            Dictionary containing estimated GPS coordinates
        """
        # Step 1: Convert pixel to normalized camera coordinates
        pixel_coords = np.array([pixel_x, pixel_y, 1.0])
        normalized_coords = self.intrinsic_inv @ pixel_coords
        
        # Step 2: Define camera-to-drone transformation at gimbal zero position
        # Camera frame: X=right, Y=down, Z=forward (optical axis)
        # ENU frame: X=east, Y=north, Z=up
        # When gimbal at (0,0,0), camera points forward (north in ENU)
        R_camera_to_drone_base = np.array([
            [1, 0, 0],    # ENU X (east) <- Camera X (right)
            [0, 0, 1],    # ENU Y (north) <- Camera Z (forward)
            [0, -1, 0]    # ENU Z (up) <- Camera -Y (up is opposite of down)
        ])
        
        # Step 3: Apply gimbal rotations
        roll, pitch, yaw = gimbal_attitude
        
        # Yaw rotation (about body Z-axis)
        R_yaw_body = Rotation.from_euler('z', -yaw, degrees=False).as_matrix()
        
        # Apply Yaw to base transformation
        R_cam2body = R_yaw_body @ R_camera_to_drone_base
        
        # Apply Pitch (about camera local X-axis)
        R_cam2body = R_cam2body @ Rotation.from_euler('x', pitch, degrees=False).as_matrix()
        
        # Apply Roll (about camera local Z-axis)
        R_cam2body = R_cam2body @ Rotation.from_euler('z', roll, degrees=False).as_matrix()
        
        # Step 4: Transform ray direction to drone body frame
        ray_camera = normalized_coords
        ray_drone = R_cam2body @ ray_camera
        
        # Step 5: Check ray is pointing downward
        if ray_drone[2] >= 0:
            raise ValueError(
                f"Ray not pointing downward!\n"
                f"Pixel: [{pixel_x:.3f}, {pixel_y:.3f}]\n"
                f"Ray in camera frame: {ray_camera}\n"
                f"Ray in drone frame: {ray_drone}\n"
                f"Gimbal attitude: roll={np.degrees(roll):.2f}°, "
                f"pitch={np.degrees(pitch):.2f}°, yaw={np.degrees(yaw):.2f}°"
            )
        
        # Step 6: Calculate ground intersection (in drone body ENU frame)
        scale_factor = -drone_altitude / ray_drone[2]
        ground_point_body = scale_factor * ray_drone
        
        body_east = ground_point_body[0]
        body_north = ground_point_body[1]
        
        # Step 7: Transform from drone body frame to world frame
        cos_heading = np.cos(drone_heading)
        sin_heading = np.sin(drone_heading)
        
        north_offset = body_north * cos_heading - body_east * sin_heading
        east_offset = body_north * sin_heading + body_east * cos_heading
        
        # Step 8: Calculate GPS coordinates
        lat, lon = drone_gps
        lat_rad = np.radians(lat)
        
        # WGS84 approximation conversion
        meters_per_deg_lon = self.earth_radius_lat * np.cos(lat_rad)
        
        lat_offset_deg = north_offset / self.earth_radius_lat
        lon_offset_deg = east_offset / meters_per_deg_lon
        
        estimated_lat = lat + lat_offset_deg
        estimated_lon = lon + lon_offset_deg
        
        return {
            "estimated_latitude": estimated_lat,
            "estimated_longitude": estimated_lon,
            "offset_north": north_offset,
            "offset_east": east_offset,
            "lateral_distance": np.sqrt(east_offset**2 + north_offset**2)
        }
    
    def estimate_from_snapshot(
        self,
        pixel_x: float,
        pixel_y: float,
        sensor_snapshot: Dict[str, Any]
    ) -> Dict[str, Any]:
        """
        Estimate GPS coordinates using sensor snapshot
        
        Args:
            pixel_x: Pixel X coordinate
            pixel_y: Pixel Y coordinate
            sensor_snapshot: Sensor data snapshot
            
        Returns:
            Estimation result dictionary
        """
        # Verify all required data exists
        missing_data = []
        if sensor_snapshot['gps'] is None:
            missing_data.append("GPS")
        if sensor_snapshot['heading'] is None:
            missing_data.append("Heading")
        if sensor_snapshot['altitude'] is None:
            missing_data.append("Altitude")
        if sensor_snapshot['gimbal'] is None:
            missing_data.append("Gimbal Attitude")
        
        if missing_data:
            raise ValueError(
                f"Cannot estimate GPS for pixel [{pixel_x:.1f}, {pixel_y:.1f}]: "
                f"Missing sensor data: {', '.join(missing_data)}"
            )
        
        return self.estimate_target_gps(
            pixel_x=pixel_x,
            pixel_y=pixel_y,
            drone_gps=sensor_snapshot['gps'],
            drone_heading=sensor_snapshot['heading'],
            drone_altitude=sensor_snapshot['altitude'],
            gimbal_attitude=sensor_snapshot['gimbal']
        )
