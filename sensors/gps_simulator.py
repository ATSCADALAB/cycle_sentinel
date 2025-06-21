"""
Cycle Sentinel GPS Simulator
Bộ mô phỏng GPS cho Cycle Sentinel

This module simulates GPS data for testing without real GPS hardware.
Module này mô phỏng dữ liệu GPS để test mà không cần phần cứng GPS thật.

Features / Tính năng:
- Realistic GPS track simulation / Mô phỏng track GPS thực tế
- Speed violation scenarios / Kịch bản vi phạm tốc độ  
- Route following with waypoints / Theo dõi tuyến đường với waypoints
- Configurable noise and accuracy / Nhiễu và độ chính xác có thể cấu hình
- Ho Chi Minh City street simulation / Mô phỏng đường phố TP.HCM
- Multiple test scenarios / Nhiều kịch bản test khác nhau

Author: Cycle Sentinel Team
Version: 1.0.0
"""

import time
import math
import random
import json
from datetime import datetime, timedelta
from typing import Dict, Any, List, Tuple, Optional
from dataclasses import dataclass
from enum import Enum
import threading

# Import cấu hình và logging / Import config and logging
from config.settings import GPS_CONFIG, GPS_DATA_SCHEMA, SYSTEM_CONFIG, MAPS_DIR
from utils.logger import get_logger

class SimulationMode(Enum):
    """
    Chế độ mô phỏng khác nhau
    Different simulation modes
    """
    STATIC = "static"                    # Đứng yên / Stationary
    NORMAL_RIDE = "normal_ride"          # Đi bình thường / Normal riding
    SPEED_VIOLATION = "speed_violation"  # Vi phạm tốc độ / Speed violation
    ZONE_VIOLATION = "zone_violation"    # Vi phạm khu vực / Zone violation
    MIXED_SCENARIO = "mixed_scenario"    # Kịch bản hỗn hợp / Mixed scenario
    CUSTOM_ROUTE = "custom_route"        # Tuyến đường tùy chỉnh / Custom route

@dataclass
class Waypoint:
    """
    Điểm waypoint trong tuyến đường
    Waypoint in route
    """
    latitude: float
    longitude: float
    speed_limit: float  # km/h
    zone_name: str
    is_restricted: bool = False

@dataclass
class SimulationConfig:
    """
    Cấu hình mô phỏng
    Simulation configuration
    """
    mode: SimulationMode
    duration_seconds: float
    update_interval: float = 1.0  # seconds
    noise_level: float = 0.1      # GPS noise factor
    accuracy_meters: float = 5.0  # Simulated GPS accuracy
    satellites: int = 8           # Number of satellites
    start_position: Tuple[float, float] = (10.7749, 106.7004)  # HCM City center

class GPSSimulator:
    """
    Bộ mô phỏng GPS cho Cycle Sentinel
    GPS simulator for Cycle Sentinel
    
    Provides realistic GPS data simulation for testing violation detection
    Cung cấp mô phỏng dữ liệu GPS thực tế để test phát hiện vi phạm
    """
    
    def __init__(self, config: Optional[SimulationConfig] = None):
        """
        Khởi tạo GPS simulator
        Initialize GPS simulator
        
        Args:
            config: Cấu hình mô phỏng / Simulation configuration
        """
        self.logger = get_logger()
        
        # Cấu hình mặc định / Default configuration
        self.config = config or SimulationConfig(
            mode=SimulationMode.NORMAL_RIDE,
            duration_seconds=300,  # 5 minutes
            update_interval=1.0
        )
        
        # Trạng thái mô phỏng / Simulation state
        self.is_running = False
        self.start_time = None
        self.current_position = self.config.start_position
        self.current_speed = 0.0  # km/h
        self.current_course = 0.0  # degrees
        self.waypoints: List[Waypoint] = []
        self.current_waypoint_index = 0
        
        # Thread cho mô phỏng / Simulation thread
        self._simulation_thread = None
        self._stop_event = threading.Event()
        
        # Dữ liệu mô phỏng / Simulation data
        self._current_data: Optional[Dict[str, Any]] = None
        self._data_lock = threading.Lock()
        
        # Load tuyến đường mặc định / Load default routes
        self._load_default_routes()
        
        self.logger.system_info("GPS Simulator initialized", {
            "mode": self.config.mode.value,
            "duration": self.config.duration_seconds,
            "start_position": self.config.start_position
        })
    
    def _load_default_routes(self):
        """
        Load các tuyến đường mặc định cho TP.HCM
        Load default routes for Ho Chi Minh City
        """
        # Tuyến đường qua trung tâm TP.HCM / Route through HCM City center
        hcm_route = [
            Waypoint(10.7749, 106.7004, 25, "District 1 - Center"),
            Waypoint(10.7760, 106.7010, 20, "Nguyen Hue Walking Street"),  
            Waypoint(10.7770, 106.7020, 15, "School Zone"),  # Speed limit 15
            Waypoint(10.7780, 106.7030, 25, "Commercial Area"),
            Waypoint(10.7790, 106.7040, 30, "Main Road"),
            Waypoint(10.7800, 106.7050, 25, "Residential Area"),
            Waypoint(10.7810, 106.7060, 20, "Hospital Zone"),  # Speed limit 20
            Waypoint(10.7820, 106.7070, 0, "Pedestrian Only", True),  # Restricted zone
            Waypoint(10.7830, 106.7080, 25, "District 3 Border")
        ]
        
        self.default_routes = {
            "hcm_center": hcm_route
        }
        
        # Set route mặc định / Set default route
        self.waypoints = hcm_route.copy()
    
    def start_simulation(self) -> bool:
        """
        Bắt đầu mô phỏng GPS
        Start GPS simulation
        
        Returns:
            bool: True nếu bắt đầu thành công / True if started successfully
        """
        if self.is_running:
            self.logger.system_warning("GPS simulation already running")
            return False
        
        try:
            self.is_running = True
            self.start_time = datetime.now()
            self._stop_event.clear()
            
            # Bắt đầu thread mô phỏng / Start simulation thread
            self._simulation_thread = threading.Thread(
                target=self._simulation_loop,
                daemon=True
            )
            self._simulation_thread.start()
            
            self.logger.system_info("GPS simulation started", {
                "mode": self.config.mode.value,
                "waypoints": len(self.waypoints)
            })
            
            return True
            
        except Exception as e:
            self.logger.system_error("Failed to start GPS simulation", {
                "error": str(e)
            }, e)
            self.is_running = False
            return False
    
    def stop_simulation(self):
        """
        Dừng mô phỏng GPS
        Stop GPS simulation
        """
        if not self.is_running:
            return
        
        self.logger.system_info("Stopping GPS simulation")
        
        self.is_running = False
        self._stop_event.set()
        
        if self._simulation_thread and self._simulation_thread.is_alive():
            self._simulation_thread.join(timeout=2.0)
        
        self.logger.system_info("GPS simulation stopped")
    
    def _simulation_loop(self):
        """
        Vòng lặp chính của mô phỏng
        Main simulation loop
        """
        try:
            while self.is_running and not self._stop_event.is_set():
                # Tính thời gian đã trôi qua / Calculate elapsed time
                elapsed = (datetime.now() - self.start_time).total_seconds()
                
                # Kiểm tra timeout / Check timeout
                if elapsed >= self.config.duration_seconds:
                    self.logger.system_info("Simulation duration completed")
                    break
                
                # Generate GPS data theo mode / Generate GPS data by mode
                gps_data = self._generate_gps_data(elapsed)
                
                # Cập nhật dữ liệu hiện tại / Update current data
                with self._data_lock:
                    self._current_data = gps_data
                
                # Log GPS data / Log GPS data
                self.logger.log_gps_data(gps_data)
                
                # Chờ interval tiếp theo / Wait for next interval
                time.sleep(self.config.update_interval)
                
        except Exception as e:
            self.logger.system_error("Simulation loop error", {"error": str(e)}, e)
        finally:
            self.is_running = False
    
    def _generate_gps_data(self, elapsed_time: float) -> Dict[str, Any]:
        """
        Generate dữ liệu GPS theo thời gian đã trôi qua
        Generate GPS data based on elapsed time
        
        Args:
            elapsed_time: Thời gian đã trôi qua (giây) / Elapsed time (seconds)
            
        Returns:
            Dict: GPS data theo GPS_DATA_SCHEMA
        """
        if self.config.mode == SimulationMode.STATIC:
            return self._generate_static_data()
        elif self.config.mode == SimulationMode.NORMAL_RIDE:
            return self._generate_normal_ride_data(elapsed_time)
        elif self.config.mode == SimulationMode.SPEED_VIOLATION:
            return self._generate_speed_violation_data(elapsed_time)
        elif self.config.mode == SimulationMode.ZONE_VIOLATION:
            return self._generate_zone_violation_data(elapsed_time)
        elif self.config.mode == SimulationMode.MIXED_SCENARIO:
            return self._generate_mixed_scenario_data(elapsed_time)
        else:
            return self._generate_normal_ride_data(elapsed_time)
    
    def _generate_static_data(self) -> Dict[str, Any]:
        """
        Generate dữ liệu GPS cho trạng thái đứng yên
        Generate GPS data for stationary state
        """
        # Thêm noise nhỏ để mô phỏng GPS drift / Add small noise to simulate GPS drift
        noise_lat = random.uniform(-0.00001, 0.00001) * self.config.noise_level
        noise_lon = random.uniform(-0.00001, 0.00001) * self.config.noise_level
        
        return {
            "timestamp": datetime.now().isoformat(),
            "latitude": self.current_position[0] + noise_lat,
            "longitude": self.current_position[1] + noise_lon,
            "speed_kmh": 0.0,
            "course": self.current_course,
            "altitude": 10.0,  # Sea level for HCM
            "accuracy": self.config.accuracy_meters,
            "satellites": self.config.satellites,
            "valid": True
        }
    
    def _generate_normal_ride_data(self, elapsed_time: float) -> Dict[str, Any]:
        """
        Generate dữ liệu GPS cho đi xe bình thường
        Generate GPS data for normal riding
        """
        # Di chuyển theo waypoints / Move along waypoints
        if self.waypoints:
            self._update_position_along_route(elapsed_time, respect_speed_limits=True)
        
        # Thêm noise / Add noise
        noise_lat = random.uniform(-0.0001, 0.0001) * self.config.noise_level
        noise_lon = random.uniform(-0.0001, 0.0001) * self.config.noise_level
        speed_noise = random.uniform(-2, 2) * self.config.noise_level
        
        return {
            "timestamp": datetime.now().isoformat(),
            "latitude": self.current_position[0] + noise_lat,
            "longitude": self.current_position[1] + noise_lon,
            "speed_kmh": max(0, self.current_speed + speed_noise),
            "course": self.current_course,
            "altitude": 10.0,
            "accuracy": self.config.accuracy_meters,
            "satellites": self.config.satellites,
            "valid": True
        }
    
    def _generate_speed_violation_data(self, elapsed_time: float) -> Dict[str, Any]:
        """
        Generate dữ liệu GPS có vi phạm tốc độ
        Generate GPS data with speed violations
        """
        # Di chuyển theo waypoints nhưng vượt tốc độ / Move along waypoints but exceed speed
        if self.waypoints:
            self._update_position_along_route(elapsed_time, respect_speed_limits=False)
            
            # Thêm excess speed / Add excess speed
            current_waypoint = self.waypoints[self.current_waypoint_index]
            excess_speed = random.uniform(5, 15)  # 5-15 km/h over limit
            self.current_speed = current_waypoint.speed_limit + excess_speed
        
        # Thêm noise / Add noise
        noise_lat = random.uniform(-0.0001, 0.0001) * self.config.noise_level
        noise_lon = random.uniform(-0.0001, 0.0001) * self.config.noise_level
        
        return {
            "timestamp": datetime.now().isoformat(),
            "latitude": self.current_position[0] + noise_lat,
            "longitude": self.current_position[1] + noise_lon,
            "speed_kmh": self.current_speed,
            "course": self.current_course,
            "altitude": 10.0,
            "accuracy": self.config.accuracy_meters,
            "satellites": self.config.satellites,
            "valid": True
        }
    
    def _generate_zone_violation_data(self, elapsed_time: float) -> Dict[str, Any]:
        """
        Generate dữ liệu GPS có vi phạm khu vực cấm
        Generate GPS data with zone violations
        """
        # Tìm restricted zone và đi vào / Find restricted zone and enter it
        restricted_waypoints = [wp for wp in self.waypoints if wp.is_restricted]
        
        if restricted_waypoints and elapsed_time > 30:  # After 30 seconds, enter restricted zone
            target = restricted_waypoints[0]
            self.current_position = (target.latitude, target.longitude)
            self.current_speed = 15.0  # Moving through restricted zone
        else:
            # Di chuyển bình thường / Normal movement
            self._update_position_along_route(elapsed_time, respect_speed_limits=True)
        
        # Thêm noise / Add noise
        noise_lat = random.uniform(-0.0001, 0.0001) * self.config.noise_level
        noise_lon = random.uniform(-0.0001, 0.0001) * self.config.noise_level
        
        return {
            "timestamp": datetime.now().isoformat(),
            "latitude": self.current_position[0] + noise_lat,
            "longitude": self.current_position[1] + noise_lon,
            "speed_kmh": self.current_speed,
            "course": self.current_course,
            "altitude": 10.0,
            "accuracy": self.config.accuracy_meters,
            "satellites": self.config.satellites,
            "valid": True
        }
    
    def _generate_mixed_scenario_data(self, elapsed_time: float) -> Dict[str, Any]:
        """
        Generate dữ liệu GPS kịch bản hỗn hợp
        Generate GPS data for mixed scenario
        """
        # Đổi behavior theo thời gian / Change behavior over time
        phase_duration = self.config.duration_seconds / 4
        
        if elapsed_time < phase_duration:
            # Phase 1: Normal riding
            return self._generate_normal_ride_data(elapsed_time)
        elif elapsed_time < phase_duration * 2:
            # Phase 2: Speed violation
            return self._generate_speed_violation_data(elapsed_time)
        elif elapsed_time < phase_duration * 3:
            # Phase 3: Normal riding again
            return self._generate_normal_ride_data(elapsed_time)
        else:
            # Phase 4: Zone violation
            return self._generate_zone_violation_data(elapsed_time)
    
    def _update_position_along_route(self, elapsed_time: float, respect_speed_limits: bool = True):
        """
        Cập nhật vị trí theo tuyến đường
        Update position along route
        
        Args:
            elapsed_time: Thời gian đã trôi qua / Elapsed time
            respect_speed_limits: Có tuân thủ giới hạn tốc độ không / Whether to respect speed limits
        """
        if not self.waypoints or self.current_waypoint_index >= len(self.waypoints) - 1:
            return
        
        current_wp = self.waypoints[self.current_waypoint_index]
        next_wp = self.waypoints[self.current_waypoint_index + 1]
        
        # Tính khoảng cách đến waypoint tiếp theo / Calculate distance to next waypoint
        distance = self._calculate_distance(
            self.current_position[0], self.current_position[1],
            next_wp.latitude, next_wp.longitude
        )
        
        # Set speed theo zone / Set speed according to zone
        if respect_speed_limits:
            self.current_speed = current_wp.speed_limit * random.uniform(0.7, 0.95)  # Slightly under limit
        else:
            self.current_speed = current_wp.speed_limit * random.uniform(1.1, 1.4)  # Over limit
        
        # Tính bearing đến waypoint tiếp theo / Calculate bearing to next waypoint
        self.current_course = self._calculate_bearing(
            self.current_position[0], self.current_position[1],
            next_wp.latitude, next_wp.longitude
        )
        
        # Di chuyển về phía waypoint tiếp theo / Move towards next waypoint
        if distance > 50:  # meters
            # Tính khoảng cách di chuyển / Calculate movement distance
            movement_distance = (self.current_speed / 3.6) * self.config.update_interval  # meters per update
            
            # Tính vị trí mới / Calculate new position
            new_lat, new_lon = self._move_towards_point(
                self.current_position[0], self.current_position[1],
                next_wp.latitude, next_wp.longitude,
                movement_distance
            )
            
            self.current_position = (new_lat, new_lon)
        else:
            # Đến waypoint, chuyển sang waypoint tiếp theo / Reached waypoint, move to next
            self.current_waypoint_index += 1
            self.current_position = (next_wp.latitude, next_wp.longitude)
    
    def _calculate_distance(self, lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        """
        Tính khoảng cách giữa 2 điểm GPS (Haversine formula)
        Calculate distance between two GPS points (Haversine formula)
        
        Returns:
            float: Khoảng cách tính bằng mét / Distance in meters
        """
        R = 6371000  # Earth radius in meters
        
        lat1_rad = math.radians(lat1)
        lat2_rad = math.radians(lat2)
        delta_lat = math.radians(lat2 - lat1)
        delta_lon = math.radians(lon2 - lon1)
        
        a = (math.sin(delta_lat/2) * math.sin(delta_lat/2) + 
             math.cos(lat1_rad) * math.cos(lat2_rad) * 
             math.sin(delta_lon/2) * math.sin(delta_lon/2))
        
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        distance = R * c
        
        return distance
    
    def _calculate_bearing(self, lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        """
        Tính bearing từ điểm 1 đến điểm 2
        Calculate bearing from point 1 to point 2
        
        Returns:
            float: Bearing in degrees (0-360)
        """
        lat1_rad = math.radians(lat1)
        lat2_rad = math.radians(lat2)
        delta_lon = math.radians(lon2 - lon1)
        
        y = math.sin(delta_lon) * math.cos(lat2_rad)
        x = (math.cos(lat1_rad) * math.sin(lat2_rad) - 
             math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(delta_lon))
        
        bearing = math.atan2(y, x)
        bearing_degrees = math.degrees(bearing)
        
        return (bearing_degrees + 360) % 360
    
    def _move_towards_point(self, lat1: float, lon1: float, lat2: float, lon2: float, 
                           distance: float) -> Tuple[float, float]:
        """
        Di chuyển từ điểm 1 về phía điểm 2 với khoảng cách cho trước
        Move from point 1 towards point 2 by given distance
        
        Args:
            lat1, lon1: Điểm bắt đầu / Starting point
            lat2, lon2: Điểm đích / Target point
            distance: Khoảng cách di chuyển (mét) / Movement distance (meters)
            
        Returns:
            Tuple[float, float]: Vị trí mới (lat, lon) / New position (lat, lon)
        """
        R = 6371000  # Earth radius in meters
        
        bearing = self._calculate_bearing(lat1, lon1, lat2, lon2)
        bearing_rad = math.radians(bearing)
        
        lat1_rad = math.radians(lat1)
        lon1_rad = math.radians(lon1)
        
        lat2_new = math.asin(math.sin(lat1_rad) * math.cos(distance / R) +
                            math.cos(lat1_rad) * math.sin(distance / R) * math.cos(bearing_rad))
        
        lon2_new = lon1_rad + math.atan2(math.sin(bearing_rad) * math.sin(distance / R) * math.cos(lat1_rad),
                                        math.cos(distance / R) - math.sin(lat1_rad) * math.sin(lat2_new))
        
        return math.degrees(lat2_new), math.degrees(lon2_new)
    
    def read_position(self) -> Optional[Dict[str, Any]]:
        """
        Đọc vị trí hiện tại từ simulator (tương thích với GPSHandler)
        Read current position from simulator (compatible with GPSHandler)
        
        Returns:
            Dict: GPS data hoặc None nếu chưa start
        """
        if not self.is_running:
            return None
        
        with self._data_lock:
            return self._current_data.copy() if self._current_data else None
    
    def connect(self) -> bool:
        """
        Giả lập kết nối GPS (tương thích với GPSHandler)
        Simulate GPS connection (compatible with GPSHandler)
        """
        return self.start_simulation()
    
    def disconnect(self):
        """
        Giả lập ngắt kết nối GPS (tương thích với GPSHandler)
        Simulate GPS disconnection (compatible with GPSHandler)
        """
        self.stop_simulation()
    
    def is_position_valid(self) -> bool:
        """
        Kiểm tra vị trí có hợp lệ không (tương thích với GPSHandler)
        Check if position is valid (compatible with GPSHandler)
        """
        return self.is_running and self._current_data is not None
    
    def get_statistics(self) -> Dict[str, Any]:
        """
        Lấy thống kê simulator
        Get simulator statistics
        """
        elapsed = 0
        if self.start_time:
            elapsed = (datetime.now() - self.start_time).total_seconds()
        
        return {
            "is_running": self.is_running,
            "elapsed_time": elapsed,
            "simulation_mode": self.config.mode.value,
            "current_waypoint": self.current_waypoint_index,
            "total_waypoints": len(self.waypoints),
            "current_speed": self.current_speed,
            "current_position": self.current_position
        }
    
    def set_custom_route(self, waypoints: List[Waypoint]):
        """
        Thiết lập tuyến đường tùy chỉnh
        Set custom route
        
        Args:
            waypoints: Danh sách waypoints / List of waypoints
        """
        self.waypoints = waypoints.copy()
        self.current_waypoint_index = 0
        self.logger.system_info(f"Custom route set with {len(waypoints)} waypoints")

# =============================================================================
# CONVENIENCE FUNCTIONS / HÀM TIỆN ÍCH
# =============================================================================

def create_speed_violation_scenario() -> GPSSimulator:
    """
    Tạo kịch bản vi phạm tốc độ
    Create speed violation scenario
    """
    config = SimulationConfig(
        mode=SimulationMode.SPEED_VIOLATION,
        duration_seconds=180,  # 3 minutes
        update_interval=1.0
    )
    return GPSSimulator(config)

def create_zone_violation_scenario() -> GPSSimulator:
    """
    Tạo kịch bản vi phạm khu vực
    Create zone violation scenario
    """
    config = SimulationConfig(
        mode=SimulationMode.ZONE_VIOLATION,
        duration_seconds=120,  # 2 minutes
        update_interval=1.0
    )
    return GPSSimulator(config)

def create_mixed_scenario() -> GPSSimulator:
    """
    Tạo kịch bản hỗn hợp
    Create mixed scenario
    """
    config = SimulationConfig(
        mode=SimulationMode.MIXED_SCENARIO,
        duration_seconds=300,  # 5 minutes
        update_interval=1.0
    )
    return GPSSimulator(config)

def load_route_from_file(filename: str) -> List[Waypoint]:
    """
    Load tuyến đường từ file JSON
    Load route from JSON file
    
    Args:
        filename: Tên file (trong thư mục maps/) / Filename (in maps/ directory)
        
    Returns:
        List[Waypoint]: Danh sách waypoints
    """
    try:
        filepath = MAPS_DIR / filename
        with open(filepath, 'r', encoding='utf-8') as f:
            data = json.load(f)
        
        waypoints = []
        for wp_data in data.get('waypoints', []):
            waypoint = Waypoint(
                latitude=wp_data['latitude'],
                longitude=wp_data['longitude'],
                speed_limit=wp_data.get('speed_limit', 25),
                zone_name=wp_data.get('zone_name', 'Unknown'),
                is_restricted=wp_data.get('is_restricted', False)
            )
            waypoints.append(waypoint)
        
        return waypoints
        
    except Exception as e:
        logger = get_logger()
        logger.system_error(f"Failed to load route from {filename}", {"error": str(e)}, e)
        return []

# =============================================================================
# TESTING / KIỂM TRA
# =============================================================================

if __name__ == "__main__":
    print("Testing Cycle Sentinel GPS Simulator...")
    
    # Test different scenarios
    scenarios = [
        ("Normal Ride", create_mixed_scenario),
        ("Speed Violation", create_speed_violation_scenario),
        ("Zone Violation", create_zone_violation_scenario)
    ]
    
    for scenario_name, create_func in scenarios:
        print(f"\n=== Testing {scenario_name} ===")
        
        simulator = create_func()
        
        if simulator.connect():
            print(f"✅ {scenario_name} simulation started")
            
            # Read some positions
            for i in range(10):
                position = simulator.read_position()
                if position:
                    print(f"Position {i+1}: {position['latitude']:.6f}, {position['longitude']:.6f}, "
                          f"Speed: {position['speed_kmh']:.1f} km/h")
                else:
                    print(f"Position {i+1}: No data")
                
                time.sleep(1)
            
            # Show statistics
            stats = simulator.get_statistics()
            print(f"Statistics: {stats}")
            
            simulator.disconnect()
            print(f"✅ {scenario_name} simulation stopped")
        else:
            print(f"❌ {scenario_name} simulation failed to start")
    
    print("\nGPS Simulator test completed!")