"""
Cycle Sentinel GPS Handler with Aggressive Stabilizer
Bộ xử lý GPS cho Cycle Sentinel với bộ ổn định tích hợp

This module handles GPS communication, NMEA parsing, and position tracking with advanced noise filtering.
Module này xử lý giao tiếp GPS, phân tích NMEA và theo dõi vị trí với bộ lọc nhiễu nâng cao.

Features / Tính năng:
- NMEA sentence parsing / Phân tích câu NMEA
- Position tracking and validation / Theo dõi và xác thực vị trí
- Speed calculation and smoothing / Tính toán và làm mượt tốc độ
- Aggressive GPS noise filtering / Lọc nhiễu GPS tích cực
- Position locking when stationary / Khóa vị trí khi đứng yên
- Connection management / Quản lý kết nối

Author: Cycle Sentinel Team
Version: 2.0.0 - With Aggressive Stabilizer
"""

import serial
import pynmea2
import time
import math
from datetime import datetime, timedelta
from typing import Optional, Dict, Any, List, Tuple
from dataclasses import dataclass
from threading import Lock
import queue

# Import cấu hình và logging / Import config and logging
from config.settings import GPS_CONFIG, GPS_DATA_SCHEMA, SYSTEM_CONFIG
from utils.logger import get_logger, PerformanceTimer

@dataclass
class GPSFix:
    """
    Thông tin về chất lượng tín hiệu GPS
    GPS fix quality information
    """
    timestamp: datetime
    latitude: float
    longitude: float
    altitude: float
    speed_kmh: float
    course: float
    accuracy: float
    satellites: int
    fix_quality: int  # 0=Invalid, 1=GPS fix, 2=DGPS fix
    hdop: float  # Horizontal Dilution of Precision
    valid: bool

class AggressiveGPSStabilizer:
    """
    Bộ ổn định GPS aggressive - ngăn GPS drift và noise hoàn toàn
    Aggressive GPS stabilizer - completely prevents GPS drift and noise
    """
    
    def __init__(self,
                 movement_threshold: float = 5.0,     # meters - ngưỡng di chuyển thực
                 speed_threshold: float = 4.0,        # km/h - ngưỡng tốc độ thực
                 stability_samples: int = 3,          # số samples để xác định ổn định
                 position_lock_radius: float = 3.0):  # meters - bán kính khóa vị trí
        """
        Khởi tạo GPS stabilizer với các ngưỡng aggressive
        """
        self.movement_threshold = movement_threshold
        self.speed_threshold = speed_threshold
        self.stability_samples = stability_samples
        self.position_lock_radius = position_lock_radius
        
        # State tracking
        self.stable_position: Optional[Tuple[float, float]] = None
        self.stable_count = 0
        self.is_position_locked = False
        self.last_valid_movement_time = None
        
        # History for analysis
        self.position_history = []
        self.speed_history = []
        self.max_history = 10
        
        # Statistics
        self.total_processed = 0
        self.drift_corrections = 0
        self.speed_corrections = 0
    
    def stabilize_gps_data(self, gps_data: Dict[str, Any]) -> Dict[str, Any]:
        """
        Ổn định GPS data - fix cả position và speed
        
        Args:
            gps_data: Raw GPS data
            
        Returns:
            Dict: Stabilized GPS data
        """
        self.total_processed += 1
        
        current_lat = gps_data["latitude"]
        current_lon = gps_data["longitude"]
        current_speed = gps_data["speed_kmh"]
        
        # Analyze movement
        is_really_moving = self._analyze_real_movement(current_lat, current_lon, current_speed)
        
        # Stabilize position
        stabilized_lat, stabilized_lon = self._stabilize_position(
            current_lat, current_lon, is_really_moving
        )
        
        # Stabilize speed
        stabilized_speed = self._stabilize_speed(current_speed, is_really_moving)
        
        # Create stabilized data
        stabilized_data = gps_data.copy()
        stabilized_data["latitude"] = stabilized_lat
        stabilized_data["longitude"] = stabilized_lon
        stabilized_data["speed_kmh"] = stabilized_speed
        
        # Update history
        self._update_history(current_lat, current_lon, current_speed)
        
        return stabilized_data
    
    def _analyze_real_movement(self, lat: float, lon: float, speed: float) -> bool:
        """
        Phân tích có thực sự di chuyển hay không
        
        Returns:
            bool: True nếu thực sự di chuyển
        """
        # Kiểm tra tốc độ
        speed_indicates_movement = speed >= self.speed_threshold
        
        # Kiểm tra khoảng cách từ vị trí ổn định
        distance_from_stable = 0.0
        if self.stable_position:
            distance_from_stable = self._calculate_distance(
                self.stable_position[0], self.stable_position[1], lat, lon
            )
        
        position_indicates_movement = distance_from_stable >= self.movement_threshold
        
        # Kiểm tra pattern từ lịch sử
        pattern_indicates_movement = self._analyze_movement_pattern()
        
        # Logic quyết định: CẦN CẢ 3 ĐIỀU KIỆN để coi là di chuyển thật
        really_moving = (
            speed_indicates_movement and 
            position_indicates_movement and 
            pattern_indicates_movement
        )
        
        return really_moving
    
    def _stabilize_position(self, lat: float, lon: float, is_moving: bool) -> Tuple[float, float]:
        """
        Ổn định vị trí GPS
        
        Returns:
            Tuple[float, float]: (stabilized_lat, stabilized_lon)
        """
        if not is_moving:
            # Đang đứng yên
            if not self.is_position_locked:
                # Chưa lock position
                if self.stable_position is None:
                    # Lần đầu tiên ổn định
                    self.stable_position = (lat, lon)
                    self.stable_count = 1
                else:
                    # Kiểm tra có gần vị trí ổn định không
                    distance = self._calculate_distance(
                        self.stable_position[0], self.stable_position[1], lat, lon
                    )
                    
                    if distance <= self.position_lock_radius:
                        # Gần vị trí ổn định → tăng counter
                        self.stable_count += 1
                        
                        # Cập nhật vị trí ổn định (làm mượt)
                        alpha = 0.1  # Smoothing factor
                        self.stable_position = (
                            self.stable_position[0] * (1 - alpha) + lat * alpha,
                            self.stable_position[1] * (1 - alpha) + lon * alpha
                        )
                        
                        # Lock position nếu đủ ổn định
                        if self.stable_count >= self.stability_samples:
                            self.is_position_locked = True
                    else:
                        # Xa vị trí ổn định → reset
                        self.stable_position = (lat, lon)
                        self.stable_count = 1
            
            # Nếu đã lock position → return vị trí ổn định
            if self.is_position_locked:
                self.drift_corrections += 1
                return self.stable_position
        
        else:
            # Đang di chuyển → unlock position
            self.is_position_locked = False
            self.stable_position = None
            self.stable_count = 0
            self.last_valid_movement_time = datetime.now()
        
        # Return vị trí gốc nếu không stabilize
        return (lat, lon)
    
    def _stabilize_speed(self, speed: float, is_moving: bool) -> float:
        """
        Ổn định tốc độ GPS
        
        Returns:
            float: Stabilized speed
        """
        if not is_moving:
            # Đang đứng yên → speed = 0
            if speed > 0:
                self.speed_corrections += 1
            return 0.0
        
        # Đang di chuyển → giữ nguyên speed nhưng có thể làm mượt
        if len(self.speed_history) >= 3:
            # Làm mượt với lịch sử
            recent_speeds = self.speed_history[-3:]
            avg_speed = sum(recent_speeds) / len(recent_speeds)
            
            # Weighted average: 70% history, 30% current
            smoothed_speed = avg_speed * 0.7 + speed * 0.3
            return smoothed_speed
        
        return speed
    
    def _analyze_movement_pattern(self) -> bool:
        """
        Phân tích pattern di chuyển từ lịch sử
        
        Returns:
            bool: True nếu pattern cho thấy di chuyển thật
        """
        if len(self.position_history) < 3:
            return False
        
        # Tính tổng khoảng cách di chuyển trong lịch sử gần đây
        total_distance = 0.0
        for i in range(len(self.position_history) - 2):
            pos1 = self.position_history[i]
            pos2 = self.position_history[i + 1]
            distance = self._calculate_distance(pos1[0], pos1[1], pos2[0], pos2[1])
            total_distance += distance
        
        # Tính khoảng cách trực tiếp từ điểm đầu đến cuối
        start_pos = self.position_history[0]
        end_pos = self.position_history[-1]
        direct_distance = self._calculate_distance(
            start_pos[0], start_pos[1], end_pos[0], end_pos[1]
        )
        
        # Tỷ lệ hiệu quả di chuyển
        if total_distance > 0:
            efficiency = direct_distance / total_distance
        else:
            efficiency = 0
        
        # Pattern cho thấy di chuyển thật nếu:
        # - Tổng khoảng cách > threshold
        # - Hiệu quả di chuyển > 0.3 (không đi lung tung)
        return total_distance > self.movement_threshold and efficiency > 0.3
    
    def _calculate_distance(self, lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        """Tính khoảng cách Haversine (meters)"""
        R = 6371000  # Earth radius in meters
        lat1_rad = math.radians(lat1)
        lat2_rad = math.radians(lat2)
        delta_lat = math.radians(lat2 - lat1)
        delta_lon = math.radians(lon2 - lon1)
        
        a = (math.sin(delta_lat/2) * math.sin(delta_lat/2) + 
             math.cos(lat1_rad) * math.cos(lat2_rad) * 
             math.sin(delta_lon/2) * math.sin(delta_lon/2))
        
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        return R * c
    
    def _update_history(self, lat: float, lon: float, speed: float):
        """Cập nhật lịch sử"""
        self.position_history.append((lat, lon))
        self.speed_history.append(speed)
        
        if len(self.position_history) > self.max_history:
            self.position_history.pop(0)
        if len(self.speed_history) > self.max_history:
            self.speed_history.pop(0)
    
    def get_stats(self) -> Dict[str, Any]:
        """Lấy thống kê stabilizer"""
        return {
            "total_processed": self.total_processed,
            "drift_corrections": self.drift_corrections,
            "speed_corrections": self.speed_corrections,
            "is_position_locked": self.is_position_locked,
            "stable_position": self.stable_position,
            "correction_rate": f"{(self.drift_corrections + self.speed_corrections) / max(1, self.total_processed) * 100:.1f}%"
        }

class GPSDataFilter:
    """
    Bộ lọc dữ liệu GPS để loại bỏ nhiễu
    GPS data filter to remove noise and invalid readings
    """
    
    def __init__(self, max_speed_change: float = 50.0, max_distance_jump: float = 100.0):
        """
        Khởi tạo bộ lọc GPS
        Initialize GPS filter
        
        Args:
            max_speed_change: Thay đổi tốc độ tối đa cho phép (km/h/s)
                            Maximum allowed speed change (km/h/s)
            max_distance_jump: Khoảng cách nhảy tối đa cho phép (m)
                             Maximum allowed distance jump (m)
        """
        self.max_speed_change = max_speed_change
        self.max_distance_jump = max_distance_jump
        self.last_fix: Optional[GPSFix] = None
        self.speed_history: List[float] = []
        self.max_history = 5
        
    def is_valid_fix(self, fix: GPSFix) -> bool:
        """
        Kiểm tra tính hợp lệ của GPS fix
        Check if GPS fix is valid
        
        Args:
            fix: GPS fix data
            
        Returns:
            bool: True nếu fix hợp lệ / True if fix is valid
        """
        try:
            # Kiểm tra cơ bản / Basic checks
            if not fix.valid:
                return False
            
            if fix.satellites < GPS_CONFIG.get("min_satellites", 4):
                return False
            
            if fix.accuracy > GPS_CONFIG.get("min_accuracy", 50.0):
                return False
            
            # Kiểm tra tọa độ hợp lệ / Check valid coordinates
            if not (-90 <= fix.latitude <= 90) or not (-180 <= fix.longitude <= 180):
                return False
            
            # Kiểm tra với fix trước đó / Check against previous fix
            if self.last_fix:
                # Kiểm tra thay đổi tốc độ đột ngột / Check sudden speed change
                time_diff = (fix.timestamp - self.last_fix.timestamp).total_seconds()
                if time_diff > 0:
                    speed_change = abs(fix.speed_kmh - self.last_fix.speed_kmh) / time_diff
                    if speed_change > self.max_speed_change:
                        return False
                
                # Kiểm tra khoảng cách nhảy / Check distance jump
                distance = self._calculate_distance(
                    self.last_fix.latitude, self.last_fix.longitude,
                    fix.latitude, fix.longitude
                )
                if distance > self.max_distance_jump:
                    return False
            
            return True
            
        except Exception:
            return False
    
    def apply_filter(self, fix: GPSFix) -> Optional[GPSFix]:
        """
        Áp dụng bộ lọc lên GPS fix
        Apply filter to GPS fix
        
        Args:
            fix: Raw GPS fix
            
        Returns:
            GPSFix: Filtered fix hoặc None nếu không hợp lệ
                   Filtered fix or None if invalid
        """
        if not self.is_valid_fix(fix):
            return None
        
        # Làm mượt tốc độ / Smooth speed
        filtered_fix = fix
        if len(self.speed_history) >= 2:
            # Sử dụng trung bình trượt / Use moving average
            avg_speed = sum(self.speed_history[-3:]) / min(3, len(self.speed_history))
            # Làm mượt với 70% dữ liệu cũ, 30% dữ liệu mới / Smooth with 70% old, 30% new
            filtered_fix.speed_kmh = avg_speed * 0.7 + fix.speed_kmh * 0.3
        
        # Cập nhật lịch sử / Update history
        self.speed_history.append(fix.speed_kmh)
        if len(self.speed_history) > self.max_history:
            self.speed_history.pop(0)
        
        self.last_fix = filtered_fix
        return filtered_fix
    
    def _calculate_distance(self, lat1: float, lon1: float, 
                          lat2: float, lon2: float) -> float:
        """
        Tính khoảng cách giữa 2 điểm GPS (Haversine formula)
        Calculate distance between two GPS points (Haversine formula)
        
        Returns:
            float: Khoảng cách tính bằng mét / Distance in meters
        """
        R = 6371000  # Bán kính Trái Đất (m) / Earth radius (m)
        
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

class GPSHandler:
    """
    Bộ xử lý GPS chính cho Cycle Sentinel với GPS Stabilizer tích hợp
    Main GPS handler for Cycle Sentinel with integrated GPS Stabilizer
    
    Handles connection, data parsing, filtering, and position tracking
    Xử lý kết nối, phân tích dữ liệu, lọc và theo dõi vị trí
    """
    
    def __init__(self):
        """
        Khởi tạo GPS handler với GPS Stabilizer
        Initialize GPS handler with GPS Stabilizer
        """
        self.logger = get_logger()
        self.serial_connection: Optional[serial.Serial] = None
        self.is_connected = False
        self.data_filter = GPSDataFilter()
        
        # ✅ THÊM: GPS Stabilizer tích hợp
        self.gps_stabilizer = AggressiveGPSStabilizer(
            movement_threshold=5.0,    # 5m = di chuyển thật
            speed_threshold=4.0,       # 4 km/h = tốc độ thật  
            stability_samples=3,       # 3 samples để lock position
            position_lock_radius=3.0   # 3m = bán kính lock
        )
        
        # Thread safety / An toàn luồng
        self._lock = Lock()
        
        # Data caching / Cache dữ liệu
        self.current_fix: Optional[GPSFix] = None
        self.last_valid_fix: Optional[GPSFix] = None
        self.fix_history: List[GPSFix] = []
        self.max_history = 10
        
        # Statistics / Thống kê
        self.stats = {
            "total_reads": 0,
            "valid_fixes": 0,
            "invalid_fixes": 0,
            "connection_attempts": 0,
            "last_fix_time": None,
            "uptime_start": datetime.now()
        }
        self.non_blocking_mode = True
        self.read_buffer = ""
        self.logger.system_info("GPS Handler with Stabilizer initialized", {
            "device_id": SYSTEM_CONFIG["device_id"],
            "gps_port": GPS_CONFIG.get("port", "/dev/ttyUSB0"),
            "baudrate": GPS_CONFIG.get("baudrate", 9600),
            "stabilizer_enabled": True
        })
    
    def connect(self) -> bool:
        """
        Kết nối tới GPS module
        Connect to GPS module
        
        Returns:
            bool: True nếu kết nối thành công / True if connection successful
        """
        with self._lock:
            try:
                self.stats["connection_attempts"] += 1
                
                port = GPS_CONFIG.get("port", "/dev/ttyUSB0")
                baudrate = GPS_CONFIG.get("baudrate", 9600)
                timeout = GPS_CONFIG.get("timeout", 1.0)
                
                self.logger.system_info("Attempting GPS connection", {
                    "port": port,
                    "baudrate": baudrate,
                    "timeout": timeout
                })
                
                # Đóng kết nối cũ nếu có / Close old connection if exists
                if self.serial_connection and self.serial_connection.is_open:
                    self.serial_connection.close()
                
                # Tạo kết nối mới / Create new connection
                self.serial_connection = serial.Serial(
                    port=port,
                    baudrate=baudrate,
                    timeout=timeout,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    bytesize=serial.EIGHTBITS
                )
                
                # Kiểm tra kết nối / Test connection
                if self.serial_connection.is_open:
                    self.is_connected = True
                    
                    self.logger.system_info("GPS connected successfully", {
                        "port": port,
                        "is_open": self.serial_connection.is_open
                    })
                    
                    # Test đọc vài dòng để kiểm tra / Test read a few lines to verify
                    test_success = self._test_connection()
                    if test_success:
                        return True
                    else:
                        self.logger.system_warning("GPS connection test failed")
                        self.disconnect()
                        return False
                
                return False
                
            except serial.SerialException as e:
                self.logger.system_error("GPS Serial connection failed", {
                    "port": GPS_CONFIG.get("port", "/dev/ttyUSB0"),
                    "error": str(e)
                }, e)
                self.is_connected = False
                return False
                
            except Exception as e:
                self.logger.system_error("GPS connection failed", {
                    "error": str(e)
                }, e)
                self.is_connected = False
                return False
    
    def _test_connection(self) -> bool:
        """
        Test kết nối GPS - IMPROVED VERSION
        Test GPS connection with better error handling
        """
        try:
            # 🚀 FLUSH BUFFER trước khi test
            if self.serial_connection.in_waiting > 0:
                self.serial_connection.reset_input_buffer()
            
            # 🚀 WAIT cho GPS module ready
            time.sleep(0.5)  # GPS module initialization time
            
            # 🚀 TĂNG timeout và attempts
            test_attempts = 15  # Tăng từ 5 → 15
            valid_sentences = 0
            
            self.logger.system_debug("Testing GPS connection...")
            
            for attempt in range(test_attempts):
                try:
                    # Đọc với timeout ngắn
                    line = self.serial_connection.readline().decode('ascii', errors='replace').strip()
                    
                    if line.startswith('$') and len(line) > 10:
                        valid_sentences += 1
                        self.logger.system_debug(f"GPS test #{attempt+1}: {line[:30]}...")
                        
                        # 🚀 CẦN ÍT NHẤT 2 VALID SENTENCES
                        if valid_sentences >= 2:
                            self.logger.system_info(f"GPS test passed after {attempt+1} attempts")
                            return True
                            
                    elif line:
                        self.logger.system_debug(f"GPS test #{attempt+1}: Invalid format: {line[:20]}")
                    else:
                        self.logger.system_debug(f"GPS test #{attempt+1}: No data")
                    
                    time.sleep(0.2)  # Tăng từ 0.1s → 0.2s
                    
                except Exception as e:
                    self.logger.system_debug(f"GPS test #{attempt+1} error: {e}")
                    time.sleep(0.1)
            
            self.logger.system_warning(f"GPS test failed - only {valid_sentences} valid sentences in {test_attempts} attempts")
            return False
            
        except Exception as e:
            self.logger.system_error(f"GPS connection test error: {e}")
            return False
    
    def disconnect(self):
        """
        Ngắt kết nối GPS
        Disconnect GPS
        """
        with self._lock:
            try:
                if self.serial_connection and self.serial_connection.is_open:
                    self.serial_connection.close()
                    
                self.is_connected = False
                self.serial_connection = None
                
                self.logger.system_info("GPS disconnected")
                
            except Exception as e:
                self.logger.system_error("Error during GPS disconnection", {"error": str(e)}, e)
    
    def read_position(self) -> Optional[Dict[str, Any]]:
        """
        Đọc vị trí hiện tại từ GPS với GPS Stabilizer
        Read current position from GPS with GPS Stabilizer
        
        Returns:
            Dict: Stabilized GPS data theo GPS_DATA_SCHEMA hoặc None
                 Stabilized GPS data following GPS_DATA_SCHEMA or None
        """
        if not self.is_connected or not self.serial_connection:
            return None
        
        with PerformanceTimer("gps_read"):
            try:
                self.stats["total_reads"] += 1
                
                # Đọc dữ liệu từ GPS / Read data from GPS
                line = self.serial_connection.readline().decode('ascii', errors='replace').strip()
                
                if not line.startswith('$'):
                    return None
                
                # Parse NMEA sentence / Phân tích câu NMEA
                parsed_data = self._parse_nmea_sentence(line)
                
                if parsed_data:
                    # Áp dụng bộ lọc cơ bản / Apply basic filter
                    filtered_fix = self.data_filter.apply_filter(parsed_data)
                    
                    if filtered_fix:
                        # Cập nhật dữ liệu hiện tại / Update current data
                        self._update_current_fix(filtered_fix)
                        
                        # Convert sang format chuẩn / Convert to standard format
                        gps_data = self._fix_to_dict(filtered_fix)
                        
                        # ✅ ÁP DỤNG GPS STABILIZER
                        stabilized_data = self.gps_stabilizer.stabilize_gps_data(gps_data)
                        
                        # Debug info về stabilization
                        if GPS_CONFIG.get("debug", False):
                            stats = self.gps_stabilizer.get_stats()
                            self.logger.system_debug("GPS Stabilized", {
                                "raw_speed": gps_data["speed_kmh"],
                                "stabilized_speed": stabilized_data["speed_kmh"],
                                "raw_position": f"{gps_data['latitude']:.6f},{gps_data['longitude']:.6f}",
                                "stabilized_position": f"{stabilized_data['latitude']:.6f},{stabilized_data['longitude']:.6f}",
                                "position_locked": stats["is_position_locked"],
                                "corrections": stats["correction_rate"]
                            })
                        
                        self.stats["valid_fixes"] += 1
                        self.stats["last_fix_time"] = datetime.now()
                        
                        return stabilized_data
                    else:
                        self.stats["invalid_fixes"] += 1
                        return None
                
                return None
                
            except Exception as e:
                self.logger.system_error("Error reading GPS position", {"error": str(e)}, e)
                return None
    
    def _parse_nmea_sentence(self, sentence: str) -> Optional[GPSFix]:
        """
        Phân tích câu NMEA thành GPSFix - FIXED VERSION
        Parse NMEA sentence into GPSFix - FIXED VERSION
        
        Args:
            sentence: NMEA sentence string
            
        Returns:
            GPSFix: Parsed GPS fix hoặc None
        """
        try:
            msg = pynmea2.parse(sentence)
            current_time = datetime.now()
            
            # DEBUG: Log sentence type để debug
            if GPS_CONFIG.get("debug", False):
                self.logger.system_debug(f"NMEA sentence type: {type(msg).__name__}", {
                    "sentence_type": type(msg).__name__,
                    "sentence": sentence[:50]
                })
            
            # ✅ XỬ LÝ RMC SENTENCE - CÓ TỐC ĐỘ VÀ VỊ TRÍ
            if isinstance(msg, pynmea2.RMC) or getattr(msg, 'sentence_type', '') == 'RMC':
                # GNRMC,023640.00,A,1047.41491,N,10637.84658,E,12.608,190.38,040725,,,A*46
                if msg.latitude is None or msg.longitude is None:
                    return None
                
                latitude = float(msg.latitude)
                longitude = float(msg.longitude)
                
                # ✅ TỐC ĐỘ TỪ RMC
                speed_knots = 0
                if hasattr(msg, 'spd_over_grnd') and msg.spd_over_grnd is not None:
                    speed_knots = float(msg.spd_over_grnd)
                
                speed_kmh = speed_knots * 1.852  # Convert knots to km/h
                
                # Course từ RMC
                course = 0
                if hasattr(msg, 'true_course') and msg.true_course is not None:
                    course = float(msg.true_course)
                
                # Status từ RMC
                valid = True
                if hasattr(msg, 'status'):
                    valid = msg.status == 'A'  # A = Active, V = Void
                
                # DEBUG: Log speed để kiểm tra
                if GPS_CONFIG.get("debug", False):
                    self.logger.system_debug(f"RMC Speed parsed", {
                        "speed_knots": speed_knots,
                        "speed_kmh": speed_kmh,
                        "raw_spd_over_grnd": getattr(msg, 'spd_over_grnd', 'MISSING')
                    })
                
                return GPSFix(
                    timestamp=current_time,
                    latitude=latitude,
                    longitude=longitude,
                    altitude=0.0,  # RMC không có altitude
                    speed_kmh=speed_kmh,
                    course=course,
                    accuracy=5.0,  # Default accuracy cho RMC
                    satellites=0,  # RMC không có satellite count
                    fix_quality=1 if valid else 0,
                    hdop=1.0,     # Default HDOP cho RMC
                    valid=valid
                )
            
            # ✅ XỬ LÝ GGA SENTENCE - CÓ VỊ TRÍ VÀ CHẤT LƯỢNG
            elif isinstance(msg, pynmea2.GGA) or getattr(msg, 'sentence_type', '') == 'GGA':
                # GNGGA,023640.00,1047.41491,N,10637.84658,E,1,12,0.82,9.2,M,-2.7,M,,*54
                if msg.latitude is None or msg.longitude is None:
                    return None
                
                latitude = float(msg.latitude)
                longitude = float(msg.longitude)
                
                # Altitude từ GGA
                altitude = 0
                if hasattr(msg, 'altitude') and msg.altitude is not None:
                    altitude = float(msg.altitude)
                
                # Fix quality từ GGA
                fix_quality = 0
                if hasattr(msg, 'gps_qual') and msg.gps_qual is not None:
                    fix_quality = int(msg.gps_qual)
                
                # Number of satellites từ GGA
                satellites = 0
                if hasattr(msg, 'num_sats') and msg.num_sats is not None:
                    satellites = int(msg.num_sats)
                
                # HDOP từ GGA
                hdop = 99.9
                if hasattr(msg, 'horizontal_dil') and msg.horizontal_dil is not None:
                    hdop = float(msg.horizontal_dil)
                
                # Tính accuracy từ HDOP
                accuracy = hdop * 3.0  # Rough estimation
                
                # ⚠️ GGA KHÔNG CÓ TỐC ĐỘ - sử dụng tốc độ từ fix trước hoặc tính toán
                speed_kmh = 0.0
                if hasattr(self, 'current_fix') and self.current_fix and self.current_fix.speed_kmh > 0:
                    # Sử dụng tốc độ từ fix trước (thường từ RMC)
                    speed_kmh = self.current_fix.speed_kmh
                
                return GPSFix(
                    timestamp=current_time,
                    latitude=latitude,
                    longitude=longitude,
                    altitude=altitude,
                    speed_kmh=speed_kmh,
                    course=0.0,   # GGA không có course
                    accuracy=accuracy,
                    satellites=satellites,
                    fix_quality=fix_quality,
                    hdop=hdop,
                    valid=fix_quality > 0
                )
            
            # ✅ XỬ LÝ VTG SENTENCE - CHỈ CÓ TỐC ĐỘ VÀ COURSE
            elif isinstance(msg, pynmea2.VTG) or getattr(msg, 'sentence_type', '') == 'VTG':
                # GNVTG,190.38,T,,M,12.608,N,23.349,K,A*22
                speed_kmh = 0
                
                # Lấy tốc độ từ VTG (km/h)
                if hasattr(msg, 'spd_over_grnd_kmph') and msg.spd_over_grnd_kmph is not None:
                    speed_kmh = float(msg.spd_over_grnd_kmph)
                elif hasattr(msg, 'spd_over_grnd_kts') and msg.spd_over_grnd_kts is not None:
                    speed_kmh = float(msg.spd_over_grnd_kts) * 1.852
                
                # Course từ VTG
                course = 0
                if hasattr(msg, 'true_track') and msg.true_track is not None:
                    course = float(msg.true_track)
                
                # ⚠️ VTG KHÔNG CÓ VỊ TRÍ - cần combine với fix trước
                if hasattr(self, 'current_fix') and self.current_fix:
                    updated_fix = self.current_fix
                    updated_fix.speed_kmh = speed_kmh
                    updated_fix.course = course
                    updated_fix.timestamp = current_time
                    return updated_fix
                
                return None  # Không có vị trí để tạo fix hoàn chỉnh
            
            # ✅ XỬ LÝ GENERIC - CHO CÁC SENTENCE KHÁC
            elif hasattr(msg, 'latitude') and hasattr(msg, 'longitude'):
                if msg.latitude is None or msg.longitude is None:
                    return None
                
                latitude = float(msg.latitude)
                longitude = float(msg.longitude)
                
                # Thử lấy tốc độ từ nhiều field khác nhau
                speed_knots = 0
                speed_attrs = ['spd_over_grnd', 'speed', 'speed_over_ground', 'sog']
                for attr in speed_attrs:
                    if hasattr(msg, attr):
                        val = getattr(msg, attr)
                        if val is not None and val != '':
                            speed_knots = float(val)
                            break
                
                speed_kmh = speed_knots * 1.852
                
                # Các thông tin khác
                course = getattr(msg, 'true_course', 0) or 0
                altitude = getattr(msg, 'altitude', 0) or 0
                fix_quality = getattr(msg, 'gps_qual', 0) or 0
                satellites = getattr(msg, 'num_sats', 0) or 0
                hdop = getattr(msg, 'horizontal_dil', 99.9) or 99.9
                accuracy = float(hdop) * 3.0
                
                # Check validity
                valid = True
                if hasattr(msg, 'status'):
                    valid = msg.status == 'A'
                
                # DEBUG: Log speed parsing
                if GPS_CONFIG.get("debug", False):
                    self.logger.system_debug(f"Generic NMEA parsed", {
                        "sentence_type": type(msg).__name__,
                        "speed_knots": speed_knots,
                        "speed_kmh": speed_kmh,
                        "found_speed_attr": [attr for attr in speed_attrs if hasattr(msg, attr)]
                    })
                
                return GPSFix(
                    timestamp=current_time,
                    latitude=latitude,
                    longitude=longitude,
                    altitude=float(altitude),
                    speed_kmh=speed_kmh,
                    course=float(course),
                    accuracy=accuracy,
                    satellites=int(satellites),
                    fix_quality=int(fix_quality),
                    hdop=float(hdop),
                    valid=valid and fix_quality > 0
                )
            
            # Nếu không match sentence nào thì return None
            return None
            
        except (pynmea2.ParseError, ValueError, AttributeError) as e:
            if GPS_CONFIG.get("debug", False):
                self.logger.system_debug(f"NMEA parse error: {e}", {
                    "sentence": sentence[:50],
                    "error": str(e),
                    "error_type": type(e).__name__
                })
            return None
        except Exception as e:
            self.logger.system_error(f"Unexpected error parsing NMEA: {e}", {
                "sentence": sentence[:50],
                "error": str(e)
            }, e)
            return None
    
    def _update_current_fix(self, fix: GPSFix):
        """
        Cập nhật fix hiện tại và lịch sử
        Update current fix and history
        
        Args:
            fix: New GPS fix
        """
        with self._lock:
            self.current_fix = fix
            self.last_valid_fix = fix
            
            # Thêm vào lịch sử / Add to history
            self.fix_history.append(fix)
            if len(self.fix_history) > self.max_history:
                self.fix_history.pop(0)
    
    def _fix_to_dict(self, fix: GPSFix) -> Dict[str, Any]:
        """
        Convert GPSFix thành dictionary theo GPS_DATA_SCHEMA
        Convert GPSFix to dictionary following GPS_DATA_SCHEMA
        
        Args:
            fix: GPS fix data
            
        Returns:
            Dict: GPS data dictionary
        """
        return {
            "timestamp": fix.timestamp.isoformat(),
            "latitude": fix.latitude,
            "longitude": fix.longitude,
            "speed_kmh": fix.speed_kmh,
            "course": fix.course,
            "altitude": fix.altitude,
            "accuracy": fix.accuracy,
            "satellites": fix.satellites,
            "valid": fix.valid
        }
    
    def get_last_position(self) -> Optional[Dict[str, Any]]:
        """
        Lấy vị trí cuối cùng (từ cache)
        Get last position (from cache)
        
        Returns:
            Dict: Last GPS data hoặc None
        """
        with self._lock:
            if self.last_valid_fix:
                return self._fix_to_dict(self.last_valid_fix)
            return None
    
    def get_statistics(self) -> Dict[str, Any]:
        """
        Lấy thống kê GPS và GPS Stabilizer
        Get GPS and GPS Stabilizer statistics
        
        Returns:
            Dict: Combined GPS statistics
        """
        with self._lock:
            uptime = (datetime.now() - self.stats["uptime_start"]).total_seconds()
            
            stats = self.stats.copy()
            stats.update({
                "uptime_seconds": uptime,
                "is_connected": self.is_connected,
                "has_fix": self.current_fix is not None,
                "fix_rate": self.stats["valid_fixes"] / max(1, self.stats["total_reads"]) * 100,
                "current_accuracy": self.current_fix.accuracy if self.current_fix else None,
                "current_satellites": self.current_fix.satellites if self.current_fix else None,
                # ✅ GPS Stabilizer stats
                "stabilizer": self.gps_stabilizer.get_stats()
            })
            
            return stats
    
    def is_position_valid(self) -> bool:
        """
        Kiểm tra xem có vị trí hợp lệ không
        Check if current position is valid
        
        Returns:
            bool: True if valid position available
        """
        with self._lock:
            if not self.current_fix:
                return False
            
            # Kiểm tra tuổi của fix / Check fix age
            age = (datetime.now() - self.current_fix.timestamp).total_seconds()
            if age > 30:  # Fix quá 30 giây thì coi như cũ / Fix older than 30s is stale
                return False
            
            return self.current_fix.valid
    
    def wait_for_fix(self, timeout: float = 10.0) -> bool:
        """
        Chờ có GPS fix hợp lệ
        Wait for valid GPS fix
        
        Args:
            timeout: Thời gian chờ tối đa (giây) / Maximum wait time (seconds)
            
        Returns:
            bool: True nếu có fix hợp lệ / True if valid fix obtained
        """
        self.logger.system_info(f"Waiting for GPS fix (timeout: {timeout}s)")
        
        start_time = time.time()
        while time.time() - start_time < timeout:
            if self.read_position() and self.is_position_valid():
                self.logger.system_info("GPS fix acquired", {
                    "wait_time": round(time.time() - start_time, 1),
                    "satellites": self.current_fix.satellites,
                    "accuracy": self.current_fix.accuracy
                })
                return True
            
            time.sleep(0.2)
        
        self.logger.system_warning("GPS fix timeout", {"timeout": timeout})
        return False
    
    def get_stabilizer_stats(self) -> Dict[str, Any]:
        """
        Lấy thống kê GPS Stabilizer riêng
        Get GPS Stabilizer statistics separately
        
        Returns:
            Dict: GPS Stabilizer statistics
        """
        return self.gps_stabilizer.get_stats()
    
    def __del__(self):
        """Destructor - đảm bảo đóng kết nối / Destructor - ensure connection is closed"""
        try:
            self.disconnect()
        except:
            pass

# =============================================================================
# CONVENIENCE FUNCTIONS / HÀM TIỆN ÍCH
# =============================================================================

def create_gps_handler() -> GPSHandler:
    """
    Tạo GPS handler instance với GPS Stabilizer
    Create GPS handler instance with GPS Stabilizer
    
    Returns:
        GPSHandler: Configured GPS handler with stabilizer
    """
    return GPSHandler()

def test_gps_connection() -> bool:
    """
    Test kết nối GPS với GPS Stabilizer
    Test GPS connection with GPS Stabilizer
    
    Returns:
        bool: True nếu kết nối thành công / True if connection successful
    """
    logger = get_logger()
    logger.system_info("Testing GPS connection with Stabilizer...")
    
    gps = create_gps_handler()
    
    try:
        if gps.connect():
            logger.system_info("GPS connection test successful")
            
            # Test đọc vài position / Test reading a few positions
            for i in range(10):
                position = gps.read_position()
                if position:
                    logger.system_info(f"Test position {i+1}", {
                        "lat": position['latitude'],
                        "lon": position['longitude'],
                        "speed": position['speed_kmh'],
                        "stabilized": True
                    })
                else:
                    logger.system_debug(f"No position data on attempt {i+1}")
                time.sleep(1)
            
            # Show stabilizer stats
            stabilizer_stats = gps.get_stabilizer_stats()
            logger.system_info("GPS Stabilizer Performance", stabilizer_stats)
            
            return True
        else:
            logger.system_error("GPS connection test failed")
            return False
            
    finally:
        gps.disconnect()

# =============================================================================
# TESTING / KIỂM TRA
# =============================================================================

if __name__ == "__main__":
    print("Testing Cycle Sentinel GPS Handler with Aggressive Stabilizer...")
    
    # Test GPS connection
    if test_gps_connection():
        print("✅ GPS connection with stabilizer test passed")
    else:
        print("❌ GPS connection test failed")
    
    # Test GPS handler functionality
    gps = create_gps_handler()
    
    if gps.connect():
        print("✅ GPS connected")
        
        # Wait for fix
        if gps.wait_for_fix(timeout=15):
            print("✅ GPS fix acquired")
            
            # Read some positions to test stabilizer
            print("\n🔧 Testing GPS Stabilizer (stay stationary)...")
            for i in range(20):
                pos = gps.read_position()
                if pos:
                    stabilizer_stats = gps.get_stabilizer_stats()
                    print(f"Position {i+1}: LAT:{pos['latitude']:.6f} LON:{pos['longitude']:.6f} "
                          f"SPEED:{pos['speed_kmh']:.1f}kmh "
                          f"LOCKED:{stabilizer_stats['is_position_locked']}")
                else:
                    print(f"Position {i+1}: No data")
                time.sleep(2)
            
            # Show final statistics
            final_stats = gps.get_statistics()
            print("\n📊 Final GPS Statistics:")
            for key, value in final_stats.items():
                if key != 'stabilizer':
                    print(f"  {key}: {value}")
            
            print("\n📊 GPS Stabilizer Statistics:")
            for key, value in final_stats['stabilizer'].items():
                print(f"  {key}: {value}")
                
        else:
            print("❌ GPS fix timeout")
        
        gps.disconnect()
        print("✅ GPS disconnected")
    else:
        print("❌ GPS connection failed")
    
    print("GPS Handler with Stabilizer test completed!")