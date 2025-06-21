"""
Cycle Sentinel GPS Handler
Bộ xử lý GPS cho Cycle Sentinel

This module handles GPS communication, NMEA parsing, and position tracking.
Module này xử lý giao tiếp GPS, phân tích NMEA và theo dõi vị trí.

Features / Tính năng:
- NMEA sentence parsing / Phân tích câu NMEA
- Position tracking and validation / Theo dõi và xác thực vị trí
- Speed calculation and smoothing / Tính toán và làm mượt tốc độ
- GPS fix quality monitoring / Giám sát chất lượng tín hiệu GPS
- Connection management / Quản lý kết nối
- Data caching and recovery / Cache dữ liệu và phục hồi

Author: Cycle Sentinel Team
Version: 1.0.0
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
            
            if fix.satellites < GPS_CONFIG["min_satellites"]:
                return False
            
            if fix.accuracy > GPS_CONFIG["min_accuracy"]:
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
    Bộ xử lý GPS chính cho Cycle Sentinel
    Main GPS handler for Cycle Sentinel
    
    Handles connection, data parsing, filtering, and position tracking
    Xử lý kết nối, phân tích dữ liệu, lọc và theo dõi vị trí
    """
    
    def __init__(self):
        """
        Khởi tạo GPS handler
        Initialize GPS handler
        """
        self.logger = get_logger()
        self.serial_connection: Optional[serial.Serial] = None
        self.is_connected = False
        self.data_filter = GPSDataFilter()
        
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
        
        self.logger.system_info("GPS Handler initialized", {
            "device_id": SYSTEM_CONFIG["device_id"],
            "gps_port": GPS_CONFIG["port"],
            "baudrate": GPS_CONFIG["baudrate"]
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
                
                self.logger.system_info("Attempting GPS connection", {
                    "port": GPS_CONFIG["port"],
                    "baudrate": GPS_CONFIG["baudrate"],
                    "timeout": GPS_CONFIG["timeout"]
                })
                
                # Đóng kết nối cũ nếu có / Close old connection if exists
                if self.serial_connection and self.serial_connection.is_open:
                    self.serial_connection.close()
                
                # Tạo kết nối mới / Create new connection
                self.serial_connection = serial.Serial(
                    port=GPS_CONFIG["port"],
                    baudrate=GPS_CONFIG["baudrate"],
                    timeout=GPS_CONFIG["timeout"],
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    bytesize=serial.EIGHTBITS
                )
                
                # Kiểm tra kết nối / Test connection
                if self.serial_connection.is_open:
                    self.is_connected = True
                    
                    self.logger.system_info("GPS connected successfully", {
                        "port": GPS_CONFIG["port"],
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
                    "port": GPS_CONFIG["port"],
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
        Test kết nối GPS bằng cách đọc vài dòng dữ liệu
        Test GPS connection by reading a few lines of data
        
        Returns:
            bool: True nếu test thành công / True if test successful
        """
        try:
            test_attempts = 5
            for attempt in range(test_attempts):
                line = self.serial_connection.readline().decode('ascii', errors='replace').strip()
                
                if line.startswith('$'):
                    self.logger.system_debug(f"GPS test read successful: {line[:20]}...")
                    return True
                
                time.sleep(0.1)
            
            self.logger.system_warning("GPS test failed - no valid NMEA data received")
            return False
            
        except Exception as e:
            self.logger.system_error("GPS connection test failed", {"error": str(e)}, e)
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
        Đọc vị trí hiện tại từ GPS
        Read current position from GPS
        
        Returns:
            Dict: GPS data theo GPS_DATA_SCHEMA hoặc None
                 GPS data following GPS_DATA_SCHEMA or None
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
                    # Áp dụng bộ lọc / Apply filter
                    filtered_fix = self.data_filter.apply_filter(parsed_data)
                    
                    if filtered_fix:
                        # Cập nhật dữ liệu hiện tại / Update current data
                        self._update_current_fix(filtered_fix)
                        
                        # Convert sang format chuẩn / Convert to standard format
                        gps_data = self._fix_to_dict(filtered_fix)
                        
                        # Log GPS data nếu bật / Log GPS data if enabled
                        self.logger.log_gps_data(gps_data)
                        
                        self.stats["valid_fixes"] += 1
                        self.stats["last_fix_time"] = datetime.now()
                        
                        return gps_data
                    else:
                        self.stats["invalid_fixes"] += 1
                        return None
                
                return None
                
            except Exception as e:
                self.logger.system_error("Error reading GPS position", {"error": str(e)}, e)
                return None
    
    def _parse_nmea_sentence(self, sentence: str) -> Optional[GPSFix]:
        """
        Phân tích câu NMEA thành GPSFix
        Parse NMEA sentence into GPSFix
        
        Args:
            sentence: NMEA sentence string
            
        Returns:
            GPSFix: Parsed GPS fix hoặc None
        """
        try:
            msg = pynmea2.parse(sentence)
            current_time = datetime.now()
            
            # Xử lý các loại NMEA khác nhau / Handle different NMEA types
            if hasattr(msg, 'latitude') and hasattr(msg, 'longitude'):
                # RMC, GGA sentences
                if msg.latitude is None or msg.longitude is None:
                    return None
                
                # Lấy thông tin cơ bản / Get basic information
                latitude = float(msg.latitude)
                longitude = float(msg.longitude)
                
                # Speed (RMC có speed, GGA không có) / Speed (RMC has speed, GGA doesn't)
                speed_knots = getattr(msg, 'spd_over_grnd', 0) or 0
                speed_kmh = float(speed_knots) * 1.852  # Convert knots to km/h
                
                # Course / Hướng
                course = getattr(msg, 'true_course', 0) or 0
                
                # Altitude / Độ cao
                altitude = getattr(msg, 'altitude', 0) or 0
                
                # Fix quality / Chất lượng tín hiệu
                fix_quality = getattr(msg, 'gps_qual', 0) or 0
                
                # Number of satellites / Số vệ tinh
                satellites = getattr(msg, 'num_sats', 0) or 0
                
                # HDOP (Horizontal Dilution of Precision)
                hdop = getattr(msg, 'horizontal_dil', 99.9) or 99.9
                
                # Tính accuracy từ HDOP / Calculate accuracy from HDOP
                accuracy = float(hdop) * 3.0  # Rough estimation
                
                # Check validity / Kiểm tra tính hợp lệ
                valid = True
                if hasattr(msg, 'status'):
                    valid = msg.status == 'A'  # A = Active, V = Void
                
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
            
            return None
            
        except (pynmea2.ParseError, ValueError, AttributeError) as e:
            if GPS_CONFIG["debug"]:
                self.logger.system_debug(f"NMEA parse error: {e}", {
                    "sentence": sentence[:50],
                    "error": str(e)
                })
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
        Lấy thống kê GPS
        Get GPS statistics
        
        Returns:
            Dict: GPS statistics
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
                "current_satellites": self.current_fix.satellites if self.current_fix else None
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
    
    def wait_for_fix(self, timeout: float = 30.0) -> bool:
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
            
            time.sleep(1)
        
        self.logger.system_warning("GPS fix timeout", {"timeout": timeout})
        return False
    
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
    Tạo GPS handler instance
    Create GPS handler instance
    
    Returns:
        GPSHandler: Configured GPS handler
    """
    return GPSHandler()

def test_gps_connection() -> bool:
    """
    Test kết nối GPS
    Test GPS connection
    
    Returns:
        bool: True nếu kết nối thành công / True if connection successful
    """
    logger = get_logger()
    logger.system_info("Testing GPS connection...")
    
    gps = create_gps_handler()
    
    try:
        if gps.connect():
            logger.system_info("GPS connection test successful")
            
            # Test đọc vài position / Test reading a few positions
            for i in range(5):
                position = gps.read_position()
                if position:
                    logger.system_info(f"Test position {i+1}", position)
                else:
                    logger.system_debug(f"No position data on attempt {i+1}")
                time.sleep(1)
            
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
    print("Testing Cycle Sentinel GPS Handler...")
    
    # Test GPS connection
    if test_gps_connection():
        print("✅ GPS connection test passed")
    else:
        print("❌ GPS connection test failed")
    
    # Test GPS handler functionality
    gps = create_gps_handler()
    
    if gps.connect():
        print("✅ GPS connected")
        
        # Wait for fix
        if gps.wait_for_fix(timeout=10):
            print("✅ GPS fix acquired")
            
            # Read some positions
            for i in range(10):
                pos = gps.read_position()
                if pos:
                    print(f"Position {i+1}: {pos['latitude']:.6f}, {pos['longitude']:.6f}, "
                          f"Speed: {pos['speed_kmh']:.1f} km/h")
                else:
                    print(f"Position {i+1}: No data")
                time.sleep(1)
            
            # Show statistics
            stats = gps.get_statistics()
            print("\nGPS Statistics:")
            for key, value in stats.items():
                print(f"  {key}: {value}")
        else:
            print("❌ GPS fix timeout")
        
        gps.disconnect()
        print("✅ GPS disconnected")
    else:
        print("❌ GPS connection failed")
    
    print("GPS Handler test completed!")