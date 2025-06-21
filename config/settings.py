"""
Cycle Sentinel Configuration
Cấu hình cho toàn bộ hệ thống Cycle Sentinel

This file contains all configuration settings for the Cycle Sentinel system.
File này chứa tất cả các cấu hình cho hệ thống Cycle Sentinel.

Author: Cycle Sentinel Team
Version: 1.0.0
"""

import os
from typing import Dict, Any, List
from pathlib import Path

# =============================================================================
# BASE PATHS / ĐƯỜNG DẪN CƠ BẢN
# =============================================================================

# Thư mục gốc của project / Project root directory
PROJECT_ROOT = Path(__file__).parent.parent

# Thư mục data / Data directory
DATA_DIR = PROJECT_ROOT / "data"
MAPS_DIR = PROJECT_ROOT / "maps"
LOGS_DIR = PROJECT_ROOT / "logs"

# Tạo thư mục nếu chưa tồn tại / Create directories if they don't exist
for directory in [DATA_DIR, LOGS_DIR]:
    directory.mkdir(exist_ok=True)

# =============================================================================
# GPS CONFIGURATION / CÂU HÌNH GPS
# =============================================================================

GPS_CONFIG: Dict[str, Any] = {
    # Cổng kết nối GPS / GPS connection port
    # Linux: /dev/ttyUSB0, /dev/ttyACM0
    # Windows: COM3, COM4, etc.
    "port": os.getenv("GPS_PORT", "/dev/ttyUSB0"),
    
    # Tốc độ baud / Baud rate
    "baudrate": int(os.getenv("GPS_BAUDRATE", "9600")),
    
    # Timeout đọc dữ liệu (giây) / Read timeout (seconds)
    "timeout": int(os.getenv("GPS_TIMEOUT", "1")),
    
    # Tần suất đọc GPS (giây) / GPS read frequency (seconds)
    "read_interval": float(os.getenv("GPS_READ_INTERVAL", "1.0")),
    
    # Độ chính xác tối thiểu (mét) / Minimum accuracy (meters)
    "min_accuracy": float(os.getenv("GPS_MIN_ACCURACY", "10.0")),
    
    # Số vệ tinh tối thiểu / Minimum satellites
    "min_satellites": int(os.getenv("GPS_MIN_SATELLITES", "4")),
    
    # Bật chế độ debug / Enable debug mode
    "debug": os.getenv("GPS_DEBUG", "False").lower() == "true"
}

# =============================================================================
# MAP CONFIGURATION / CÂU HÌNH BẢN ĐỒ
# =============================================================================

MAP_CONFIG: Dict[str, Any] = {
    # File bản đồ zone / Zone map file
    "map_file": str(MAPS_DIR / "hcmc_zones.json"),
    
    # Backup map file / File bản đồ dự phòng
    "backup_map_file": str(MAPS_DIR / "hcmc_zones_backup.json"),
    
    # Khoảng cách buffer cho zone (mét) / Zone buffer distance (meters)
    "zone_buffer_meters": float(os.getenv("ZONE_BUFFER", "5.0")),
    
    # Tự động reload map khi file thay đổi / Auto reload map when file changes
    "auto_reload": os.getenv("MAP_AUTO_RELOAD", "True").lower() == "true",
    
    # Kiểm tra tính hợp lệ của map / Validate map data
    "validate_map": True,
    
    # Encoding của file map / Map file encoding
    "encoding": "utf-8"
}

# =============================================================================
# VIOLATION DETECTION / PHÁT HIỆN VI PHẠM
# =============================================================================

VIOLATION_CONFIG: Dict[str, Any] = {
    # Dung sai tốc độ (km/h) - để tránh false positive do GPS error
    # Speed tolerance (km/h) - to avoid false positives from GPS errors
    "speed_tolerance_kmh": float(os.getenv("SPEED_TOLERANCE", "2.0")),
    
    # Thời gian vi phạm tối thiểu (giây) - vi phạm phải kéo dài ít nhất thời gian này
    # Minimum violation duration (seconds) - violation must last at least this long
    "min_violation_duration": float(os.getenv("MIN_VIOLATION_DURATION", "3.0")),
    
    # Tốc độ tối thiểu để tính vi phạm (km/h) - tránh false positive khi đỗ xe
    # Minimum speed for violation (km/h) - avoid false positives when parked
    "min_speed_for_violation": float(os.getenv("MIN_SPEED_VIOLATION", "5.0")),
    
    # Số điểm GPS liên tiếp để xác nhận vi phạm / Consecutive GPS points to confirm violation
    "consecutive_points_required": int(os.getenv("CONSECUTIVE_POINTS", "3")),
    
    # Khoảng cách tối thiểu giữa các vi phạm (mét) / Minimum distance between violations (meters)
    "min_distance_between_violations": float(os.getenv("MIN_VIOLATION_DISTANCE", "50.0")),
    
    # Các loại vi phạm được bật / Enabled violation types
    "enabled_violations": [
        "speed_violation",      # Vi phạm tốc độ / Speed violation
        "restricted_zone",      # Vào khu vực cấm / Restricted zone entry
        "wrong_direction"       # Đi sai chiều / Wrong direction (if implemented)
    ]
}

# =============================================================================
# LOGGING CONFIGURATION / CẤU HÌNH LOGGING
# =============================================================================

LOGGING_CONFIG: Dict[str, Any] = {
    # File log vi phạm / Violation log file
    "violation_log_file": str(LOGS_DIR / "violations.log"),
    
    # File log hệ thống / System log file
    "system_log_file": str(LOGS_DIR / "system.log"),
    
    # File log GPS data (cho debug) / GPS data log file (for debugging)
    "gps_log_file": str(LOGS_DIR / "gps_data.log"),
    
    # Mức độ log / Log level
    # DEBUG, INFO, WARNING, ERROR, CRITICAL
    "log_level": os.getenv("LOG_LEVEL", "INFO"),
    
    # Format log / Log format
    "log_format": "%(asctime)s - %(name)s - %(levelname)s - %(message)s",
    
    # Format thời gian / Time format
    "time_format": "%Y-%m-%d %H:%M:%S",
    
    # Kích thước file log tối đa (bytes) / Maximum log file size (bytes)
    "max_log_size": int(os.getenv("MAX_LOG_SIZE", "10485760")),  # 10MB
    
    # Số file backup / Number of backup files
    "backup_count": int(os.getenv("LOG_BACKUP_COUNT", "5")),
    
    # Log ra console / Log to console
    "log_to_console": os.getenv("LOG_TO_CONSOLE", "True").lower() == "true",
    
    # Log GPS data (chỉ cho debug) / Log GPS data (debug only)
    "log_gps_data": os.getenv("LOG_GPS_DATA", "False").lower() == "true"
}

# =============================================================================
# SYSTEM CONFIGURATION / CẤU HÌNH HỆ THỐNG
# =============================================================================

SYSTEM_CONFIG: Dict[str, Any] = {
    # Tên hệ thống / System name
    "system_name": "Cycle Sentinel",
    
    # Phiên bản / Version
    "version": "1.0.0",
    
    # ID thiết bị (unique cho mỗi device) / Device ID (unique per device)
    "device_id": os.getenv("DEVICE_ID", "CS_DEV_001"),
    
    # Múi giờ / Timezone
    "timezone": os.getenv("TIMEZONE", "Asia/Ho_Chi_Minh"),
    
    # Chế độ debug / Debug mode
    "debug_mode": os.getenv("DEBUG", "False").lower() == "true",
    
    # Tự động khởi động / Auto start
    "auto_start": os.getenv("AUTO_START", "True").lower() == "true",
    
    # Thời gian delay khi khởi động (giây) / Startup delay (seconds)
    "startup_delay": float(os.getenv("STARTUP_DELAY", "2.0")),
    
    # Chu kỳ heartbeat (giây) / Heartbeat interval (seconds)
    "heartbeat_interval": float(os.getenv("HEARTBEAT_INTERVAL", "30.0"))
}

# =============================================================================
# DATA FORMATS / ĐỊNH DẠNG DỮ LIỆU
# =============================================================================

# Cấu trúc dữ liệu GPS chuẩn / Standard GPS data structure
GPS_DATA_SCHEMA = {
    "timestamp": str,           # ISO format: 2024-01-01T12:00:00Z
    "latitude": float,          # Độ vĩ / Latitude
    "longitude": float,         # Kinh độ / Longitude
    "speed_kmh": float,         # Tốc độ km/h / Speed in km/h
    "course": float,            # Hướng di chuyển (độ) / Course in degrees
    "altitude": float,          # Độ cao (m) / Altitude in meters
    "accuracy": float,          # Độ chính xác (m) / Accuracy in meters
    "satellites": int,          # Số vệ tinh / Number of satellites
    "valid": bool               # Dữ liệu hợp lệ / Data is valid
}

# Cấu trúc dữ liệu vi phạm / Violation data structure
VIOLATION_DATA_SCHEMA = {
    "id": str,                  # ID vi phạm / Violation ID
    "timestamp": str,           # Thời gian / Timestamp
    "device_id": str,           # ID thiết bị / Device ID
    "violation_type": str,      # Loại vi phạm / Violation type
    "location": {               # Vị trí / Location
        "latitude": float,
        "longitude": float
    },
    "speed_data": {             # Dữ liệu tốc độ / Speed data
        "current_speed": float,
        "speed_limit": float,
        "excess_speed": float
    },
    "zone_info": {              # Thông tin zone / Zone information
        "zone_id": str,
        "zone_name": str,
        "zone_type": str
    },
    "confidence": float,        # Độ tin cậy (0-1) / Confidence (0-1)
    "duration": float           # Thời gian vi phạm (giây) / Violation duration (seconds)
}

# =============================================================================
# DEFAULT VALUES / GIÁ TRỊ MẶC ĐỊNH
# =============================================================================

# Giới hạn tốc độ mặc định / Default speed limits
DEFAULT_SPEED_LIMITS = {
    "residential": 25,          # Khu dân cư / Residential area (km/h)
    "commercial": 20,           # Khu thương mại / Commercial area (km/h)
    "school_zone": 10,          # Khu trường học / School zone (km/h)
    "hospital_zone": 15,        # Khu bệnh viện / Hospital zone (km/h)
    "park": 15,                 # Công viên / Park (km/h)
    "pedestrian_only": 0        # Chỉ dành cho người đi bộ / Pedestrian only
}

# =============================================================================
# VALIDATION FUNCTIONS / HÀM KIỂM TRA
# =============================================================================

def validate_config() -> bool:
    """
    Kiểm tra tính hợp lệ của cấu hình
    Validate configuration settings
    
    Returns:
        bool: True nếu config hợp lệ / True if config is valid
    """
    try:
        # Kiểm tra GPS config / Check GPS config
        if GPS_CONFIG["baudrate"] <= 0:
            raise ValueError("GPS baudrate must be positive")
        
        if GPS_CONFIG["read_interval"] <= 0:
            raise ValueError("GPS read interval must be positive")
        
        # Kiểm tra violation config / Check violation config
        if VIOLATION_CONFIG["speed_tolerance_kmh"] < 0:
            raise ValueError("Speed tolerance cannot be negative")
        
        if VIOLATION_CONFIG["min_violation_duration"] <= 0:
            raise ValueError("Minimum violation duration must be positive")
        
        # Kiểm tra file paths / Check file paths
        if not MAPS_DIR.exists():
            raise ValueError(f"Maps directory does not exist: {MAPS_DIR}")
        
        return True
        
    except Exception as e:
        print(f"Configuration validation error: {e}")
        return False

def get_config_summary() -> Dict[str, Any]:
    """
    Lấy tóm tắt cấu hình hệ thống
    Get system configuration summary
    
    Returns:
        Dict: Tóm tắt cấu hình / Configuration summary
    """
    return {
        "system": {
            "name": SYSTEM_CONFIG["system_name"],
            "version": SYSTEM_CONFIG["version"],
            "device_id": SYSTEM_CONFIG["device_id"],
            "debug_mode": SYSTEM_CONFIG["debug_mode"]
        },
        "gps": {
            "port": GPS_CONFIG["port"],
            "baudrate": GPS_CONFIG["baudrate"],
            "read_interval": GPS_CONFIG["read_interval"]
        },
        "violations": {
            "speed_tolerance": VIOLATION_CONFIG["speed_tolerance_kmh"],
            "min_duration": VIOLATION_CONFIG["min_violation_duration"],
            "enabled_types": VIOLATION_CONFIG["enabled_violations"]
        },
        "logging": {
            "level": LOGGING_CONFIG["log_level"],
            "log_to_console": LOGGING_CONFIG["log_to_console"]
        }
    }

# =============================================================================
# INITIALIZE / KHỞI TẠO
# =============================================================================

# Tự động validate config khi import module
# Automatically validate config when importing module
if __name__ != "__main__":
    if not validate_config():
        raise RuntimeError("Invalid configuration detected")

# Để test và debug / For testing and debugging
if __name__ == "__main__":
    import json
    print("=== Cycle Sentinel Configuration ===")
    print(json.dumps(get_config_summary(), indent=2, ensure_ascii=False))
    print(f"\nValidation result: {validate_config()}")