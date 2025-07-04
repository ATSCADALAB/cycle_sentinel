"""
Cycle Sentinel Logging System
Hệ thống logging cho Cycle Sentinel

This module provides comprehensive logging functionality for the Cycle Sentinel system.
Module này cung cấp chức năng logging toàn diện cho hệ thống Cycle Sentinel.

Features / Tính năng:
- System logging (GPS, violations, errors) / Logging hệ thống
- Violation logging with structured data / Logging vi phạm với dữ liệu có cấu trúc
- File rotation and size management / Quản lý xoay vòng và kích thước file
- Console and file output / Xuất ra console và file
- Performance logging / Logging hiệu suất

Author: Cycle Sentinel Team
Version: 1.0.0
"""

import logging
import logging.handlers
import json
import os
import sys
from datetime import datetime
from typing import Dict, Any, Optional, Union
from pathlib import Path

# Import cấu hình từ settings / Import config from settings
from config.settings import (
    LOGGING_CONFIG, 
    SYSTEM_CONFIG, 
    VIOLATION_DATA_SCHEMA,
    LOGS_DIR
)

class CycleSentinelLogger:
    """
    Logger chính cho hệ thống Cycle Sentinel
    Main logger for Cycle Sentinel system
    
    Provides structured logging for system events, violations, and debugging
    Cung cấp logging có cấu trúc cho các sự kiện hệ thống, vi phạm và debug
    """
    
    def __init__(self):
        """
        Khởi tạo logging system
        Initialize logging system
        """
        self.system_logger = None
        self.violation_logger = None
        self.gps_logger = None
        self.performance_logger = None
        
        # Thiết lập loggers / Setup loggers
        self._setup_loggers()
        
        # Log khởi động hệ thống / Log system startup
        self.system_info("Logging system initialized", {
            "device_id": SYSTEM_CONFIG["device_id"],
            "version": SYSTEM_CONFIG["version"],
            "log_level": LOGGING_CONFIG["log_level"]
        })
    
    def _setup_loggers(self):
        """
        Thiết lập các logger khác nhau
        Setup different loggers for different purposes
        """
        # Tạo thư mục logs nếu chưa có / Create logs directory if not exists
        LOGS_DIR.mkdir(exist_ok=True)
        
        # System Logger - cho các sự kiện hệ thống / System Logger - for system events
        self.system_logger = self._create_logger(
            name="cycle_sentinel.system",
            log_file=LOGGING_CONFIG["system_log_file"],
            level=LOGGING_CONFIG["log_level"]
        )
        
        # Violation Logger - cho các vi phạm / Violation Logger - for violations
        self.violation_logger = self._create_logger(
            name="cycle_sentinel.violations",
            log_file=LOGGING_CONFIG["violation_log_file"],
            level="INFO",  # Luôn log tất cả violations / Always log all violations
            formatter_type="violation"
        )
        
        # GPS Logger - cho debug GPS data / GPS Logger - for GPS data debugging
        if LOGGING_CONFIG["log_gps_data"]:
            self.gps_logger = self._create_logger(
                name="cycle_sentinel.gps",
                log_file=LOGGING_CONFIG["gps_log_file"],
                level="DEBUG",
                formatter_type="gps"
            )
        
        # Performance Logger - cho giám sát hiệu suất / Performance Logger - for performance monitoring
        self.performance_logger = self._create_logger(
            name="cycle_sentinel.performance",
            log_file=str(LOGS_DIR / "performance.log"),
            level="INFO",
            formatter_type="performance"
        )
    
    def _create_logger(self, name: str, log_file: str, level: str, 
                      formatter_type: str = "standard") -> logging.Logger:
        """
        Tạo một logger với cấu hình cụ thể
        Create a logger with specific configuration
        
        Args:
            name: Tên logger / Logger name
            log_file: Đường dẫn file log / Log file path
            level: Mức độ log / Log level
            formatter_type: Loại formatter / Formatter type
            
        Returns:
            logging.Logger: Logger đã cấu hình / Configured logger
        """
        logger = logging.getLogger(name)
        logger.setLevel(getattr(logging, level.upper()))
        
        # Xóa handlers cũ nếu có / Remove old handlers if any
        logger.handlers.clear()
        
        # File Handler với rotation / File Handler with rotation
        file_handler = logging.handlers.RotatingFileHandler(
            filename=log_file,
            maxBytes=LOGGING_CONFIG["max_log_size"],
            backupCount=LOGGING_CONFIG["backup_count"],
            encoding='utf-8'
        )
        
        # Console Handler / Console Handler
        console_handler = logging.StreamHandler(sys.stdout)
        
        # Thiết lập formatter / Setup formatter
        formatter = self._get_formatter(formatter_type)
        file_handler.setFormatter(formatter)
        console_handler.setFormatter(formatter)
        
        # Thêm handlers / Add handlers
        logger.addHandler(file_handler)
        
        # if LOGGING_CONFIG["log_to_console"]:
        #     logger.addHandler(console_handler)
        
        # Ngăn duplicate logs / Prevent duplicate logs
        logger.propagate = False
        
        return logger
    
    def _get_formatter(self, formatter_type: str) -> logging.Formatter:
        """
        Lấy formatter theo loại
        Get formatter by type
        
        Args:
            formatter_type: Loại formatter / Formatter type
            
        Returns:
            logging.Formatter: Formatter object
        """
        if formatter_type == "violation":
            # Format đặc biệt cho violations / Special format for violations
            return logging.Formatter(
                '%(asctime)s | VIOLATION | %(message)s',
                datefmt=LOGGING_CONFIG["time_format"]
            )
        elif formatter_type == "gps":
            # Format cho GPS data / Format for GPS data
            return logging.Formatter(
                '%(asctime)s | GPS | %(message)s',
                datefmt=LOGGING_CONFIG["time_format"]
            )
        elif formatter_type == "performance":
            # Format cho performance / Format for performance
            return logging.Formatter(
                '%(asctime)s | PERF | %(message)s',
                datefmt=LOGGING_CONFIG["time_format"]
            )
        else:
            # Standard format / Format chuẩn
            return logging.Formatter(
                LOGGING_CONFIG["log_format"],
                datefmt=LOGGING_CONFIG["time_format"]
            )
    
    # =============================================================================
    # SYSTEM LOGGING METHODS / PHƯƠNG THỨC LOGGING HỆ THỐNG
    # =============================================================================
    
    def system_debug(self, message: str, extra_data: Optional[Dict[str, Any]] = None):
        """Log debug message cho hệ thống / Log debug message for system"""
        self._log_with_extra(self.system_logger, logging.DEBUG, message, extra_data)
    
    def system_info(self, message: str, extra_data: Optional[Dict[str, Any]] = None):
        """Log info message cho hệ thống / Log info message for system"""
        self._log_with_extra(self.system_logger, logging.INFO, message, extra_data)
    
    def system_warning(self, message: str, extra_data: Optional[Dict[str, Any]] = None):
        """Log warning message cho hệ thống / Log warning message for system"""
        self._log_with_extra(self.system_logger, logging.WARNING, message, extra_data)
    
    def system_error(self, message: str, extra_data: Optional[Dict[str, Any]] = None, 
                    exception: Optional[Exception] = None):
        """Log error message cho hệ thống / Log error message for system"""
        if exception:
            extra_data = extra_data or {}
            extra_data["exception"] = str(exception)
            extra_data["exception_type"] = type(exception).__name__
        
        self._log_with_extra(self.system_logger, logging.ERROR, message, extra_data)
    
    def system_critical(self, message: str, extra_data: Optional[Dict[str, Any]] = None):
        """Log critical message cho hệ thống / Log critical message for system"""
        self._log_with_extra(self.system_logger, logging.CRITICAL, message, extra_data)
    
    # =============================================================================
    # VIOLATION LOGGING METHODS / PHƯƠNG THỨC LOGGING VI PHẠM
    # =============================================================================
    
    def log_violation(self, violation_data: Dict[str, Any]):
        """
        Log vi phạm với dữ liệu có cấu trúc
        Log violation with structured data
        
        Args:
            violation_data: Dữ liệu vi phạm theo VIOLATION_DATA_SCHEMA
                          Violation data following VIOLATION_DATA_SCHEMA
        """
        try:
            # Validate dữ liệu vi phạm / Validate violation data
            validated_data = self._validate_violation_data(violation_data)
            
            # Thêm metadata / Add metadata
            validated_data["logged_at"] = datetime.now().isoformat()
            validated_data["device_id"] = SYSTEM_CONFIG["device_id"]
            
            # Log dưới dạng JSON / Log as JSON
            self.violation_logger.info(json.dumps(validated_data, ensure_ascii=False))
            
            # Log vào system log / Log to system log
            self.system_info(f"Violation logged: {validated_data['violation_type']}", {
                "violation_id": validated_data["id"],
                "zone": validated_data["zone_info"]["zone_name"],
                "speed": validated_data["speed_data"]["current_speed"]
            })
            
        except Exception as e:
            self.system_error("Failed to log violation", {
                "error": str(e),
                "violation_data": violation_data
            }, e)
    
    def _validate_violation_data(self, data: Dict[str, Any]) -> Dict[str, Any]:
        """
        Validate và chuẩn hóa dữ liệu vi phạm
        Validate and normalize violation data
        
        Args:
            data: Raw violation data
            
        Returns:
            Dict: Validated violation data
        """
        # Tạo violation ID nếu chưa có / Create violation ID if not exists
        if "id" not in data:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            data["id"] = f"VL_{SYSTEM_CONFIG['device_id']}_{timestamp}"
        
        # Đảm bảo timestamp format chuẩn / Ensure standard timestamp format
        if "timestamp" not in data:
            data["timestamp"] = datetime.now().isoformat()
        
        # Validate required fields / Kiểm tra các trường bắt buộc
        required_fields = ["violation_type", "location", "speed_data", "zone_info"]
        for field in required_fields:
            if field not in data:
                raise ValueError(f"Missing required field: {field}")
        
        return data
    
    # =============================================================================
    # GPS LOGGING METHODS / PHƯƠNG THỨC LOGGING GPS
    # =============================================================================
    
    def log_gps_data(self, gps_data: Dict[str, Any]):
        """
        Log GPS data cho debug
        Log GPS data for debugging
        
        Args:
            gps_data: GPS data theo GPS_DATA_SCHEMA / GPS data following GPS_DATA_SCHEMA
        """
        if self.gps_logger and LOGGING_CONFIG["log_gps_data"]:
            try:
                # Tạo log entry compact / Create compact log entry
                log_entry = {
                    "ts": gps_data.get("timestamp", datetime.now().isoformat()),
                    "lat": gps_data.get("latitude", 0.0),
                    "lon": gps_data.get("longitude", 0.0),
                    "spd": gps_data.get("speed_kmh", 0.0),
                    "acc": gps_data.get("accuracy", 0.0),
                    "sat": gps_data.get("satellites", 0),
                    "val": gps_data.get("valid", False)
                }
                
                self.gps_logger.debug(json.dumps(log_entry, ensure_ascii=False))
                
            except Exception as e:
                self.system_error("Failed to log GPS data", {"error": str(e)}, e)
    
    # =============================================================================
    # PERFORMANCE LOGGING METHODS / PHƯƠNG THỨC LOGGING HIỆU SUẤT
    # =============================================================================
    
    def log_performance(self, operation: str, duration: float, 
                       extra_data: Optional[Dict[str, Any]] = None):
        """
        Log thông tin hiệu suất
        Log performance information
        
        Args:
            operation: Tên thao tác / Operation name
            duration: Thời gian thực hiện (giây) / Execution time (seconds)
            extra_data: Dữ liệu bổ sung / Additional data
        """
        try:
            perf_data = {
                "operation": operation,
                "duration_seconds": round(duration, 4),
                "timestamp": datetime.now().isoformat()
            }
            
            if extra_data:
                perf_data.update(extra_data)
            
            self.performance_logger.info(json.dumps(perf_data, ensure_ascii=False))
            
            # Cảnh báo nếu quá chậm / Warning if too slow
            if duration > 5.0:  # > 5 seconds
                self.system_warning(f"Slow operation detected: {operation}", {
                    "duration": duration,
                    "operation": operation
                })
                
        except Exception as e:
            self.system_error("Failed to log performance", {"error": str(e)}, e)
    
    # =============================================================================
    # UTILITY METHODS / PHƯƠNG THỨC TIỆN ÍCH
    # =============================================================================
    
    def _log_with_extra(self, logger: logging.Logger, level: int, 
                       message: str, extra_data: Optional[Dict[str, Any]] = None):
        """
        Log message với dữ liệu bổ sung
        Log message with additional data
        
        Args:
            logger: Logger object
            level: Log level
            message: Log message / Thông điệp log
            extra_data: Dữ liệu bổ sung / Additional data
        """
        if extra_data:
            # Format message với extra data / Format message with extra data
            extra_str = json.dumps(extra_data, ensure_ascii=False)
            full_message = f"{message} | DATA: {extra_str}"
        else:
            full_message = message
        
        logger.log(level, full_message)
    
    def get_log_stats(self) -> Dict[str, Any]:
        """
        Lấy thống kê về logs
        Get logging statistics
        
        Returns:
            Dict: Log statistics / Thống kê log
        """
        stats = {
            "log_files": {},
            "system_info": {
                "device_id": SYSTEM_CONFIG["device_id"],
                "version": SYSTEM_CONFIG["version"],
                "log_level": LOGGING_CONFIG["log_level"]
            }
        }
        
        # Thống kê các file log / Statistics for log files
        log_files = [
            LOGGING_CONFIG["system_log_file"],
            LOGGING_CONFIG["violation_log_file"],
            LOGGING_CONFIG["gps_log_file"]
        ]
        
        for log_file in log_files:
            if os.path.exists(log_file):
                file_stats = os.stat(log_file)
                stats["log_files"][os.path.basename(log_file)] = {
                    "size_bytes": file_stats.st_size,
                    "size_mb": round(file_stats.st_size / (1024 * 1024), 2),
                    "modified": datetime.fromtimestamp(file_stats.st_mtime).isoformat()
                }
        
        return stats
    
    def cleanup_old_logs(self, days_to_keep: int = 7):
        """
        Dọn dẹp logs cũ
        Cleanup old log files
        
        Args:
            days_to_keep: Số ngày giữ lại logs / Days to keep logs
        """
        try:
            import time
            current_time = time.time()
            cutoff_time = current_time - (days_to_keep * 24 * 3600)
            
            cleaned_files = []
            
            for log_file in LOGS_DIR.glob("*.log*"):
                if log_file.stat().st_mtime < cutoff_time:
                    log_file.unlink()
                    cleaned_files.append(str(log_file))
            
            if cleaned_files:
                self.system_info(f"Cleaned up {len(cleaned_files)} old log files", {
                    "files_cleaned": cleaned_files,
                    "days_to_keep": days_to_keep
                })
            
        except Exception as e:
            self.system_error("Failed to cleanup old logs", {"error": str(e)}, e)

# =============================================================================
# GLOBAL LOGGER INSTANCE / INSTANCE LOGGER TOÀN CỤC
# =============================================================================

# Tạo instance logger toàn cục / Create global logger instance
_logger_instance = None

def get_logger() -> CycleSentinelLogger:
    """
    Lấy instance logger toàn cục (Singleton pattern)
    Get global logger instance (Singleton pattern)
    
    Returns:
        CycleSentinelLogger: Logger instance
    """
    global _logger_instance
    if _logger_instance is None:
        _logger_instance = CycleSentinelLogger()
    return _logger_instance

# =============================================================================
# CONVENIENCE FUNCTIONS / HÀM TIỆN LỢI
# =============================================================================

def log_system_startup():
    """Log khởi động hệ thống / Log system startup"""
    logger = get_logger()
    logger.system_info("=== CYCLE SENTINEL STARTUP ===", {
        "system_name": SYSTEM_CONFIG["system_name"],
        "version": SYSTEM_CONFIG["version"],
        "device_id": SYSTEM_CONFIG["device_id"],
        "timestamp": datetime.now().isoformat()
    })

def log_system_shutdown():
    """Log tắt hệ thống / Log system shutdown"""
    logger = get_logger()
    logger.system_info("=== CYCLE SENTINEL SHUTDOWN ===", {
        "timestamp": datetime.now().isoformat()
    })
def log_gps_tracking(self, gps_data: Dict[str, Any], zone_info: Optional[Dict[str, Any]] = None):
    """
    ✅ MỚI: Log GPS tracking data vào file riêng biệt
    Log GPS tracking data to separate file
    
    Args:
        gps_data: GPS data theo GPS_DATA_SCHEMA
        zone_info: Thông tin zone hiện tại (optional)
    """
    try:
        # Tạo tracking entry với đầy đủ thông tin
        tracking_entry = {
            "timestamp": gps_data.get("timestamp", datetime.now().isoformat()),
            "device_id": SYSTEM_CONFIG["device_id"],
            "location": {
                "latitude": gps_data.get("latitude", 0.0),
                "longitude": gps_data.get("longitude", 0.0),
                "accuracy": gps_data.get("accuracy", 0.0)
            },
            "speed_data": {
                "current_speed": gps_data.get("speed_kmh", 0.0),
                "speed_limit": zone_info.get("speed_limit", 25.0) if zone_info else 25.0
            },
            "gps_quality": {
                "satellites": gps_data.get("satellites", 0),
                "valid": gps_data.get("valid", False),
                "hdop": gps_data.get("hdop", 0.0)
            },
            "zone_info": zone_info if zone_info else {
                "zone_name": "Unknown",
                "zone_type": "unknown"
            },
            "logged_at": datetime.now().isoformat()
        }
        
        # ✅ Log vào file tracking.log
        if hasattr(self, 'tracking_logger') and self.tracking_logger:
            self.tracking_logger.info(json.dumps(tracking_entry, ensure_ascii=False))
        
        # Log summary vào system log
        self.system_info(f"GPS tracking logged", {
            "lat": tracking_entry["location"]["latitude"],
            "lon": tracking_entry["location"]["longitude"], 
            "speed": tracking_entry["speed_data"]["current_speed"],
            "zone": tracking_entry["zone_info"]["zone_name"]
        })
        
    except Exception as e:
        self.system_error("Failed to log GPS tracking", {
            "error": str(e),
            "gps_data": gps_data
        }, e)

def _setup_loggers(self):
    """
    ✅ ENHANCED: Thêm tracking logger vào setup
    Enhanced setup with tracking logger
    """
    # Tạo thư mục logs nếu chưa có
    LOGS_DIR.mkdir(exist_ok=True)
    
    # System Logger - cho các sự kiện hệ thống
    self.system_logger = self._create_logger(
        name="cycle_sentinel.system",
        log_file=LOGGING_CONFIG["system_log_file"],
        level=LOGGING_CONFIG["log_level"]
    )
    
    # Violation Logger - cho các vi phạm
    self.violation_logger = self._create_logger(
        name="cycle_sentinel.violations",
        log_file=LOGGING_CONFIG["violation_log_file"],
        level="INFO",
        formatter_type="violation"
    )
    
    # ✅ TRACKING LOGGER - MỚI: Cho GPS tracking data
    self.tracking_logger = self._create_logger(
        name="cycle_sentinel.tracking",
        log_file=LOGGING_CONFIG["tracking_log_file"],  # File mới
        level="INFO",
        formatter_type="tracking"
    )
    
    # GPS Logger - cho debug GPS data thô
    self.gps_logger = self._create_logger(
        name="cycle_sentinel.gps",
        log_file=LOGGING_CONFIG["gps_log_file"],
        level="DEBUG",
        formatter_type="simple"
    )
    
    # Performance Logger - cho monitoring hiệu suất
    self.performance_logger = self._create_logger(
        name="cycle_sentinel.performance", 
        log_file=str(LOGS_DIR / "performance.log"),
        level="INFO",
        formatter_type="simple"
    )
# Context manager cho performance logging / Context manager for performance logging
class PerformanceTimer:
    """
    Context manager để đo thời gian thực hiện
    Context manager for measuring execution time
    
    Usage:
        with PerformanceTimer("gps_read"):
            # Code to measure
            pass
    """
    
    def __init__(self, operation_name: str, extra_data: Optional[Dict[str, Any]] = None):
        self.operation_name = operation_name
        self.extra_data = extra_data or {}
        self.start_time = None
        self.logger = get_logger()
    
    def __enter__(self):
        self.start_time = datetime.now()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        if self.start_time:
            duration = (datetime.now() - self.start_time).total_seconds()
            self.logger.log_performance(self.operation_name, duration, self.extra_data)

# =============================================================================
# TESTING / KIỂM TRA
# =============================================================================

if __name__ == "__main__":
    # Test logging system / Kiểm tra hệ thống logging
    print("Testing Cycle Sentinel Logging System...")
    
    logger = get_logger()
    
    # Test system logs
    logger.system_info("Testing system logging")
    logger.system_warning("This is a warning test")
    logger.system_error("This is an error test")
    
    # Test violation log
    test_violation = {
        "violation_type": "speed_violation",
        "location": {"latitude": 10.7749, "longitude": 106.7004},
        "speed_data": {
            "current_speed": 30.5,
            "speed_limit": 25.0,
            "excess_speed": 5.5
        },
        "zone_info": {
            "zone_id": "test_zone_1",
            "zone_name": "Test Zone",
            "zone_type": "residential"
        },
        "confidence": 0.95,
        "duration": 4.2
    }
    logger.log_violation(test_violation)
    
    # Test performance logging
    with PerformanceTimer("test_operation"):
        import time
        time.sleep(0.1)  # Simulate work
    
    # Show stats
    stats = logger.get_log_stats()
    print("Log Stats:", json.dumps(stats, indent=2, ensure_ascii=False))
    
    print("Logging test completed!")