"""
Cycle Sentinel Violation Checker
Bộ kiểm tra vi phạm cho Cycle Sentinel

This module detects and analyzes traffic violations based on GPS data and map zones.
Module này phát hiện và phân tích vi phạm giao thông dựa trên dữ liệu GPS và zones bản đồ.

Features / Tính năng:
- Speed violation detection / Phát hiện vi phạm tốc độ
- Restricted zone entry detection / Phát hiện vào khu vực cấm
- Persistent violation tracking / Theo dõi vi phạm liên tục
- Confidence scoring / Tính điểm tin cậy
- False positive prevention / Ngăn chặn false positive
- Violation history management / Quản lý lịch sử vi phạm

Author: Cycle Sentinel Team
Version: 1.0.0
"""

import uuid
import time
import math
from datetime import datetime, timedelta
from typing import Dict, Any, List, Optional, Tuple
from dataclasses import dataclass, field
from enum import Enum
import threading

# Import cấu hình và logging / Import config and logging
from config.settings import (
    VIOLATION_CONFIG, 
    VIOLATION_DATA_SCHEMA,
    SYSTEM_CONFIG,
    GPS_DATA_SCHEMA
)
from utils.logger import get_logger, PerformanceTimer
from enforcement.map_handler import MapHandler, Zone

class ViolationType(Enum):
    """
    Các loại vi phạm
    Types of violations
    """
    SPEED_VIOLATION = "speed_violation"           # Vượt tốc độ / Speeding
    RESTRICTED_ZONE = "restricted_zone"           # Vào khu vực cấm / Restricted zone entry
    WRONG_DIRECTION = "wrong_direction"           # Đi sai chiều / Wrong direction
    PARKING_VIOLATION = "parking_violation"      # Đỗ xe sai quy định / Parking violation

@dataclass
class ViolationEvent:
    """
    Sự kiện vi phạm đang được theo dõi
    Violation event being tracked
    """
    violation_type: ViolationType
    start_time: datetime
    start_position: Tuple[float, float]  # (lat, lon)
    current_position: Tuple[float, float]
    zone: Optional[Zone]
    speed_data: Dict[str, float] = field(default_factory=dict)
    duration: float = 0.0  # seconds
    confidence: float = 0.0  # 0.0 to 1.0
    max_speed: float = 0.0
    min_speed: float = float('inf')
    sample_count: int = 0
    metadata: Dict[str, Any] = field(default_factory=dict)

@dataclass
class ViolationResult:
    """
    Kết quả vi phạm đã được xác nhận
    Confirmed violation result
    """
    id: str
    violation_type: ViolationType
    timestamp: datetime
    location: Tuple[float, float]
    zone_info: Dict[str, Any]
    speed_data: Dict[str, float]
    duration: float
    confidence: float
    metadata: Dict[str, Any]

class ViolationTracker:
    """
    Theo dõi vi phạm đang diễn ra
    Track ongoing violations
    """
    
    def __init__(self):
        """Khởi tạo violation tracker / Initialize violation tracker"""
        self.active_violations: Dict[str, ViolationEvent] = {}
        self.violation_history: List[ViolationResult] = []
        self.max_history = 100
        
    def start_violation(self, violation_type: ViolationType, 
                       position: Tuple[float, float], zone: Optional[Zone],
                       speed_data: Dict[str, float]) -> str:
        """
        Bắt đầu theo dõi vi phạm mới
        Start tracking new violation
        
        Args:
            violation_type: Loại vi phạm / Violation type
            position: Vị trí (lat, lon) / Position (lat, lon)
            zone: Zone information
            speed_data: Speed related data
            
        Returns:
            str: Violation tracking ID
        """
        violation_id = f"{violation_type.value}_{int(time.time())}"
        
        violation_event = ViolationEvent(
            violation_type=violation_type,
            start_time=datetime.now(),
            start_position=position,
            current_position=position,
            zone=zone,
            speed_data=speed_data.copy(),
            metadata={}
        )
        
        self.active_violations[violation_id] = violation_event
        return violation_id
    
    def update_violation(self, violation_id: str, position: Tuple[float, float],
                        speed_data: Dict[str, float]) -> bool:
        """
        Cập nhật vi phạm đang theo dõi
        Update ongoing violation
        
        Args:
            violation_id: ID của vi phạm
            position: Vị trí hiện tại / Current position
            speed_data: Dữ liệu tốc độ / Speed data
            
        Returns:
            bool: True nếu cập nhật thành công
        """
        if violation_id not in self.active_violations:
            return False
        
        violation = self.active_violations[violation_id]
        
        # Cập nhật thông tin / Update information
        violation.current_position = position
        violation.duration = (datetime.now() - violation.start_time).total_seconds()
        violation.sample_count += 1
        
        # Cập nhật speed data / Update speed data
        current_speed = speed_data.get("current_speed", 0.0)
        violation.max_speed = max(violation.max_speed, current_speed)
        violation.min_speed = min(violation.min_speed, current_speed)
        
        # Cập nhật confidence dựa trên duration và consistency
        # Update confidence based on duration and consistency
        violation.confidence = self._calculate_confidence(violation)
        
        return True
    
    def end_violation(self, violation_id: str) -> Optional[ViolationResult]:
        """
        Kết thúc và finalize vi phạm
        End and finalize violation
        
        Args:
            violation_id: ID của vi phạm
            
        Returns:
            ViolationResult: Kết quả vi phạm nếu hợp lệ
        """
        if violation_id not in self.active_violations:
            return None
        
        violation = self.active_violations.pop(violation_id)
        
        # Kiểm tra vi phạm có đủ điều kiện không / Check if violation meets criteria
        if (violation.duration >= VIOLATION_CONFIG["min_violation_duration"] and
            violation.confidence >= 0.5):  # Minimum confidence threshold
            
            # Tạo violation result / Create violation result
            result = ViolationResult(
                id=str(uuid.uuid4()),
                violation_type=violation.violation_type,
                timestamp=violation.start_time,
                location=violation.start_position,
                zone_info=self._get_zone_info(violation.zone),
                speed_data={
                    "current_speed": violation.speed_data.get("current_speed", 0.0),
                    "speed_limit": violation.speed_data.get("speed_limit", 0.0),
                    "excess_speed": violation.speed_data.get("excess_speed", 0.0),
                    "max_speed": violation.max_speed,
                    "min_speed": violation.min_speed
                },
                duration=violation.duration,
                confidence=violation.confidence,
                metadata=violation.metadata
            )
            
            # Thêm vào lịch sử / Add to history
            self.violation_history.append(result)
            if len(self.violation_history) > self.max_history:
                self.violation_history.pop(0)
            
            return result
        
        return None
    
    def _calculate_confidence(self, violation: ViolationEvent) -> float:
        """
        Tính confidence score cho vi phạm
        Calculate confidence score for violation
        
        Args:
            violation: Violation event
            
        Returns:
            float: Confidence score (0.0 to 1.0)
        """
        confidence = 0.0
        
        # Duration factor / Yếu tố thời gian
        duration_factor = min(violation.duration / VIOLATION_CONFIG["min_violation_duration"], 1.0)
        confidence += duration_factor * 0.4
        
        # Sample count factor / Yếu tố số lượng mẫu
        sample_factor = min(violation.sample_count / VIOLATION_CONFIG["consecutive_points_required"], 1.0)
        confidence += sample_factor * 0.3
        
        # Consistency factor / Yếu tố nhất quán
        if violation.violation_type == ViolationType.SPEED_VIOLATION:
            speed_range = violation.max_speed - violation.min_speed
            consistency_factor = max(0, 1.0 - (speed_range / 20.0))  # Less variation = higher confidence
            confidence += consistency_factor * 0.3
        else:
            confidence += 0.3  # Default for non-speed violations
        
        return min(confidence, 1.0)
    
    def _get_zone_info(self, zone: Optional[Zone]) -> Dict[str, Any]:
        """
        Lấy thông tin zone cho violation result
        Get zone information for violation result
        
        Args:
            zone: Zone object
            
        Returns:
            Dict: Zone information
        """
        if zone:
            return {
                "zone_id": zone.id,
                "zone_name": zone.name,
                "zone_type": zone.zone_type,
                "speed_limit": zone.speed_limit,
                "is_restricted": zone.is_restricted
            }
        else:
            return {
                "zone_id": "unknown",
                "zone_name": "Unknown Zone",
                "zone_type": "unknown",
                "speed_limit": 25.0,
                "is_restricted": False
            }
    
    def cleanup_stale_violations(self, max_age_seconds: float = 300.0):
        """
        Dọn dẹp các vi phạm quá cũ
        Cleanup stale violations
        
        Args:
            max_age_seconds: Tuổi tối đa của vi phạm (giây)
        """
        current_time = datetime.now()
        stale_ids = []
        
        for violation_id, violation in self.active_violations.items():
            age = (current_time - violation.start_time).total_seconds()
            if age > max_age_seconds:
                stale_ids.append(violation_id)
        
        for violation_id in stale_ids:
            self.active_violations.pop(violation_id, None)

class ViolationChecker:
    """
    Bộ kiểm tra vi phạm chính cho Cycle Sentinel
    Main violation checker for Cycle Sentinel
    
    Analyzes GPS data against map zones to detect traffic violations
    Phân tích dữ liệu GPS với zones bản đồ để phát hiện vi phạm giao thông
    """
    
    def __init__(self, map_handler: MapHandler):
        """
        Khởi tạo Violation Checker
        Initialize Violation Checker
        
        Args:
            map_handler: Map handler instance
        """
        self.logger = get_logger()
        self.map_handler = map_handler
        self.tracker = ViolationTracker()
        
        # Thread safety / An toàn luồng
        self._lock = threading.RLock()
        
        # Position history cho smoothing / Position history for smoothing
        self.position_history: List[Dict[str, Any]] = []
        self.max_position_history = 10
        
        # Last violation positions để tránh duplicate / Last violation positions to avoid duplicates
        self.recent_violation_positions: List[Tuple[float, float, datetime]] = []
        self.violation_position_timeout = 60.0  # seconds
        
        # Statistics / Thống kê
        self.stats = {
            "total_checks": 0,
            "violations_detected": 0,
            "false_positives_prevented": 0,
            "speed_violations": 0,
            "zone_violations": 0,
            "last_check_time": None,
            "average_check_time": 0.0
        }
        
        self.logger.system_info("Violation Checker initialized", {
            "device_id": SYSTEM_CONFIG["device_id"],
            "enabled_violations": VIOLATION_CONFIG["enabled_violations"]
        })
    
    def check_violation(self, gps_data: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        """
        Kiểm tra vi phạm từ dữ liệu GPS
        Check for violations from GPS data
        
        Args:
            gps_data: GPS data theo GPS_DATA_SCHEMA
            
        Returns:
            Dict: Violation data nếu phát hiện vi phạm, None nếu không
        """
        with self._lock:
            with PerformanceTimer("violation_check") as timer:
                self.stats["total_checks"] += 1
                self.stats["last_check_time"] = datetime.now()
                
                try:
                    # Validate GPS data / Xác thực dữ liệu GPS
                    if not self._validate_gps_data(gps_data):
                        return None
                    
                    # Thêm vào position history / Add to position history
                    self._update_position_history(gps_data)
                    
                    # Cleanup stale violations / Dọn dẹp vi phạm cũ
                    self.tracker.cleanup_stale_violations()
                    
                    # Cleanup old violation positions / Dọn dẹp vị trí vi phạm cũ
                    self._cleanup_old_violation_positions()
                    
                    # Kiểm tra các loại vi phạm / Check different violation types
                    violation_result = None
                    
                    # 1. Speed violation check / Kiểm tra vi phạm tốc độ
                    if "speed_violation" in VIOLATION_CONFIG["enabled_violations"]:
                        violation_result = self._check_speed_violation(gps_data)
                    
                    # 2. Restricted zone check / Kiểm tra khu vực cấm
                    if not violation_result and "restricted_zone" in VIOLATION_CONFIG["enabled_violations"]:
                        violation_result = self._check_restricted_zone_violation(gps_data)
                    
                    # Update performance stats / Cập nhật thống kê hiệu suất
                    self._update_performance_stats(timer.start_time)
                    
                    return violation_result
                    
                except Exception as e:
                    self.logger.system_error("Error in violation check", {
                        "gps_data": gps_data,
                        "error": str(e)
                    }, e)
                    return None
    
    def _validate_gps_data(self, gps_data: Dict[str, Any]) -> bool:
        """
        Xác thực dữ liệu GPS
        Validate GPS data
        
        Args:
            gps_data: GPS data
            
        Returns:
            bool: True nếu hợp lệ
        """
        try:
            # Kiểm tra các trường bắt buộc / Check required fields
            required_fields = ["latitude", "longitude", "speed_kmh", "valid"]
            for field in required_fields:
                if field not in gps_data:
                    return False
            
            # Kiểm tra validity / Check validity
            if not gps_data.get("valid", False):
                return False
            
            # Kiểm tra tọa độ hợp lệ / Check valid coordinates
            lat = gps_data["latitude"]
            lon = gps_data["longitude"]
            if not (-90 <= lat <= 90) or not (-180 <= lon <= 180):
                return False
            
            # Kiểm tra tốc độ hợp lệ / Check valid speed
            speed = gps_data["speed_kmh"]
            if speed < 0 or speed > 200:  # Unrealistic speeds
                return False
            
            # Kiểm tra accuracy nếu có / Check accuracy if available
            accuracy = gps_data.get("accuracy", 0)
            if accuracy > 50:  # Poor accuracy
                return False
            
            return True
            
        except (ValueError, TypeError):
            return False
    
    def _update_position_history(self, gps_data: Dict[str, Any]):
        """
        Cập nhật lịch sử vị trí
        Update position history
        
        Args:
            gps_data: GPS data
        """
        self.position_history.append({
            "timestamp": datetime.now(),
            "latitude": gps_data["latitude"],
            "longitude": gps_data["longitude"],
            "speed_kmh": gps_data["speed_kmh"]
        })
        
        # Giữ history trong giới hạn / Keep history within limits
        if len(self.position_history) > self.max_position_history:
            self.position_history.pop(0)
    
    def _check_speed_violation(self, gps_data: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        """
        Kiểm tra vi phạm tốc độ
        Check for speed violations
        
        Args:
            gps_data: GPS data
            
        Returns:
            Dict: Violation data nếu có vi phạm
        """
        try:
            lat = gps_data["latitude"]
            lon = gps_data["longitude"]
            current_speed = gps_data["speed_kmh"]
            
            # Kiểm tra tốc độ tối thiểu / Check minimum speed
            if current_speed < VIOLATION_CONFIG["min_speed_for_violation"]:
                return None
            
            # Lấy zone và speed limit / Get zone and speed limit
            zone = self.map_handler.find_zone_at_position(lat, lon)
            speed_limit = self.map_handler.get_speed_limit_at_position(lat, lon)
            
            # Tính excess speed / Calculate excess speed
            tolerance = VIOLATION_CONFIG["speed_tolerance_kmh"]
            excess_speed = current_speed - speed_limit - tolerance
            
            # Kiểm tra vi phạm / Check violation
            if excess_speed > 0:
                # Kiểm tra có phải duplicate không / Check for duplicates
                if self._is_duplicate_violation((lat, lon), ViolationType.SPEED_VIOLATION):
                    return None
                
                # Tạo speed data / Create speed data
                speed_data = {
                    "current_speed": current_speed,
                    "speed_limit": speed_limit,
                    "excess_speed": excess_speed
                }
                
                # Kiểm tra persistence / Check persistence
                violation_result = self._check_violation_persistence(
                    ViolationType.SPEED_VIOLATION,
                    (lat, lon),
                    zone,
                    speed_data
                )
                
                if violation_result:
                    self.stats["violations_detected"] += 1
                    self.stats["speed_violations"] += 1
                    
                    # Thêm vào recent violations / Add to recent violations
                    self._add_recent_violation_position((lat, lon))
                    
                    return violation_result
            
            return None
            
        except Exception as e:
            self.logger.system_error("Error checking speed violation", {"error": str(e)}, e)
            return None
    
    def _check_restricted_zone_violation(self, gps_data: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        """
        Kiểm tra vi phạm khu vực cấm
        Check for restricted zone violations
        
        Args:
            gps_data: GPS data
            
        Returns:
            Dict: Violation data nếu có vi phạm
        """
        try:
            lat = gps_data["latitude"]
            lon = gps_data["longitude"]
            current_speed = gps_data["speed_kmh"]
            
            # Kiểm tra có trong khu vực cấm không / Check if in restricted zone
            zone = self.map_handler.find_zone_at_position(lat, lon)
            
            if zone and zone.is_restricted:
                # Kiểm tra có đang di chuyển không / Check if moving
                if current_speed > 1.0:  # Moving faster than 1 km/h
                    
                    # Kiểm tra có phải duplicate không / Check for duplicates
                    if self._is_duplicate_violation((lat, lon), ViolationType.RESTRICTED_ZONE):
                        return None
                    
                    # Tạo speed data / Create speed data
                    speed_data = {
                        "current_speed": current_speed,
                        "speed_limit": 0.0,  # Restricted zones have 0 speed limit
                        "excess_speed": current_speed
                    }
                    
                    # Kiểm tra persistence / Check persistence
                    violation_result = self._check_violation_persistence(
                        ViolationType.RESTRICTED_ZONE,
                        (lat, lon),
                        zone,
                        speed_data
                    )
                    
                    if violation_result:
                        self.stats["violations_detected"] += 1
                        self.stats["zone_violations"] += 1
                        
                        # Thêm vào recent violations / Add to recent violations
                        self._add_recent_violation_position((lat, lon))
                        
                        return violation_result
            
            return None
            
        except Exception as e:
            self.logger.system_error("Error checking restricted zone violation", {"error": str(e)}, e)
            return None
    
    def _check_violation_persistence(self, violation_type: ViolationType,
                                   position: Tuple[float, float], zone: Optional[Zone],
                                   speed_data: Dict[str, float]) -> Optional[Dict[str, Any]]:
        """
        Kiểm tra tính persistence của vi phạm
        Check violation persistence
        
        Args:
            violation_type: Loại vi phạm
            position: Vị trí
            zone: Zone information
            speed_data: Speed data
            
        Returns:
            Dict: Violation data nếu đủ persistent
        """
        # Tìm vi phạm đang active của cùng loại / Find active violation of same type
        active_violation_id = None
        for vid, violation in self.tracker.active_violations.items():
            if (violation.violation_type == violation_type and
                violation.zone == zone):
                active_violation_id = vid
                break
        
        if active_violation_id:
            # Update existing violation / Cập nhật vi phạm hiện có
            self.tracker.update_violation(active_violation_id, position, speed_data)
            
            # Kiểm tra có đủ điều kiện finalize không / Check if ready to finalize
            violation = self.tracker.active_violations[active_violation_id]
            if (violation.duration >= VIOLATION_CONFIG["min_violation_duration"] and
                violation.sample_count >= VIOLATION_CONFIG["consecutive_points_required"]):
                
                # Finalize violation / Hoàn thành vi phạm
                result = self.tracker.end_violation(active_violation_id)
                if result:
                    return self._format_violation_result(result)
        else:
            # Start new violation tracking / Bắt đầu theo dõi vi phạm mới
            self.tracker.start_violation(violation_type, position, zone, speed_data)
        
        return None
    
    def _is_duplicate_violation(self, position: Tuple[float, float], 
                              violation_type: ViolationType) -> bool:
        """
        Kiểm tra có phải duplicate violation không
        Check if this is a duplicate violation
        
        Args:
            position: Vị trí hiện tại / Current position
            violation_type: Loại vi phạm / Violation type
            
        Returns:
            bool: True nếu là duplicate
        """
        lat, lon = position
        min_distance = VIOLATION_CONFIG["min_distance_between_violations"]
        
        for prev_lat, prev_lon, timestamp in self.recent_violation_positions:
            # Tính khoảng cách / Calculate distance
            distance = self._calculate_distance(lat, lon, prev_lat, prev_lon)
            
            if distance < min_distance:
                return True
        
        return False
    
    def _calculate_distance(self, lat1: float, lon1: float, 
                          lat2: float, lon2: float) -> float:
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
    
    def _add_recent_violation_position(self, position: Tuple[float, float]):
        """
        Thêm vị trí vi phạm vào recent list
        Add violation position to recent list
        
        Args:
            position: Vị trí vi phạm / Violation position
        """
        self.recent_violation_positions.append((position[0], position[1], datetime.now()))
    
    def _cleanup_old_violation_positions(self):
        """
        Dọn dẹp vị trí vi phạm cũ
        Cleanup old violation positions
        """
        current_time = datetime.now()
        timeout = timedelta(seconds=self.violation_position_timeout)
        
        self.recent_violation_positions = [
            (lat, lon, timestamp) for lat, lon, timestamp in self.recent_violation_positions
            if current_time - timestamp < timeout
        ]
    
    def _format_violation_result(self, result: ViolationResult) -> Dict[str, Any]:
        """
        Format violation result theo VIOLATION_DATA_SCHEMA
        Format violation result according to VIOLATION_DATA_SCHEMA
        
        Args:
            result: Violation result
            
        Returns:
            Dict: Formatted violation data
        """
        return {
            "id": result.id,
            "timestamp": result.timestamp.isoformat(),
            "device_id": SYSTEM_CONFIG["device_id"],
            "violation_type": result.violation_type.value,
            "location": {
                "latitude": result.location[0],
                "longitude": result.location[1]
            },
            "speed_data": result.speed_data,
            "zone_info": result.zone_info,
            "confidence": result.confidence,
            "duration": result.duration
        }
    
    def _update_performance_stats(self, start_time: datetime):
        """
        Cập nhật thống kê hiệu suất
        Update performance statistics
        
        Args:
            start_time: Thời gian bắt đầu check / Check start time
        """
        check_time = (datetime.now() - start_time).total_seconds()
        
        # Update average check time / Cập nhật thời gian check trung bình
        if self.stats["average_check_time"] == 0:
            self.stats["average_check_time"] = check_time
        else:
            # Moving average / Trung bình trượt
            self.stats["average_check_time"] = (
                self.stats["average_check_time"] * 0.9 + check_time * 0.1
            )
    
    def get_statistics(self) -> Dict[str, Any]:
        """
        Lấy thống kê violation checker
        Get violation checker statistics
        
        Returns:
            Dict: Statistics data
        """
        with self._lock:
            stats = self.stats.copy()
            
            # Thêm thông tin real-time / Add real-time information
            stats.update({
                "active_violations": len(self.tracker.active_violations),
                "violation_history_count": len(self.tracker.violation_history),
                "recent_violation_positions": len(self.recent_violation_positions),
                "position_history_count": len(self.position_history),
                "map_loaded": self.map_handler.is_map_loaded()
            })
            
            # Tính violation rate / Calculate violation rate
            if stats["total_checks"] > 0:
                stats["violation_rate"] = (stats["violations_detected"] / stats["total_checks"]) * 100
            else:
                stats["violation_rate"] = 0.0
            
            return stats
    
    def reset_statistics(self):
        """
        Reset thống kê
        Reset statistics
        """
        with self._lock:
            self.stats = {
                "total_checks": 0,
                "violations_detected": 0,
                "false_positives_prevented": 0,
                "speed_violations": 0,
                "zone_violations": 0,
                "last_check_time": None,
                "average_check_time": 0.0
            }
            
            self.logger.system_info("Violation checker statistics reset")

# =============================================================================
# CONVENIENCE FUNCTIONS / HÀM TIỆN ÍCH
# =============================================================================

def create_violation_checker(map_handler: MapHandler) -> ViolationChecker:
    """
    Tạo violation checker instance
    Create violation checker instance
    
    Args:
        map_handler: Map handler instance
        
    Returns:
        ViolationChecker: Configured violation checker
    """
    return ViolationChecker(map_handler)

def test_violation_scenarios(checker: ViolationChecker) -> Dict[str, Any]:
    """
    Test các scenario vi phạm khác nhau
    Test different violation scenarios
    
    Args:
        checker: Violation checker instance
        
    Returns:
        Dict: Test results
    """
    test_results = {
        "speed_violation_tests": 0,
        "zone_violation_tests": 0,
        "false_positive_prevention": 0,
        "total_tests": 0
    }
    
    logger = get_logger()
    logger.system_info("Running violation checker tests")
    
    # Test 1: Speed violation scenario / Test 1: Kịch bản vi phạm tốc độ
    speed_test_data = [
        {
            "latitude": 10.7750,
            "longitude": 106.7000,
            "speed_kmh": 35.0,  # Over 25 km/h limit
            "valid": True,
            "accuracy": 5.0,
            "timestamp": datetime.now().isoformat()
        }
    ]
    
    for data in speed_test_data:
        result = checker.check_violation(data)
        test_results["total_tests"] += 1
        if result and result["violation_type"] == "speed_violation":
            test_results["speed_violation_tests"] += 1
    
    # Test 2: Restricted zone violation / Test 2: Vi phạm khu vực cấm
    zone_test_data = [
        {
            "latitude": 10.7750,
            "longitude": 106.7010,  # Nguyen Hue Walking Street
            "speed_kmh": 10.0,  # Moving in restricted zone
            "valid": True,
            "accuracy": 5.0,
            "timestamp": datetime.now().isoformat()
        }
    ]
    
    for data in zone_test_data:
        result = checker.check_violation(data)
        test_results["total_tests"] += 1
        if result and result["violation_type"] == "restricted_zone":
            test_results["zone_violation_tests"] += 1
    
    # Test 3: Normal riding (no violation) / Test 3: Đi xe bình thường (không vi phạm)
    normal_test_data = [
        {
            "latitude": 10.7750,
            "longitude": 106.7000,
            "speed_kmh": 20.0,  # Within speed limit
            "valid": True,
            "accuracy": 5.0,
            "timestamp": datetime.now().isoformat()
        }
    ]
    
    for data in normal_test_data:
        result = checker.check_violation(data)
        test_results["total_tests"] += 1
        if not result:
            test_results["false_positive_prevention"] += 1
    
    logger.system_info("Violation checker tests completed", test_results)
    return test_results

# =============================================================================
# TESTING / KIỂM TRA
# =============================================================================

if __name__ == "__main__":
    print("Testing Cycle Sentinel Violation Checker...")
    
    # Import dependencies / Import các dependency
    from enforcement.map_handler import create_map_handler, create_sample_hcm_map
    import json
    import tempfile
    import os
    
    # Create sample map / Tạo sample map
    sample_map = create_sample_hcm_map()
    
    # Save to temporary file / Lưu vào file tạm
    with tempfile.NamedTemporaryFile(mode='w', suffix='.json', delete=False) as f:
        json.dump(sample_map, f, indent=2)
        temp_map_file = f.name
    
    try:
        # Create map handler / Tạo map handler
        map_handler = create_map_handler(temp_map_file)
        
        if not map_handler.is_map_loaded():
            print("❌ Failed to load map for testing")
            exit(1)
        
        print("✅ Map loaded successfully")
        
        # Create violation checker / Tạo violation checker
        violation_checker = create_violation_checker(map_handler)
        print("✅ Violation checker created")
        
        # Test scenarios / Test các scenarios
        print("\n=== Testing Violation Scenarios ===")
        
        # Scenario 1: Speed violation / Kịch bản 1: Vi phạm tốc độ
        print("\n1. Speed Violation Test:")
        speed_violation_data = {
            "latitude": 10.7750,
            "longitude": 106.7000,  # District 1 center (20 km/h limit)
            "speed_kmh": 35.0,      # Exceeds limit by 15 km/h
            "valid": True,
            "accuracy": 5.0,
            "satellites": 8,
            "timestamp": datetime.now().isoformat()
        }
        
        # Test multiple times để trigger persistence / Test multiple times to trigger persistence
        for i in range(5):
            result = violation_checker.check_violation(speed_violation_data)
            if result:
                print(f"  ⚠️  Speed violation detected:")
                print(f"     Type: {result['violation_type']}")
                print(f"     Speed: {result['speed_data']['current_speed']} km/h")
                print(f"     Limit: {result['speed_data']['speed_limit']} km/h")
                print(f"     Excess: {result['speed_data']['excess_speed']} km/h")
                print(f"     Confidence: {result['confidence']:.2f}")
                break
            else:
                print(f"  Check {i+1}: Building violation evidence...")
        
        # Scenario 2: Restricted zone violation / Kịch bản 2: Vi phạm khu vực cấm
        print("\n2. Restricted Zone Violation Test:")
        zone_violation_data = {
            "latitude": 10.7750,
            "longitude": 106.7010,  # Nguyen Hue Walking Street (restricted)
            "speed_kmh": 15.0,      # Moving in restricted area
            "valid": True,
            "accuracy": 5.0,
            "satellites": 8,
            "timestamp": datetime.now().isoformat()
        }
        
        # Test multiple times để trigger persistence / Test multiple times to trigger persistence
        for i in range(5):
            result = violation_checker.check_violation(zone_violation_data)
            if result:
                print(f"  ⚠️  Zone violation detected:")
                print(f"     Type: {result['violation_type']}")
                print(f"     Zone: {result['zone_info']['zone_name']}")
                print(f"     Speed: {result['speed_data']['current_speed']} km/h")
                print(f"     Confidence: {result['confidence']:.2f}")
                break
            else:
                print(f"  Check {i+1}: Building violation evidence...")
        
        # Scenario 3: Normal riding (no violation) / Kịch bản 3: Đi xe bình thường
        print("\n3. Normal Riding Test:")
        normal_data = {
            "latitude": 10.7750,
            "longitude": 106.7000,  # District 1 center
            "speed_kmh": 18.0,      # Within limit (20 km/h)
            "valid": True,
            "accuracy": 5.0,
            "satellites": 8,
            "timestamp": datetime.now().isoformat()
        }
        
        result = violation_checker.check_violation(normal_data)
        if result:
            print(f"  ❌ Unexpected violation detected: {result['violation_type']}")
        else:
            print("  ✅ No violation detected (correct)")
        
        # Scenario 4: Invalid GPS data / Kịch bản 4: Dữ liệu GPS không hợp lệ
        print("\n4. Invalid GPS Data Test:")
        invalid_data = {
            "latitude": 10.7750,
            "longitude": 106.7000,
            "speed_kmh": 25.0,
            "valid": False,  # Invalid GPS fix
            "accuracy": 50.0,  # Poor accuracy
            "satellites": 2,
            "timestamp": datetime.now().isoformat()
        }
        
        result = violation_checker.check_violation(invalid_data)
        if result:
            print(f"  ❌ Violation detected from invalid data: {result['violation_type']}")
        else:
            print("  ✅ Invalid data correctly rejected")
        
        # Show statistics / Hiển thị thống kê
        print("\n=== Violation Checker Statistics ===")
        stats = violation_checker.get_statistics()
        for key, value in stats.items():
            if isinstance(value, float):
                print(f"  {key}: {value:.3f}")
            else:
                print(f"  {key}: {value}")
        
        # Test comprehensive scenarios / Test các scenarios toàn diện
        print("\n=== Running Comprehensive Tests ===")
        test_results = test_violation_scenarios(violation_checker)
        
        print("Test Results:")
        for key, value in test_results.items():
            print(f"  {key}: {value}")
        
        # Verify persistence mechanism / Kiểm tra cơ chế persistence
        print("\n=== Testing Violation Persistence ===")
        print("Testing that violations require sustained behavior...")
        
        # Single speed violation (should not trigger) / Vi phạm tốc độ đơn lẻ (không nên trigger)
        single_violation_data = {
            "latitude": 10.7880,
            "longitude": 106.7130,  # Residential area (25 km/h limit)
            "speed_kmh": 40.0,      # High speed
            "valid": True,
            "accuracy": 3.0,
            "timestamp": datetime.now().isoformat()
        }
        
        result = violation_checker.check_violation(single_violation_data)
        if result:
            print("  ❌ Single violation incorrectly triggered enforcement")
        else:
            print("  ✅ Single violation correctly ignored (building evidence)")
        
        print("\n✅ All violation checker tests completed successfully!")
        
    finally:
        # Cleanup / Dọn dẹp
        try:
            os.unlink(temp_map_file)
            print(f"✅ Temporary map file cleaned up")
        except:
            pass
    
    print("Violation Checker test completed!")