"""
Cycle Sentinel Map Handler
Bộ xử lý bản đồ cho Cycle Sentinel

This module handles map loading, zone management, and geospatial operations.
Module này xử lý load bản đồ, quản lý zones và các phép toán địa lý.

Features / Tính năng:
- GeoJSON map parsing / Phân tích bản đồ GeoJSON
- Zone detection and management / Phát hiện và quản lý zones
- Point-in-polygon calculations / Tính toán điểm trong đa giác
- Speed limit lookup / Tra cứu giới hạn tốc độ
- Map validation and caching / Xác thực và cache bản đồ
- Real-time zone updates / Cập nhật zones theo thời gian thực

Author: Cycle Sentinel Team
Version: 1.0.0
"""

import json
import os
import hashlib
import time
from datetime import datetime, timedelta
from typing import Dict, Any, List, Tuple, Optional, Union
from dataclasses import dataclass
from pathlib import Path
import threading
from shapely.geometry import Point, Polygon, MultiPolygon
from shapely.prepared import prep
import geojson

# Import cấu hình và logging / Import config and logging
from config.settings import (
    MAP_CONFIG, 
    DEFAULT_SPEED_LIMITS, 
    MAPS_DIR,
    SYSTEM_CONFIG
)
from utils.logger import get_logger, PerformanceTimer

@dataclass
class Zone:
    """
    Thông tin về một zone trên bản đồ
    Information about a map zone
    """
    id: str
    name: str
    zone_type: str  # residential, commercial, school_zone, etc.
    speed_limit: float  # km/h
    is_restricted: bool  # Có phải khu vực cấm không / Is restricted zone
    is_active: bool  # Zone có đang hoạt động không / Is zone currently active
    geometry: Union[Polygon, MultiPolygon]  # Shapely geometry
    prepared_geometry: Any  # Prepared geometry for fast operations
    metadata: Dict[str, Any]  # Additional zone information
    
    def __post_init__(self):
        """Chuẩn bị geometry để tăng tốc operations / Prepare geometry for faster operations"""
        if self.geometry:
            self.prepared_geometry = prep(self.geometry)

@dataclass
class MapInfo:
    """
    Thông tin metadata về bản đồ
    Map metadata information
    """
    file_path: str
    file_hash: str
    load_time: datetime
    version: str
    total_zones: int
    coverage_bounds: Tuple[float, float, float, float]  # min_lat, min_lon, max_lat, max_lon
    last_modified: datetime

class MapHandler:
    """
    Bộ xử lý bản đồ chính cho Cycle Sentinel
    Main map handler for Cycle Sentinel
    
    Handles loading, parsing, and querying of geospatial zone data
    Xử lý load, phân tích và truy vấn dữ liệu zone địa lý
    """
    
    def __init__(self, map_file_path: Optional[str] = None):
        """
        Khởi tạo Map Handler
        Initialize Map Handler
        
        Args:
            map_file_path: Đường dẫn file bản đồ / Map file path
        """
        self.logger = get_logger()
        
        # File path và cache / File path and cache
        self.map_file_path = map_file_path or MAP_CONFIG["map_file"]
        self.backup_file_path = MAP_CONFIG["backup_map_file"]
        
        # Zone data / Dữ liệu zones
        self.zones: Dict[str, Zone] = {}
        self.zones_by_type: Dict[str, List[Zone]] = {}
        self.spatial_index: List[Zone] = []  # For spatial queries
        
        # Map metadata / Metadata bản đồ
        self.map_info: Optional[MapInfo] = None
        self.is_loaded = False
        
        # Thread safety / An toàn luồng
        self._lock = threading.RLock()
        
        # Auto-reload monitoring / Giám sát auto-reload
        self._last_file_check = 0
        self._file_check_interval = 5.0  # seconds
        
        # Statistics / Thống kê
        self.stats = {
            "total_queries": 0,
            "zone_hits": 0,
            "zone_misses": 0,
            "load_time": 0.0,
            "last_query_time": None
        }
        
        self.logger.system_info("Map Handler initialized", {
            "map_file": self.map_file_path,
            "auto_reload": MAP_CONFIG["auto_reload"]
        })
        
        # Load bản đồ ban đầu / Load initial map
        if os.path.exists(self.map_file_path):
            self.load_map()
    
    def load_map(self, file_path: Optional[str] = None) -> bool:
        """
        Load bản đồ từ file
        Load map from file
        
        Args:
            file_path: Đường dẫn file (nếu khác với mặc định) / File path (if different from default)
            
        Returns:
            bool: True nếu load thành công / True if loaded successfully
        """
        target_file = file_path or self.map_file_path
        
        with self._lock:
            try:
                with PerformanceTimer("map_load"):
                    self.logger.system_info(f"Loading map from {target_file}")
                    
                    # Kiểm tra file tồn tại / Check file exists
                    if not os.path.exists(target_file):
                        self.logger.system_error(f"Map file not found: {target_file}")
                        return self._try_load_backup()
                    
                    # Đọc và parse file / Read and parse file
                    map_data = self._read_map_file(target_file)
                    if not map_data:
                        return self._try_load_backup()
                    
                    # Validate map data / Xác thực dữ liệu bản đồ
                    if not self._validate_map_data(map_data):
                        self.logger.system_error("Map validation failed")
                        return self._try_load_backup()
                    
                    # Parse zones / Phân tích zones
                    zones = self._parse_zones(map_data)
                    if not zones:
                        self.logger.system_error("No valid zones found in map")
                        return self._try_load_backup()
                    
                    # Cập nhật internal state / Update internal state
                    self._update_zones(zones)
                    self._build_spatial_index()
                    self._create_map_info(target_file, map_data)
                    
                    self.is_loaded = True
                    
                    self.logger.system_info("Map loaded successfully", {
                        "file": target_file,
                        "zones_count": len(self.zones),
                        "zone_types": list(self.zones_by_type.keys())
                    })
                    
                    # Backup map nếu load thành công / Backup map if loaded successfully
                    if target_file != self.backup_file_path:
                        self._create_backup()
                    
                    return True
                    
            except Exception as e:
                self.logger.system_error("Failed to load map", {
                    "file": target_file,
                    "error": str(e)
                }, e)
                return self._try_load_backup()
    
    def _read_map_file(self, file_path: str) -> Optional[Dict[str, Any]]:
        """
        Đọc file bản đồ
        Read map file
        
        Args:
            file_path: Đường dẫn file / File path
            
        Returns:
            Dict: Map data hoặc None nếu lỗi
        """
        try:
            with open(file_path, 'r', encoding=MAP_CONFIG["encoding"]) as f:
                content = f.read()
            
            # Parse JSON
            map_data = json.loads(content)
            
            # Validate basic structure / Kiểm tra cấu trúc cơ bản
            if not isinstance(map_data, dict):
                raise ValueError("Map data must be a dictionary")
            
            return map_data
            
        except json.JSONDecodeError as e:
            self.logger.system_error(f"JSON parse error in {file_path}", {"error": str(e)}, e)
            return None
        except Exception as e:
            self.logger.system_error(f"Error reading {file_path}", {"error": str(e)}, e)
            return None
    
    def _validate_map_data(self, map_data: Dict[str, Any]) -> bool:
        """
        Xác thực dữ liệu bản đồ
        Validate map data
        
        Args:
            map_data: Dữ liệu bản đồ / Map data
            
        Returns:
            bool: True nếu hợp lệ / True if valid
        """
        try:
            # Kiểm tra có zones không / Check if zones exist
            if "zones" not in map_data and "features" not in map_data:
                self.logger.system_error("Map data missing 'zones' or 'features' field")
                return False
            
            # Nếu là GeoJSON format / If GeoJSON format
            if "features" in map_data:
                if not isinstance(map_data["features"], list):
                    self.logger.system_error("GeoJSON features must be a list")
                    return False
                
                if len(map_data["features"]) == 0:
                    self.logger.system_error("No features found in GeoJSON")
                    return False
            
            # Nếu là custom zones format / If custom zones format
            elif "zones" in map_data:
                if not isinstance(map_data["zones"], list):
                    self.logger.system_error("Zones must be a list")
                    return False
                
                if len(map_data["zones"]) == 0:
                    self.logger.system_error("No zones found in map data")
                    return False
            
            return True
            
        except Exception as e:
            self.logger.system_error("Map validation error", {"error": str(e)}, e)
            return False
    
    def _parse_zones(self, map_data: Dict[str, Any]) -> List[Zone]:
        """
        Parse zones từ map data
        Parse zones from map data
        
        Args:
            map_data: Dữ liệu bản đồ / Map data
            
        Returns:
            List[Zone]: Danh sách zones đã parse
        """
        zones = []
        
        try:
            # Parse GeoJSON format / Phân tích format GeoJSON
            if "features" in map_data:
                zones.extend(self._parse_geojson_features(map_data["features"]))
            
            # Parse custom zones format / Phân tích format zones tùy chỉnh
            elif "zones" in map_data:
                zones.extend(self._parse_custom_zones(map_data["zones"]))
            
            self.logger.system_debug(f"Parsed {len(zones)} zones from map data")
            return zones
            
        except Exception as e:
            self.logger.system_error("Error parsing zones", {"error": str(e)}, e)
            return []
    
    def _parse_geojson_features(self, features: List[Dict[str, Any]]) -> List[Zone]:
        """
        Parse GeoJSON features thành zones
        Parse GeoJSON features into zones
        
        Args:
            features: Danh sách GeoJSON features
            
        Returns:
            List[Zone]: Danh sách zones
        """
        zones = []
        
        for i, feature in enumerate(features):
            try:
                # Extract properties / Lấy properties
                properties = feature.get("properties", {})
                geometry_data = feature.get("geometry", {})
                
                # Create zone ID nếu chưa có / Create zone ID if not exists
                zone_id = properties.get("id", f"zone_{i}")
                
                # Extract zone info / Lấy thông tin zone
                zone_name = properties.get("name", f"Zone {i}")
                zone_type = properties.get("zone_type", "residential")
                speed_limit = float(properties.get("speed_limit", DEFAULT_SPEED_LIMITS.get(zone_type, 25)))
                is_restricted = bool(properties.get("is_restricted", False))
                is_active = bool(properties.get("is_active", True))
                
                # Parse geometry / Phân tích geometry
                geometry = self._parse_geometry(geometry_data)
                if not geometry:
                    self.logger.system_warning(f"Invalid geometry for zone {zone_id}")
                    continue
                
                # Create zone / Tạo zone
                zone = Zone(
                    id=zone_id,
                    name=zone_name,
                    zone_type=zone_type,
                    speed_limit=speed_limit,
                    is_restricted=is_restricted,
                    is_active=is_active,
                    geometry=geometry,
                    prepared_geometry=None,  # Will be set in __post_init__
                    metadata=properties
                )
                
                zones.append(zone)
                
            except Exception as e:
                self.logger.system_warning(f"Error parsing feature {i}", {"error": str(e)})
                continue
        
        return zones
    
    def _parse_custom_zones(self, zones_data: List[Dict[str, Any]]) -> List[Zone]:
        """
        Parse custom zones format
        Parse định dạng zones tùy chỉnh
        
        Args:
            zones_data: Dữ liệu zones
            
        Returns:
            List[Zone]: Danh sách zones
        """
        zones = []
        
        for i, zone_data in enumerate(zones_data):
            try:
                # Extract zone info / Lấy thông tin zone
                zone_id = zone_data.get("id", f"zone_{i}")
                zone_name = zone_data.get("name", f"Zone {i}")
                zone_type = zone_data.get("zone_type", "residential")
                speed_limit = float(zone_data.get("speed_limit", DEFAULT_SPEED_LIMITS.get(zone_type, 25)))
                is_restricted = bool(zone_data.get("is_restricted", False))
                is_active = bool(zone_data.get("is_active", True))
                
                # Parse geometry / Phân tích geometry
                geometry = None
                
                # Support bounding box format / Hỗ trợ format bounding box
                if "bounds" in zone_data:
                    bounds = zone_data["bounds"]
                    geometry = self._create_polygon_from_bounds(
                        bounds["south"], bounds["west"],
                        bounds["north"], bounds["east"]
                    )
                
                # Support polygon coordinates / Hỗ trợ tọa độ polygon
                elif "coordinates" in zone_data:
                    coords = zone_data["coordinates"]
                    geometry = Polygon(coords)
                
                # Support geometry object / Hỗ trợ object geometry
                elif "geometry" in zone_data:
                    geometry = self._parse_geometry(zone_data["geometry"])
                
                if not geometry:
                    self.logger.system_warning(f"No valid geometry for zone {zone_id}")
                    continue
                
                # Create zone / Tạo zone
                zone = Zone(
                    id=zone_id,
                    name=zone_name,
                    zone_type=zone_type,
                    speed_limit=speed_limit,
                    is_restricted=is_restricted,
                    is_active=is_active,
                    geometry=geometry,
                    prepared_geometry=None,  # Will be set in __post_init__
                    metadata=zone_data
                )
                
                zones.append(zone)
                
            except Exception as e:
                self.logger.system_warning(f"Error parsing zone {i}", {"error": str(e)})
                continue
        
        return zones
    
    def _parse_geometry(self, geometry_data: Dict[str, Any]) -> Optional[Union[Polygon, MultiPolygon]]:
        """
        Parse geometry data thành Shapely object
        Parse geometry data into Shapely object
        
        Args:
            geometry_data: GeoJSON geometry data
            
        Returns:
            Shapely geometry hoặc None
        """
        try:
            geom_type = geometry_data.get("type", "").lower()
            coordinates = geometry_data.get("coordinates", [])
            
            if geom_type == "polygon":
                if coordinates:
                    return Polygon(coordinates[0])  # Exterior ring only for simplicity
            
            elif geom_type == "multipolygon":
                polygons = []
                for poly_coords in coordinates:
                    if poly_coords:
                        polygons.append(Polygon(poly_coords[0]))
                if polygons:
                    return MultiPolygon(polygons)
            
            return None
            
        except Exception as e:
            self.logger.system_debug(f"Geometry parse error: {e}")
            return None
    
    def _create_polygon_from_bounds(self, south: float, west: float, 
                                  north: float, east: float) -> Polygon:
        """
        Tạo polygon từ bounding box
        Create polygon from bounding box
        
        Args:
            south, west, north, east: Tọa độ bounds
            
        Returns:
            Polygon: Shapely polygon
        """
        coordinates = [
            (west, south),   # Bottom-left
            (east, south),   # Bottom-right
            (east, north),   # Top-right
            (west, north),   # Top-left
            (west, south)    # Close polygon
        ]
        return Polygon(coordinates)
    
    def _update_zones(self, zones: List[Zone]):
        """
        Cập nhật internal zones data
        Update internal zones data
        
        Args:
            zones: Danh sách zones mới / New zones list
        """
        # Clear old data / Xóa dữ liệu cũ
        self.zones.clear()
        self.zones_by_type.clear()
        
        # Add new zones / Thêm zones mới
        for zone in zones:
            self.zones[zone.id] = zone
            
            # Group by type / Nhóm theo loại
            if zone.zone_type not in self.zones_by_type:
                self.zones_by_type[zone.zone_type] = []
            self.zones_by_type[zone.zone_type].append(zone)
    
    def _build_spatial_index(self):
        """
        Xây dựng spatial index để tăng tốc queries
        Build spatial index for faster queries
        """
        # Simple list-based index (có thể upgrade thành R-tree sau / Can upgrade to R-tree later)
        self.spatial_index = [zone for zone in self.zones.values() if zone.is_active]
        
        self.logger.system_debug(f"Built spatial index with {len(self.spatial_index)} active zones")
    
    def _create_map_info(self, file_path: str, map_data: Dict[str, Any]):
        """
        Tạo metadata thông tin bản đồ
        Create map metadata information
        
        Args:
            file_path: Đường dẫn file / File path
            map_data: Dữ liệu bản đồ / Map data
        """
        try:
            # Calculate file hash / Tính hash file
            with open(file_path, 'rb') as f:
                file_hash = hashlib.md5(f.read()).hexdigest()
            
            # Calculate bounds / Tính bounds
            bounds = self._calculate_coverage_bounds()
            
            # Get file modification time / Lấy thời gian sửa đổi file
            file_stat = os.stat(file_path)
            last_modified = datetime.fromtimestamp(file_stat.st_mtime)
            
            self.map_info = MapInfo(
                file_path=file_path,
                file_hash=file_hash,
                load_time=datetime.now(),
                version=map_data.get("version", "1.0"),
                total_zones=len(self.zones),
                coverage_bounds=bounds,
                last_modified=last_modified
            )
            
        except Exception as e:
            self.logger.system_error("Error creating map info", {"error": str(e)}, e)
    
    def _calculate_coverage_bounds(self) -> Tuple[float, float, float, float]:
        """
        Tính bounds tổng thể của bản đồ
        Calculate overall bounds of the map
        
        Returns:
            Tuple: (min_lat, min_lon, max_lat, max_lon)
        """
        if not self.zones:
            return (0.0, 0.0, 0.0, 0.0)
        
        min_lat = min_lon = float('inf')
        max_lat = max_lon = float('-inf')
        
        for zone in self.zones.values():
            bounds = zone.geometry.bounds  # (minx, miny, maxx, maxy)
            min_lon = min(min_lon, bounds[0])
            min_lat = min(min_lat, bounds[1])
            max_lon = max(max_lon, bounds[2])
            max_lat = max(max_lat, bounds[3])
        
        return (min_lat, min_lon, max_lat, max_lon)
    
    def _try_load_backup(self) -> bool:
        """
        Thử load backup map
        Try to load backup map
        
        Returns:
            bool: True nếu load backup thành công
        """
        if os.path.exists(self.backup_file_path):
            self.logger.system_warning("Trying to load backup map")
            return self.load_map(self.backup_file_path)
        else:
            self.logger.system_error("No backup map available")
            return False
    
    def _create_backup(self):
        """
        Tạo backup của map hiện tại
        Create backup of current map
        """
        try:
            import shutil
            shutil.copy2(self.map_file_path, self.backup_file_path)
            self.logger.system_debug("Map backup created")
        except Exception as e:
            self.logger.system_warning("Failed to create map backup", {"error": str(e)})
    
    def find_zone_at_position(self, latitude: float, longitude: float) -> Optional[Zone]:
        """
        Tìm zone tại vị trí cho trước
        Find zone at given position
        
        Args:
            latitude: Vĩ độ / Latitude
            longitude: Kinh độ / Longitude
            
        Returns:
            Zone: Zone chứa vị trí hoặc None
        """
        with self._lock:
            self.stats["total_queries"] += 1
            self.stats["last_query_time"] = datetime.now()
            
            # Kiểm tra auto-reload / Check auto-reload
            if MAP_CONFIG["auto_reload"]:
                self._check_file_update()
            
            try:
                point = Point(longitude, latitude)
                
                # Tìm trong spatial index / Search in spatial index
                for zone in self.spatial_index:
                    if zone.prepared_geometry.contains(point):
                        self.stats["zone_hits"] += 1
                        return zone
                
                self.stats["zone_misses"] += 1
                return None
                
            except Exception as e:
                self.logger.system_error("Error in zone lookup", {
                    "latitude": latitude,
                    "longitude": longitude,
                    "error": str(e)
                }, e)
                return None
    
    def get_speed_limit_at_position(self, latitude: float, longitude: float) -> float:
        """
        Lấy giới hạn tốc độ tại vị trí cho trước
        Get speed limit at given position
        
        Args:
            latitude: Vĩ độ / Latitude
            longitude: Kinh độ / Longitude
            
        Returns:
            float: Speed limit (km/h), mặc định 25 km/h nếu không tìm thấy zone
        """
        zone = self.find_zone_at_position(latitude, longitude)
        if zone:
            return zone.speed_limit
        
        # Default speed limit / Giới hạn tốc độ mặc định
        return DEFAULT_SPEED_LIMITS["residential"]
    
    def is_position_restricted(self, latitude: float, longitude: float) -> bool:
        """
        Kiểm tra vị trí có bị cấm không
        Check if position is in restricted area
        
        Args:
            latitude: Vĩ độ / Latitude
            longitude: Kinh độ / Longitude
            
        Returns:
            bool: True nếu vị trí bị cấm
        """
        zone = self.find_zone_at_position(latitude, longitude)
        return zone.is_restricted if zone else False
    
    def get_zones_by_type(self, zone_type: str) -> List[Zone]:
        """
        Lấy tất cả zones theo loại
        Get all zones by type
        
        Args:
            zone_type: Loại zone (residential, school_zone, etc.)
            
        Returns:
            List[Zone]: Danh sách zones
        """
        with self._lock:
            return self.zones_by_type.get(zone_type, []).copy()
    
    def get_all_zones(self) -> List[Zone]:
        """
        Lấy tất cả zones
        Get all zones
        
        Returns:
            List[Zone]: Danh sách tất cả zones
        """
        with self._lock:
            return list(self.zones.values())
    
    def get_zone_by_id(self, zone_id: str) -> Optional[Zone]:
        """
        Lấy zone theo ID
        Get zone by ID
        
        Args:
            zone_id: ID của zone
            
        Returns:
            Zone: Zone object hoặc None
        """
        with self._lock:
            return self.zones.get(zone_id)
    
    def _check_file_update(self):
        """
        Kiểm tra file có được update không
        Check if map file has been updated
        """
        current_time = time.time()
        
        # Kiểm tra không quá thường xuyên / Don't check too frequently
        if current_time - self._last_file_check < self._file_check_interval:
            return
        
        self._last_file_check = current_time
        
        try:
            if not os.path.exists(self.map_file_path):
                return
            
            file_stat = os.stat(self.map_file_path)
            file_modified = datetime.fromtimestamp(file_stat.st_mtime)
            
            # So sánh với thời gian load / Compare with load time
            if self.map_info and file_modified > self.map_info.last_modified:
                self.logger.system_info("Map file updated, reloading...")
                self.load_map()
                
        except Exception as e:
            self.logger.system_debug(f"Error checking file update: {e}")
    
    def get_statistics(self) -> Dict[str, Any]:
        """
        Lấy thống kê map handler
        Get map handler statistics
        
        Returns:
            Dict: Statistics data
        """
        with self._lock:
            stats = self.stats.copy()
            
            if self.map_info:
                stats.update({
                    "map_loaded": self.is_loaded,
                    "total_zones": self.map_info.total_zones,
                    "map_version": self.map_info.version,
                    "load_time": self.map_info.load_time.isoformat(),
                    "coverage_bounds": self.map_info.coverage_bounds,
                    "zone_types": list(self.zones_by_type.keys()),
                    "active_zones": len(self.spatial_index)
                })
            
            # Calculate hit rate / Tính tỷ lệ hit
            total_queries = stats["total_queries"]
            if total_queries > 0:
                stats["hit_rate"] = (stats["zone_hits"] / total_queries) * 100
            else:
                stats["hit_rate"] = 0.0
            
            return stats
    
    def reload_map(self) -> bool:
        """
        Reload bản đồ từ file
        Reload map from file
        
        Returns:
            bool: True nếu reload thành công
        """
        self.logger.system_info("Manual map reload requested")
        return self.load_map()
    
    def is_map_loaded(self) -> bool:
        """
        Kiểm tra bản đồ đã được load chưa
        Check if map is loaded
        
        Returns:
            bool: True nếu map đã load
        """
        return self.is_loaded and len(self.zones) > 0

# =============================================================================
# CONVENIENCE FUNCTIONS / HÀM TIỆN ÍCH
# =============================================================================

def create_map_handler(map_file: Optional[str] = None) -> MapHandler:
    """
    Tạo map handler instance
    Create map handler instance
    
    Args:
        map_file: Đường dẫn file map tùy chọn / Optional map file path
        
    Returns:
        MapHandler: Configured map handler
    """
    return MapHandler(map_file)

def create_sample_hcm_map() -> Dict[str, Any]:
    """
    Tạo sample map cho TP.HCM để testing
    Create sample HCM map for testing
    
    Returns:
        Dict: Sample map data
    """
    sample_map = {
        "version": "1.0",
        "name": "Ho Chi Minh City Sample Zones",
        "zones": [
            {
                "id": "school_zone_1",
                "name": "School Zone - Le Van Tam Park",
                "zone_type": "school_zone",
                "speed_limit": 15,
                "is_restricted": False,
                "bounds": {
                    "south": 10.7780,
                    "north": 10.7820,
                    "west": 106.7030,
                    "east": 106.7070
                }
            },
            {
                "id": "hospital_zone_1",
                "name": "Hospital Zone - FV Hospital",
                "zone_type": "hospital_zone",
                "speed_limit": 15,
                "is_restricted": False,
                "bounds": {
                    "south": 10.7820,
                    "north": 10.7860,
                    "west": 106.7070,
                    "east": 106.7110
                }
            },
            {
                "id": "pedestrian_only_1",
                "name": "Nguyen Hue Walking Street",
                "zone_type": "pedestrian_only",
                "speed_limit": 0,
                "is_restricted": True,
                "bounds": {
                    "south": 10.7740,
                    "north": 10.7760,
                    "west": 106.7000,
                    "east": 106.7020
                }
            },
            {
                "id": "residential_1",
                "name": "District 3 - Residential Area",
                "zone_type": "residential",
                "speed_limit": 25,
                "is_restricted": False,
                "bounds": {
                    "south": 10.7860,
                    "north": 10.7900,
                    "west": 106.7110,
                    "east": 106.7150
                }
            }
        ]
    }
    
    return sample_map

def validate_map_file(file_path: str) -> bool:
    """
    Validate map file mà không load vào memory
    Validate map file without loading into memory
    
    Args:
        file_path: Đường dẫn file map / Map file path
        
    Returns:
        bool: True nếu file hợp lệ / True if file is valid
    """
    try:
        handler = MapHandler()
        map_data = handler._read_map_file(file_path)
        if not map_data:
            return False
        
        return handler._validate_map_data(map_data)
        
    except Exception as e:
        logger = get_logger()
        logger.system_error(f"Map validation failed for {file_path}", {"error": str(e)}, e)
        return False

def create_map_from_geojson(geojson_data: Dict[str, Any]) -> Dict[str, Any]:
    """
    Convert GeoJSON data thành format map của Cycle Sentinel
    Convert GeoJSON data to Cycle Sentinel map format
    
    Args:
        geojson_data: GeoJSON FeatureCollection
        
    Returns:
        Dict: Cycle Sentinel map format
    """
    try:
        if geojson_data.get("type") != "FeatureCollection":
            raise ValueError("Input must be a GeoJSON FeatureCollection")
        
        features = geojson_data.get("features", [])
        zones = []
        
        for i, feature in enumerate(features):
            properties = feature.get("properties", {})
            
            # Extract zone properties với defaults / Extract zone properties with defaults
            zone = {
                "id": properties.get("id", f"imported_zone_{i}"),
                "name": properties.get("name", f"Imported Zone {i}"),
                "zone_type": properties.get("zone_type", "residential"),
                "speed_limit": properties.get("speed_limit", 25),
                "is_restricted": properties.get("is_restricted", False),
                "is_active": properties.get("is_active", True),
                "geometry": feature.get("geometry", {})
            }
            
            zones.append(zone)
        
        return {
            "version": "1.0",
            "name": "Imported from GeoJSON",
            "created": datetime.now().isoformat(),
            "features": features  # Keep original GeoJSON format
        }
        
    except Exception as e:
        logger = get_logger()
        logger.system_error("Failed to convert GeoJSON", {"error": str(e)}, e)
        return {}