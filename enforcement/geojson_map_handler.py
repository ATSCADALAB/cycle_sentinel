"""
Cycle Sentinel GeoJSON Map Handler
Bộ xử lý bản đồ GeoJSON cho Cycle Sentinel

This module handles OpenStreetMap GeoJSON data for cycling infrastructure.
Module này xử lý dữ liệu GeoJSON OpenStreetMap cho cơ sở hạ tầng xe đạp.

Features / Tính năng:
- GeoJSON parsing with cycling-specific attributes
- Speed limit extraction from OSM data
- Bicycle infrastructure detection
- Point-to-line distance calculations
- Dynamic speed limit lookup

Author: Cycle Sentinel Team
Version: 1.0.0
"""

import json
import math
from typing import Dict, Any, List, Tuple, Optional, Union
from dataclasses import dataclass
from pathlib import Path
import threading
from shapely.geometry import Point, LineString, MultiLineString
from shapely.ops import nearest_points
import geojson

from config.settings import MAP_CONFIG, MAPS_DIR, DEFAULT_SPEED_LIMITS
from utils.logger import get_logger

@dataclass
class CyclingWay:
    """
    Thông tin về một đường xe đạp từ OSM
    Information about a cycling way from OSM
    """
    id: str
    name: str
    highway_type: str  # cycleway, residential, trunk, etc.
    bicycle: str  # yes, no, designated, etc.
    maxspeed: Optional[int]  # km/h from OSM data
    maxspeed_type: Optional[str]  # sign, legal, etc.
    surface: Optional[str]  # asphalt, concrete, etc.
    oneway: Optional[str]  # yes, no, -1
    geometry: LineString
    properties: Dict[str, Any]  # All OSM properties

class GeoJSONMapHandler:
    """
    Xử lý bản đồ GeoJSON cho hệ thống Cycle Sentinel
    GeoJSON map handler for Cycle Sentinel system
    """
    
    def __init__(self, map_file: Optional[str] = None):
        """
        Khởi tạo GeoJSON map handler
        Initialize GeoJSON map handler
        """
        self.logger = get_logger()
        self.map_file = map_file or MAP_CONFIG["map_file"]
        self.cycling_ways: List[CyclingWay] = []
        self._lock = threading.RLock()
        self._last_loaded = None
        
        # Default speed limits for different highway types
        self.default_speeds = {
            'cycleway': 25,  # Dedicated cycle paths
            'residential': 30,  # Residential roads
            'secondary': 50,  # Secondary roads
            'trunk': 60,  # Major roads
            'primary': 60,  # Primary roads
            'tertiary': 40,  # Tertiary roads
            'unclassified': 30,  # Unclassified roads
            'service': 20,  # Service roads
            'footway': 15,  # Footpaths (walking speed)
            'path': 20,  # Mixed use paths
            'track': 25,  # Tracks
            'bridleway': 20,  # Bridleways
            'pedestrian': 15  # Pedestrian areas
        }
        
        # Load map on initialization
        self.load_map()
    
    def load_map(self) -> bool:
        """
        Load GeoJSON map file
        
        Returns:
            bool: True if successful
        """
        try:
            with self._lock:
                if not Path(self.map_file).exists():
                    self.logger.system_error(f"Map file not found: {self.map_file}")
                    return False
                
                with open(self.map_file, 'r', encoding='utf-8') as f:
                    geojson_data = json.load(f)
                
                if not self._validate_geojson(geojson_data):
                    return False
                
                self.cycling_ways = self._parse_geojson(geojson_data)
                self._last_loaded = Path(self.map_file).stat().st_mtime
                
                self.logger.system_info(
                    f"Successfully loaded {len(self.cycling_ways)} cycling ways from {self.map_file}"
                )
                return True
                
        except Exception as e:
            self.logger.system_error(f"Failed to load map: {e}", error=e)
            return False
    
    def _validate_geojson(self, data: Dict[str, Any]) -> bool:
        """
        Validate GeoJSON structure
        """
        if data.get("type") != "FeatureCollection":
            self.logger.system_error("Invalid GeoJSON: must be FeatureCollection")
            return False
        
        features = data.get("features", [])
        if not features:
            self.logger.system_warning("GeoJSON contains no features")
            return False
        
        return True
    
    def _parse_geojson(self, geojson_data: Dict[str, Any]) -> List[CyclingWay]:
        """
        Parse GeoJSON features into CyclingWay objects
        """
        ways = []
        
        for feature in geojson_data.get("features", []):
            try:
                way = self._parse_feature(feature)
                if way and self._is_cycling_relevant(way):
                    ways.append(way)
                    
            except Exception as e:
                self.logger.system_warning(f"Failed to parse feature: {e}")
                continue
        
        return ways
    
    def _parse_feature(self, feature: Dict[str, Any]) -> Optional[CyclingWay]:
        """
        Parse single GeoJSON feature into CyclingWay
        """
        properties = feature.get("properties", {})
        geometry = feature.get("geometry", {})
        
        # Extract basic info
        way_id = str(properties.get("@id", properties.get("id", f"unknown_{hash(str(feature))}")))
        name = properties.get("name", "Unnamed")
        highway_type = properties.get("highway", "unknown")
        
        # Extract cycling-specific properties
        bicycle = properties.get("bicycle", "unknown")
        
        # Extract speed limit - key change here!
        maxspeed = self._extract_speed_limit(properties)
        maxspeed_type = properties.get("maxspeed:type")
        
        # Extract other properties
        surface = properties.get("surface")
        oneway = properties.get("oneway")
        
        # Parse geometry
        try:
            if geometry.get("type") == "LineString":
                coords = geometry.get("coordinates", [])
                # Convert to (lat, lon) format
                shapely_coords = [(lon, lat) for lon, lat in coords]
                line_geom = LineString(shapely_coords)
                
            else:
                self.logger.system_warning(f"Unsupported geometry type: {geometry.get('type')}")
                return None
                
        except Exception as e:
            self.logger.system_warning(f"Failed to parse geometry: {e}")
            return None
        
        return CyclingWay(
            id=way_id,
            name=name,
            highway_type=highway_type,
            bicycle=bicycle,
            maxspeed=maxspeed,
            maxspeed_type=maxspeed_type,
            surface=surface,
            oneway=oneway,
            geometry=line_geom,
            properties=properties
        )
    
    def _extract_speed_limit(self, properties: Dict[str, Any]) -> Optional[int]:
        """
        Extract speed limit from OSM properties
        
        This is the key function for getting speed from JSON instead of config!
        """
        # Try different maxspeed formats from OSM
        maxspeed_raw = properties.get("maxspeed")
        
        if maxspeed_raw:
            # Handle different formats: "50", "50 mph", "50 km/h", etc.
            if isinstance(maxspeed_raw, (int, float)):
                return int(maxspeed_raw)
            
            if isinstance(maxspeed_raw, str):
                # Remove units and convert
                speed_str = maxspeed_raw.lower().replace(" km/h", "").replace(" mph", "").replace("km/h", "").replace("mph", "").strip()
                
                try:
                    speed = int(speed_str)
                    # Convert mph to km/h if needed
                    if "mph" in maxspeed_raw.lower():
                        speed = int(speed * 1.60934)
                    return speed
                    
                except ValueError:
                    # Handle special values like "none", "signals", etc.
                    if speed_str in ["none"]:
                        return None
                    # For other special values, fall back to highway type
        
        # If no explicit speed limit, use highway type default
        highway_type = properties.get("highway", "unknown")
        return self.default_speeds.get(highway_type, 25)  # 25 km/h default for e-bikes
    
    def _is_cycling_relevant(self, way: CyclingWay) -> bool:
        """
        Check if this way is relevant for cycling
        """
        # Include if explicitly allows bicycles
        if way.bicycle in ["yes", "designated", "permissive"]:
            return True
        
        # Include dedicated cycling infrastructure
        if way.highway_type in ["cycleway", "path"]:
            return True
        
        # Include roads where cycling is typically allowed
        if way.highway_type in ["residential", "secondary", "tertiary", "unclassified", "service"]:
            # Exclude if explicitly forbidden
            if way.bicycle == "no":
                return False
            return True
        
        # Include footways that allow cycling
        if way.highway_type == "footway" and way.bicycle in ["yes", "designated"]:
            return True
        
        return False
    
    def get_speed_limit_at_point(self, latitude: float, longitude: float, buffer_meters: float = 20.0) -> int:
        """
        Get speed limit at specific GPS coordinates
        
        Args:
            latitude: GPS latitude
            longitude: GPS longitude  
            buffer_meters: Search radius in meters
            
        Returns:
            int: Speed limit in km/h
        """
        try:
            with self._lock:
                point = Point(longitude, latitude)  # Note: shapely uses (lon, lat)
                
                closest_way = None
                min_distance = float('inf')
                
                # Find closest cycling way
                for way in self.cycling_ways:
                    try:
                        distance = point.distance(way.geometry)
                        # Convert degrees to meters (approximate)
                        distance_meters = distance * 111320  # rough conversion
                        
                        if distance_meters <= buffer_meters and distance_meters < min_distance:
                            min_distance = distance_meters
                            closest_way = way
                            
                    except Exception as e:
                        continue
                
                if closest_way and closest_way.maxspeed:
                    self.logger.system_debug(
                        f"Found speed limit {closest_way.maxspeed} km/h on {closest_way.name} "
                        f"({closest_way.highway_type}) at distance {min_distance:.1f}m"
                    )
                    return closest_way.maxspeed
                
                # Fallback to default e-bike speed limit
                default_speed = 25
                self.logger.system_debug(f"No speed limit found, using default: {default_speed} km/h")
                return default_speed
                
        except Exception as e:
            self.logger.system_error(f"Error getting speed limit: {e}", error=e)
            return 25  # Safe default
    
    def get_way_info_at_point(self, latitude: float, longitude: float, buffer_meters: float = 20.0) -> Optional[Dict[str, Any]]:
        """
        Get detailed way information at point
        
        Returns:
            Dict with way information or None
        """
        try:
            with self._lock:
                point = Point(longitude, latitude)
                
                closest_way = None
                min_distance = float('inf')
                
                for way in self.cycling_ways:
                    try:
                        distance = point.distance(way.geometry)
                        distance_meters = distance * 111320
                        
                        if distance_meters <= buffer_meters and distance_meters < min_distance:
                            min_distance = distance_meters
                            closest_way = way
                            
                    except Exception:
                        continue
                
                if closest_way:
                    return {
                        "id": closest_way.id,
                        "name": closest_way.name,
                        "highway_type": closest_way.highway_type,
                        "bicycle": closest_way.bicycle,
                        "maxspeed": closest_way.maxspeed,
                        "surface": closest_way.surface,
                        "oneway": closest_way.oneway,
                        "distance_meters": min_distance,
                        "properties": closest_way.properties
                    }
                
                return None
                
        except Exception as e:
            self.logger.system_error(f"Error getting way info: {e}", error=e)
            return None
    
    def is_cycling_allowed(self, latitude: float, longitude: float, buffer_meters: float = 20.0) -> bool:
        """
        Check if cycling is allowed at this location
        """
        way_info = self.get_way_info_at_point(latitude, longitude, buffer_meters)
        
        if not way_info:
            return True  # Assume allowed if no data
        
        bicycle = way_info.get("bicycle", "unknown")
        highway_type = way_info.get("highway_type", "unknown")
        
        # Explicitly forbidden
        if bicycle == "no":
            return False
        
        # Explicitly allowed
        if bicycle in ["yes", "designated", "permissive"]:
            return True
        
        # Check highway type
        allowed_types = [
            "cycleway", "path", "residential", "secondary", 
            "tertiary", "unclassified", "service", "track"
        ]
        
        return highway_type in allowed_types
    
    def reload_if_changed(self) -> bool:
        """
        Reload map if file has changed
        """
        try:
            if not Path(self.map_file).exists():
                return False
            
            current_mtime = Path(self.map_file).stat().st_mtime
            
            if self._last_loaded is None or current_mtime > self._last_loaded:
                self.logger.system_info("Map file changed, reloading...")
                return self.load_map()
            
            return True
            
        except Exception as e:
            self.logger.system_error(f"Error checking map file: {e}", error=e)
            return False
    
    def get_statistics(self) -> Dict[str, Any]:
        """
        Get map statistics for debugging
        """
        with self._lock:
            highway_types = {}
            bicycle_types = {}
            speed_limits = {}
            
            for way in self.cycling_ways:
                # Count highway types
                highway_types[way.highway_type] = highway_types.get(way.highway_type, 0) + 1
                
                # Count bicycle access
                bicycle_types[way.bicycle] = bicycle_types.get(way.bicycle, 0) + 1
                
                # Count speed limits
                if way.maxspeed:
                    speed_limits[way.maxspeed] = speed_limits.get(way.maxspeed, 0) + 1
            
            return {
                "total_ways": len(self.cycling_ways),
                "highway_types": highway_types,
                "bicycle_access": bicycle_types,
                "speed_limits": speed_limits,
                "map_file": self.map_file,
                "last_loaded": self._last_loaded
            }


# Global instance
_map_handler_instance = None
_map_handler_lock = threading.Lock()

def get_geojson_map_handler() -> GeoJSONMapHandler:
    """
    Get singleton GeoJSON map handler instance
    """
    global _map_handler_instance
    
    with _map_handler_lock:
        if _map_handler_instance is None:
            _map_handler_instance = GeoJSONMapHandler()
        
        return _map_handler_instance

def validate_geojson_map_file(file_path: str) -> bool:
    """
    Validate a GeoJSON map file
    """
    try:
        handler = GeoJSONMapHandler(file_path)
        return len(handler.cycling_ways) > 0
        
    except Exception as e:
        logger = get_logger()
        logger.system_error(f"GeoJSON validation failed for {file_path}", error=e)
        return False