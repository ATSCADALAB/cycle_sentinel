#!/usr/bin/env python3
"""
GeoJSON to Cycle Sentinel Map Converter
Chuyển đổi dữ liệu GeoJSON từ OpenStreetMap thành format map của Cycle Sentinel

Usage:
    python geojson_converter.py input.json output.json
    
Author: Cycle Sentinel Team
Version: 1.0.0
"""

import json
import sys
import uuid
from datetime import datetime
from typing import Dict, Any, List, Optional, Tuple
from pathlib import Path

class GeoJSONConverter:
    """Converter class để chuyển đổi GeoJSON thành Cycle Sentinel format"""
    
    # Mapping OSM tags thành zone types của Cycle Sentinel
    OSM_TO_ZONE_TYPE = {
        'pedestrian': 'pedestrian_only',
        'residential': 'residential', 
        'commercial': 'commercial',
        'industrial': 'industrial',
        'school': 'school_zone',
        'hospital': 'hospital_zone',
        'park': 'park_zone',
        'primary': 'main_road',
        'secondary': 'secondary_road',
        'tertiary': 'local_road',
        'cycleway': 'bike_path',
        'footway': 'pedestrian_only'
    }
    
    # Speed limits mặc định cho từng loại zone
    DEFAULT_SPEED_LIMITS = {
        'pedestrian_only': 0,
        'school_zone': 15,
        'hospital_zone': 15,
        'residential': 25,
        'commercial': 20,
        'park_zone': 15,
        'bike_path': 20,
        'local_road': 30,
        'secondary_road': 40,
        'main_road': 50,
        'industrial': 30
    }
    
    def __init__(self):
        self.converted_zones = []
        self.stats = {
            'total_features': 0,
            'converted_zones': 0,
            'skipped_features': 0,
            'errors': []
        }
    
    def detect_zone_type(self, properties: Dict[str, Any]) -> str:
        """
        Phát hiện zone type từ OSM properties
        
        Args:
            properties: Properties từ GeoJSON feature
            
        Returns:
            str: Zone type cho Cycle Sentinel
        """
        # Ưu tiên các tag quan trọng
        highway = properties.get('highway', '').lower()
        if highway in self.OSM_TO_ZONE_TYPE:
            return self.OSM_TO_ZONE_TYPE[highway]
        
        # Kiểm tra area/building types
        amenity = properties.get('amenity', '').lower()
        if 'school' in amenity or 'education' in amenity:
            return 'school_zone'
        elif 'hospital' in amenity or 'clinic' in amenity:
            return 'hospital_zone'
        elif 'park' in amenity or 'garden' in amenity:
            return 'park_zone'
        
        # Kiểm tra landuse
        landuse = properties.get('landuse', '').lower()
        if landuse == 'residential':
            return 'residential'
        elif landuse == 'commercial':
            return 'commercial'
        elif landuse == 'industrial':
            return 'industrial'
        
        # Kiểm tra bicycle tags
        bicycle = properties.get('bicycle', '').lower()
        if bicycle == 'designated':
            return 'bike_path'
        elif bicycle == 'no':
            return 'pedestrian_only'
        
        # Default fallback
        return 'residential'
    
    def extract_speed_limit(self, properties: Dict[str, Any], zone_type: str) -> int:
        """
        Trích xuất speed limit từ properties hoặc dùng default
        
        Args:
            properties: Properties từ GeoJSON
            zone_type: Loại zone đã detect
            
        Returns:
            int: Speed limit in km/h
        """
        # Tìm speed limit trong properties
        for key in ['maxspeed', 'speed_limit', 'max_speed']:
            if key in properties:
                speed_str = str(properties[key]).lower()
                # Parse speed (handle "30 km/h", "30", etc.)
                if speed_str.replace(' ', '') in ['0', 'none', 'walk', 'walking']:
                    return 0
                
                # Extract number
                import re
                numbers = re.findall(r'\d+', speed_str)
                if numbers:
                    speed = int(numbers[0])
                    # Convert mph to km/h if needed
                    if 'mph' in speed_str:
                        speed = int(speed * 1.60934)
                    return max(0, min(speed, 80))  # Clamp between 0-80 km/h
        
        # Dùng default cho zone type
        return self.DEFAULT_SPEED_LIMITS.get(zone_type, 25)
    
    def is_restricted_zone(self, properties: Dict[str, Any], zone_type: str) -> bool:
        """
        Xác định zone có bị restricted không
        
        Args:
            properties: Properties từ GeoJSON
            zone_type: Zone type
            
        Returns:
            bool: True nếu zone bị restricted
        """
        # Pedestrian only zones luôn restricted
        if zone_type == 'pedestrian_only':
            return True
        
        # Kiểm tra access restrictions
        access = properties.get('access', '').lower()
        bicycle = properties.get('bicycle', '').lower()
        
        if access in ['no', 'private', 'restricted'] or bicycle == 'no':
            return True
        
        return False
    
    def geometry_to_bounds(self, geometry: Dict[str, Any]) -> Optional[Dict[str, float]]:
        """
        Chuyển đổi geometry thành bounding box
        
        Args:
            geometry: GeoJSON geometry
            
        Returns:
            Dict với south, north, west, east bounds hoặc None
        """
        try:
            coords = []
            geom_type = geometry.get('type', '')
            
            if geom_type == 'Polygon':
                # Polygon có array of arrays (exterior + holes)
                coords = geometry['coordinates'][0]  # Chỉ lấy exterior ring
            elif geom_type == 'MultiPolygon':
                # MultiPolygon có array of polygons
                for polygon in geometry['coordinates']:
                    coords.extend(polygon[0])  # Exterior ring của mỗi polygon
            elif geom_type == 'Point':
                coords = [geometry['coordinates']]
            elif geom_type == 'LineString':
                coords = geometry['coordinates']
            elif geom_type == 'MultiLineString':
                for line in geometry['coordinates']:
                    coords.extend(line)
            else:
                print(f"Warning: Unsupported geometry type: {geom_type}")
                return None
            
            if not coords:
                return None
            
            # Tính bounding box
            lons = [coord[0] for coord in coords]
            lats = [coord[1] for coord in coords]
            
            return {
                'west': min(lons),
                'east': max(lons),
                'south': min(lats),
                'north': max(lats)
            }
            
        except Exception as e:
            print(f"Error processing geometry: {e}")
            return None
    
    def convert_feature(self, feature: Dict[str, Any], index: int) -> Optional[Dict[str, Any]]:
        """
        Chuyển đổi một GeoJSON feature thành Cycle Sentinel zone
        
        Args:
            feature: GeoJSON feature
            index: Index của feature
            
        Returns:
            Dict: Cycle Sentinel zone hoặc None nếu lỗi
        """
        try:
            properties = feature.get('properties', {})
            geometry = feature.get('geometry', {})
            
            # Detect zone info
            zone_type = self.detect_zone_type(properties)
            speed_limit = self.extract_speed_limit(properties, zone_type)
            is_restricted = self.is_restricted_zone(properties, zone_type)
            
            # Convert geometry to bounds
            bounds = self.geometry_to_bounds(geometry)
            if not bounds:
                self.stats['errors'].append(f"Feature {index}: Could not extract bounds")
                return None
            
            # Create zone name
            name = properties.get('name', f'Imported Zone {index + 1}')
            if not name or name.strip() == '':
                name = f'Zone {index + 1} ({zone_type})'
            
            # Create zone ID
            zone_id = properties.get('@id', properties.get('id', f'imported_zone_{index}'))
            if isinstance(zone_id, str) and zone_id.startswith('way/'):
                zone_id = zone_id.replace('way/', 'osm_way_')
            elif isinstance(zone_id, str) and zone_id.startswith('relation/'):
                zone_id = zone_id.replace('relation/', 'osm_rel_')
            else:
                zone_id = f'imported_zone_{index}'
            
            # Tạo zone object
            zone = {
                'id': str(zone_id),
                'name': name,
                'zone_type': zone_type,
                'speed_limit': speed_limit,
                'is_restricted': is_restricted,
                'is_active': True,
                'bounds': bounds,
                'metadata': {
                    'source': 'openstreetmap',
                    'original_properties': properties,
                    'imported_at': datetime.now().isoformat()
                }
            }
            
            return zone
            
        except Exception as e:
            error_msg = f"Feature {index}: {str(e)}"
            self.stats['errors'].append(error_msg)
            print(f"Error converting feature {index}: {e}")
            return None
    
    def convert_geojson(self, geojson_data: Dict[str, Any]) -> Dict[str, Any]:
        """
        Chuyển đổi toàn bộ GeoJSON thành Cycle Sentinel format
        
        Args:
            geojson_data: GeoJSON FeatureCollection
            
        Returns:
            Dict: Cycle Sentinel map format
        """
        # Validate input
        if geojson_data.get('type') != 'FeatureCollection':
            raise ValueError("Input must be a GeoJSON FeatureCollection")
        
        features = geojson_data.get('features', [])
        self.stats['total_features'] = len(features)
        
        print(f"Converting {len(features)} features...")
        
        # Convert each feature
        converted_zones = []
        for i, feature in enumerate(features):
            if i % 100 == 0 and i > 0:
                print(f"Processed {i}/{len(features)} features...")
            
            zone = self.convert_feature(feature, i)
            if zone:
                converted_zones.append(zone)
                self.stats['converted_zones'] += 1
            else:
                self.stats['skipped_features'] += 1
        
        # Create Cycle Sentinel map structure
        cycle_sentinel_map = {
            'version': '1.0',
            'name': 'Imported from OpenStreetMap',
            'created': datetime.now().isoformat(),
            'metadata': {
                'source': 'openstreetmap_geojson',
                'generator': geojson_data.get('generator', 'unknown'),
                'copyright': geojson_data.get('copyright', ''),
                'timestamp': geojson_data.get('timestamp', ''),
                'conversion_stats': self.stats
            },
            'zones': converted_zones
        }
        
        return cycle_sentinel_map
    
    def print_stats(self):
        """In thống kê conversion"""
        print("\n" + "="*50)
        print("CONVERSION STATISTICS")
        print("="*50)
        print(f"Total features processed: {self.stats['total_features']}")
        print(f"Successfully converted: {self.stats['converted_zones']}")
        print(f"Skipped features: {self.stats['skipped_features']}")
        print(f"Conversion rate: {self.stats['converted_zones']/max(1,self.stats['total_features'])*100:.1f}%")
        
        if self.stats['errors']:
            print(f"\nErrors encountered: {len(self.stats['errors'])}")
            for error in self.stats['errors'][:5]:  # Show first 5 errors
                print(f"  - {error}")
            if len(self.stats['errors']) > 5:
                print(f"  ... and {len(self.stats['errors'])-5} more errors")

def main():
    """Main function"""
    if len(sys.argv) != 3:
        print("Usage: python geojson_converter.py <input_geojson> <output_json>")
        print("\nExample:")
        print("  python geojson_converter.py queens_square.json cycle_sentinel_map.json")
        sys.exit(1)
    
    input_file = Path(sys.argv[1])
    output_file = Path(sys.argv[2])
    
    # Validate input file
    if not input_file.exists():
        print(f"Error: Input file '{input_file}' not found")
        sys.exit(1)
    
    try:
        # Load GeoJSON
        print(f"Loading GeoJSON from: {input_file}")
        with open(input_file, 'r', encoding='utf-8') as f:
            geojson_data = json.load(f)
        
        # Convert
        converter = GeoJSONConverter()
        cycle_sentinel_map = converter.convert_geojson(geojson_data)
        
        # Save output
        print(f"Saving Cycle Sentinel map to: {output_file}")
        with open(output_file, 'w', encoding='utf-8') as f:
            json.dump(cycle_sentinel_map, f, indent=2, ensure_ascii=False)
        
        # Print stats
        converter.print_stats()
        print(f"\n✅ Conversion completed successfully!")
        print(f"📄 Output saved to: {output_file}")
        
    except Exception as e:
        print(f"❌ Error during conversion: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()