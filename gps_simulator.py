#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Cycle Sentinel GPS Simulator - Giả lập Map Signals và Violation Detection
Dựa trên source code thực của hệ thống Cycle Sentinel
"""

import time
import math
import random
import json
from datetime import datetime, timedelta
from typing import Dict, Any, List, Tuple, Optional
from dataclasses import dataclass
from enum import Enum

class SimulationMode(Enum):
    """Chế độ mô phỏng khác nhau"""
    STATIC = "static"
    NORMAL_RIDE = "normal_ride"
    SPEED_VIOLATION = "speed_violation"
    ZONE_VIOLATION = "zone_violation"
    MIXED_SCENARIO = "mixed_scenario"

@dataclass
class Waypoint:
    """Điểm waypoint trong tuyến đường"""
    latitude: float
    longitude: float
    speed_limit: float  # km/h
    zone_name: str
    is_restricted: bool = False

class GPSSimulator:
    """Bộ mô phỏng GPS cho Cycle Sentinel"""
    
    def __init__(self):
        # Starting position in Melbourne CBD (from original source code)
        self.current_position = [-37.8136, 144.9631]
        self.current_speed = 0.0
        self.current_course = 0.0
        self.start_time = None
        self.is_running = False
        
        # Setup waypoints for Melbourne route from source code
        self.setup_melbourne_route()
        self.current_waypoint_index = 0
        
    def setup_melbourne_route(self):
        """Setup Melbourne CBD route with Australian locations"""
        self.waypoints = [
            Waypoint(-37.8136, 144.9631, 25.0, "Flinders Street Station", False),
            Waypoint(-37.8152, 144.9641, 15.0, "School Zone - Collins Street", False),
            Waypoint(-37.8166, 144.9651, 0.0, "Pedestrian Mall - Bourke Street", True),  # RESTRICTED
            Waypoint(-37.8180, 144.9661, 15.0, "Hospital Zone - Melbourne Central", False),
            Waypoint(-37.8194, 144.9671, 40.0, "Main Road - Elizabeth Street", False)
        ]
        
    def start_simulation(self, mode: SimulationMode):
        """Start simulation"""
        self.start_time = datetime.now()
        self.is_running = True
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        print(f"[{timestamp}] SIMULATION_STARTED MODE:{mode.value.upper()}")
        
    def stop_simulation(self):
        """Stop simulation"""
        self.is_running = False
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        print(f"[{timestamp}] SIMULATION_STOPPED DURATION:{self.get_elapsed_time():.1f}s")
        
    def get_elapsed_time(self) -> float:
        """Lấy thời gian đã trôi qua"""
        if not self.start_time:
            return 0.0
        return (datetime.now() - self.start_time).total_seconds()
        
    def calculate_distance(self, lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        """Tính khoảng cách giữa 2 điểm (từ source code gốc)"""
        R = 6371000  # Earth radius in meters
        lat1_rad = math.radians(lat1)
        lat2_rad = math.radians(lat2)
        delta_lat = math.radians(lat2 - lat1)
        delta_lon = math.radians(lon2 - lon1)
        
        a = (math.sin(delta_lat / 2) ** 2 +
             math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(delta_lon / 2) ** 2)
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        
        return R * c
        
    def update_position_along_route(self, elapsed_time: float, mode: SimulationMode):
        """Cập nhật vị trí theo tuyến đường"""
        if not self.waypoints:
            return
            
        # Thay đổi waypoint mỗi 15 giây (từ source code)
        target_index = min(int(elapsed_time / 15), len(self.waypoints) - 1)
        self.current_waypoint_index = target_index
        
        current_waypoint = self.waypoints[target_index]
        
        # Lerp position đến waypoint target
        progress = (elapsed_time % 15) / 15.0
        if target_index < len(self.waypoints) - 1:
            next_waypoint = self.waypoints[target_index + 1]
            self.current_position[0] = current_waypoint.latitude + (next_waypoint.latitude - current_waypoint.latitude) * progress
            self.current_position[1] = current_waypoint.longitude + (next_waypoint.longitude - current_waypoint.longitude) * progress
        else:
            self.current_position = [current_waypoint.latitude, current_waypoint.longitude]
            
        # Update speed based on mode
        if mode == SimulationMode.NORMAL_RIDE:
            self.current_speed = current_waypoint.speed_limit * 0.8  # Ride slower than limit
        elif mode == SimulationMode.SPEED_VIOLATION:
            if elapsed_time > 10:  # Start violation after 10s
                excess = min(20, (elapsed_time - 10) * 1.5)  # Gradual increase
                self.current_speed = current_waypoint.speed_limit + excess
            else:
                self.current_speed = current_waypoint.speed_limit * 0.8
        elif mode == SimulationMode.ZONE_VIOLATION:
            if elapsed_time > 30:  # Enter restricted zone after 30s
                self.current_speed = 12.0
            else:
                self.current_speed = current_waypoint.speed_limit * 0.8
        # Update speed progression for mixed scenario
        elif mode == SimulationMode.MIXED_SCENARIO:
            if elapsed_time < 20:
                # Phase 1: Normal riding
                self.current_speed = current_waypoint.speed_limit * 0.8
            elif elapsed_time < 35:
                # Phase 2: Speed violation (gradual increase)
                excess = min(15, (elapsed_time - 20) * 1.0)
                self.current_speed = current_waypoint.speed_limit + excess
            elif elapsed_time < 50:
                # Phase 3: Enter restricted zone
                if current_waypoint.is_restricted or elapsed_time > 45:
                    self.current_speed = 8.0  # Moving through restricted area
                else:
                    self.current_speed = current_waypoint.speed_limit * 0.9
            else:
                # Phase 4: Return to normal
                self.current_speed = current_waypoint.speed_limit * 0.8
                
    def generate_gps_data(self, mode: SimulationMode) -> Dict[str, Any]:
        """Tạo dữ liệu GPS theo mode (từ source code gốc)"""
        elapsed_time = self.get_elapsed_time()
        
        # Cập nhật vị trí
        self.update_position_along_route(elapsed_time, mode)
        
        # Thêm GPS noise (từ source code)
        noise_level = 0.1
        noise_lat = random.uniform(-0.0001, 0.0001) * noise_level
        noise_lon = random.uniform(-0.0001, 0.0001) * noise_level
        speed_noise = random.uniform(-1, 1) * noise_level
        
        return {
            "timestamp": datetime.now().isoformat(),
            "latitude": self.current_position[0] + noise_lat,
            "longitude": self.current_position[1] + noise_lon,
            "speed_kmh": max(0, self.current_speed + speed_noise),
            "course": self.current_course,
            "altitude": 10.0,
            "accuracy": 5.0,
            "satellites": 8,
            "valid": True
        }
        
    def get_current_zone_info(self) -> Dict[str, Any]:
        """Lấy thông tin zone hiện tại"""
        if self.current_waypoint_index < len(self.waypoints):
            waypoint = self.waypoints[self.current_waypoint_index]
            return {
                "zone_name": waypoint.zone_name,
                "speed_limit": waypoint.speed_limit,
                "is_restricted": waypoint.is_restricted
            }
        return {"zone_name": "Unknown", "speed_limit": 25.0, "is_restricted": False}
        
class ViolationChecker:
    """Kiểm tra vi phạm (từ source code gốc)"""
    
    def __init__(self):
        self.violation_count = 0
        
    def check_violations(self, gps_data: Dict[str, Any], zone_info: Dict[str, Any]) -> List[Dict[str, Any]]:
        """Kiểm tra vi phạm"""
        violations = []
        current_speed = gps_data["speed_kmh"]
        speed_limit = zone_info["speed_limit"]
        
        # Vi phạm tốc độ
        if current_speed > speed_limit and speed_limit > 0:
            excess_speed = current_speed - speed_limit
            severity = "CRITICAL" if excess_speed > 15 else "HIGH" if excess_speed > 10 else "MEDIUM"
            
            violations.append({
                "type": "speed_violation",
                "severity": severity,
                "current_speed": current_speed,
                "speed_limit": speed_limit,
                "excess_speed": excess_speed,
                "zone": zone_info["zone_name"]
            })
            
        # Restricted zone violation
        if zone_info["is_restricted"] and current_speed > 0:
            violations.append({
                "type": "restricted_zone",
                "severity": "CRITICAL",
                "current_speed": current_speed,
                "zone": zone_info["zone_name"],
                "message": "Entering restricted pedestrian area"
            })
            
        return violations

def run_simulation_demo():
    """Run simulation demo"""
    simulator = GPSSimulator()
    violation_checker = ViolationChecker()
    
    print("CYCLE SENTINEL GPS SIMULATOR - MELBOURNE CBD")
    print("=" * 60)
    print("1. Normal riding")
    print("2. Speed violation (gradual increase)")  
    print("3. Restricted zone violation")
    print("4. Mixed scenario")
    print("0. Exit")
    
    while True:
        try:
            choice = input("\nSelect simulation mode (0-4): ").strip()
            
            if choice == "0":
                print("SYSTEM_EXIT | GOODBYE")
                break
                
            # Choose mode
            mode_map = {
                "1": SimulationMode.NORMAL_RIDE,
                "2": SimulationMode.SPEED_VIOLATION,
                "3": SimulationMode.ZONE_VIOLATION,
                "4": SimulationMode.MIXED_SCENARIO
            }
            
            if choice not in mode_map:
                print("ERROR | INVALID_SELECTION")
                continue
                
            mode = mode_map[choice]
            duration = 60  # 60 seconds for mixed scenario
            
            # Start simulation
            simulator.start_simulation(mode)
            
            try:
                while simulator.is_running and simulator.get_elapsed_time() < duration:
                    # Generate GPS data
                    gps_data = simulator.generate_gps_data(mode)
                    zone_info = simulator.get_current_zone_info()
                    
                    # Check violations
                    violations = violation_checker.check_violations(gps_data, zone_info)
                    
                    # Display console output
                    display_check_result(gps_data, zone_info, violations)
                    
                    time.sleep(4)  # Update every second
                    
            except KeyboardInterrupt:
                timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                print(f"\n[{timestamp}] USER_INTERRUPT | STOPPING_SIMULATION")
                
            simulator.stop_simulation()
            print()
            
        except Exception as e:
            print(f"SYSTEM_ERROR | {e}")

def display_check_result(gps_data: Dict[str, Any], zone_info: Dict[str, Any], violations: List[Dict[str, Any]]):
    """Display check results in single line professional format"""
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    lat = gps_data["latitude"]
    lon = gps_data["longitude"]
    speed = gps_data["speed_kmh"]
    
    # Build violation status
    if not violations:
        violation_status = "NO_VIOLATIONS"
    else:
        violation_details = []
        for violation in violations:
            if violation["type"] == "speed_violation":
                violation_details.append(f"SPEED_EXCEEDED:{violation['excess_speed']:.1f}kmh")
            elif violation["type"] == "restricted_zone":
                violation_details.append(f"RESTRICTED_ZONE_ENTRY")
        violation_status = "|".join(violation_details)
    
    # Single line format
    print(f"[{timestamp}] LAT:{lat:.6f} LON:{lon:.6f} SPEED:{speed:.1f}kmh ZONE:{zone_info['zone_name']} LIMIT:{zone_info['speed_limit']}kmh STATUS:{violation_status}")

if __name__ == "__main__":
    run_simulation_demo()