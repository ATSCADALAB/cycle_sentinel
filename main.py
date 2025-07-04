#!/usr/bin/env python3
"""
Cycle Sentinel Main Application - Fixed Version
Ứng dụng chính của Cycle Sentinel - Phiên bản đã fix

This is the main entry point for the Cycle Sentinel e-bike monitoring system.
Đây là điểm khởi đầu chính cho hệ thống giám sát xe đạp điện Cycle Sentinel.

Features / Tính năng:
- GPS monitoring and violation detection / Giám sát GPS và phát hiện vi phạm
- Real-time speed and zone enforcement / Thực thi tốc độ và zone theo thời gian thực
- Automatic logging and reporting / Tự động ghi log và báo cáo
- System health monitoring / Giám sát sức khỏe hệ thống
- Graceful shutdown handling / Xử lý tắt hệ thống một cách nhẹ nhàng

Usage / Cách sử dụng:
    python main_fixed.py [options]
    
Options / Tùy chọn:
    --simulate    Use GPS simulator instead of real GPS / Sử dụng GPS simulator thay vì GPS thật
    --config      Specify config file / Chỉ định file cấu hình
    --debug       Enable debug mode / Bật chế độ debug
    --test        Run in test mode / Chạy ở chế độ test

Author: Cycle Sentinel Team
Version: 1.0.0
"""

import sys
import os
import argparse
import signal
import time
import threading
from datetime import datetime, timedelta
from typing import Optional, Dict, Any
import json

# Add project root to path / Thêm project root vào path
project_root = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, project_root)

# Import các modules của hệ thống / Import system modules
try:
    from config.settings import (
        SYSTEM_CONFIG, 
        GPS_CONFIG, 
        MAP_CONFIG, 
        VIOLATION_CONFIG,
        validate_config,
        get_config_summary,
        MAPS_DIR,
        LOGS_DIR
    )
    from utils.logger import (
        get_logger, 
        log_system_startup, 
        log_system_shutdown,
        PerformanceTimer
    )
    from sensors.gps_handler import GPSHandler, create_gps_handler
    from sensors.gps_simulator import GPSSimulator, create_mixed_scenario, SimulationConfig, SimulationMode
    from enforcement.map_handler import MapHandler, create_map_handler
    from enforcement.violation_checker import ViolationChecker, create_violation_checker
    
except ImportError as e:
    print(f"❌ Failed to import required modules: {e}")
    print("Please ensure all dependencies are installed and paths are correct.")
    sys.exit(1)

class CycleSentinelApp:
    """
    Ứng dụng chính Cycle Sentinel - Fixed Version
    Main Cycle Sentinel application - Fixed Version
    """
    
    def __init__(self, use_simulator: bool = False, config_file: Optional[str] = None):
        """
        Khởi tạo ứng dụng Cycle Sentinel
        Initialize Cycle Sentinel application
        """
        # Tạo thư mục cần thiết trước / Create necessary directories first
        MAPS_DIR.mkdir(exist_ok=True)
        LOGS_DIR.mkdir(exist_ok=True)
        
        # Khởi tạo logger NGAY / Initialize logger IMMEDIATELY
        self.logger = get_logger()
        self.use_simulator = use_simulator
        self.config_file = config_file
        
        # System state / Trạng thái hệ thống
        self.is_running = False
        self.shutdown_requested = False
        self.startup_time = datetime.now()
        
        # Core components / Các thành phần cốt lõi
        self.gps_handler: Optional[GPSHandler] = None
        self.map_handler: Optional[MapHandler] = None
        self.violation_checker: Optional[ViolationChecker] = None
        
        # Monitoring thread / Luồng giám sát
        self.monitoring_thread: Optional[threading.Thread] = None
        self.health_check_thread: Optional[threading.Thread] = None
        
        # Statistics / Thống kê
        self.stats = {
            "startup_time": self.startup_time,
            "total_gps_reads": 0,
            "total_violations": 0,
            "last_gps_time": None,
            "last_violation_time": None,
            "system_uptime": 0.0,
            "error_count": 0
        }
        
        # Setup signal handlers / Thiết lập signal handlers
        self._setup_signal_handlers()
        
        self.logger.system_info("Cycle Sentinel application initialized", {
            "use_simulator": use_simulator,
            "config_file": config_file,
            "device_id": SYSTEM_CONFIG["device_id"]
        })
    
    def _setup_signal_handlers(self):
        """Thiết lập signal handlers cho graceful shutdown"""
        def signal_handler(signum, frame):
            signal_name = signal.Signals(signum).name
            self.logger.system_info(f"Received signal {signal_name}, initiating shutdown")
            self.request_shutdown()
        
        # Register signal handlers / Đăng ký signal handlers
        signal.signal(signal.SIGINT, signal_handler)   # Ctrl+C
        signal.signal(signal.SIGTERM, signal_handler)  # Termination request
        
        if hasattr(signal, 'SIGHUP'):  # Unix only
            signal.signal(signal.SIGHUP, signal_handler)  # Hang up
    
    def create_sample_map_if_needed(self):
        """Tạo sample map nếu chưa có"""
        map_file = MAPS_DIR / "hcm_zones.json"
        
        if not map_file.exists():
            self.logger.system_info("Creating sample map file")
            
            sample_map = {
                "version": "1.0",
                "name": "Ho Chi Minh City Sample Zones",
                "zones": [
                    {
                        "id": "district1_center",
                        "name": "Quận 1 - Trung tâm",
                        "zone_type": "commercial",
                        "speed_limit": 20,
                        "is_restricted": False,
                        "bounds": {
                            "south": 10.7730,
                            "north": 10.7770,
                            "west": 106.6980,
                            "east": 106.7020
                        }
                    },
                    {
                        "id": "nguyen_hue_walking",
                        "name": "Phố đi bộ Nguyễn Huệ",
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
                        "id": "school_zone_1",
                        "name": "Khu trường học - Lê Văn Tám",
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
                        "id": "residential_1",
                        "name": "Quận 3 - Khu dân cư",
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
            
            with open(map_file, 'w', encoding='utf-8') as f:
                json.dump(sample_map, f, indent=2, ensure_ascii=False)
            
            self.logger.system_info(f"Sample map created: {map_file}")
        
        return str(map_file)
    
    def initialize_components(self) -> bool:
        """Khởi tạo tất cả các thành phần hệ thống"""
        try:
            self.logger.system_info("Initializing system components...")
            
            # 1. Validate configuration
            if not validate_config():
                self.logger.system_error("Configuration validation failed")
                return False
            
            # 2. Create sample map if needed
            map_file = self.create_sample_map_if_needed()
            
            # 3. Initialize Map Handler
            self.logger.system_info("Initializing map handler...")
            self.map_handler = create_map_handler(map_file)
            
            if not self.map_handler.is_map_loaded():
                self.logger.system_error("Failed to load map data")
                return False
            
            map_stats = self.map_handler.get_statistics()
            self.logger.system_info("Map handler initialized successfully", {
                "zones_count": map_stats.get("total_zones", 0),
                "zone_types": map_stats.get("zone_types", [])
            })
            
            # 4. Initialize GPS Handler
            self.logger.system_info("Initializing GPS handler...")
            
            if self.use_simulator:
                self.logger.system_info("Using GPS simulator")
                config = SimulationConfig(
                    mode=SimulationMode.MIXED_SCENARIO,
                    duration_seconds=300,  # 5 minutes
                    update_interval=1.0
                )
                self.gps_handler = GPSSimulator(config)
            else:
                self.logger.system_info("Using real GPS hardware")
                self.gps_handler = create_gps_handler()
            
            # Test GPS connection
            if not self.gps_handler.connect():
                self.logger.system_error("Failed to connect to GPS")
                return False
            
            self.logger.system_info("GPS handler initialized successfully")
            
            # 5. Initialize Violation Checker
            self.logger.system_info("Initializing violation checker...")
            self.violation_checker = create_violation_checker(self.map_handler)
            self.logger.system_info("Violation checker initialized successfully")
            
            # 6. Wait for GPS fix if using real hardware
            if not self.use_simulator:
                self.logger.system_info("Waiting for GPS fix...")
                if hasattr(self.gps_handler, 'wait_for_fix'):
                    if hasattr(self.gps_handler, 'data_filter'):
                        self.gps_handler.data_filter._startup_mode = True

                    if not self.gps_handler.wait_for_fix(timeout=10):  # 30 → 10 giây
                        self.logger.system_warning("GPS fix timeout - continuing anyway")
                    else:
                        # ✅ Disable startup mode sau khi có fix
                        if hasattr(self.gps_handler, 'data_filter'):
                            self.gps_handler.data_filter._startup_mode = False
            
            self.logger.system_info("All components initialized successfully")
            return True
            
        except Exception as e:
            self.logger.system_error("Component initialization failed", {
                "error": str(e)
            }, e)
            return False
    
    def start_monitoring(self) -> bool:
        """Bắt đầu giám sát hệ thống"""
        if self.is_running:
            self.logger.system_warning("Monitoring already running")
            return False
        
        try:
            self.is_running = True
            self.shutdown_requested = False
            
            # Start monitoring thread
            self.monitoring_thread = threading.Thread(
                target=self._monitoring_loop,
                name="MonitoringThread",
                daemon=True
            )
            self.monitoring_thread.start()
            
            # Start health check thread
            self.health_check_thread = threading.Thread(
                target=self._health_check_loop,
                name="HealthCheckThread", 
                daemon=True
            )
            self.health_check_thread.start()
            
            self.logger.system_info("System monitoring started")
            return True
            
        except Exception as e:
            self.logger.system_error("Failed to start monitoring", {
                "error": str(e)
            }, e)
            self.is_running = False
            return False
    
    def _monitoring_loop(self):
        """Vòng lặp giám sát chính"""
        self.logger.system_info("Monitoring loop started")
        
        try:
            while self.is_running and not self.shutdown_requested:
                try:
                    with PerformanceTimer("monitoring_cycle"):
                        # Read GPS data
                        gps_data = self.gps_handler.read_position()
                        
                        if gps_data:
                            self.stats["total_gps_reads"] += 1
                            self.stats["last_gps_time"] = datetime.now()
                            
                            # ✅ ĐOẠN NÀY QUAN TRỌNG - CHECK VIOLATION VÀ LOG
                            violation_data = self.violation_checker.check_violation(gps_data)
                            
                            if violation_data:
                                # ✅ LOG VIOLATION - ĐÂY LÀ CHỖ LOGGING DIỄN RA
                                self.logger.log_violation(violation_data)
                                self.stats["total_violations"] += 1
                                self.stats["last_violation_time"] = datetime.now()
                                
                                # Print violation info to console
                                self._print_professional_log(violation_data, gps_data, has_violation=True)
                            else:
                                # Print normal GPS info
                                if SYSTEM_CONFIG["debug_mode"]:
                                    self._print_professional_log(None, gps_data, has_violation=False)
                        
                        else:
                            # No GPS data available
                            # if SYSTEM_CONFIG["debug_mode"]:
                            #     print(".", end="", flush=True)  # Progress indicator
                            pass
                    
                    # Sleep for next cycle
                    time.sleep(GPS_CONFIG["read_interval"])
                    
                except Exception as e:
                    self.logger.system_error("Error in monitoring loop", {
                        "error": str(e)
                    }, e)
                    self.stats["error_count"] += 1
                    time.sleep(1)  # Brief pause on error
        
        except Exception as e:
            self.logger.system_critical("Critical error in monitoring loop", {
                "error": str(e)
            }, e)
        
        finally:
            self.logger.system_info("Monitoring loop ended")
    
    # def _monitoring_loop(self):
    #     """Vòng lặp giám sát chính - Enhanced với GPS tracking logging"""
    #     self.logger.system_info("Monitoring loop started")
        
    #     try:
    #         while self.is_running and not self.shutdown_requested:
    #             try:
    #                 with PerformanceTimer("monitoring_cycle"):
    #                     # Read GPS data
    #                     gps_data = self.gps_handler.read_position()
                        
    #                     if gps_data:
    #                         self.stats["total_gps_reads"] += 1
    #                         self.stats["last_gps_time"] = datetime.now()
                            
    #                         # ✅ GET ZONE INFO
    #                         lat, lon = gps_data["latitude"], gps_data["longitude"]
    #                         current_zone = self.map_handler.find_zone_at_position(lat, lon)
    #                         zone_info = {
    #                             "zone_name": current_zone.name if current_zone else "Outside Zone",
    #                             "zone_type": current_zone.zone_type if current_zone else "unknown",
    #                             "speed_limit": current_zone.speed_limit if current_zone else 25.0,
    #                             "zone_id": current_zone.zone_id if current_zone else None
    #                         }
                            
    #                         # ✅ LOG GPS TRACKING DATA VÀO FILE
    #                         if LOGGING_CONFIG["log_gps_tracking"]:
    #                             self.logger.log_gps_tracking(gps_data, zone_info)
                            
    #                         # Check violation
    #                         violation_data = self.violation_checker.check_violation(gps_data)
                            
    #                         if violation_data:
    #                             # Log violation
    #                             self.logger.log_violation(violation_data)
    #                             self.stats["total_violations"] += 1
    #                             self.stats["last_violation_time"] = datetime.now()
                                
    #                             # Print violation info to console
    #                             self._print_professional_log(violation_data, gps_data, has_violation=True)
    #                         else:
    #                             # Print normal GPS info
    #                             if SYSTEM_CONFIG["debug_mode"]:
    #                                 self._print_gps_info(gps_data)
                        
    #                     else:
    #                         # No GPS data available
    #                         if SYSTEM_CONFIG["debug_mode"]:
    #                             print(".", end="", flush=True)
                    
    #                 # Sleep for next cycle
    #                 time.sleep(GPS_CONFIG["read_interval"])
                    
    #             except Exception as e:
    #                 self.logger.system_error("Error in monitoring loop", {
    #                     "error": str(e)
    #                 }, e)
    #                 self.stats["error_count"] += 1
    #                 time.sleep(1)
        
    #     except Exception as e:
    #         self.logger.system_critical("Critical error in monitoring loop", {
    #             "error": str(e)
    #         }, e)
        
    #     finally:
    #         self.logger.system_info("Monitoring loop ended")
    
    def _health_check_loop(self):
        """Vòng lặp kiểm tra sức khỏe hệ thống"""
        self.logger.system_info("Health check loop started")
        
        try:
            while self.is_running and not self.shutdown_requested:
                try:
                    # Update system stats
                    self._update_system_stats()
                    
                    # Check component health
                    self._check_component_health()
                    
                    # Sleep for next check
                    time.sleep(SYSTEM_CONFIG["heartbeat_interval"])
                    
                except Exception as e:
                    self.logger.system_error("Error in health check loop", {
                        "error": str(e)
                    }, e)
                    time.sleep(5)  # Brief pause on error
        
        except Exception as e:
            self.logger.system_error("Critical error in health check loop", {
                "error": str(e)
            }, e)
        
        finally:
            self.logger.system_info("Health check loop ended")
    
    def _update_system_stats(self):
        """Cập nhật thống kê hệ thống"""
        self.stats["system_uptime"] = (datetime.now() - self.startup_time).total_seconds()
    
    def _check_component_health(self):
        """Kiểm tra sức khỏe các thành phần"""
        health_issues = []
        
        # Check GPS handler
        if self.gps_handler:
            if not self.use_simulator and hasattr(self.gps_handler, 'is_position_valid'):
                if not self.gps_handler.is_position_valid():
                    health_issues.append("GPS position invalid or stale")
        
        # Check map handler
        if self.map_handler and not self.map_handler.is_map_loaded():
            health_issues.append("Map not loaded")
        
        # Check last GPS reading
        if self.stats["last_gps_time"]:
            time_since_gps = (datetime.now() - self.stats["last_gps_time"]).total_seconds()
            if time_since_gps > 60:  # No GPS data for 1 minute
                health_issues.append(f"No GPS data for {time_since_gps:.0f} seconds")
        
        # Log health issues
        if health_issues:
            self.logger.system_warning("System health issues detected", {
                "issues": health_issues
            })
        else:
            self.logger.system_debug("System health check passed")
    
    def _print_professional_log(self, violation_data: Optional[Dict[str, Any]], gps_data: Dict[str, Any], has_violation: bool):
        """In log theo format chuyên nghiệp 1 dòng"""
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        lat = gps_data["latitude"]
        lon = gps_data["longitude"]
        speed = gps_data["speed_kmh"]
        
        # Get zone info
        zone = self.map_handler.find_zone_at_position(lat, lon)
        zone_name = zone.name if zone else "Unknown Zone"
        speed_limit = zone.speed_limit if zone else 25.0
        
        # Build violation status
        if not has_violation:
            violation_status = "NO_VIOLATIONS"
        else:
            violation_details = []
            violation_type = violation_data["violation_type"]
            
            if violation_type == "speed_violation":
                excess_speed = violation_data["speed_data"]["excess_speed"]
                violation_details.append(f"SPEED_EXCEEDED:{excess_speed:.1f}kmh")
            elif violation_type == "restricted_zone":
                violation_details.append("RESTRICTED_ZONE_ENTRY")
            
            violation_status = "|".join(violation_details) if violation_details else "VIOLATION_DETECTED"
        
        # Single line professional format
        print(f"[{timestamp}] LAT:{lat:.6f} LON:{lon:.6f} SPEED:{speed:.1f}kmh LIMIT:{speed_limit}kmh STATUS:{violation_status}")

    def _print_gps_info(self, gps_data: Dict[str, Any]):
        """In thông tin GPS bình thường ra console"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        lat = gps_data["latitude"]
        lon = gps_data["longitude"]
        speed = gps_data["speed_kmh"]
        
        # Get current zone info
        zone = self.map_handler.find_zone_at_position(lat, lon)
        zone_name = zone.name if zone else "Ngoài khu vực"
        speed_limit = zone.speed_limit if zone else 25.0
        
        print(f"[{timestamp}] 📍 {lat:.6f}, {lon:.6f} | "
              f"🏃 {speed:.1f}/{speed_limit:.0f} km/h | "
              f"🎯 {zone_name}")
    
    def request_shutdown(self):
        """Yêu cầu tắt hệ thống"""
        self.logger.system_info("Shutdown requested")
        self.shutdown_requested = True
    
    def stop_monitoring(self):
        """Dừng giám sát hệ thống"""
        if not self.is_running:
            return
        
        self.logger.system_info("Stopping system monitoring...")
        
        self.is_running = False
        self.shutdown_requested = True
        
        # Wait for threads to finish
        if self.monitoring_thread and self.monitoring_thread.is_alive():
            self.monitoring_thread.join(timeout=5.0)
        
        if self.health_check_thread and self.health_check_thread.is_alive():
            self.health_check_thread.join(timeout=5.0)
        
        self.logger.system_info("System monitoring stopped")
    
    def cleanup(self):
        """Dọn dẹp tài nguyên hệ thống"""
        self.logger.system_info("Cleaning up system resources...")
        
        try:
            # Disconnect GPS
            if self.gps_handler:
                self.gps_handler.disconnect()
                self.logger.system_info("GPS disconnected")
            
            # Log final statistics
            self._log_final_statistics()
            
        except Exception as e:
            self.logger.system_error("Error during cleanup", {"error": str(e)}, e)
        
        self.logger.system_info("System cleanup completed")
    
    def _log_final_statistics(self):
        """Ghi log thống kê cuối cùng"""
        final_stats = self.get_system_statistics()
        self.logger.system_info("Final system statistics", final_stats)
    
    def get_system_statistics(self) -> Dict[str, Any]:
        """Lấy thống kê toàn hệ thống"""
        stats = self.stats.copy()
        
        # Add component statistics
        if self.gps_handler and hasattr(self.gps_handler, 'get_statistics'):
            stats["gps_stats"] = self.gps_handler.get_statistics()
        
        if self.map_handler:
            stats["map_stats"] = self.map_handler.get_statistics()
        
        if self.violation_checker:
            stats["violation_stats"] = self.violation_checker.get_statistics()
        
        # Calculate rates
        uptime_hours = stats["system_uptime"] / 3600
        if uptime_hours > 0:
            stats["gps_reads_per_hour"] = stats["total_gps_reads"] / uptime_hours
            stats["violations_per_hour"] = stats["total_violations"] / uptime_hours
        
        return stats
    
    def run(self) -> int:
        """Chạy ứng dụng chính"""
        try:
            # Log system startup
            log_system_startup()
            
            # Show configuration summary
            config_summary = get_config_summary()
            self.logger.system_info("System configuration", config_summary)
            
            # Initialize components
            if not self.initialize_components():
                self.logger.system_error("Component initialization failed")
                return 1
            
            # Start monitoring
            if not self.start_monitoring():
                self.logger.system_error("Failed to start monitoring")
                return 1
            
            # Show startup message
            self._print_startup_message()
            
            # Main loop
            try:
                while not self.shutdown_requested:
                    time.sleep(1)
                    
                    # Check if monitoring thread is still alive
                    if self.monitoring_thread and not self.monitoring_thread.is_alive():
                        self.logger.system_error("Monitoring thread died unexpectedly")
                        break
            
            except KeyboardInterrupt:
                self.logger.system_info("Keyboard interrupt received")
            
            return 0
            
        except Exception as e:
            self.logger.system_critical("Critical application error", {
                "error": str(e)
            }, e)
            return 1
        
        finally:
            # Cleanup
            self.stop_monitoring()
            self.cleanup()
            log_system_shutdown()
    
    def _print_startup_message(self):
        """In thông báo khởi động ra console"""
        print("\n" + "="*60)
        print("🚴 CYCLE SENTINEL - E-BIKE MONITORING SYSTEM")
        print("="*60)
        print(f" Device ID: {SYSTEM_CONFIG['device_id']}")
        print(f"GPS Mode: {'Simulator' if self.use_simulator else 'Hardware'}")
        print(f"Map Zones: {len(self.map_handler.zones)} loaded")
        print(f"Debug Mode: {'ON' if SYSTEM_CONFIG['debug_mode'] else 'OFF'}")
        print(f"Monitoring: GPS + Violations")
        print(f"Update Interval: {GPS_CONFIG['read_interval']}s")
        print(f"Logs Directory: {LOGS_DIR}")
        print("="*60)
        print("="*60)
        print()

def parse_arguments():
    """Parse command line arguments"""
    parser = argparse.ArgumentParser(
        description="Cycle Sentinel - E-bike Speed and Zone Monitoring System",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples / Ví dụ:
  python main_fixed.py                    # Run with real GPS / Chạy với GPS thật
  python main_fixed.py --simulate         # Run with GPS simulator / Chạy với GPS simulator
  python main_fixed.py --debug            # Run in debug mode / Chạy ở chế độ debug
  python main_fixed.py --test             # Run test scenarios / Chạy các kịch bản test
        """
    )
    
    parser.add_argument(
        "--simulate",
        action="store_true",
        help="Use GPS simulator instead of real GPS hardware / Sử dụng GPS simulator thay vì GPS thật"
    )
    
    parser.add_argument(
        "--config",
        type=str,
        help="Path to configuration file / Đường dẫn đến file cấu hình"
    )
    
    parser.add_argument(
        "--debug",
        action="store_true",
        help="Enable debug mode with verbose output / Bật chế độ debug với output chi tiết"
    )
    
    parser.add_argument(
        "--test",
        action="store_true", 
        help="Run in test mode with predefined scenarios / Chạy ở chế độ test với các kịch bản định sẵn"
    )
    
    parser.add_argument(
        "--version",
        action="version",
        version=f"Cycle Sentinel {SYSTEM_CONFIG['version']}"
    )
    
    return parser.parse_args()

def main():
    """Hàm main chính"""
    # Parse command line arguments
    args = parse_arguments()
    
    # Override config with command line arguments
    if args.debug:
        SYSTEM_CONFIG["debug_mode"] = True
        print("🐛 Debug mode enabled")
    
    # Create and run application
    try:
        app = CycleSentinelApp(
            use_simulator=args.simulate or args.test,
            config_file=args.config
        )
        
        exit_code = app.run()
        sys.exit(exit_code)
        
    except KeyboardInterrupt:
        print("\n🛑 Application interrupted by user")
        sys.exit(0)
    except Exception as e:
        print(f"💥 Critical application error: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()