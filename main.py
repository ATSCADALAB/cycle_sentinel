#!/usr/bin/env python3
"""
Cycle Sentinel Main Application
Ứng dụng chính của Cycle Sentinel

This is the main entry point for the Cycle Sentinel e-bike monitoring system.
Đây là điểm khởi đầu chính cho hệ thống giám sát xe đạp điện Cycle Sentinel.

Features / Tính năng:
- GPS monitoring and violation detection / Giám sát GPS và phát hiện vi phạm
- Real-time speed and zone enforcement / Thực thi tốc độ và zone theo thời gian thực
- Automatic logging and reporting / Tự động ghi log và báo cáo
- System health monitoring / Giám sát sức khỏe hệ thống
- Graceful shutdown handling / Xử lý tắt hệ thống một cách nhẹ nhàng

Usage / Cách sử dụng:
    python main.py [options]
    
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
        get_config_summary
    )
    from utils.logger import (
        get_logger, 
        log_system_startup, 
        log_system_shutdown,
        PerformanceTimer
    )
    from sensors.gps_handler import GPSHandler, create_gps_handler
    from sensors.gps_simulator import GPSSimulator, create_mixed_scenario
    from enforcement.map_handler import MapHandler, create_map_handler
    from enforcement.violation_checker import ViolationChecker, create_violation_checker
    
except ImportError as e:
    print(f"❌ Failed to import required modules: {e}")
    print("Please ensure all dependencies are installed and paths are correct.")
    sys.exit(1)

class CycleSentinelApp:
    """
    Ứng dụng chính Cycle Sentinel
    Main Cycle Sentinel application
    
    Coordinates all system components and manages the main monitoring loop
    Điều phối tất cả các thành phần hệ thống và quản lý vòng lặp giám sát chính
    """
    
    def __init__(self, use_simulator: bool = False, config_file: Optional[str] = None):
        """
        Khởi tạo ứng dụng Cycle Sentinel
        Initialize Cycle Sentinel application
        
        Args:
            use_simulator: Sử dụng GPS simulator thay vì GPS thật / Use GPS simulator instead of real GPS
            config_file: Đường dẫn file cấu hình tùy chọn / Optional config file path
        """
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
        """
        Thiết lập signal handlers cho graceful shutdown
        Setup signal handlers for graceful shutdown
        """
        def signal_handler(signum, frame):
            signal_name = signal.Signals(signum).name
            self.logger.system_info(f"Received signal {signal_name}, initiating shutdown")
            self.request_shutdown()
        
        # Register signal handlers / Đăng ký signal handlers
        signal.signal(signal.SIGINT, signal_handler)   # Ctrl+C
        signal.signal(signal.SIGTERM, signal_handler)  # Termination request
        
        if hasattr(signal, 'SIGHUP'):  # Unix only
            signal.signal(signal.SIGHUP, signal_handler)  # Hang up
    
    def initialize_components(self) -> bool:
        """
        Khởi tạo tất cả các thành phần hệ thống
        Initialize all system components
        
        Returns:
            bool: True nếu khởi tạo thành công / True if initialization successful
        """
        try:
            self.logger.system_info("Initializing system components...")
            
            # 1. Validate configuration / Xác thực cấu hình
            if not validate_config():
                self.logger.system_error("Configuration validation failed")
                return False
            
            # 2. Initialize Map Handler / Khởi tạo Map Handler
            self.logger.system_info("Initializing map handler...")
            self.map_handler = create_map_handler(MAP_CONFIG["map_file"])
            
            if not self.map_handler.is_map_loaded():
                self.logger.system_error("Failed to load map data")
                return False
            
            map_stats = self.map_handler.get_statistics()
            self.logger.system_info("Map handler initialized successfully", {
                "zones_count": map_stats.get("total_zones", 0),
                "zone_types": map_stats.get("zone_types", [])
            })
            
            # 3. Initialize GPS Handler / Khởi tạo GPS Handler
            self.logger.system_info("Initializing GPS handler...")
            
            if self.use_simulator:
                self.logger.system_info("Using GPS simulator")
                self.gps_handler = create_mixed_scenario()
            else:
                self.logger.system_info("Using real GPS hardware")
                self.gps_handler = create_gps_handler()
            
            # Test GPS connection / Test kết nối GPS
            if not self.gps_handler.connect():
                self.logger.system_error("Failed to connect to GPS")
                return False
            
            self.logger.system_info("GPS handler initialized successfully")
            
            # 4. Initialize Violation Checker / Khởi tạo Violation Checker
            self.logger.system_info("Initializing violation checker...")
            self.violation_checker = create_violation_checker(self.map_handler)
            self.logger.system_info("Violation checker initialized successfully")
            
            # 5. Wait for GPS fix if using real hardware / Chờ GPS fix nếu dùng phần cứng thật
            if not self.use_simulator:
                self.logger.system_info("Waiting for GPS fix...")
                if not self.gps_handler.wait_for_fix(timeout=30):
                    self.logger.system_warning("GPS fix timeout - continuing anyway")
                else:
                    self.logger.system_info("GPS fix acquired")
            
            self.logger.system_info("All components initialized successfully")
            return True
            
        except Exception as e:
            self.logger.system_error("Component initialization failed", {
                "error": str(e)
            }, e)
            return False
    
    def start_monitoring(self) -> bool:
        """
        Bắt đầu giám sát hệ thống
        Start system monitoring
        
        Returns:
            bool: True nếu bắt đầu thành công / True if started successfully
        """
        if self.is_running:
            self.logger.system_warning("Monitoring already running")
            return False
        
        try:
            self.is_running = True
            self.shutdown_requested = False
            
            # Start monitoring thread / Bắt đầu luồng giám sát
            self.monitoring_thread = threading.Thread(
                target=self._monitoring_loop,
                name="MonitoringThread",
                daemon=True
            )
            self.monitoring_thread.start()
            
            # Start health check thread / Bắt đầu luồng kiểm tra sức khỏe
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
        """
        Vòng lặp giám sát chính
        Main monitoring loop
        """
        self.logger.system_info("Monitoring loop started")
        
        try:
            while self.is_running and not self.shutdown_requested:
                try:
                    with PerformanceTimer("monitoring_cycle"):
                        # Read GPS data / Đọc dữ liệu GPS
                        gps_data = self.gps_handler.read_position()
                        
                        if gps_data:
                            self.stats["total_gps_reads"] += 1
                            self.stats["last_gps_time"] = datetime.now()
                            
                            # Check for violations / Kiểm tra vi phạm
                            violation_data = self.violation_checker.check_violation(gps_data)
                            
                            if violation_data:
                                # Log violation / Ghi log vi phạm
                                self.logger.log_violation(violation_data)
                                self.stats["total_violations"] += 1
                                self.stats["last_violation_time"] = datetime.now()
                                
                                # Print violation info to console / In thông tin vi phạm ra console
                                self._print_violation_info(violation_data, gps_data)
                            else:
                                # Print normal GPS info / In thông tin GPS bình thường
                                if SYSTEM_CONFIG["debug_mode"]:
                                    self._print_gps_info(gps_data)
                        
                        else:
                            # No GPS data available / Không có dữ liệu GPS
                            if SYSTEM_CONFIG["debug_mode"]:
                                print(".", end="", flush=True)  # Progress indicator
                    
                    # Sleep for next cycle / Nghỉ cho chu kỳ tiếp theo
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
    
    def _health_check_loop(self):
        """
        Vòng lặp kiểm tra sức khỏe hệ thống
        System health check loop
        """
        self.logger.system_info("Health check loop started")
        
        try:
            while self.is_running and not self.shutdown_requested:
                try:
                    # Update system stats / Cập nhật thống kê hệ thống
                    self._update_system_stats()
                    
                    # Check component health / Kiểm tra sức khỏe các thành phần
                    self._check_component_health()
                    
                    # Sleep for next check / Nghỉ cho lần kiểm tra tiếp theo
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
        """
        Cập nhật thống kê hệ thống
        Update system statistics
        """
        self.stats["system_uptime"] = (datetime.now() - self.startup_time).total_seconds()
    
    def _check_component_health(self):
        """
        Kiểm tra sức khỏe các thành phần
        Check component health
        """
        health_issues = []
        
        # Check GPS handler / Kiểm tra GPS handler
        if self.gps_handler:
            if not self.use_simulator and hasattr(self.gps_handler, 'is_position_valid'):
                if not self.gps_handler.is_position_valid():
                    health_issues.append("GPS position invalid or stale")
        
        # Check map handler / Kiểm tra map handler
        if self.map_handler and not self.map_handler.is_map_loaded():
            health_issues.append("Map not loaded")
        
        # Check last GPS reading / Kiểm tra lần đọc GPS cuối
        if self.stats["last_gps_time"]:
            time_since_gps = (datetime.now() - self.stats["last_gps_time"]).total_seconds()
            if time_since_gps > 60:  # No GPS data for 1 minute
                health_issues.append(f"No GPS data for {time_since_gps:.0f} seconds")
        
        # Log health issues / Ghi log các vấn đề sức khỏe
        if health_issues:
            self.logger.system_warning("System health issues detected", {
                "issues": health_issues
            })
        else:
            self.logger.system_debug("System health check passed")
    
    def _print_violation_info(self, violation_data: Dict[str, Any], gps_data: Dict[str, Any]):
        """
        In thông tin vi phạm ra console
        Print violation information to console
        
        Args:
            violation_data: Dữ liệu vi phạm / Violation data
            gps_data: Dữ liệu GPS / GPS data
        """
        timestamp = datetime.now().strftime("%H:%M:%S")
        violation_type = violation_data["violation_type"]
        
        if violation_type == "speed_violation":
            current_speed = violation_data["speed_data"]["current_speed"]
            speed_limit = violation_data["speed_data"]["speed_limit"]
            excess_speed = violation_data["speed_data"]["excess_speed"]
            
            print(f"\n🚨 [{timestamp}] SPEED VIOLATION DETECTED!")
            print(f"   📍 Location: {gps_data['latitude']:.6f}, {gps_data['longitude']:.6f}")
            print(f"   🏃 Speed: {current_speed:.1f} km/h (Limit: {speed_limit:.1f} km/h)")
            print(f"   ⚡ Excess: +{excess_speed:.1f} km/h")
            print(f"   🎯 Zone: {violation_data['zone_info']['zone_name']}")
            print(f"   📊 Confidence: {violation_data['confidence']:.2f}")
            
        elif violation_type == "restricted_zone":
            current_speed = violation_data["speed_data"]["current_speed"]
            
            print(f"\n🚫 [{timestamp}] RESTRICTED ZONE VIOLATION!")
            print(f"   📍 Location: {gps_data['latitude']:.6f}, {gps_data['longitude']:.6f}")
            print(f"   🏃 Speed: {current_speed:.1f} km/h (Zone: RESTRICTED)")
            print(f"   🎯 Zone: {violation_data['zone_info']['zone_name']}")
            print(f"   📊 Confidence: {violation_data['confidence']:.2f}")
        
        print()
    
    def _print_gps_info(self, gps_data: Dict[str, Any]):
        """
        In thông tin GPS bình thường ra console
        Print normal GPS information to console
        
        Args:
            gps_data: Dữ liệu GPS / GPS data
        """
        timestamp = datetime.now().strftime("%H:%M:%S")
        lat = gps_data["latitude"]
        lon = gps_data["longitude"]
        speed = gps_data["speed_kmh"]
        
        # Get current zone info / Lấy thông tin zone hiện tại
        zone = self.map_handler.find_zone_at_position(lat, lon)
        zone_name = zone.name if zone else "Unknown Zone"
        speed_limit = zone.speed_limit if zone else 25.0
        
        print(f"[{timestamp}] 📍 {lat:.6f}, {lon:.6f} | "
              f"🏃 {speed:.1f}/{speed_limit:.0f} km/h | "
              f"🎯 {zone_name}")
    
    def request_shutdown(self):
        """
        Yêu cầu tắt hệ thống
        Request system shutdown
        """
        self.logger.system_info("Shutdown requested")
        self.shutdown_requested = True
    
    def stop_monitoring(self):
        """
        Dừng giám sát hệ thống
        Stop system monitoring
        """
        if not self.is_running:
            return
        
        self.logger.system_info("Stopping system monitoring...")
        
        self.is_running = False
        self.shutdown_requested = True
        
        # Wait for threads to finish / Chờ các luồng kết thúc
        if self.monitoring_thread and self.monitoring_thread.is_alive():
            self.monitoring_thread.join(timeout=5.0)
        
        if self.health_check_thread and self.health_check_thread.is_alive():
            self.health_check_thread.join(timeout=5.0)
        
        self.logger.system_info("System monitoring stopped")
    
    def cleanup(self):
        """
        Dọn dẹp tài nguyên hệ thống
        Cleanup system resources
        """
        self.logger.system_info("Cleaning up system resources...")
        
        try:
            # Disconnect GPS / Ngắt kết nối GPS
            if self.gps_handler:
                self.gps_handler.disconnect()
                self.logger.system_info("GPS disconnected")
            
            # Log final statistics / Ghi log thống kê cuối
            self._log_final_statistics()
            
        except Exception as e:
            self.logger.system_error("Error during cleanup", {"error": str(e)}, e)
        
        self.logger.system_info("System cleanup completed")
    
    def _log_final_statistics(self):
        """
        Ghi log thống kê cuối cùng
        Log final statistics
        """
        final_stats = self.get_system_statistics()
        self.logger.system_info("Final system statistics", final_stats)
    
    def get_system_statistics(self) -> Dict[str, Any]:
        """
        Lấy thống kê toàn hệ thống
        Get comprehensive system statistics
        
        Returns:
            Dict: System statistics / Thống kê hệ thống
        """
        stats = self.stats.copy()
        
        # Add component statistics / Thêm thống kê các thành phần
        if self.gps_handler and hasattr(self.gps_handler, 'get_statistics'):
            stats["gps_stats"] = self.gps_handler.get_statistics()
        
        if self.map_handler:
            stats["map_stats"] = self.map_handler.get_statistics()
        
        if self.violation_checker:
            stats["violation_stats"] = self.violation_checker.get_statistics()
        
        # Calculate rates / Tính tỷ lệ
        uptime_hours = stats["system_uptime"] / 3600
        if uptime_hours > 0:
            stats["gps_reads_per_hour"] = stats["total_gps_reads"] / uptime_hours
            stats["violations_per_hour"] = stats["total_violations"] / uptime_hours
        
        return stats
    
    def run(self) -> int:
        """
        Chạy ứng dụng chính
        Run the main application
        
        Returns:
            int: Exit code (0 = success, non-zero = error)
        """
        try:
            # Log system startup / Ghi log khởi động hệ thống
            log_system_startup()
            
            # Show configuration summary / Hiển thị tóm tắt cấu hình
            config_summary = get_config_summary()
            self.logger.system_info("System configuration", config_summary)
            
            # Initialize components / Khởi tạo các thành phần
            if not self.initialize_components():
                self.logger.system_error("Component initialization failed")
                return 1
            
            # Start monitoring / Bắt đầu giám sát
            if not self.start_monitoring():
                self.logger.system_error("Failed to start monitoring")
                return 1
            
            # Show startup message / Hiển thị thông báo khởi động
            self._print_startup_message()
            
            # Main loop / Vòng lặp chính
            try:
                while not self.shutdown_requested:
                    time.sleep(1)
                    
                    # Check if monitoring thread is still alive / Kiểm tra monitoring thread còn sống không
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
            # Cleanup / Dọn dẹp
            self.stop_monitoring()
            self.cleanup()
            log_system_shutdown()
    
    def _print_startup_message(self):
        """
        In thông báo khởi động ra console
        Print startup message to console
        """
        print("\n" + "="*60)
        print("🚴 CYCLE SENTINEL - E-BIKE MONITORING SYSTEM")
        print("="*60)
        print(f"📱 Device ID: {SYSTEM_CONFIG['device_id']}")
        print(f"🌐 GPS Mode: {'Simulator' if self.use_simulator else 'Hardware'}")
        print(f"🗺️  Map Zones: {len(self.map_handler.zones)} loaded")
        print(f"⚙️  Debug Mode: {'ON' if SYSTEM_CONFIG['debug_mode'] else 'OFF'}")
        print(f"📊 Monitoring: GPS + Violations")
        print(f"🔄 Update Interval: {GPS_CONFIG['read_interval']}s")
        print("="*60)
        print("🟢 System is RUNNING - Press Ctrl+C to stop")
        print("="*60)
        print()

def parse_arguments():
    """
    Parse command line arguments
    Phân tích các tham số dòng lệnh
    
    Returns:
        argparse.Namespace: Parsed arguments
    """
    parser = argparse.ArgumentParser(
        description="Cycle Sentinel - E-bike Speed and Zone Monitoring System",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples / Ví dụ:
  python main.py                    # Run with real GPS / Chạy với GPS thật
  python main.py --simulate         # Run with GPS simulator / Chạy với GPS simulator
  python main.py --debug            # Run in debug mode / Chạy ở chế độ debug
  python main.py --test             # Run test scenarios / Chạy các kịch bản test
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
    """
    Hàm main chính
    Main entry point function
    """
    # Parse command line arguments / Phân tích tham số dòng lệnh
    args = parse_arguments()
    
    # Override config with command line arguments / Ghi đè config với tham số dòng lệnh
    if args.debug:
        SYSTEM_CONFIG["debug_mode"] = True
        print("🐛 Debug mode enabled")
    
    # Create and run application / Tạo và chạy ứng dụng
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