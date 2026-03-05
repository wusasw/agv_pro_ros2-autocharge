#!/usr/bin/env python3
"""
ABCD 循环导航脚本（最终版）
低电压取消导航后，直接启动charge_test.py自动回充（无需模拟按键）
"""

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
import math
from std_msgs.msg import Float32
import subprocess
import time
import threading
import os
import signal
import sys  # 新增：用于强制刷新日志


def set_initial_pose(navigator: BasicNavigator, x: float, y: float, yaw_deg: float):
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = x
    initial_pose.pose.position.y = y
    yaw_rad = math.radians(yaw_deg)
    initial_pose.pose.orientation.z = math.sin(yaw_rad / 2.0)
    initial_pose.pose.orientation.w = math.cos(yaw_rad / 2.0)
    navigator.setInitialPose(initial_pose)


def create_pose(navigator: BasicNavigator, x: float, y: float, yaw_deg: float) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    yaw_rad = math.radians(yaw_deg)
    pose.pose.orientation.z = math.sin(yaw_rad / 2.0)
    pose.pose.orientation.w = math.cos(yaw_rad / 2.0)
    return pose


class VoltageMonitor:
    """电压监测类（最终版）"""
    
    def __init__(self, navigator):
        self.navigator = navigator
        self.low_voltage_threshold = 21.0
        self.last_voltage = 0.0
        self.recharge_started = False  # 标记：是否已经发起过回充流程
        self.voltage_subscription = None
        self.recharge_process = None
        self.ros_env = os.environ.copy()  # 保存ROS环境变量（关键）

    def start_monitoring(self):
        self.voltage_subscription = self.navigator.create_subscription(
            Float32, 'voltage', self.voltage_callback, 10)
        print(f"✅ 电压监测已启动，阈值: {self.low_voltage_threshold}V")
        sys.stdout.flush()  # 强制刷新日志

    def voltage_callback(self, msg):
        current_voltage = msg.data
        self.last_voltage = current_voltage
        
        print(f'[电压] 当前: {current_voltage:.1f}V')
        sys.stdout.flush()
        
        # 核心逻辑：低电压+未启动回充 → 触发回充
        if current_voltage < self.low_voltage_threshold and not self.recharge_started:
            print(f'🚨 [电压] 过低！触发回充逻辑！')
            sys.stdout.flush()
            
            # ====================== 关键修复：顺序调换 ======================
            # 1. 先标记：防止重复触发
            self.recharge_started = True
            
            # 2. 先启动回充线程（绝对不阻塞！）
            print(f'🚨 [电压] 正在启动回充线程...')
            sys.stdout.flush()
            threading.Thread(
                target=self.start_recharge_process, 
                daemon=False,
                name="RechargeThread"
            ).start()
            
            # 3. 最后再去取消导航（让它慢慢取消，不影响回充启动）
            print(f'🚨 [电压] 尝试停止导航任务...')
            sys.stdout.flush()
            try:
                self.navigator.cancelTask()
                print(f'✅ [电压] 导航任务已取消（后台执行）')
            except Exception as e:
                print(f'[电压] 取消导航时出错（非致命）: {e}')
            sys.stdout.flush()

    def start_recharge_process(self):
        """启动回充进程（最终版：直接启动charge_test.py）"""
        print(f'🔋 [回充] 线程已启动，开始执行回充流程...')
        sys.stdout.flush()
        
        try:
            # 步骤1：等待5秒，确保导航完全停止、系统稳定
            print(f'🔋 [回充] 等待 5 秒，确保系统稳定...')
            sys.stdout.flush()
            time.sleep(5)
            
            # 步骤2：清理旧的回充进程（包括charge_test和旧的combined_auto_recharger）
            print(f'🔋 [回充] 清理旧的回充进程...')
            sys.stdout.flush()
            self.clean_old_recharge_process()
            
            # 步骤3：启动charge_test.py（自动触发版，无需按q）
            # ******** 关键修改 ********
            # 方式1：如果charge_test.py已配置为ROS2节点（推荐）
            print(f'🔋 [回充] 执行命令: ros2 run agv_pro_autocharge charge_test')
            cmd = ['ros2', 'run', 'agv_pro_autocharge', 'charge_test']
            
            # 方式2：如果未配置节点，直接用python3执行脚本（替换为你的实际路径）
            # cmd = ['python3', '/home/elephant/agv_pro_ros2/src/agv_pro_autocharge/charge_test.py']
            
            sys.stdout.flush()
            
            # 启动进程（继承ROS环境，捕获输出）
            self.recharge_process = subprocess.Popen(
                cmd,
                env=self.ros_env,  # 继承ROS环境变量
                stdout=subprocess.PIPE,  # 捕获标准输出
                stderr=subprocess.STDOUT,  # 错误输出重定向到标准输出
                text=True,  # 输出为文本格式
                preexec_fn=os.setsid,  # 创建新进程组，方便后续杀死
                bufsize=1,  # 行缓冲，实时打印输出
                universal_newlines=True
            )
            
            print(f'✅ 🔋 [回充] charge_test.py 进程已启动，PID: {self.recharge_process.pid}')
            sys.stdout.flush()
            
            # 步骤4：实时读取并打印回充进程的输出（调试用）
            print(f'🔋 [回充] 开始监控charge_test.py输出...')
            sys.stdout.flush()
            self.read_recharge_output()
            
        except Exception as e:
            print(f'❌ [回充] 启动异常: {str(e)}')
            sys.stdout.flush()
            import traceback
            traceback.print_exc()
            sys.stdout.flush()
            # 出错后重置状态，允许重试
            self.recharge_started = False

    def read_recharge_output(self):
        """实时读取charge_test.py的输出并打印"""
        if not self.recharge_process:
            return
        
        while True:
            try:
                # 检查进程是否已退出
                if self.recharge_process.poll() is not None:
                    exit_code = self.recharge_process.returncode
                    print(f'🔋 [回充] charge_test.py 已退出，退出码: {exit_code}')
                    sys.stdout.flush()
                    break
                
                # 读取一行输出（非阻塞）
                output = self.recharge_process.stdout.readline()
                if output:
                    print(f'[charge_test] {output.strip()}')
                    sys.stdout.flush()
                    
                time.sleep(0.1)  # 避免过度占用CPU
                
            except Exception as e:
                print(f'❌ [回充] 读取输出出错: {e}')
                sys.stdout.flush()
                break

    def clean_old_recharge_process(self):
        """清理旧的回充进程（包括charge_test和combined_auto_recharger）"""
        # 杀死当前启动的回充进程
        try:
            if self.recharge_process and self.recharge_process.poll() is None:
                os.killpg(os.getpgid(self.recharge_process.pid), signal.SIGTERM)
                self.recharge_process.wait(timeout=3)
                print(f'🔋 [回充] 旧进程已杀死')
        except Exception as e:
            print(f'[回充] 清理旧进程警告: {e}')
        
        # 暴力清理所有相关进程
        try:
            # 杀死charge_test相关进程
            subprocess.run(
                ['pkill', '-f', 'charge_test.py'],
                timeout=2,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
            # 杀死旧的combined_auto_recharger进程
            subprocess.run(
                ['pkill', '-f', 'combined_auto_recharger'],
                timeout=2,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
            print(f'🔋 [回充] 所有旧回充进程已清理')
            sys.stdout.flush()
        except Exception as e:
            print(f'[回充] 暴力清理进程警告: {e}')
            sys.stdout.flush()

    def stop_recharge_process(self):
        """停止回充进程"""
        self.clean_old_recharge_process()


def execute_waypoint_navigation(navigator: BasicNavigator, goal_poses, cycle_num: int, voltage_monitor) -> bool:
    """执行航点导航"""
    if voltage_monitor.recharge_started:
        return False

    nav_start = navigator.get_clock().now()
    navigator.followWaypoints(goal_poses)

    print(f"\n开始第 {cycle_num} 轮导航\n")
    sys.stdout.flush()

    while not navigator.isTaskComplete():
        if voltage_monitor.recharge_started:
            print("\n检测到回充信号，中断当前导航...")
            sys.stdout.flush()
            try:
                navigator.cancelTask()
            except:
                pass
            return False
            
        if navigator.get_clock().now() - nav_start > Duration(seconds=600.0):
            navigator.cancelTask()
            break
        rclpy.spin_once(navigator, timeout_sec=0.1)

    return navigator.getResult() == TaskResult.SUCCEEDED


def main():
    rclpy.init()
    navigator = BasicNavigator()

    print("="*60)
    print("           ABCD 循环导航系统（最终版）")
    print("="*60)
    sys.stdout.flush()

    voltage_monitor = VoltageMonitor(navigator)
    voltage_monitor.start_monitoring()

    print("\n等待导航激活...")
    sys.stdout.flush()
    navigator.waitUntilNav2Active()
    print("✓ 导航已激活\n")
    sys.stdout.flush()

    waypoint_configs = {
        'A': [1.504, -0.070, 3.78],
        'B': [4.107, 0.001, -114.59],
        'C': [4.203, -1.941, -171.0],
        'D': [1.663, -2.058, 98.96]
    }

    cycle_waypoints = [
        create_pose(navigator, *waypoint_configs['A']),
        create_pose(navigator, *waypoint_configs['B']),
        create_pose(navigator, *waypoint_configs['C']),
        create_pose(navigator, *waypoint_configs['D']),
        create_pose(navigator, *waypoint_configs['A'])
    ]

    cycle_num = 1
    try:
        while True:
            if voltage_monitor.recharge_started:
                print(f"\n主程序检测到回充已启动，退出导航循环。")
                sys.stdout.flush()
                break
                
            success = execute_waypoint_navigation(navigator, cycle_waypoints, cycle_num, voltage_monitor)
            
            if voltage_monitor.recharge_started:
                break
            
            if success:
                print(f"\n等待 2 秒...")
                sys.stdout.flush()
                time.sleep(2)
                cycle_num += 1
            else:
                print(f"\n等待 5 秒...")
                sys.stdout.flush()
                time.sleep(5)

    except KeyboardInterrupt:
        print("\n[主程序] 手动停止")
        sys.stdout.flush()
    finally:
        # 停止回充进程
        voltage_monitor.stop_recharge_process()
        navigator.lifecycleShutdown()
        rclpy.shutdown()
        print("\n程序已退出")
        sys.stdout.flush()


if __name__ == '__main__':
    main()