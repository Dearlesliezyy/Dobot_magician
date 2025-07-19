# version: Python3
from DobotEDU import *
import math
import time

class RobotArcController:
    def __init__(self, magician):
        """
        初始化机械臂圆弧控制器
        :param magician: 机械臂控制对象
        """
        self.magician = magician
        self.current_pose = {'x': 0, 'y': -200, 'z': -30, 'r': 0}
        
    def get_current_pose(self):
        """获取当前末端执行器位姿"""
        pose = self.magician.get_pose()
        return {
            'x': pose['x'],
            'y': pose['y'], 
            'z': pose['z'],
            'r': pose['r']
        }
        
    def calculate_arc_points(self, center_x=0, center_y=0, center_z=0, 
                           radius=None, num_points=36, start_angle=0, end_angle=360,
                           start_z=None, end_z=None, z_variation_type='linear'):
        """
        计算圆弧轨迹点
        :param center_x: 圆心X坐标
        :param center_y: 圆心Y坐标  
        :param center_z: 圆心Z坐标
        :param radius: 圆弧半径，如果为None则根据当前位置计算
        :param num_points: 轨迹点数量
        :param start_angle: 起始角度（度）
        :param end_angle: 结束角度（度）
        :param start_z: 起始Z坐标，如果为None则使用当前Z坐标
        :param end_z: 结束Z坐标，如果为None则使用当前Z坐标
        :param z_variation_type: Z轴变化类型 ('linear', 'sine', 'cosine')
        :return: 轨迹点列表
        """
        
        # 如果未指定半径，根据当前位置计算
        if radius is None:
            dx = self.current_pose['x'] - center_x
            dy = self.current_pose['y'] - center_y
            radius = math.sqrt(dx**2 + dy**2)
            print(f"计算得出的圆弧半径: {radius:.2f} mm")
        
        # 设置Z轴起始和结束值
        if start_z is None:
            start_z = center_z + self.current_pose['z']
        else:
            start_z = center_z + start_z
            
        if end_z is None:
            end_z = center_z + self.current_pose['z']
        else:
            end_z = center_z + end_z
        
        # 计算当前位置的起始角度
        current_dx = self.current_pose['x'] - center_x
        current_dy = self.current_pose['y'] - center_y
        current_angle = math.degrees(math.atan2(current_dy, current_dx))
        
        # 生成角度序列
        angle_step = (end_angle - start_angle) / (num_points - 1)
        angles = [start_angle + i * angle_step for i in range(num_points)]
        
        # 计算轨迹点
        trajectory_points = []
        
        for i, angle in enumerate(angles):
            # 将角度转换为弧度
            angle_rad = math.radians(angle)
            
            # 计算圆弧上的点(x轴向下，y轴向右)
            x = center_x + radius * math.sin(angle_rad)
            y = center_y + radius * (-math.cos(angle_rad))
            
            # 计算Z轴坐标（根据变化类型）
            progress = i / (num_points - 1)  # 轨迹进度 (0到1)
            
            if z_variation_type == 'linear':
                # 线性变化
                z = start_z + (end_z - start_z) * progress
            elif z_variation_type == 'sine':
                # 正弦波变化
                z = start_z + (end_z - start_z) * math.sin(progress * math.pi)
            elif z_variation_type == 'cosine':
                # 余弦波变化（从高到低再到高）
                z = start_z + (end_z - start_z) * (1 - math.cos(progress * math.pi)) / 2
            else:
                # 默认线性变化
                z = start_z + (end_z - start_z) * progress
            
            r = self.current_pose['r']  # 保持当前旋转角度
            
            trajectory_points.append({'x': x, 'y': y, 'z': z, 'r': r})
            
        return trajectory_points
    
    def calculate_circle_points_from_current_pos(self, radius, num_points=36, 
                                               start_angle=0, end_angle=360,
                                               z_offset=0, z_variation_type='linear'):
        """
        以当前末端执行器位置为圆心计算圆弧轨迹点
        :param radius: 圆弧半径（必须指定）
        :param num_points: 轨迹点数量
        :param start_angle: 起始角度（度）
        :param end_angle: 结束角度（度）
        :param z_offset: Z轴相对于圆心的偏移量变化范围
        :param z_variation_type: Z轴变化类型 ('linear', 'sine', 'cosine', 'none')
        :return: 轨迹点列表
        """
        
        # 获取当前位置作为圆心
        current_pose = self.get_current_pose()
        center_x = current_pose['x']
        center_y = current_pose['y']
        center_z = current_pose['z']
        current_r = current_pose['r']
        
        print(f"以当前位置为圆心: ({center_x:.2f}, {center_y:.2f}, {center_z:.2f})")
        print(f"圆弧半径: {radius} mm")
        
        # 生成角度序列
        angle_step = (end_angle - start_angle) / (num_points - 1)
        angles = [start_angle + i * angle_step for i in range(num_points)]
        
        # 计算轨迹点
        trajectory_points = []
        
        for i, angle in enumerate(angles):
            # 将角度转换为弧度
            angle_rad = math.radians(angle)
            
            # 计算圆弧上的点
            x = center_x + radius * math.cos(angle_rad)
            y = center_y + radius * math.sin(angle_rad)
            
            # 计算Z轴坐标（根据变化类型）
            progress = i / (num_points - 1) if num_points > 1 else 0  # 轨迹进度 (0到1)
            
            if z_variation_type == 'linear':
                # 线性变化
                z = center_z + z_offset * (2 * progress - 1)  # 从-z_offset到+z_offset
            elif z_variation_type == 'sine':
                # 正弦波变化
                z = center_z + z_offset * math.sin(progress * 2 * math.pi)
            elif z_variation_type == 'cosine':
                # 余弦波变化
                z = center_z + z_offset * math.cos(progress * 2 * math.pi)
            else:  # 'none' or any other value
                # 无Z轴变化
                z = center_z
            
            trajectory_points.append({'x': x, 'y': y, 'z': z, 'r': current_r})
            
        return trajectory_points
    
    def execute_arc_trajectory(self, center_x=0, center_y=0, center_z=0,
                             radius=None, num_points=36, start_angle=0, 
                             end_angle=360, start_z=None, end_z=None,
                             z_variation_type='linear', delay=0.5):
        """
        执行圆弧轨迹运动（以指定圆心）
        """
        
        print("开始执行圆弧轨迹运动...")
        print(f"当前位置: x={self.current_pose['x']}, y={self.current_pose['y']}, z={self.current_pose['z']}")
        print(f"圆心坐标: ({center_x}, {center_y}, {center_z})")
        print(f"Z轴变化: 从 {start_z} 到 {end_z} ({z_variation_type})")
        
        # 计算轨迹点
        trajectory_points = self.calculate_arc_points(
            center_x, center_y, center_z, radius, num_points, 
            start_angle, end_angle, start_z, end_z, z_variation_type
        )
        
        print(f"生成了 {len(trajectory_points)} 个轨迹点")
        
        # 逐个执行轨迹点
        for i, point in enumerate(trajectory_points):
            print(f"移动到第 {i+1}/{len(trajectory_points)} 个点: "
                  f"x={point['x']:.2f}, y={point['y']:.2f}, z={point['z']:.2f}, r={point['r']:.2f}")
            
            # 使用JUMP模式移动到目标点
            self.magician.ptp(mode=0, 
                            x=point['x'], 
                            y=point['y'], 
                            z=point['z'], 
                            r=point['r'])
            
            # 等待指定时间
            time.sleep(delay)
            
        print("圆弧轨迹运动完成！")
    
    def execute_circle_from_current_pos(self, radius, num_points=36, 
                                      start_angle=0, end_angle=360,
                                      z_offset=0, z_variation_type='none', 
                                      delay=0.1):
        """
        以当前末端执行器位置为圆心执行圆弧轨迹运动
        :param radius: 圆弧半径（必须指定）
        :param num_points: 轨迹点数量
        :param start_angle: 起始角度（度）
        :param end_angle: 结束角度（度）
        :param z_offset: Z轴相对于圆心的最大偏移量
        :param z_variation_type: Z轴变化类型 ('linear', 'sine', 'cosine', 'none')
        :param delay: 每个点之间的延时（秒）
        """
        
        print("开始以当前位置为圆心执行圆弧轨迹运动...")
        
        # 获取当前位姿
        current_pose = self.get_current_pose()
        print(f"圆心位置（当前位置）: x={current_pose['x']:.2f}, y={current_pose['y']:.2f}, z={current_pose['z']:.2f}")
        
        # 计算轨迹点
        trajectory_points = self.calculate_circle_points_from_current_pos(
            radius, num_points, start_angle, end_angle, z_offset, z_variation_type
        )
        
        print(f"生成了 {len(trajectory_points)} 个轨迹点")
        print(f"半径: {radius} mm, 角度范围: {start_angle}° 到 {end_angle}°")
        
        # 逐个执行轨迹点
        for i, point in enumerate(trajectory_points):
            print(f"移动到第 {i+1}/{len(trajectory_points)} 个点: "
                  f"x={point['x']:.2f}, y={point['y']:.2f}, z={point['z']:.2f}, r={point['r']:.2f}")
            
            # 使用PTP模式移动到目标点
            self.magician.ptp(mode=1,  # 使用PTP模式
                            x=point['x'], 
                            y=point['y'], 
                            z=point['z'], 
                            r=point['r'])
            
            # 等待指定时间
            time.sleep(delay)
            
        print("以当前位置为圆心的圆弧轨迹运动完成！")

# 使用示例
magician.jump_params(100, 2)  # 设置门字型运动参数 zlimit, height
arc_controller = RobotArcController(magician)

# 移动到初始位置
magician.ptp(mode=1, x=200, y=0, z=30, r=0)
time.sleep(1)

print("当前位姿:", magician.get_pose())

# 新功能：以当前末端执行器位置为圆心画圆
print("\n=== 示例1：以当前位置为圆心画完整的圆 ===")
arc_controller.execute_circle_from_current_pos(
    radius=20,           # 半径50mm
    num_points=50,       # 36个点
    start_angle=0,       # 从0度开始
    end_angle=360,       # 到360度结束（完整圆）
    z_offset=0,          # Z轴无变化
    z_variation_type='none',
    delay=0.0
)

# 示例2：以当前位置为圆心画半圆，带Z轴变化
print("\n=== 示例2：以当前位置为圆心画半圆，带正弦波Z轴变化 ===")
# arc_controller.execute_circle_from_current_pos(
#     radius=70,           # 半径70mm
#     num_points=20,       # 20个点
#     start_angle=0,       # 从0度开始
#     end_angle=180,       # 到180度结束（半圆）
#     z_offset=20,         # Z轴最大偏移±20mm
#     z_variation_type='sine',  # 正弦波变化
#     delay=0.2
# )

# 示例3：以当前位置为圆心画1/4圆弧
print("\n=== 示例3：以当前位置为圆心画1/4圆弧 ===")
# arc_controller.execute_circle_from_current_pos(
#     radius=80,           # 半径80mm
#     num_points=15,       # 15个点
#     start_angle=0,       # 从0度开始
#     end_angle=90,        # 到90度结束（1/4圆）
#     z_offset=0,          # Z轴无变化
#     z_variation_type='none',
#     delay=0.15
# )

# 原有功能仍然可用：以指定位置为圆心画圆弧
print("\n=== 原有功能：以底座为圆心画圆弧 ===")
# arc_controller.execute_arc_trajectory(
#     center_x=0, center_y=0, center_z=0,
#     radius=200,
#     num_points=20,
#     start_angle=0,
#     end_angle=90,
#     start_z=-35,
#     end_z=-35,
#     delay=0.1
# )