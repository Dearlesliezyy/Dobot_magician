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
    
    def execute_arc_trajectory(self, center_x=0, center_y=0, center_z=0,
                             radius=None, num_points=36, start_angle=0, 
                             end_angle=360, start_z=None, end_z=None,
                             z_variation_type='linear', delay=0.5):
        """
        执行圆弧轨迹运动
        :param center_x: 圆心X坐标
        :param center_y: 圆心Y坐标
        :param center_z: 圆心Z坐标
        :param radius: 圆弧半径
        :param num_points: 轨迹点数量
        :param start_angle: 起始角度（度）
        :param end_angle: 结束角度（度）
        :param start_z: 起始Z坐标
        :param end_z: 结束Z坐标
        :param z_variation_type: Z轴变化类型 ('linear', 'sine', 'cosine')
        :param delay: 每个点之间的延时（秒）
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

# 使用
magician.jump_params(100, 2) # 设置门字型运动参数 zlimit, height
arc_controller = RobotArcController(magician)
magician.ptp(mode=1, x=0, y=-200, z=-30, r=0)# 移动到初始位置

# 示例1：Z轴无变化 可设置参数：半径 轨迹点数 开始及终止角度 末端执行器z坐标
arc_controller.execute_arc_trajectory(
    center_x=0, center_y=0, center_z=0,
    radius=200,
    num_points=50,
    start_angle=0,
    end_angle=90,
    start_z=-35,  # 起始Z坐标
    end_z=-35,     # 结束Z坐标
    delay=0
)

# 示例2：线性Z轴变化 - 从-30mm线性上升到10mm
# arc_controller.execute_arc_trajectory(
#    center_x=0, center_y=0, center_z=0,
#    radius=200,
#    num_points=30,
#    start_angle=0,
#    end_angle=90,
#    start_z=-30,  # 起始Z坐标
#    end_z=10,     # 结束Z坐标
#    z_variation_type='linear',  # 线性变化
#    delay=0.1
#)

# 示例3：正弦波Z轴变化
# arc_controller.execute_arc_trajectory(
#     center_x=0, center_y=0, center_z=0,
#     radius=200,
#     num_points=50,
#     start_angle=0,
#     end_angle=180,
#     start_z=-40,
#     end_z=20,
#     z_variation_type='sine',  # 正弦波变化
#     delay=0.1
# )

# 示例4：余弦波Z轴变化
# arc_controller.execute_arc_trajectory(
#     center_x=0, center_y=0, center_z=0,
#     radius=150,
#     num_points=36,
#     start_angle=0,
#     end_angle=360,
#     start_z=-20,
#     end_z=-50,
#     z_variation_type='cosine',  # 余弦波变化
#     delay=0.1
# )