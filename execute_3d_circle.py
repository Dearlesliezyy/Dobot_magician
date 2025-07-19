# version: Python3
from DobotEDU import *
import math
import numpy as np
import time

class Robot3DCircleController:
    def __init__(self, magician):
        """
        初始化机械臂3D圆弧控制器
        :param magician: 机械臂控制对象
        """
        self.magician = magician
        
    def get_current_pose(self):
        """获取当前末端执行器位姿"""
        pose = self.magician.get_pose()
        return {
            'x': pose['x'],
            'y': pose['y'], 
            'z': pose['z'],
            'r': pose['r']
        }
    
    def normalize_vector(self, vector):
        """向量归一化"""
        vector = np.array(vector)
        norm = np.linalg.norm(vector)
        if norm == 0:
            return vector
        return vector / norm
    
    def create_rotation_matrix(self, normal_vector, up_vector=None):
        """
        创建旋转矩阵，将XY平面旋转到指定法向量的平面
        :param normal_vector: 圆所在平面的法向量
        :param up_vector: 用于定义圆在平面内的"上"方向，如果为None则自动选择
        :return: 3x3旋转矩阵
        """
        # 归一化法向量
        n = self.normalize_vector(normal_vector)
        
        # 如果没有指定up_vector，自动选择一个
        if up_vector is None:
            # 选择与法向量最不平行的坐标轴
            if abs(n[0]) < 0.9:
                up_vector = [1, 0, 0]
            else:
                up_vector = [0, 1, 0]
        
        up_vector = np.array(up_vector)
        
        # 创建正交基
        # u向量：在圆平面内，类似于局部X轴
        u = up_vector - np.dot(up_vector, n) * n
        u = self.normalize_vector(u)
        
        # v向量：在圆平面内，类似于局部Y轴
        v = np.cross(n, u)
        v = self.normalize_vector(v)
        
        # 构建旋转矩阵（列向量形式）
        rotation_matrix = np.column_stack([u, v, n])
        
        return rotation_matrix
    
    def calculate_3d_circle_points(self, center, radius, normal_vector, 
                                   num_points=36, start_angle=0, end_angle=360,
                                   up_vector=None):
        """
        计算3D空间中任意方向圆的轨迹点
        :param center: 圆心坐标 [x, y, z]
        :param radius: 圆的半径
        :param normal_vector: 圆所在平面的法向量 [x, y, z]
        :param num_points: 轨迹点数量
        :param start_angle: 起始角度（度）
        :param end_angle: 结束角度（度）
        :param up_vector: 定义圆在平面内的"上"方向 [x, y, z]
        :return: 轨迹点列表
        """
        center = np.array(center)
        
        # 创建旋转矩阵
        rotation_matrix = self.create_rotation_matrix(normal_vector, up_vector)
        
        # 生成角度序列
        if num_points == 1:
            angles = [start_angle]
        else:
            angle_step = (end_angle - start_angle) / (num_points - 1)
            angles = [start_angle + i * angle_step for i in range(num_points)]
        
        # 计算轨迹点
        trajectory_points = []
        current_r = self.get_current_pose()['r']  # 保持当前旋转角度
        
        for angle in angles:
            # 在标准XY平面上计算点
            angle_rad = math.radians(angle)
            local_x = radius * math.cos(angle_rad)
            local_y = radius * math.sin(angle_rad)
            local_z = 0
            
            # 局部坐标点
            local_point = np.array([local_x, local_y, local_z])
            
            # 通过旋转矩阵转换到全局坐标
            global_point = rotation_matrix @ local_point + center
            
            trajectory_points.append({
                'x': float(global_point[0]),
                'y': float(global_point[1]),
                'z': float(global_point[2]),
                'r': current_r
            })
            
        return trajectory_points
    
    def execute_3d_circle(self, center, radius, normal_vector, 
                          num_points=36, start_angle=0, end_angle=360,
                          up_vector=None, delay=0.1, mode=1):
        """
        执行3D空间中任意方向的圆弧轨迹运动
        :param center: 圆心坐标 [x, y, z]
        :param radius: 圆的半径
        :param normal_vector: 圆所在平面的法向量 [x, y, z]
        :param num_points: 轨迹点数量
        :param start_angle: 起始角度（度）
        :param end_angle: 结束角度（度）
        :param up_vector: 定义圆在平面内的"上"方向 [x, y, z]
        :param delay: 每个点之间的延时（秒）
        :param mode: 运动模式（0: JUMP, 1: MOVJ, 2: MOVL）
        """
        
        print("开始执行3D空间圆弧轨迹运动...")
        print(f"圆心: {center}")
        print(f"半径: {radius} mm")
        print(f"法向量: {normal_vector}")
        print(f"角度范围: {start_angle}° 到 {end_angle}°")
        
        # 计算轨迹点
        trajectory_points = self.calculate_3d_circle_points(
            center, radius, normal_vector, num_points, 
            start_angle, end_angle, up_vector
        )
        
        print(f"生成了 {len(trajectory_points)} 个轨迹点")
        
        # 逐个执行轨迹点
        for i, point in enumerate(trajectory_points):
            print(f"移动到第 {i+1}/{len(trajectory_points)} 个点: "
                  f"x={point['x']:.2f}, y={point['y']:.2f}, z={point['z']:.2f}")
            
            # 移动到目标点
            self.magician.ptp(mode=mode,
                            x=point['x'], 
                            y=point['y'], 
                            z=point['z'], 
                            r=point['r'])
            
            # 等待指定时间
            time.sleep(delay)
            
        print("3D圆弧轨迹运动完成！")
    
    def execute_circle_from_current_to_point(self, target_point, num_points=36, 
                                           plane_normal=None, delay=0.1):
        """
        从当前位置到目标点，以两点连线为直径绘制半圆
        :param target_point: 目标点坐标 [x, y, z]
        :param num_points: 轨迹点数量
        :param plane_normal: 圆所在平面的法向量，如果为None则自动计算
        :param delay: 每个点之间的延时（秒）
        """
        
        current_pose = self.get_current_pose()
        current_point = np.array([current_pose['x'], current_pose['y'], current_pose['z']])
        target_point = np.array(target_point)
        
        # 计算圆心（两点的中点）
        center = (current_point + target_point) / 2
        
        # 计算半径（两点距离的一半）
        radius = np.linalg.norm(target_point - current_point) / 2
        
        # 如果没有指定平面法向量，选择一个垂直于连线的方向
        if plane_normal is None:
            line_vector = target_point - current_point
            # 选择与连线向量最不平行的坐标轴来构造法向量
            if abs(line_vector[2]) < 0.9 * np.linalg.norm(line_vector):
                plane_normal = np.cross(line_vector, [0, 0, 1])
            else:
                plane_normal = np.cross(line_vector, [1, 0, 0])
        
        print(f"从当前位置 {current_point} 到目标位置 {target_point}")
        print(f"圆心: {center}, 半径: {radius:.2f} mm")
        
        # 执行半圆轨迹
        self.execute_3d_circle(center, radius, plane_normal, 
                             num_points, 0, 180, delay=delay)

# 使用示例
magician.jump_params(100, 2)  # 设置门字型运动参数
circle_3d = Robot3DCircleController(magician)

# 移动到初始位置
magician.ptp(mode=1, x=200, y=0, z=50, r=0)
time.sleep(1)

print("当前位姿:", magician.get_pose())

# 示例1：在XY平面画圆（法向量为Z轴方向）
print("\n=== 示例1：在XY平面画圆 ===")
circle_3d.execute_3d_circle(
    center=[200, 0, 50],        # 圆心
    radius=30,                   # 半径
    normal_vector=[0, 0, 1],    # Z轴方向（XY平面）
    num_points=20,
    start_angle=0,
    end_angle=360,
    delay=0.1
)

# 示例2：在XZ平面画圆（法向量为Y轴方向）
print("\n=== 示例2：在XZ平面画半圆 ===")
circle_3d.execute_3d_circle(
    center=[200, 0, 50],        # 圆心
    radius=25,                   # 半径
    normal_vector=[0, 1, 0],    # Y轴方向（XZ平面）
    num_points=15,
    start_angle=0,
    end_angle=180,              # 半圆
    delay=0.15
)

# 示例3：在YZ平面画圆（法向量为X轴方向）
print("\n=== 示例3：在YZ平面画1/4圆 ===")
circle_3d.execute_3d_circle(
    center=[200, 0, 50],        # 圆心
    radius=20,                   # 半径
    normal_vector=[1, 0, 0],    # X轴方向（YZ平面）
    num_points=10,
    start_angle=0,
    end_angle=90,               # 1/4圆
    delay=0.2
)

# 示例4：在倾斜平面画圆
print("\n=== 示例4：在倾斜平面画圆 ===")
circle_3d.execute_3d_circle(
    center=[200, 0, 50],           # 圆心
    radius=35,                      # 半径
    normal_vector=[1, 1, 1],       # 倾斜法向量
    num_points=25,
    start_angle=0,
    end_angle=360,
    up_vector=[0, 0, 1],           # 指定"上"方向
    delay=0.1
)

# 示例5：从当前位置到指定点画半圆轨迹
print("\n=== 示例5：从当前位置到目标点的半圆轨迹 ===")
# circle_3d.execute_circle_from_current_to_point(
#     target_point=[250, 50, 80],  # 目标点
#     num_points=20,
#     plane_normal=[0, 0, 1],      # 在XY平面的上方
#     delay=0.1
# )