import sys
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import seaborn as sns
from collections import defaultdict
import time

class TrafficLight:
    # 创建交通信号灯
    def __init__(self, cycle=60, green_ratio=0.5):
        # 初始化交通信号灯(这些都是默认配置，后期调用可以更改)
        # cycle: 控制信号周期长度
        # param green_ratio: 绿灯时间比例
        self.cycle = cycle
        self.green_ratio = green_ratio
        self.green_duration = int (cycle * green_ratio)
        self.yellow_duration = 3 # 这个是黄灯时间，后期可以尝试使用其他方法完成调控
        # 设置红灯时间
        self.red_duration = cycle-self.yellow_duration-self.green_duration
        # 初始状态
        self.state = "green"
        # 初始计时器
        self.timer = 0

    # 编写如歌更改信号灯状态方式
    def update(self, step):
        self.timer += 1

        # 在绿灯状态需要结束的时候
        if self.state == "green" and self.timer >= self.green_duration:
            self.state = "yellow"
            self.timer = 0

        # 在黄灯状态结束时
        if self.state == "yellow" and self.timer >= self.yellow_duration:
            self.state = "red"
            self.timer = 0

        # 在红灯状态结束时
        if self.state == "red" and self.timer >= self.red_duration:
            self.state = "green"
            self.timer = 0

    # 获取当前信号灯状态
    def get_state(self):
        return self.state

    # 获取信号剩余时间
    def get_remaining_time(self):
        if self.state == "green":
            return self.green_duration-self.timer
        elif self.state == "yellow":
            return self.yellow_duration-self.timer
        else:
            return self.red_duration-self.timer

# 设置车辆行为
class Vehicle:
    next_id = 0
    def __init__(self, direction, max_speed=5, acceleration=1, deceleration=2):
        # 汽车id号生成
        self.id = Vehicle.next_id
        Vehicle.next_id += 1
        self.direction = direction  # 汽车驾驶方向确定
        self.max_speed = max_speed  # 汽车最大形式速度
        self.speed = 0 # 初始化汽车速度，设置为0
        self.acceleration = acceleration  # 加速度
        self.deceleration = deceleration  # 减速度
        self.position = None #
        self.path = [] # 车辆行驶路径
        self.wait_time = 0 # 累计等待时间
        self.entry_time = time.time() # 进入系统时间
        self.exit_time = None # 离开系统时间
        self.target_direction = self._get_target_direction() # 确定车辆的行驶方向

    def _get_target_direction(self):
        # 设置汽车的形式方向

        r = np.random.random()
        if r < 0.6:
            return "straight"
        elif r < 0.85:
            return "left"
        else:
            return "right"

    # 实时更新车辆的行驶速度
    def update_speed(self, front_vehicle_dist, light_state, light_dist):
        # front_vehicle_dist: 和牵扯距离
        # light_state: 前方信号等状态
        # light_dist: 到前方信号灯的距离

        # 1. 加速策略
        if self.speed < self.max_speed:
            self.speed = min(self.speed+self.max_speed, self.max_speed)

        # 2. 减速策略（对车子而言）
        safe_distance = max(2, int(self.speed * 1.5)) # 计算相关的安全距离
        if front_vehicle_dist is not None and front_vehicle_dist < safe_distance:
            self.speed = max(0, min(self.speed, front_vehicle_dist-1))

        # 3. 遇到信号灯减速策略
        if light_state == "red" or light_state == "yellow":
            # 根据安全距离提起进行减速
            if light_dist < safe_distance * 2:
                # 计算安全减速距离
                decel_distance = (self.speed ** 2) / (2 * self.deceleration)
                if light_dist < decel_distance:
                    self.speed = max(0, self.speed - self.deceleration)
        
    def move(self):
        #
        if self.position is not None:
            # 记录路径
            self.path.append((self.position[0], self.position[1], time.time()))

            # 根据方向移动
            if self.direction == "north":
                self.position = (self.position[0], self.position[1]+self.speed)

            elif self.direction == "south":
                self.position = (self.position[0], self.position[1]-self.speed)

            elif self.direction == "east":
                self.position = (self.position[0]+self.speed, self.position[1])
            
            elif self.direction == "west":
                self.position = (self.position[0]-self.speed, self.position[1])

        # 记录等待时间
        if self.speed == 0:
            self.wait_time += 1

# 十字路口模拟系统
class Intersection:
    def __init__(self, road_length=100, lane_width=10, spawn_rate=0.05):
        pass