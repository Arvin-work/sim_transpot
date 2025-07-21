import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import matplotlib.patches as patches
import seaborn as sns
from collections import defaultdict
import time
import matplotlib.font_manager as fm  # 添加字体管理模块

# 设置支持中文的字体
plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'WenQuanYi Micro Hei']  # 设置中文字体
plt.rcParams['axes.unicode_minus'] = False  # 正常显示负号

class TrafficLight:
    """交通信号灯类"""
    def __init__(self, cycle=60, green_ratio=0.5):
        """
        初始化交通信号灯
        :param cycle: 信号周期长度（秒）
        :param green_ratio: 绿灯时间比例
        """
        self.cycle = cycle
        self.green_duration = int(cycle * green_ratio)
        self.yellow_duration = 3  # 黄灯时间固定为3秒
        self.red_duration = cycle - self.green_duration - self.yellow_duration
        self.state = "green"  # 初始状态
        self.timer = 0  # 当前状态计时器
        
    def update(self, step):
        """更新信号灯状态"""
        self.timer += 1
        
        if self.state == "green" and self.timer >= self.green_duration:
            self.state = "yellow"
            self.timer = 0
        elif self.state == "yellow" and self.timer >= self.yellow_duration:
            self.state = "red"
            self.timer = 0
        elif self.state == "red" and self.timer >= self.red_duration:
            self.state = "green"
            self.timer = 0
            
    def get_state(self):
        """获取当前信号灯状态"""
        return self.state
    
    def get_remaining_time(self):
        """获取当前状态剩余时间"""
        if self.state == "green":
            return self.green_duration - self.timer
        elif self.state == "yellow":
            return self.yellow_duration - self.timer
        else:
            return self.red_duration - self.timer

class Vehicle:
    """车辆类"""
    next_id = 0
    
    def __init__(self, direction, max_speed=5, acceleration=1, deceleration=2):
        """
        初始化车辆
        :param direction: 行驶方向 ('north', 'south', 'east', 'west')
        :param max_speed: 最大速度
        :param acceleration: 加速度
        :param deceleration: 减速度
        """
        self.id = Vehicle.next_id
        Vehicle.next_id += 1
        self.direction = direction
        self.max_speed = max_speed
        self.speed = 0
        self.acceleration = acceleration
        self.deceleration = deceleration
        self.position = None
        self.path = []  # 车辆行驶路径
        self.wait_time = 0  # 累计等待时间
        self.entry_time = time.time()  # 进入系统时间
        self.exit_time = None  # 离开系统时间
        self.target_direction = self._get_target_direction()
        
    def _get_target_direction(self):
        """随机生成目标方向（直行、左转、右转）"""
        # 直行60%，左转25%，右转15%
        r = np.random.random()
        if r < 0.6:
            return "straight"
        elif r < 0.85:
            return "left"
        else:
            return "right"
            
    def update_speed(self, front_vehicle_dist, light_state, light_dist):
        """
        更新车辆速度
        :param front_vehicle_dist: 与前车距离
        :param light_state: 前方信号灯状态
        :param light_dist: 到信号灯距离
        """
        # 1. 加速
        if self.speed < self.max_speed:
            self.speed = min(self.speed + self.acceleration, self.max_speed)
            
        # 2. 减速（避免碰撞）
        safe_distance = max(2, self.speed * 1.5)  # 安全距离
        if front_vehicle_dist is not None and front_vehicle_dist < safe_distance:
            self.speed = max(0, min(self.speed, front_vehicle_dist - 1))
            
        # 3. 信号灯减速
        if light_state == "red" or light_state == "yellow":
            # 在安全距离外开始减速
            if light_dist < safe_distance * 2:
                # 计算安全减速距离
                decel_distance = (self.speed ** 2) / (2 * self.deceleration)
                if light_dist < decel_distance:
                    self.speed = max(0, self.speed - self.deceleration)
        
        # 4. 随机波动（模拟驾驶员行为）
        if np.random.random() < 0.1 and self.speed > 0:
            self.speed = max(0, self.speed - 1)
            
    def move(self):
        """根据速度移动车辆"""
        if self.position is not None:
            # 记录路径
            self.path.append((self.position[0], self.position[1], time.time()))
            
            # 根据方向移动
            if self.direction == "north":
                self.position = (self.position[0], self.position[1] + self.speed)
            elif self.direction == "south":
                self.position = (self.position[0], self.position[1] - self.speed)
            elif self.direction == "east":
                self.position = (self.position[0] + self.speed, self.position[1])
            elif self.direction == "west":
                self.position = (self.position[0] - self.speed, self.position[1])
                
        # 记录等待时间
        if self.speed == 0:
            self.wait_time += 1

class Intersection:
    """十字路口模拟系统"""
    def __init__(self, road_length=100, lane_width=10, spawn_rate=0.05):
        """
        初始化十字路口
        :param road_length: 道路长度
        :param lane_width: 车道宽度
        :param spawn_rate: 车辆生成率
        """
        self.road_length = road_length
        self.lane_width = lane_width
        self.spawn_rate = spawn_rate
        
        # 创建交通信号灯 (NS: 南北, EW: 东西)
        self.light_NS = TrafficLight(cycle=60, green_ratio=0.5)
        self.light_EW = TrafficLight(cycle=60, green_ratio=0.5)
        
        # 初始状态：南北绿灯，东西红灯
        self.light_NS.state = "green"
        self.light_EW.state = "red"
        
        # 车辆存储
        self.vehicles = []
        self.removed_vehicles = []
        
        # 统计数据
        self.stats = {
            "total_vehicles": 0,
            "avg_wait_time": 0,
            "avg_travel_time": 0,
            "throughput": 0,
            "queue_lengths": defaultdict(list)
        }
        
        # 道路边界
        self.road_boundaries = {
            "north": (road_length//2 - lane_width//2, road_length//2 + lane_width//2),
            "south": (road_length//2 - lane_width//2, road_length//2 + lane_width//2),
            "east": (road_length//2 - lane_width//2, road_length//2 + lane_width//2),
            "west": (road_length//2 - lane_width//2, road_length//2 + lane_width//2)
        }
        
        # 交叉口中心区域
        self.intersection_center = (road_length // 2, road_length // 2)
        self.intersection_size = lane_width * 1.5
        
    def spawn_vehicle(self):
        """生成新车辆"""
        if np.random.random() < self.spawn_rate:
            direction = np.random.choice(["north", "south", "east", "west"])
            
            # 设置初始位置
            if direction == "north":
                start_x = np.random.uniform(
                    self.road_boundaries["north"][0], 
                    self.road_boundaries["north"][1]
                )
                start_pos = (start_x, 0)
            elif direction == "south":
                start_x = np.random.uniform(
                    self.road_boundaries["south"][0], 
                    self.road_boundaries["south"][1]
                )
                start_pos = (start_x, self.road_length)
            elif direction == "east":
                start_y = np.random.uniform(
                    self.road_boundaries["east"][0], 
                    self.road_boundaries["east"][1]
                )
                start_pos = (0, start_y)
            else:  # west
                start_y = np.random.uniform(
                    self.road_boundaries["west"][0], 
                    self.road_boundaries["west"][1]
                )
                start_pos = (self.road_length, start_y)
                
            vehicle = Vehicle(direction)
            vehicle.position = start_pos
            self.vehicles.append(vehicle)
            self.stats["total_vehicles"] += 1
            
    def is_in_intersection(self, pos):
        """检查位置是否在交叉口区域内"""
        x, y = pos
        center_x, center_y = self.intersection_center
        size = self.intersection_size
        return (center_x - size <= x <= center_x + size and 
                center_y - size <= y <= center_y + size)
                
    def get_light_state(self, direction):
        """获取当前方向的信号灯状态"""
        if direction in ["north", "south"]:
            return self.light_NS.get_state()
        else:
            return self.light_EW.get_state()
            
    def get_distance_to_light(self, vehicle):
        """计算车辆到信号灯的距离"""
        x, y = vehicle.position
        
        if vehicle.direction == "north":
            return self.road_length - y - self.road_length//2
        elif vehicle.direction == "south":
            return y - self.road_length//2
        elif vehicle.direction == "east":
            return self.road_length - x - self.road_length//2
        else:  # west
            return x - self.road_length//2
            
    def check_collision(self, vehicle):
        """检查碰撞（简化版）"""
        for other in self.vehicles:
            if other.id != vehicle.id:
                dist = np.sqrt((vehicle.position[0]-other.position[0])**2 + 
                              (vehicle.position[1]-other.position[1])**2)
                if dist < 2:  # 碰撞阈值
                    return True
        return False
        
    def check_exit(self, vehicle):
        """检查车辆是否已离开系统"""
        x, y = vehicle.position
        if vehicle.direction == "north" and y > self.road_length:
            return True
        elif vehicle.direction == "south" and y < 0:
            return True
        elif vehicle.direction == "east" and x > self.road_length:
            return True
        elif vehicle.direction == "west" and x < 0:
            return True
        return False
        
    def get_front_vehicle_distance(self, vehicle):
        """获取与前车的距离"""
        min_dist = float('inf')
        for other in self.vehicles:
            if other.id != vehicle.id and other.direction == vehicle.direction:
                # 计算同方向车辆距离
                if vehicle.direction == "north":
                    dist = other.position[1] - vehicle.position[1]
                elif vehicle.direction == "south":
                    dist = vehicle.position[1] - other.position[1]
                elif vehicle.direction == "east":
                    dist = other.position[0] - vehicle.position[0]
                else:  # west
                    dist = vehicle.position[0] - other.position[0]
                    
                if dist > 0 and dist < min_dist:
                    min_dist = dist
                    
        return min_dist if min_dist != float('inf') else None
        
    def update(self, step):
        """更新整个系统状态"""
        # 更新信号灯
        self.light_NS.update(step)
        self.light_EW.update(step)
        
        # 生成新车辆
        self.spawn_vehicle()
        
        # 更新每辆车
        to_remove = []
        for i, vehicle in enumerate(self.vehicles):
            # 获取信号灯状态
            light_state = self.get_light_state(vehicle.direction)
            light_dist = self.get_distance_to_light(vehicle)
            
            # 获取前车距离
            front_dist = self.get_front_vehicle_distance(vehicle)
            
            # 更新速度
            vehicle.update_speed(front_dist, light_state, light_dist)
            
            # 移动车辆
            vehicle.move()
            
            # 检查碰撞
            if self.check_collision(vehicle):
                # 发生碰撞，移除车辆
                vehicle.exit_time = time.time()  # 设置退出时间
                to_remove.append(i)
                continue
                
            # 检查是否离开系统
            if self.check_exit(vehicle):
                vehicle.exit_time = time.time()
                to_remove.append(i)
                self.stats["throughput"] += 1
                # 记录行程时间
                travel_time = vehicle.exit_time - vehicle.entry_time
                self.stats["avg_travel_time"] = (
                    self.stats["avg_travel_time"] * (self.stats["throughput"] - 1) + travel_time
                ) / self.stats["throughput"]
                # 记录等待时间
                self.stats["avg_wait_time"] = (
                    self.stats["avg_wait_time"] * (self.stats["throughput"] - 1) + vehicle.wait_time
                ) / self.stats["throughput"]
        
        # 移除离开的车辆
        for i in sorted(to_remove, reverse=True):
            self.removed_vehicles.append(self.vehicles.pop(i))
            
        # 记录队列长度
        self._record_queue_lengths()
        
    def _record_queue_lengths(self):
        """记录各方向队列长度"""
        queues = {"north": 0, "south": 0, "east": 0, "west": 0}
        
        for vehicle in self.vehicles:
            # 只统计在信号灯前等待的车辆
            light_dist = self.get_distance_to_light(vehicle)
            if light_dist < 30 and vehicle.speed == 0:  # 在30单位内且停止
                queues[vehicle.direction] += 1
                
        for direction, length in queues.items():
            self.stats["queue_lengths"][direction].append(length)
            
    def get_efficiency_metrics(self):
        """获取效率指标"""
        return {
            "throughput": self.stats["throughput"],
            "avg_wait_time": self.stats["avg_wait_time"],
            "avg_travel_time": self.stats["avg_travel_time"],
            "queue_lengths": self.stats["queue_lengths"]
        }
        
    def visualize(self, steps=1000, interval=50):
        """可视化模拟过程"""
        fig, ax = plt.subplots(figsize=(10, 10))
        
        # 绘制道路
        road_color = '#D3D3D3'
        ax.add_patch(patches.Rectangle(
            (0, self.road_length//2 - self.lane_width//2), 
            self.road_length, 
            self.lane_width, 
            color=road_color
        ))
        ax.add_patch(patches.Rectangle(
            (self.road_length//2 - self.lane_width//2, 0), 
            self.lane_width, 
            self.road_length, 
            color=road_color
        ))
        
        # 绘制交叉口
        ax.add_patch(patches.Rectangle(
            (self.road_length//2 - self.intersection_size//2, 
             self.road_length//2 - self.intersection_size//2),
            self.intersection_size,
            self.intersection_size,
            color='#A9A9A9'
        ))
        
        # 设置坐标轴
        ax.set_xlim(0, self.road_length)
        ax.set_ylim(0, self.road_length)
        ax.set_aspect('equal')
        ax.set_title("十字路口交通流模拟")
        
        # 车辆点
        vehicle_dots = ax.scatter([], [], s=50, c='blue')
        
        # 信号灯指示器
        light_text = ax.text(
            self.road_length - 20, 
            self.road_length - 20, 
            "",
            fontsize=12,
            bbox=dict(facecolor='white', alpha=0.8)
        )
        
        # 效率指标
        metrics_text = ax.text(
            10, 
            self.road_length - 20, 
            "",
            fontsize=10,
            bbox=dict(facecolor='white', alpha=0.8)
        )
        
        def init():
            vehicle_dots.set_offsets(np.empty((0, 2)))
            return vehicle_dots,
        
        def update(frame):
            # 更新模拟
            self.update(frame)
            
            # 更新车辆位置
            positions = np.array([v.position for v in self.vehicles])
            if len(positions) > 0:
                vehicle_dots.set_offsets(positions)
            else:
                vehicle_dots.set_offsets(np.empty((0, 2)))
            
            # 更新信号灯状态
            ns_state = self.light_NS.get_state()
            ew_state = self.light_EW.get_state()
            light_info = (
                f"南北灯: {ns_state} ({self.light_NS.get_remaining_time()}s)\n"
                f"东西灯: {ew_state} ({self.light_EW.get_remaining_time()}s)"
            )
            light_text.set_text(light_info)
            
            # 更新效率指标
            metrics = self.get_efficiency_metrics()
            metrics_info = (
                f"通行量: {metrics['throughput']}\n"
                f"平均等待: {metrics['avg_wait_time']:.1f}步\n"
                f"平均行程: {metrics['avg_travel_time']:.1f}s"
            )
            metrics_text.set_text(metrics_info)
            
            return vehicle_dots, light_text, metrics_text
        
        ani = FuncAnimation(
            fig, 
            update, 
            frames=steps,
            init_func=init,
            interval=interval,
            blit=True
        )
        
        plt.tight_layout()
        plt.show()
        return ani
    
    def plot_metrics(self):
        """绘制效率指标图表并分别保存为独立文件"""
        metrics = self.get_efficiency_metrics()
        timestamp = time.strftime("%Y%m%d-%H%M%S")
    
        # 1. 队列长度时间序列图
        plt.figure(figsize=(10, 6))
        for direction, lengths in metrics["queue_lengths"].items():
            if lengths:  # 确保有数据
                plt.plot(lengths, label=f"{direction}方向", linewidth=2)
    
        if plt.gca().lines:  # 如果有线条才设置标题等
            plt.title("各方向队列长度变化", fontsize=14)
            plt.xlabel("时间步", fontsize=12)
            plt.ylabel("队列长度", fontsize=12)
            plt.legend(fontsize=10, loc='best', framealpha=0.7)
            plt.grid(True, linestyle='--', alpha=0.7)
            plt.xlim(left=0)
            plt.tight_layout()
        
            # 保存图像
            filename = f"queue_length_ts_{timestamp}.png"
            plt.savefig(filename, dpi=150, bbox_inches='tight')
            print(f"已保存队列长度时间序列图: {filename}")
        else:
            print("无队列长度数据，跳过队列长度时间序列图")
    
        plt.close()
    
        # 2. 平均队列长度柱状图
        directions = ["north", "south", "east", "west"]
        direction_names = ["北向", "南向", "东向", "西向"]
        wait_times = []
        for direction in directions:
            if metrics["queue_lengths"].get(direction) and metrics["queue_lengths"][direction]:
                avg_wait = np.mean(metrics["queue_lengths"][direction])
                wait_times.append(avg_wait)
            else:
                wait_times.append(0)  # 没有数据时设为0
    
        if any(wait_times):
            plt.figure(figsize=(8, 6))
            colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728']
            bars = plt.bar(direction_names, wait_times, color=colors)
            plt.title("各方向平均队列长度", fontsize=14)
            plt.ylabel("平均队列长度", fontsize=12)
            plt.grid(True, axis='y', linestyle='--', alpha=0.7)
        
            # 在柱子上方添加数值标签
            for bar in bars:
                height = bar.get_height()
                plt.annotate(f'{height:.1f}',
                            xy=(bar.get_x() + bar.get_width() / 2, height),
                            xytext=(0, 3),  # 3点垂直偏移
                            textcoords="offset points",
                            ha='center', va='bottom', fontsize=10)
        
            plt.tight_layout()
        
            # 保存图像
            filename = f"avg_queue_length_{timestamp}.png"
            plt.savefig(filename, dpi=150, bbox_inches='tight')
            print(f"已保存平均队列长度图: {filename}")
        else:
            print("无平均队列长度数据，跳过平均队列长度图")
    
        plt.close()
    
        # 3. 通行量分布饼图
        throughput_data = []
        for vehicle in self.removed_vehicles:
            if vehicle.exit_time:  # 确保有退出时间
                throughput_data.append(vehicle.direction)
    
        if throughput_data:  # 确保有数据
            plt.figure(figsize=(8, 6))
            # 统计各方向通行量
            counts = {d: throughput_data.count(d) for d in directions}
            colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728']
            wedges, texts, autotexts = plt.pie(
                [counts[d] for d in directions],
                labels=direction_names,
                colors=colors,
                autopct='%1.1f%%',
                startangle=90,
                textprops={'fontsize': 12}
            )
            plt.title("各方向通行量分布", fontsize=14)
            plt.axis('equal')  # 保证饼图是圆形
        
            # 添加图例
            plt.legend(wedges, direction_names,
                    title="方向",
                    loc="center left",
                    bbox_to_anchor=(1, 0, 0.5, 1),
                    fontsize=10)
        
            plt.tight_layout()
        
            # 保存图像
            filename = f"throughput_distribution_{timestamp}.png"
            plt.savefig(filename, dpi=150, bbox_inches='tight')
            print(f"已保存通行量分布图: {filename}")
        else:
            print("无通行量数据，跳过通行量分布图")
    
        plt.close()
    
        # 4. 行程时间分布直方图
        travel_times = [v.exit_time - v.entry_time for v in self.removed_vehicles if v.exit_time is not None]
        if travel_times:  # 确保有数据
            plt.figure(figsize=(10, 6))
            # 使用KDE曲线和直方图结合
            sns.histplot(travel_times, bins=20, kde=True, color='green', alpha=0.7)
        
            # 添加平均线
            mean_time = np.mean(travel_times)
            plt.axvline(mean_time, color='red', linestyle='--', linewidth=2)
            plt.text(mean_time*1.05, plt.gca().get_ylim()[1]*0.9, 
                    f'平均: {mean_time:.2f}s', 
                    color='red', fontsize=12)
        
            plt.title("车辆行程时间分布", fontsize=14)
            plt.xlabel("行程时间 (秒)", fontsize=12)
            plt.ylabel("车辆数", fontsize=12)
            plt.grid(True, linestyle='--', alpha=0.7)
            plt.tight_layout()
        
            # 保存图像
            filename = f"travel_time_distribution_{timestamp}.png"
            plt.savefig(filename, dpi=150, bbox_inches='tight')
            print(f"已保存行程时间分布图: {filename}")
        else:
            print("无行程时间数据，跳出行程时间分布图")
    
        plt.close()
    
        # 5. 可选：同时创建组合图表（如果需要）
        # 这里可以选择是否创建组合图表
        create_composite = True
        if create_composite:
        # 创建组合图表
            fig, axs = plt.subplots(2, 2, figsize=(16, 12))
            fig.suptitle("十字路口交通效率指标分析", fontsize=18, fontweight='bold', y=0.98)
        
            # 队列长度时间序列
            for direction, lengths in metrics["queue_lengths"].items():
                if lengths:  # 确保有数据
                    axs[0, 0].plot(lengths, label=f"{direction}方向", linewidth=2)
        
            if axs[0, 0].lines:  # 如果有线条才设置标题等
                axs[0, 0].set_title("各方向队列长度变化", fontsize=14)
                axs[0, 0].set_xlabel("时间步", fontsize=12)
                axs[0, 0].set_ylabel("队列长度", fontsize=12)
                axs[0, 0].legend(fontsize=10, loc='best', framealpha=0.7)
                axs[0, 0].grid(True, linestyle='--', alpha=0.7)
                axs[0, 0].set_xlim(left=0)
        
            # 平均队列长度
            if any(wait_times):
                colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728']
                bars = axs[0, 1].bar(direction_names, wait_times, color=colors)
                axs[0, 1].set_title("各方向平均队列长度", fontsize=14)
                axs[0, 1].set_ylabel("平均队列长度", fontsize=12)
                axs[0, 1].grid(True, axis='y', linestyle='--', alpha=0.7)
            
                # 在柱子上方添加数值标签
                for bar in bars:
                    height = bar.get_height()
                    axs[0, 1].annotate(f'{height:.1f}',
                                xy=(bar.get_x() + bar.get_width() / 2, height),
                                xytext=(0, 3),  # 3点垂直偏移
                                textcoords="offset points",
                                ha='center', va='bottom', fontsize=10)
        
            # 通行量分布
            if throughput_data:  # 确保有数据
                colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728']
                wedges, texts, autotexts = axs[1, 0].pie(
                    [counts[d] for d in directions],
                    labels=direction_names,
                    colors=colors,
                    autopct='%1.1f%%',
                    startangle=90,
                    textprops={'fontsize': 12}
                )
                axs[1, 0].set_title("各方向通行量分布", fontsize=14)
                axs[1, 0].axis('equal')  # 保证饼图是圆形
            
                # 添加图例
                axs[1, 0].legend(wedges, direction_names,
                        title="方向",
                        loc="center left",
                        bbox_to_anchor=(1, 0, 0.5, 1),
                        fontsize=10)
        
            # 行程时间分布
            if travel_times:  # 确保有数据
                sns.histplot(travel_times, bins=20, kde=True, color='green', alpha=0.7, ax=axs[1, 1])
            
                # 添加平均线
                mean_time = np.mean(travel_times)
                axs[1, 1].axvline(mean_time, color='red', linestyle='--', linewidth=2)
                axs[1, 1].text(mean_time*1.05, axs[1, 1].get_ylim()[1]*0.9, 
                        f'平均: {mean_time:.2f}s', 
                        color='red', fontsize=12)
            
                axs[1, 1].set_title("车辆行程时间分布", fontsize=14)
                axs[1, 1].set_xlabel("行程时间 (秒)", fontsize=12)
                axs[1, 1].set_ylabel("车辆数", fontsize=12)
                axs[1, 1].grid(True, linestyle='--', alpha=0.7)
        
            # 添加调整布局
            plt.tight_layout(rect=[0, 0, 1, 0.96])  # 为整体标题留出空间
            plt.subplots_adjust(hspace=0.3, wspace=0.3)  # 调整子图间距
        
            # 保存组合图表
            filename = f"traffic_metrics_composite_{timestamp}.png"
            plt.savefig(filename, dpi=150, bbox_inches='tight')
            print(f"已保存组合图表: {filename}")
            plt.close()
# 运行模拟
if __name__ == "__main__":
    # 创建十字路口模拟
    intersection = Intersection(
        road_length=200,
        lane_width=15,
        spawn_rate=0.50  # 车辆生成率
    )
    
    # 运行可视化模拟
    print("启动十字路口交通流模拟...")
    ani = intersection.visualize(steps=500, interval=100)
    
    # 模拟结束后绘制效率指标
    print("模拟结束，绘制效率指标...")
    intersection.plot_metrics()
    
    # 打印最终效率指标
    metrics = intersection.get_efficiency_metrics()
    print("\n最终效率指标:")
    print(f"总通行量: {metrics['throughput']} 辆车")
    print(f"平均等待时间: {metrics['avg_wait_time']:.2f} 步")
    print(f"平均行程时间: {metrics['avg_travel_time']:.2f} 秒")
    
    # 各方向平均队列长度
    for direction in ["north", "south", "east", "west"]:
        if metrics["queue_lengths"].get(direction) and metrics["queue_lengths"][direction]:
            avg_queue = np.mean(metrics["queue_lengths"][direction])
            print(f"{direction}方向平均队列长度: {avg_queue:.2f}")
        else:
            print(f"{direction}方向平均队列长度: 0.00")