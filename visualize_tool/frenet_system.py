import math

import numpy as np
from scipy.interpolate import CubicSpline


def calc_distance(point1, point2):
    # 计算两点之间的欧几里德距离
    return math.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)
def nearest_point(points, test_point):
    # 使用 min 函数和 distance 函数找到最近的点
    nearest_index = min(range(len(points)), key=lambda i: calc_distance(points[i], test_point))
    return nearest_index

class FrenetSystem:
    def __init__(self, vertices):
        self.vertices = np.array(vertices)
        self.vertices_size = len(self.vertices)
        self.start_vertex = self.vertices[0]
        self.end_vertex = self.vertices[-1]

        # 将 xy 分别与 s 建立五次多项式方程
        self.x_values = self.vertices[:, 0]
        self.y_values = self.vertices[:, 1]
        self.s_values = self.generate_s_values()

        self.s_x_interpolator = CubicSpline(self.s_values, self.x_values, bc_type='natural')
        self.s_y_interpolator = CubicSpline(self.s_values, self.y_values, bc_type='natural')

    def reference_path(self):
        return self.vertices

    def generate_s_values(self):
        s_values = [0]
        total_distance = 0
        for i in range(self.vertices_size - 1):
            current_vertex = self.vertices[i]
            next_vertex = self.vertices[i + 1]
            current_distance = np.linalg.norm(current_vertex - next_vertex)
            total_distance += current_distance
            s_values.append(total_distance)

        return np.array(s_values)


    def get_k_with_s_in_xy(self, s_value):
        dx_divide_ds = self.s_x_interpolator.derivative()(s_value)
        dy_divide_ds = self.s_y_interpolator.derivative()(s_value)
        dy_divide_dx = dy_divide_ds / dx_divide_ds
        return dy_divide_dx

    def find_point_r(self, input_point, step=1.0, tolerance=1e-4, max_iterations=5000):
        nearest_index = nearest_point(self.vertices, input_point)
        s_current = self.s_values[nearest_index]
        step_direction = 1
        best_distance = float('inf')
        current_iterations = 0

        # Function to minimize (distance between the normal line and input_point)
        def distance_function(s):
            x = self.s_x_interpolator(s)
            y = self.s_y_interpolator(s)
            curve_point = np.array([x, y])
            k = self.get_k_with_s_in_xy(s)
            c = -(curve_point[0] + k * curve_point[1])
            a = 1
            b = k
            distance = abs(a * input_point[0] + b * input_point[1] + c) / math.sqrt(a * a + b * b)
            return distance, k

        while best_distance > tolerance and current_iterations < max_iterations:
            s_next = s_current + step_direction * step
            distance_current, _ = distance_function(s_current)
            distance_next, _ = distance_function(s_next)

            if distance_next < distance_current:
                s_current = s_next
                best_distance = distance_next
            else:
                step_direction *= -1
                step /= 2
                best_distance = distance_current

            current_iterations += 1
        x_current = self.s_x_interpolator(s_current)
        y_current = self.s_y_interpolator(s_current)
        return np.array([x_current, y_current]), s_current

    def cartesian2ds_frame(self, input_point):
        best_point, s = self.find_point_r(input_point)
        abs_dist = calc_distance(input_point, best_point)
        s_next = s + 0.1
        x0 = self.s_x_interpolator(s)
        y0 = self.s_y_interpolator(s)
        x1 = self.s_x_interpolator(s_next)
        y1 = self.s_y_interpolator(s_next)
        x2, y2 = input_point
        vector1 = np.array([x1 - x0, y1 - y0])
        vector2 = np.array([x2 - x0, y2 - y0])
        sign = np.sign(np.cross(vector1, vector2))
        d_value = sign * abs_dist

        return np.array([round(d_value, 7), round(s, 7)]), best_point

    def ds_frame2cartesian(self, input_point):
        d_value, s_value = input_point
        s_next = s_value + 0.1
        x0 = self.s_x_interpolator(s_value)
        y0 = self.s_y_interpolator(s_value)
        x1 = self.s_x_interpolator(s_next)
        y1 = self.s_y_interpolator(s_next)
        vector1 = np.array([x1 - x0, y1 - y0])

        # vector1 逆时针旋转 90 度得到法线方向
        vector_norm = np.array([-vector1[1], vector1[0]])

        # 将 vector_norm 标准化
        vector_norm = vector_norm / np.linalg.norm(vector_norm)

        # 使用 d_value 确定最终的目标点
        # 由于 vector_norm 已经是单位向量，直接乘以 d_value 来得到沿法线方向的偏移
        dx = vector_norm[0] * d_value
        dy = vector_norm[1] * d_value

        # 计算并返回最终目标点的坐标
        x_result = x0 + dx
        y_result = y0 + dy

        return np.array([x_result, y_result])