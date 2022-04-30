"""

   Follow-Trajectory Simulation for Differential Drive Robot with Python
   
   author: Abdullah DANGAC (@abdullahdangac)
   
   Created 21.10.2021
   
"""

import math
import time
import pygame

# delta t
dt = 0.04  # recommended dt for pygame

# === ENVIRONMENT PROPERTIES (INITIAL VALUES) ===
# map (window) dimensions
map_width = 1400
map_height = 750

# === ROBOT PROPERTIES (INITIAL VALUES) ===
# start position of the robot (pose)
start_x = 300
start_y = 700
start_theta = math.pi

# follow distance between robot and target
follow_distance = 100

# PID-Controller Gains
Kp_v = 0.5  # P-Control gain for linear velocity
Ki_v = 0.001  # I-Control gain for linear velocity
Kd_v = 0.1  # D-Control gain for linear velocity
Kp_w = 1  # P-Control gain for angular velocity


class Environment:
    def __init__(self, window_width, window_height):

        # colors
        self.black = (0, 0, 0)
        self.white = (255, 255, 255)
        self.green = (0, 255, 0)
        self.red = (255, 0, 0)

        # window (map) dimensions
        self.width = window_width
        self.height = window_height

        # window settings
        pygame.display.set_caption('Differential Drive Robot')
        self.map = pygame.display.set_mode((self.width, self.height))

        # text variables
        self.font = pygame.font.Font('freesansbold.ttf', 30)
        self.text = self.font.render('default', True, self.white, self.black)
        self.textRect = self.text.get_rect()

        # trails
        self.trail_set_target = []
        self.trail_set_robot = []

    def trail(self, pose_x, pose_y, trail_set, color):
        for i in range(0, len(trail_set) - 1):
            pygame.draw.line(self.map, color, (trail_set[i][0], trail_set[i][1]),
                             (trail_set[i + 1][0], trail_set[i + 1][1]))
        if trail_set.__sizeof__() > 10000:
            trail_set.pop(0)
        trail_set.append((pose_x, pose_y))

    def write_info(self, Vl, Vr, theta, follow_dist):
        txt = f"Vl = {round(Vl, 2)}"
        self.text = self.font.render(txt, True, self.white, self.black)
        self.textRect.center = (self.width - 200, self.height - 200)
        self.map.blit(self.text, self.textRect)

        txt = f"Vr = {round(Vr, 2)}"
        self.text = self.font.render(txt, True, self.white, self.black)
        self.textRect.center = (self.width - 200, self.height - 150)
        self.map.blit(self.text, self.textRect)

        txt = f"theta = {round(math.degrees(theta), 2)}"
        self.text = self.font.render(txt, True, self.white, self.black)
        self.textRect.center = (self.width - 200, self.height - 100)
        self.map.blit(self.text, self.textRect)

        txt = f"Follow Distance = {round(follow_dist, 2)}"
        self.text = self.font.render(txt, True, self.white, self.black)
        self.textRect.center = (self.width - 350, self.height - 50)
        self.map.blit(self.text, self.textRect)


class Target:
    def __init__(self, targetImg):

        # target data
        self.x = 0
        self.y = 0

        # target graphics
        self.img = pygame.image.load(targetImg)
        self.rotated = self.img
        self.rect = self.rotated.get_rect(center=(self.x, self.y))

    def draw(self, map):
        map.blit(self.rotated, self.rect)

    def move(self, t):  # movement function of the target
        self.x = 300 + t * 50
        self.y = 200 + 50 * math.cos(t)

        self.rotated = pygame.transform.rotozoom(self.img, math.degrees(0), 1)
        self.rect = self.rotated.get_rect(center=(self.x, self.y))


class Robot:
    def __init__(self, robot_x, robot_y, robot_theta, distance_star, robotImg, robotWidth=0.03):

        # meter -> pixel transform
        self.meter_to_pixel = 3779.52

        # robot data
        self.x = robot_x
        self.y = robot_y
        self.theta = robot_theta
        self.x_target = 0
        self.y_target = 0
        self.width = robotWidth * self.meter_to_pixel
        self.R = 0.1 * self.meter_to_pixel  # radius of the wheels
        self.d_star = distance_star  # desired follow distance
        self.follow_dist = 0  # current follow distance
        self.vr = 0  # vr = wr * self.R
        self.vl = 0
        self.wr = 0
        self.wl = 0
        self.w_velocity = 0
        self.v_velocity = 0

        # PID attributes
        self.e_distance_sum = 0
        self.e_distance_prev = 0

        # robot graphics
        self.img = pygame.image.load(robotImg)
        self.rotated = self.img
        self.rect = self.rotated.get_rect(center=(self.x, self.y))

    def draw(self, map):
        map.blit(self.rotated, self.rect)

    def error_distance(self):
        self.follow_dist = math.sqrt((self.x_target - self.x) ** 2 + (self.y_target - self.y) ** 2)
        e_distance = self.follow_dist - self.d_star
        return e_distance

    def linear_velocity(self):
        e_distance = self.error_distance()
        P = Kp_v * e_distance
        I = Ki_v * self.e_distance_sum * dt
        D = Kd_v * (e_distance - self.e_distance_prev) / dt
        self.v_velocity = P + I + D
        self.e_distance_prev = e_distance
        self.e_distance_sum += e_distance
        return self.v_velocity

    def target_angle(self):
        theta_star = math.atan2(self.y_target - self.y, self.x_target - self.x)
        return theta_star

    def error_theta(self):
        theta_star = self.target_angle()
        theta = theta_star - self.theta
        e_theta = math.atan2(math.sin(theta), math.cos(theta))
        return e_theta

    def angular_velocity(self):
        e_theta = self.error_theta()
        self.w_velocity = Kp_w * e_theta
        return self.w_velocity

    def wheel_linear_velocity(self):
        v_velocity = self.linear_velocity()
        w_velocity = self.angular_velocity()
        self.vr = (2 * v_velocity + w_velocity * self.width) / 2  # vr = wr * self.R
        self.vl = (2 * v_velocity - w_velocity * self.width) / 2  # vl = wl * self.R
        return self.vr, self.vl

    def wheel_angular_velocity(self):
        vr = self.wheel_linear_velocity()[0]
        vl = self.wheel_linear_velocity()[1]
        self.wr = vr / self.R
        self.wl = vl / self.R
        return self.wr, self.wl

    def controller(self, target_pos):
        """
        CONTROLLER
        control input: robot pose (x, y, theta) and target position (x_target, y_target)
        control outputs: angular velocity of right and left wheel (wr, wl)
        """
        self.x_target = target_pos[0]
        self.y_target = target_pos[1]

        """if self.follow_dist > self.d_star:
            self.e_distance_sum = 0"""

        wr = self.wheel_angular_velocity()[0]
        wl = self.wheel_angular_velocity()[1]
        return wr, wl

    def move(self, target_pos):  # movement function of the robot
        [wr, wl] = self.controller(target_pos)

        vr = wr * self.R
        vl = wl * self.R
        v_velocity = (vr + vl) / 2
        w_velocity = self.angular_velocity()

        # robot pose update
        self.x = self.x + v_velocity * math.cos(self.theta) * dt
        self.y = self.y + v_velocity * math.sin(self.theta) * dt
        self.theta = self.theta + w_velocity * dt

        # reset theta
        if self.theta > 2 * math.pi or self.theta < -2 * math.pi:
            self.theta = 0

        # movement of robot image on the map
        self.rotated = pygame.transform.rotozoom(self.img, -math.degrees(self.theta), 1)
        self.rect = self.rotated.get_rect(center=(self.x, self.y))


def main():
    pygame.init()

    # environment object
    environment = Environment(map_width, map_height)

    # robot object
    robot = Robot(start_x, start_y, start_theta, follow_distance,
                  r"C:\Users\AbdullahDangac\Desktop\follow_trajectory_simulation\robot\diff_drive_robot.png")

    # target object
    target = Target(r"C:\Users\AbdullahDangac\Desktop\follow_trajectory_simulation\robot\targetImg.png")

    # program start time
    start_time = time.time()

    # simulation loop
    loop = True
    while loop:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                loop = False

        pygame.display.update()
        environment.map.fill(environment.black)

        runtime = time.time() - start_time

        target.move(runtime)
        target.draw(environment.map)
        environment.trail(target.x, target.y, environment.trail_set_target, environment.red)

        robot.move((target.x, target.y))
        robot.draw(environment.map)
        environment.trail(robot.x, robot.y, environment.trail_set_robot, environment.green)

        if robot.theta < 0:
            robot.theta = 2 * math.pi + robot.theta

        environment.write_info(robot.vl, robot.vr, robot.theta, robot.follow_dist)


if __name__ == '__main__':
    try:
        main()
    except RuntimeError:
        pass
