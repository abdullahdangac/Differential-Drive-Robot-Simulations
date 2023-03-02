"""

   Go-To-Goal Simulation for Differential Drive Robot with Python

   author: Abdullah DANGAC (@abdullahdangac)

   Created 21.10.2021

"""

import math
import pygame

# delta t
dt = 0.005

# === ENVIRONMENT PROPERTIES (INITIAL VALUES) ===
# map (window) dimensions
map_width = 1400
map_height = 750

# === ROBOT PROPERTIES (INITIAL VALUES) ===
# start position of the robot (pose)
start_x = 200
start_y = 600
start_theta = 2 * math.pi

# goal point
goal_x = 800
goal_y = 200

# PID-Control Gains
Kp_v = 0.5  # P-Control gain for linear velocity
Kp_w = 1  # P-Control gain for angular velocity


class Environment:
    def __init__(self, window_width, window_height):

        # colors
        self.black = (0, 0, 0)
        self.white = (255, 255, 255)
        self.green = (0, 255, 0)

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

        # trail
        self.trail_set = []
        
    def trail(self, pose_x, pose_y):
        for i in range(0, len(self.trail_set) - 1):
            pygame.draw.line(self.map, self.green, (self.trail_set[i][0], self.trail_set[i][1]),
                             (self.trail_set[i + 1][0], self.trail_set[i + 1][1]))
        if self.trail_set.__sizeof__() > 10000:
            self.trail_set.pop(0)
        self.trail_set.append((pose_x, pose_y))
        
    def write_info(self, Vl, Vr, theta, x, y):
        txt = f"Vl = {round(Vl, 2)}"
        self.text = self.font.render(txt, True, self.white, self.black)
        self.textRect.center = (self.width - 200, self.height - 150)
        self.map.blit(self.text, self.textRect)

        txt = f"Vr = {round(Vr, 2)}"
        self.text = self.font.render(txt, True, self.white, self.black)
        self.textRect.center = (self.width - 200, self.height - 100)
        self.map.blit(self.text, self.textRect)

        txt = f"theta = {round(math.degrees(theta), 2)}"
        self.text = self.font.render(txt, True, self.white, self.black)
        self.textRect.center = (self.width - 200, self.height - 50)
        self.map.blit(self.text, self.textRect)

        txt = f"x = {round(x, 2)}"
        self.text = self.font.render(txt, True, self.white, self.black)
        self.textRect.center = (self.width - 200, self.height - 300)
        self.map.blit(self.text, self.textRect)

        txt = f"y = {round(y, 2)}"
        self.text = self.font.render(txt, True, self.white, self.black)
        self.textRect.center = (self.width - 200, self.height - 250)
        self.map.blit(self.text, self.textRect)


class Robot:
    def __init__(self, robot_x, robot_y, robot_theta, x_goal, y_goal, robotImg, robotWidth=0.03):

        # meter -> pixel transform
        self.meter_to_pixel = 3779.52

        # robot data
        self.x = robot_x
        self.y = robot_y
        self.theta = robot_theta
        self.width = robotWidth * self.meter_to_pixel
        self.R = 0.1 * self.meter_to_pixel  # radius of the wheels
        self.vr = 0  # vr = wr * self.R
        self.vl = 0
        self.wr = 0
        self.wl = 0
        self.w_velocity = 0
        self.v_velocity = 0

        # goal point data
        self.x_g = x_goal
        self.y_g = y_goal

        # robot graphics
        self.img = pygame.image.load(robotImg)
        self.rotated = self.img
        self.rect = self.rotated.get_rect(center=(self.x, self.y))

    def draw(self, map):
        map.blit(self.rotated, self.rect)

    def distance(self):  # calculate distance between the robot and goal
        distance = math.sqrt((self.x_g - self.x) ** 2 + (self.y_g - self.y) ** 2)
        return distance

    def linear_velocity(self):
        distance = self.distance()
        self.v_velocity = Kp_v * distance
        return self.v_velocity

    def goal_angle(self):
        theta_star = math.atan2(self.y_g - self.y, self.x_g - self.x)
        return theta_star

    def error_theta(self):  # calculate error angle between heading and target direction
        theta_star = self.goal_angle()
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

    def move(self):
        # for angular velocity input
        """
        [wr, wl] = self.wheel_angular_velocity()

        vr = wr * self.R
        vl = wl * self.R
        """

        # for linear velocity input
        [vr, vl] = self.wheel_linear_velocity()

        if self.distance() == 0.1 * self.meter_to_pixel:
            v_velocity = 0
            w_velocity = 0
        else:
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
    robot = Robot(start_x, start_y, start_theta, goal_x, goal_y,
                  r"C:/Users/AbdullahDangac/Differential-Drive-Robot-Simulations/Go_to_Goal_Simulation/images/differential_drive_robot.png")

    # simulation loop
    loop = True
    while loop:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                loop = False

        pygame.display.update()
        environment.map.fill(environment.black)

        robot.move()
        robot.draw(environment.map)

        # for robot information displayed on the screen
        if robot.theta < 0:
            robot.theta = 2 * math.pi + robot.theta

        environment.write_info(robot.vl, robot.vr, robot.theta, robot.x, robot.y)
        environment.trail(robot.x, robot.y)


if __name__ == '__main__':
    try:
        main()
    except RuntimeError:
        pass
