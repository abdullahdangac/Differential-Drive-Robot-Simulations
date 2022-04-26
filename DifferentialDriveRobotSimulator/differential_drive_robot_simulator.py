"""

   Differential Drive Robot Simulation with Python

   author: Abdullah DANGAC (@abdullahdangac)

   Created 21.10.2021

"""

import math
import pygame

# delta t
dt = 0.004  # recommended dt for pygame

# === ENVIRONMENT PROPERTIES (INITIAL VALUES) ===
# map (window) dimensions
map_width = 1400
map_height = 750

# === ROBOT PROPERTIES (INITIAL VALUES) ===
# robot initial pose
start_x = 200
start_y = 600
start_theta = math.pi / 4


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

    def write_info(self, Vl, Vr, theta):
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


class Robot:
    def __init__(self, robot_x, robot_y, robot_theta, robotImg, robotWidth=0.03):
        # meter -> pixel transform
        self.meter_to_pixel = 3779.52

        # robot data
        self.x = robot_x
        self.y = robot_y
        self.theta = robot_theta
        self.width = robotWidth * self.meter_to_pixel
        self.vr = 0
        self.vl = 0
        self.w_velocity = 0
        self.v_velocity = 0

        # differential drive behaviour (Instantaneous Center of Curvature - ICC) data
        self.ICCx = 0
        self.ICCy = 0
        self.r_distance = 0

        # robot graphics
        self.img = pygame.image.load(robotImg)
        self.rotated = self.img
        self.rect = self.rotated.get_rect(center=(self.x, self.y))

    def draw(self, map):
        map.blit(self.rotated, self.rect)

    def calc_v_velocity(self):
        self.v_velocity = (self.vr + self.vl) / 2

    def calc_w_velocity(self):
        self.w_velocity = (self.vr - self.vl) / self.width

    def calc_r_distance(self):  # calculate distance to ICC
        self.r_distance = ((self.width / 2) * ((self.vr + self.vl) / (self.vr - self.vl)))

    def calc_icc(self):  # calculate ICC point
        self.ICCx = self.x + (self.r_distance * math.sin(self.theta))
        self.ICCy = self.y + (self.r_distance * math.cos(self.theta))

    def move(self, event=None):  # robot move function
        if event is not None:
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_KP4:
                    self.vl += 0.001 * self.meter_to_pixel
                elif event.key == pygame.K_KP1:
                    self.vl -= 0.001 * self.meter_to_pixel
                elif event.key == pygame.K_KP6:
                    self.vr += 0.001 * self.meter_to_pixel
                elif event.key == pygame.K_KP3:
                    self.vr -= 0.001 * self.meter_to_pixel
                elif event.key == pygame.K_KP8:
                    self.vr += 0.001 * self.meter_to_pixel
                    self.vl += 0.001 * self.meter_to_pixel
                elif event.key == pygame.K_KP2:
                    self.vr = 0
                    self.vl = 0
                elif event.key == pygame.K_KP5:
                    if self.vr < self.vl:
                        self.vl = self.vr
                    else:
                        self.vr = self.vl

        self.calc_w_velocity()
        self.calc_v_velocity()

        # robot pose update (differential drive behaviour)
        if self.vl == self.vr:
            self.x += self.v_velocity * math.cos(self.theta) * dt
            self.y -= self.v_velocity * math.sin(self.theta) * dt
            self.theta += self.w_velocity * dt

            # reset theta
            if self.theta > 2 * math.pi or self.theta < -2 * math.pi:
                self.theta = 0

        elif self.vl != self.vr:
            self.calc_icc()
            self.calc_r_distance()

            # rotation matrix (pose update)
            self.x = (math.cos(self.w_velocity * dt) * (self.x - self.ICCx)) - (math.sin(self.w_velocity * dt) * (self.y - self.ICCy)) + self.ICCx
            self.y = (math.sin(self.w_velocity * dt) * (self.x - self.ICCx)) + (math.cos(self.w_velocity * dt) * (self.y - self.ICCy)) + self.ICCy
            self.theta = self.theta + (self.w_velocity * dt)

            # reset theta
            if self.theta > 2 * math.pi or self.theta < -2 * math.pi:
                self.theta = 0

        # show moving of robot
        self.rotated = pygame.transform.rotozoom(self.img, math.degrees(self.theta), 1)
        self.rect = self.rotated.get_rect(center=(self.x, self.y))


def main():
    pygame.init()

    # environment object
    environment = Environment(map_width, map_height)

    # robot object
    robot = Robot(start_x, start_y, start_theta,
                  r"C:\Users\AbdullahDangac\Desktop\differential_drive_robot_simulator\robot\diff_drive_robot.png")

    # simulation loop
    loop = True
    while loop:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                loop = False
            robot.move(event)

        pygame.display.update()
        environment.map.fill(environment.black)

        robot.move()
        robot.draw(environment.map)

        if robot.theta < 0:
            robot.theta = 2 * math.pi + robot.theta

        environment.write_info(robot.vl, robot.vr, robot.theta)
        environment.trail(robot.x, robot.y)


if __name__ == '__main__':
    try:
        main()
    except RuntimeError:
        pass
