# -*- coding: utf-8 -*-
#CSCI3010 Final Project
#Michael DeMelo & Mohammadreza Rahbar
#100779096 & 100781952

import pygame, sys, random, os
import numpy as np
from scipy.integrate import ode
import time

WHITE      = (255, 255, 255)
WIN_HEIGHT = 800
WIN_WIDTH  = 800
GREEN      = (50, 205, 50)
WALLS      = ["left", "right", "top", "bottom"]
DT         = 0.05
paused     = False
in_windows = False

class Disk2D(pygame.sprite.Sprite):
    def __init__(self, imgfile, radius, mass):
        pygame.sprite.Sprite.__init__(self)
        

        self.image = pygame.image.load(imgfile)
        self.image = pygame.transform.smoothscale(self.image, (radius*2, radius*2))


        self.state = [0, 0, 0, 0]
        self.mass = mass
        self.t = 0
        self.radius = radius
        self.peg_state = False
        self.cursor_state = False
        self.goal_state = False
        
        self.g = 9.8

        self.solver = ode(self.f)
        self.solver.set_integrator('dop853')
        self.solver.set_initial_value(self.state, self.t)

    def f(self, t, y):
        return [y[2], y[3], 0, -10*self.g]

    def set_pos(self, pos):
        self.state[0:2] = pos
        self.solver.set_initial_value(self.state, self.t)
        return self

    def set_vel(self, vel):
        self.state[2:] = vel
        self.solver.set_initial_value(self.state, self.t)
        return self
    
    def set_type(self, peg, curse, goal):
        self.peg_state = peg
        self.cursor_state = curse
        self.goal_state = goal
        if peg == True:
            self.image = pygame.transform.smoothscale(self.image, (self.radius*2, self.radius*2))
        if goal == True:
            self.image.fill(GREEN)

    def update(self, dt):
        if self.peg_state == False:
            if self.cursor_state == False:
                self.t += dt
                self.state = self.solver.integrate(self.t)
    

    def draw(self, surface):

        rect = self.image.get_rect()
        rect.center = (self.state[0], WIN_HEIGHT-self.state[1])
        surface.blit(self.image, rect)


class World:
    def __init__(self):
        self.paused = paused
        self.score = 0
        self.balls_left = 10
        self.disks = []
        self.dt = DT
        self.e= 0.3

    def add(self, imgfile, radius, mass):
        disk = Disk2D(imgfile,radius, mass)
        self.disks.append(disk)
        return disk

    def draw(self, screen):
        for d in self.disks:
            d.draw(screen)

    def update(self, dt):
        self.collision_check()

        for d in self.disks:
            d.update(dt)

    def pause(self):
        self.paused = True

    def resume(self):
        self.paused = False


    def collision_check(self):
        for i in range(0, len(self.disks)):

            for k in range(0, len(WALLS)):
                if self.compute_collision_response(i, -1, WALLS[k]):
                    if (WALLS[k] == 'bottom'):
                        self.balls_left -= 1
                        self.disks[i].set_pos([900, 900])
                        self.disks[i].set_type(True, False, False)
                        if self.balls_left == 0:
                            print(f'Your score is {self.score}/5')
                            pygame.quit()

            for j in range(i+1, len(self.disks)):
                if i == j:
                    continue

                if self.compute_collision_response(i, j):
                    if self.disks[i].goal_state == True:
                        self.balls_left -= 1
                        self.disks[i].set_pos([900, 900])
                        self.disks[j].set_pos([900, 900])
                        self.score += 1
                        if self.balls_left == 0:
                            print(f'Your score is {self.score}/5')
                            pygame.quit()
                    if self.disks[j].goal_state == True:
                        self.balls_left -= 1
                        self.disks[j].set_pos([900, 900])
                        self.disks[i].set_pos([900, 900])
                        self.score += 1
                        if self.balls_left == 0:
                            print(f'Your score is {self.score}/5')
                            pygame.quit()
                    break

    def compute_collision_response(self, i, j, wall=""):
        pos_i = np.array(self.disks[i].state[0:2])
        mass_i = self.disks[i].mass

        if j == -1:
            if wall == "left":
                pos_j = [0, pos_i[1]]
            elif wall == "right":
                pos_j = [WIN_WIDTH, pos_i[1]]
            elif wall == "top":
                pos_j = [pos_i[0], WIN_HEIGHT]
            else:
                pos_j = [pos_i[0], 0]

            radius_j = 0
            vel_j = [0, 0]
            mass_denom = (1./mass_i)

        else:
            pos_j = np.array(self.disks[j].state[0:2])
            vel_j = self.disks[j].state[2:]
            radius_j = self.disks[j].radius
            mass_j = self.disks[j].mass
            mass_denom = (1./mass_i) + (1./mass_j)

        diff_pos = pos_i - pos_j
        dist = np.sqrt(np.sum(diff_pos**2))

        if dist <= (self.disks[i].radius + radius_j):
            vel_i = np.array(self.disks[i].state[2:])

            relative_vel_ij = vel_i - vel_j
            n_ij = normalize(diff_pos)

            dot_rel_vel = np.dot(relative_vel_ij, n_ij)
            if dot_rel_vel < 0:
                J = -(1 + self.e) * dot_rel_vel / mass_denom

                vel_i_aftercollision = vel_i + n_ij * J / mass_i 
                self.disks[i].set_vel(vel_i_aftercollision)

                if j != -1:
                    vel_j_aftercollision = vel_j - n_ij * J / mass_j 
                    self.disks[j].set_vel(vel_j_aftercollision)

                return True
        return False


def normalize(v):
    return v / np.linalg.norm(v)


def update(screen, world):
    img = pygame.image.load('back.jpg')
    screen.fill(WHITE)
    screen.blit(img, (0,0))
    world.draw(screen)
    world.update(world.dt)
    pygame.display.update()



def main():
    global in_windows
    if sys.platform == "win32":
        in_windows = True

    pygame.init()

    clock = pygame.time.Clock()

    win_width = WIN_WIDTH
    win_height = WIN_HEIGHT
    screen = pygame.display.set_mode((win_width, win_height))

    disk_img = []
    for img_name in os.listdir("images/"):
        disk_img.append(os.path.join("images", img_name))

    world = World()

    x = 38
    y = 600
    row = 0
    while y > 180:
        if x > WIN_WIDTH:
            if (row % 2) == 0:
                x = 90
            else:
                x = 38
            y -= 80
            row += 1
        if y > 180:
            world.add(disk_img[1], 20, 1000).set_pos([x, y]).set_vel([0,0]).set_type(True, False, False)
        x += 80

    rand_goals = []
    for i in range(5):
        random.seed(time.time())
        rand_x = random.randrange(0, 800, 3)
        if len(rand_goals) > 0:
            for j in range(len(rand_goals)):
                if rand_x - rand_goals[j] <= 30:
                    rand_x = random.randrange(0, 800, 3)
        rand_goals.append(rand_x)
        world.add(disk_img[1],25, 1000).set_pos([rand_x, 50]).set_vel([0,0]).set_type(True, False, True)

    world.add(disk_img[0],10, 1).set_pos([400,700]).set_vel([0,0]).set_type(False, True, False)

    print('Welcome to BALL DROP\nChoose where to dop the balls and try to hit the GREEN goals\nControls:')
    print('\'q\' to quit')
    print('\'<-\' & \'->\' keys to move ball left and right\n\'SPACE\' to drop ball')
    print('10 balls remaining')

    ball_count = 10

    while True:

        clock.tick(144)

        event = pygame.event.poll()
        
        if (ball_count > 0):
            if event.type == pygame.KEYDOWN and event.key == pygame.K_LEFT:
                world.disks[-1].set_pos([world.disks[-1].state[0] -7, world.disks[-1].state[1]])
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_RIGHT:
                world.disks[-1].set_pos([world.disks[-1].state[0] +7, world.disks[-1].state[1]])
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_SPACE:
                prev_x = world.disks[-1].state[0]
                world.disks[-1].set_type(False, False, False)
                ball_count -= 1
                if (ball_count > 0):
                    print(f'{ball_count} balls remaining')
                    world.add(disk_img[0],10, 1).set_pos([prev_x,700]).set_vel([0,0]).set_type(False, True, False)
                else:
                    print('No more balls remaining')

        if event.type == pygame.KEYDOWN and event.key == pygame.K_q:
            pygame.quit()
            break
        else:
            pass
        
        if not world.paused:
            update(screen, world)
        else:
            if event.type == pygame.KEYDOWN and event.key == pygame.K_SPACE:
                update(screen, world)
        
if __name__ == '__main__':
    main()


