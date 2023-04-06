import pygame, sys, random, os
import numpy as np
from scipy.integrate import ode

WHITE      = (255, 255, 255)
WIN_HEIGHT = 200
WIN_WIDTH  = 200
BLACK = (0,0,0)
WALLS      = ["left", "right", "top", "bottom"]
DT         = 0.01
paused     = False
in_windows = False

class Disk2D(pygame.sprite.Sprite):
    def __init__(self, radius, mass=1.0):
        pygame.sprite.Sprite.__init__(self)
        
        self.image = pygame.Surface([radius*2, radius*2])
        self.image.fill(BLACK)
        pygame.draw.circle(self.image, BLACK, (radius, radius), radius, radius)
        

        self.state = [0, 0, 0, 0]
        self.mass = mass
        self.t = 0
        self.radius = radius

        self.solver = ode(self.f)
        self.solver.set_integrator('dop853')
        self.solver.set_initial_value(self.state, self.t)

    def f(self, t, y):
        return [y[2], y[3], 0, 0]

    def set_pos(self, pos):
        self.state[0:2] = pos
        self.solver.set_initial_value(self.state, self.t)
        return self

    def set_vel(self, vel):
        self.state[2:] = vel
        self.solver.set_initial_value(self.state, self.t)
        return self

    def update(self, dt):
        self.t += dt
        self.state = self.solver.integrate(self.t)
    

    def draw(self, surface):

        """
        self.image = pygame.Surface([20, 20])
        self.image.fill(BLACK)
        self.image.set_colorkey(BLACK)
        pygame.draw.circle(self.image, BLACK, (10, 10), 10)
        """
        rect = self.image.get_rect()
        rect.center = (self.state[0], WIN_HEIGHT-self.state[1])
        surface.blit(self.image, rect)


class World:
    def __init__(self):
        self.paused = paused
        self.disks = []
        self.dt = DT
        self.e= 1.

    def add(self, radius, mass=1.0):
        disk = Disk2D(radius, mass)
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
                self.compute_collision_response(i, -1, WALLS[k])

            for j in range(i+1, len(self.disks)):
                if i == j:
                    continue

                if self.compute_collision_response(i, j):
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
    screen.fill(WHITE)
    world.draw(screen)
    world.update(world.dt)
    pygame.display.update()



def main():
    print('Press q to quit')
    global in_windows
    if sys.platform == "win32":
        in_windows = True



    pygame.init()

    clock = pygame.time.Clock()

    win_width = WIN_WIDTH
    win_height = WIN_HEIGHT
    screen = pygame.display.set_mode((win_width, win_height))


    world = World()
    
    world.add(10, 1).set_pos([100,100]).set_vel([0,0])
    world.add(10, 1).set_pos([110,160]).set_vel([0,-400])


    while True:
        # 30 fps
        clock.tick(30)

        event = pygame.event.poll()
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
