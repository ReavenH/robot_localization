import pygame
from pygame.locals import *

from OpenGL.GL import *
from OpenGL.GLU import *

import numpy as np

import random

'''
Nodes, connections, faces.
'''

vertices = np.array([[1, -1, -1],
                     [1, 1, -1],
                     [-1, 1, -1],
                     [-1, -1, -1],
                     [1, -1, 1],
                     [1, 1, 1],
                     [-1, -1, 1],
                     [-1, 1, 1]])

edges = np.array([[0, 1],
                  [0, 3],
                  [0, 4],
                  [2, 1],
                  [2, 3],
                  [2, 7],
                  [6, 3],
                  [6, 4],
                  [6, 7],
                  [5, 1],
                  [5, 4],
                  [5, 7]])

surfaces = np.array([[0, 1, 2, 3],
                     [3, 2, 7, 6],
                     [6, 7, 5, 4],
                     [4, 5, 1, 0],
                     [1, 5, 7, 2],
                     [4, 0, 3, 6]])

colors = np.array([[1, 0, 0],
                   [1, 1, 0],
                   [1, 1, 1],
                   [0, 0, 0],
                   [0, 1, 0],
                   [0, 1, 1],
                   [1, 0, 1],
                   [0, 1, 0]])

def Cube():

    glBegin(GL_QUADS)

    for i, surface in enumerate(surfaces):
        
        for j, vertex in enumerate(surface):
            glColor3fv(colors[j])
            glVertex3fv(vertices[vertex])

    glEnd()

    glBegin(GL_LINES)  # specify task inside brackets

    for edge in edges:
        for vertex in edge:
            glVertex3fv(vertices[vertex])  # alternative of glVertex3f. It supports input the pointer to a vertex, instead of the coordinates.

    glEnd()  # empty inside brackets

def main():
    pygame.init()
    display = (800, 600)

    # double buffer in the background to reduce screen flashing, use opengl for rendering. "|" is the bitwise OR operator.
    pygame.display.set_mode(display, DOUBLEBUF|OPENGL)  

    # view angle (the larger, the more distorted and the broader view angle), 
    # perspect ratio (same as the window pr to maintain shape), 
    # near clipping plane (any object closer than this range to the viewpoint will not be rendered),
    # far clipping plane.
    gluPerspective(60, (display[0] / display[1]), 0.1, 50.0)  

    # translation vector: x, y, z. Z+ axis goes perpendicular out of the screen; X+ goes to the right of the screen; Y+ goes to the top of screen,
    # when Z is far below 0, meaning the viewpoint is far from the object, same pixels for dX and dY will require larger X and Y input.
    glTranslatef(random.randrange(-5, 5), random.randrange(-5, 5), -40)

    # no rotation.
    glRotatef(0, 0, 0, 0)  # deg, x, y, z

    cameraPose = np.array([0, 0, 0]).astype('float')
    objectPassed = False

    while(not objectPassed):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                quit()
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_LEFT:
                    glTranslatef(0.2, 0, 0)
                if event.key == pygame.K_RIGHT:
                    glTranslatef(-0.2, 0, 0)
                if event.key == pygame.K_UP:
                    glTranslatef(0, -0.2, 0)
                if event.key == pygame.K_DOWN:
                    glTranslatef(0, 0.2, 0)
            '''if event.type == MOUSEBUTTONDOWN:
                if event.button == 4:  # mouse wheel downwards
                    glTranslatef(0, 0, 0.2)
                if event.button == 5:  # mouse wheel upwards
                    glTranslatef(0, 0, -0.2)'''
            
        x = np.array(glGetDoublev(GL_MODELVIEW_MATRIX))
        cameraPose = x[-1, 0:3]

        if cameraPose[-1] < -1:
            objectPassed = True

        glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT)  # clear the previous frame

        # rotate 1-deg, centered at the vector (3, 1, 1).
        # glRotatef(1, 0, 1, 0)

        # bring the cube closer
        glTranslatef(0, 0, 0.1)
        Cube()

        # display the current buffer and clear the back buffer (prevent ripping)
        pygame.display.flip()

        # refresh rate: every 10 milisec.
        pygame.time.wait(10)

    pygame.quit()
    

if __name__ == "__main__":
    for i in range(5):
        main()
    quit()