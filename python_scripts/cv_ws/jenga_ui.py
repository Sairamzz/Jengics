from jenga_stack import JengaStack

import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *

import time

class JengaViewer:
    def __init__(self, stack):
        """
        Initializes the 3D Jenga Viewer.

        :param stack: A list of lists representing the Jenga tower.
                      True -> Block present, False -> Block removed.
        """
        self.stack = stack
        self.levels = len(stack)
        self.angle_x = 0
        self.angle_y = 0
        self.zoom = -20
        self.mouse_down = False
        self.last_mouse_pos = (0, 0)
        self.setup_pygame()

    def setup_pygame(self):
        """Initializes pygame and OpenGL settings."""
        pygame.init()
        self.screen = pygame.display.set_mode((800, 600), DOUBLEBUF | OPENGL)
        glEnable(GL_DEPTH_TEST)
        gluPerspective(45, (800 / 600), 0.1, 50.0)
        glTranslatef(0, -5, self.zoom)

    def draw_block(self, x, y, z, present, horizontal=True):
        """Draws a Jenga block at (x, y, z) with black borders."""
        glPushMatrix()
        glTranslatef(x, y, z)
        
        if not horizontal:
            glRotatef(90, 0, 1, 0)

        # Block color (Brown if present, Gray if removed)
        glColor3f(0.8, 0.5, 0.2) if present else glColor3f(0.5, 0.5, 0.5)

        length = 3.0
        width = 1.0

        # --- Draw Solid Block ---
        glBegin(GL_QUADS)
        # Top
        glVertex3f(-length / 2, 0.3, width / 2)
        glVertex3f(length / 2, 0.3, width / 2)
        glVertex3f(length / 2, 0.3, -width / 2)
        glVertex3f(-length / 2, 0.3, -width / 2)

        # Bottom
        glVertex3f(-length / 2, -0.3, width / 2)
        glVertex3f(length / 2, -0.3, width / 2)
        glVertex3f(length / 2, -0.3, -width / 2)
        glVertex3f(-length / 2, -0.3, -width / 2)

        # Front
        glVertex3f(-length / 2, -0.3, width / 2)
        glVertex3f(length / 2, -0.3, width / 2)
        glVertex3f(length / 2, 0.3, width / 2)
        glVertex3f(-length / 2, 0.3, width / 2)

        # Back
        glVertex3f(-length / 2, -0.3, -width / 2)
        glVertex3f(length / 2, -0.3, -width / 2)
        glVertex3f(length / 2, 0.3, -width / 2)
        glVertex3f(-length / 2, 0.3, -width / 2)

        # Left
        glVertex3f(-length / 2, -0.3, -width / 2)
        glVertex3f(-length / 2, -0.3, width / 2)
        glVertex3f(-length / 2, 0.3, width / 2)
        glVertex3f(-length / 2, 0.3, -width / 2)

        # Right
        glVertex3f(length / 2, -0.3, -width / 2)
        glVertex3f(length / 2, -0.3, width / 2)
        glVertex3f(length / 2, 0.3, width / 2)
        glVertex3f(length / 2, 0.3, -width / 2)
        glEnd()

        # --- Draw Wireframe Edges ---
        glColor3f(0, 0, 0)  # Set color to black
        glLineWidth(2.0)  # Thicker edges for visibility

        glBegin(GL_LINE_LOOP)  # Top border
        glVertex3f(-length / 2, 0.3, width / 2)
        glVertex3f(length / 2, 0.3, width / 2)
        glVertex3f(length / 2, 0.3, -width / 2)
        glVertex3f(-length / 2, 0.3, -width / 2)
        glEnd()

        glBegin(GL_LINE_LOOP)  # Bottom border
        glVertex3f(-length / 2, -0.3, width / 2)
        glVertex3f(length / 2, -0.3, width / 2)
        glVertex3f(length / 2, -0.3, -width / 2)
        glVertex3f(-length / 2, -0.3, -width / 2)
        glEnd()

        glBegin(GL_LINES)  # Vertical edges
        edges = [
            (-length / 2, -0.3, width / 2), (-length / 2, 0.3, width / 2),
            (length / 2, -0.3, width / 2), (length / 2, 0.3, width / 2),
            (-length / 2, -0.3, -width / 2), (-length / 2, 0.3, -width / 2),
            (length / 2, -0.3, -width / 2), (length / 2, 0.3, -width / 2)
        ]
        for i in range(0, len(edges), 2):
            glVertex3f(*edges[i])
            glVertex3f(*edges[i + 1])
        glEnd()

        glPopMatrix()

    def draw_jenga_tower(self):
        """Draws the full Jenga tower based on stack data in reverse order."""
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glPushMatrix()
        glRotatef(self.angle_x, 1, 0, 0)
        glRotatef(self.angle_y, 0, 1, 0)

        for level, blocks in enumerate(reversed(self.stack)):  # Reverse order
            y = (self.levels - 1 - level) * 0.7  # Adjust y-axis to keep correct positioning
            horizontal = level % 2 == 0

            if horizontal:  # Alternating X and Z-axis positioning
                positions = [(0, y, -1), (0, y, 0), (0, y, 1)]  # X-axis
            else:
                positions = [(-1, y, 0), (0, y, 0), (1, y, 0)]  # Z-axis

            for i, present in enumerate(blocks):
                self.draw_block(*positions[i], present, horizontal=horizontal)

        glPopMatrix()
        pygame.display.flip()


    def update_blocks(self, new_stack):
        """Updates the Jenga stack and refreshes the display.

        :param new_stack: A new 2D list representing the updated Jenga stack.
        """
        self.stack = new_stack
        self.levels = len(new_stack)

    def update_screen(self, stack, running=True):
        """Handles events and updates the screen when called, without infinite loop.

        :param stack: The updated Jenga stack.
        :param running: Boolean flag to control execution externally.
        """

        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()

                elif event.type == pygame.MOUSEBUTTONDOWN:
                    if event.button == 1:
                        self.mouse_down = True
                        self.last_mouse_pos = event.pos

                elif event.type == pygame.MOUSEBUTTONUP:
                    if event.button == 1:
                        self.mouse_down = False

                elif event.type == pygame.MOUSEMOTION:
                    if self.mouse_down:
                        dx, dy = event.rel
                        self.angle_y += dx * 0.5
                        self.angle_x += dy * 0.5

                if event.type == pygame.KEYDOWN:
                    running = False
                    self.update_blocks(stack)

            self.draw_jenga_tower()
            pygame.time.wait(5)

        




# if __name__ == "__main__":

    
#     jenga_stack = [
#         [True, True, True],  
#         [True, True, True],  
#         [True, True, True],  
#         [True, True, True],  
#         [True, True, True],  
#         [True, True, True],  
#         [True, False, True]  
#     ]
#     viewer = JengaViewer(jenga_stack)
#     viewer.run()

