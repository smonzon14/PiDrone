import time
import struct
import pygame
import socket
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *
from OpenGL.WGL import *
surfaces = (
    (1, 2, 3, 4),
    (3, 2, 7, 6),
    (6, 7, 5, 4),
    (4, 5, 1, 0),
    (1, 5, 7, 2),
    (4, 0, 3, 6)
)

vertices = (
    ( 1, -1, -1),
    ( 1,  1, -1),
    (-1,  1, -1),
    (-1, -1, -1),
    ( 1, -1,  1),
    ( 1,  1,  1),
    (-1, -1,  1),
    (-1,  1,  1)
)


edges = (
    (0, 1),
    (0, 3),
    (0, 4),
    (2, 1),
    (2, 3),
    (2, 7),
    (6, 3),
    (6, 4),
    (6, 7),
    (5, 1),
    (5, 4),
    (5, 7)
)


# -------------------------------------------------------------
btn_square = 0
btn_cross = 1
btn_circle = 2
btn_tri = 3

btn_l1 = 4
btn_l2 = 6
btn_l3 = 10

btn_r1 = 5
btn_r2 = 7
btn_r3 = 11

btn_share = 8
btn_option = 9

axis_x = 14
axis_y = 13

btn_ps4 = 12
btn_touch = 13


axis_l3_x = 0
axis_l3_y = 1
axis_r3_x = 2
axis_r3_y = 3

axis_r2 = 4
axis_l2 = 5


axis_touch_x = 13
axis_touch_y = 14
# -------------------------------------------------------------

def mixColors(colors):
    reds =   [ c[0] for c in colors ]
    greens = [ c[1] for c in colors ]
    blues =  [ c[2] for c in colors ]

    r = sum(reds) / len(colors)
    g = sum(greens) / len(colors)
    b = sum(blues) / len(colors)

    return (r, g, b)

def getColorFromButtonsPressed(buttons):
    colors = {
        btn_square: (1.00, 0.41, 0.97),
        btn_cross:  (0.49, 0.69, 0.97),
        btn_circle: (1.00, 0.40, 0.40),
        btn_tri:    (0.25, 0.88, 0.62)
    }

    pressedButtons = [ b for b in buttons.keys() if buttons[b] and b in colors.keys() ]

    if not pressedButtons:
        return (0, 0, 0)

    return mixColors([ colors[b] for b in pressedButtons ])

def shiftCube(axis_x, axis_y, axis_z):

    glTranslatef(axis_x, axis_y, axis_z)


def Cube(color):


    glBegin(GL_QUADS)
    for surface in surfaces:
        for vertex in surface:
            glColor3f(color[0],color[1],color[2])
            glVertex3f(vertices[vertex][0],vertices[vertex][1],vertices[vertex][2])
    glEnd()

    glBegin(GL_LINES)
    i = 0
    for edge in edges:

        for vertex in edge:
            if(i == 3): glColor3f(1,0,0)
            elif(i == 9): glColor3f(0,1,0)
            elif(i == 13): glColor3f(0,0,1)
            else: glColor3f(1,1,1)
            i+=1
            glVertex3f(vertices[vertex][0],vertices[vertex][1],vertices[vertex][2])

    glEnd()


def main():
    pygame.init()
    display = (800,600)
    pygame.display.set_mode(display, DOUBLEBUF|OPENGL)

    gluPerspective(45, (display[0]/display[1]), 0.1, 50.0)
    glTranslatef(0.0,0.0, -5)

    pygame.init()
    pygame.joystick.init()
    controller = pygame.joystick.Joystick(0)
    controller.init()

    axis = {x:0.0 for x in range(controller.get_numaxes())}

    button = {}
    for i in range(controller.get_numbuttons()):
        button[i] = False

    mode = 0

    UDP_IP = "192.168.1.11"
    UDP_PORT = 5005
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    while True:
        for event in pygame.event.get():

            if event.type == pygame.JOYAXISMOTION:
                axis[event.axis] = round(event.value,2)
            elif event.type == pygame.JOYBUTTONDOWN:
                button[event.button] = True
            elif event.type == pygame.JOYBUTTONUP:
                button[event.button] = False

        kill = button[btn_circle]
        arm = button[btn_r3]
        calibrate = button[btn_option]

        trans_x = axis[axis_l3_x]
        trans_y = axis[axis_r3_y]
        trans_z = axis[axis_l3_y]
        rot_y = axis[axis_r3_x]


        glPushMatrix()
        shiftCube(trans_x, -1 * trans_y, trans_z)

        glRotatef(10 * trans_z, 1, 0, 0)
        glRotatef(-60 * rot_y, 0, 1, 0)
        glRotatef(-10 * trans_x, 0, 0, 1)
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        Cube(getColorFromButtonsPressed(button))
        glPopMatrix()
        pygame.display.flip()

        msg_string = struct.pack('3?4d', kill, arm, calibrate, trans_x, -1 * trans_y, -1 * trans_z, rot_y)
        print([kill, arm, calibrate, trans_x, -1 * trans_y, -1 * trans_z, rot_y])
        sock.sendto(msg_string, (UDP_IP, UDP_PORT))
        time.sleep(0.03)

main()


