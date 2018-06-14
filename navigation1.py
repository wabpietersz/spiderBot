#INCLUDES--------------------------------------------------------

import cv2
import numpy as np
import serial
import time
import math
import micropython

#SerialCommand SCmd;



#SERVOS---------------------------------------------------------------------------
# defining 12 servos for 4 legs
servo[4][3]=None
servo_pin = [ [2, 3, 4], [5, 6, 7], [8, 9, 10], [11, 12, 13]] #check again


#SIZE OF THE ROBOT--------------------------------------------------------------------

length_a = 55
length_b = 77.5
length_c = 27.5
length_side = 71
z_absolute = -28

#CONSTANTS FOR MOVEMENT---------------------------------------------------------------

z_default = -50
z_up = -30
z_boot = z_absolute
x_default = 62
x_offset = 0
y_start = 0
y_step = 40

#VARIABLES FOR MOVEMENT-------------------------------------------------------------

site_now[4][3]=None
site_expect[4][3]=None
temp_speed[4][3]=None
move_speed=None
speed_multiple = 1 //movement speed multiple
spot_turn_speed = 4
leg_move_speed = 8
body_move_speed = 3
stand_seat_speed = 1
rest_counter= None

#functions' parameter----
KEEP = 255
#define PI for calculation----
pi = 3.1415926

#CONSTANTS FOR TURN ----------------------------------------------------------------
        #temp length
temp_a = sqrt(pow(2 * x_default + length_side, 2) + pow(y_step, 2))
temp_b = 2 * (y_start + y_step) + length_side
temp_c = sqrt(pow(2 * x_default + length_side, 2) + pow(2 * y_start + y_step + length_side, 2))
temp_alpha = acos((pow(temp_a, 2) + pow(temp_b, 2) - pow(temp_c, 2)) / 2 / temp_a / temp_b)
        #site for turn
turn_x1 = (temp_a - length_side) / 2
turn_y1 = y_start + y_step / 2
turn_x0 = turn_x1 - temp_b * cos(temp_alpha)
turn_y0 = temp_b * sin(temp_alpha) - turn_y1 - length_side


# pin-A0
#define IR_Detect_IO 14


#setup function ----------------------------------------------

def setup():
    ser = serial.Serial('/dev/ttyUSB0')
    ser.open()

#pinMode(IR_Detect_IO, INPUT)

  #SCmd.addCommand("w", action_cmd);

 # SCmd.setDefaultHandler(unrecognized);

 #initialize default parameter, the default position of the robot
set_site(0, x_default - x_offset, y_start + y_step, z_boot)
set_site(1, x_default - x_offset, y_start + y_step, z_boot)
set_site(2, x_default + x_offset, y_start, z_boot)
set_site(3, x_default + x_offset, y_start, z_boot)

for i in range(0,4):
    for j in range(0,3):
        site_now[i][j] = site_expect[i][j]



#//start servo service
#FlexiTimer2::set(20, servo_service);
#FlexiTimer2::start();
#Serial.println("Servo service started");
#//initialize servos
#servo_attach();
#Serial.println("Servos initialized");
#Serial.println("Robot initialization Complete");


def servo_attach():
    for i in range(0,4):
        for j in range(0,3):
            #servo[i][j].attach(servo_pin[i][j]);
            #delay(100);




def servo_detach():
    for i in range(0,4):
        for j in range(0,3):
            #servo[i][j].detach();
            #delay(100);
"""/*
  - microservos service /timer interrupt function/50Hz
  - when set site expected,this function move the end point to it in a straight line
  - temp_speed[4][3] should be set before set expect site,it make sure the end point
   move in a straight line,and decide move speed.
   ---------------------------------------------------------------------------*/"""

def servo_service():
    micropython.alloc_emergency_exception_buf(100)

    for i in range(0,4):
        for j in range(0,3):
            if (abs(site_now[i][j] - site_expect[i][j]) >= abs(temp_speed[i][j])):
                site_now[i][j] += temp_speed[i][j]
            else:
                site_now[i][j] = site_expect[i][j]

        cartesian_to_polar(alpha, beta, gamma, site_now[i][0], site_now[i][1], site_now[i][2])
        polar_to_servo(i, alpha, beta, gamma)

    rest_counter+=1

"""  - set one of end points' expect site
  - this founction will set temp_speed[4][3] at same time
  - non - blocking function"""



def set_site(leg,x,y,z):
      length_x = 0
      length_y = 0
      length_z = 0


      if (x != KEEP)
        length_x = x - site_now[leg][0]
      elif (y != KEEP)
        length_y = y - site_now[leg][1]
      elif (z != KEEP)
        length_z = z - site_now[leg][2]

      length = sqrt(pow(length_x, 2) + pow(length_y, 2) + pow(length_z, 2))

      temp_speed[leg][0] = length_x / length * move_speed * speed_multiple
      temp_speed[leg][1] = length_y / length * move_speed * speed_multiple
      temp_speed[leg][2] = length_z / length * move_speed * speed_multiple

      if (x != KEEP)
        site_expect[leg][0] = x
      if (y != KEEP)
        site_expect[leg][1] = y
      if (z != KEEP)
        site_expect[leg][2] = z

"""- wait one of end points move to expect site
- blocking function"""

def wait_reach(leg):
      while (1):
        if (site_now[leg][0] == site_expect[leg][0]):
          if (site_now[leg][1] == site_expect[leg][1]):
            if (site_now[leg][2] == site_expect[leg][2]):
              break

"""- wait all of end points move to expect site
- blocking function"""

def wait_all_reach():
    for i in range(0,4):
        wait_reach(i)


"""- trans site from cartesian to polar
- mathematical model 2/2"""
def  cartesian_to_polar():





"""- trans site from polar to microservos
- mathematical model map to fact
- the errors saved in eeprom will be add"""

def polar_to_servo(leg,alpha,beta,gamma):
    if (leg==0):
        alpha = 90 - alpha
        beta = beta
        gamma += 90

    elif (leg==1):
        alpha += 90
        beta = 180 - beta
        gamma = 90 - gamma

    elif (leg == 2)

        alpha += 90
        beta = 180 - beta
        gamma = 90 - gamma

    elif (leg == 3)

        alpha = 90 - alpha
        beta = beta
        gamma += 90

    #servo[leg][0].write(alpha)
    #servo[leg][1].write(beta)
    #servo[leg][2].write(gamma)
