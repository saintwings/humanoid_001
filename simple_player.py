import multiprocessing

import argparse
import serial

from lowlevel_control import Lowlevel
import time


print("Number of cpu : ", multiprocessing.cpu_count())



parser = argparse.ArgumentParser(description='Enter Comports : con, head' )

parser.add_argument('--headType',help='motor type on head Mx , Mx ',default='Mx')
parser.add_argument('-c','--con',help='Controller Comport',default='/dev/ttyACM0')
parser.add_argument('--head',help='Head Comport',default='/dev/ttyUSB0')
parser.add_argument('--D_version',help='Dynamixel Protocal Version',default='2')

args = parser.parse_args()

if args.D_version == '1':
    from dynamixel_control import Dynamixel
    print("Dynamixel V.1")
else:
    from dynamixel_control2 import Dynamixel
    print("Dynamixel V.2")






robot_head_type = args.headType
print("HEAD TYPE =", robot_head_type)


step_flag =False

motorHead = Dynamixel(args.head,115200)
motorHead.connect()

lowLevel_Control = Lowlevel(args.con,115200)
lowLevel_Control.connect()

motorHead.setEnableMotorTurque(41)
motorHead.setEnableMotorTurque(42)
motorHead.setStatusReturnLevel(1)

if robot_head_type == 'Mx':
    motorHead.setDeviceMoving(41, robot_head_type, 2048, 1023, 1023)
    motorHead.setDeviceMoving(42, robot_head_type, 2048, 1023, 1023)
else:
    motorHead.setDeviceMoving(41, robot_head_type, 512, 1023, 1023)
    motorHead.setDeviceMoving(42, robot_head_type, 512, 1023, 1023)

head_step = [0,0] # motor pan41,tilt42

pygame.display.set_mode([screen_width,screen_height])
while True:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit(); #sys.exit() if sys is imported
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_w:
                print("Forward ")
                lowLevel_Control.forward_walk(step_flag)
            if event.key == pygame.K_a:
                print("Left Slide")
                lowLevel_Control.left_walk(step_flag)
            if event.key == pygame.K_s:
                print("Backward")
                lowLevel_Control.backward_walk(step_flag)
            if event.key == pygame.K_d:
                print("Right Slide")
                lowLevel_Control.right_walk(step_flag)
            if event.key == pygame.K_q:
                print("Turn Left")
                lowLevel_Control.turn_left(step_flag)
            if event.key == pygame.K_e:
                print("Turn Right")
                lowLevel_Control.turn_right(step_flag)
            if event.key == pygame.K_SPACE:
                print("Stop")
                lowLevel_Control.stop_walk()
            if event.key == pygame.K_z:
                if step_flag == False:
                    step_flag = True
                   
                elif step_flag == True:
                    step_flag = False
                print(step_flag)

            #######################################################
            if event.key == pygame.K_KP1:
                print("Sit")
                lowLevel_Control.sit()
            if event.key == pygame.K_KP2:
                print("Stand")
                lowLevel_Control.stand()
            if event.key == pygame.K_KP3:
                print("Getup")
                lowLevel_Control.getup()
            if event.key == pygame.K_KP4:
                print("Left Kick")
                lowLevel_Control.left_kick()
            if event.key == pygame.K_KP6:
                print("Right Kick")
                lowLevel_Control.right_kick()
            if event.key == pygame.K_KP7:
                print("Left Save")
                lowLevel_Control.left_save()
            if event.key == pygame.K_KP9:
                print("Right Save")
                lowLevel_Control.right_save()


            #######################################################
            ######### HEAD CONTROL ###############
            if event.key == pygame.K_UP:
                print("Head Up")
                head_step[1] = head_step[1] + (-1)
                if robot_head_type == 'Mx':
                    #position = motorHead.getMotorPosition(42)
                    
                    motorHead.setDeviceMoving(42, robot_head_type, 2018 + (350*head_step[1]), 300, 1023)
                else:
                    #position = motorHead.getMotorPosition(42)
                    motorHead.setDeviceMoving(42, robot_head_type, 512 + (70*head_step[1]), 300, 1023)
            if event.key == pygame.K_DOWN:
                head_step[1] = head_step[1] + (1)
                print("Head Down")
                if robot_head_type == 'Mx':
                    #position = motorHead.getMotorPosition(42)
                    motorHead.setDeviceMoving(42, robot_head_type, 2018 + (350*head_step[1]), 300, 1023)
                else:
                    #position = motorHead.getMotorPosition(42)
                    motorHead.setDeviceMoving(42, robot_head_type, 512 + (70*head_step[1]), 300, 1023)
            if event.key == pygame.K_LEFT:
                print("Head LEFT")
                head_step[0] = head_step[0] + (1)
                if robot_head_type == 'Mx':
                    #position = motorHead.getMotorPosition(41)
                    motorHead.setDeviceMoving(41, robot_head_type, 2018 + (350*head_step[0]), 300, 1023)
                else:
                    #position = motorHead.getMotorPosition(41)
                    motorHead.setDeviceMoving(41, robot_head_type, 512 + (70*head_step[0]), 300, 1023)
            if event.key == pygame.K_RIGHT:
                print("Head RIGHT")
                head_step[0] = head_step[0] + (-1)
                if robot_head_type == 'Mx':
                    #position = motorHead.getMotorPosition(41)
                    motorHead.setDeviceMoving(41, robot_head_type, 2018 + (350*head_step[0]), 300, 1023)
                else:
                    #position = motorHead.getMotorPosition(41)
                    motorHead.setDeviceMoving(41, robot_head_type, 512 + (70*head_step[0]), 300, 1023)
            if event.key == pygame.K_RSHIFT :
                head_step[0] = 0
                head_step[1] = 0
                print("Head Center")
                if robot_head_type == 'Mx':
                    motorHead.setDeviceMoving(41, robot_head_type, 2048, 1023, 1023)
                    motorHead.setDeviceMoving(42, robot_head_type, 2048, 1023, 1023)
                else:
                    motorHead.setDeviceMoving(41, robot_head_type, 512, 1023, 1023)
                    motorHead.setDeviceMoving(42, robot_head_type, 512, 1023, 1023)
            if event.key == pygame.K_RCTRL :
                head_step[0] = 0
                head_step[1] = 0
                if robot_head_type == 'Mx':
                    motorHead.setDeviceMoving(41, robot_head_type, 2048, 1023, 1023)
                    motorHead.setDeviceMoving(42, robot_head_type, 2048, 1023, 1023)
                    time.sleep(0.5)
                    motorHead.setDeviceMoving(41, robot_head_type, (2048-1200), 200, 1023)
                    time.sleep(0.8)
                    motorHead.setDeviceMoving(42, robot_head_type, (2048+400), 1023, 1023)
                    motorHead.setDeviceMoving(41, robot_head_type, (2048+1200), 200, 1023)
                    time.sleep(1.6)
                    motorHead.setDeviceMoving(42, robot_head_type, (2048), 1023, 1023)
                    motorHead.setDeviceMoving(41, robot_head_type, (2048), 200, 1023)

                else:

                    motorHead.setDeviceMoving(41, robot_head_type, 512, 1023, 1023)
                    motorHead.setDeviceMoving(42, robot_head_type, 512, 1023, 1023)
                    time.sleep(0.5)
                    motorHead.setDeviceMoving(41, robot_head_type, (512-300), 200, 1023)
                    time.sleep(0.8)
                    motorHead.setDeviceMoving(42, robot_head_type, (512+100), 1023, 1023)
                    motorHead.setDeviceMoving(41, robot_head_type, (512+300), 200, 1023)
                    time.sleep(1.6)
                    motorHead.setDeviceMoving(42, robot_head_type, (512), 1023, 1023)
                    motorHead.setDeviceMoving(41, robot_head_type, (512), 200, 1023)


            

            