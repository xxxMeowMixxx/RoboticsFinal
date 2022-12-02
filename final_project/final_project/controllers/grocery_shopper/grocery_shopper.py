"""grocery controller."""

# Nov 2, 2022

from controller import Robot, Keyboard
from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink
import math
import time
import numpy as np

#Initialization
print("=== Initializing Grocery Shopper...")
#Consts
MAX_SPEED = 7.0  # [rad/s]
MAX_SPEED_MS = 0.633 # [m/s]
AXLE_LENGTH = 0.4044 # m
MOTOR_LEFT = 10
MOTOR_RIGHT = 11
N_PARTS = 12
LIDAR_ANGLE_BINS = 667
LIDAR_SENSOR_MAX_RANGE = 5.5 # Meters
LIDAR_ANGLE_RANGE = math.radians(240)

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# The Tiago robot has multiple motors, each identified by their names below
part_names = ("head_2_joint", "head_1_joint", "torso_lift_joint", "arm_1_joint",
              "arm_2_joint",  "arm_3_joint",  "arm_4_joint",      "arm_5_joint",
              "arm_6_joint",  "arm_7_joint",  "wheel_left_joint", "wheel_right_joint",
              "gripper_left_finger_joint","gripper_right_finger_joint")

# 


# All motors except the wheels are controlled by position control. The wheels
# are controlled by a velocity controller. We therefore set their position to infinite.
target_pos = (0.0, 0.0, 0.35, 0.07, 1.02, -3.16, 1.27, 1.32, 0.0, 1.41, 'inf', 'inf',0.045,0.045)

robot_parts={}
for i, part_name in enumerate(part_names):
    robot_parts[part_name]=robot.getDevice(part_name)
    robot_parts[part_name].setPosition(float(target_pos[i]))
    robot_parts[part_name].setVelocity(robot_parts[part_name].getMaxVelocity() / 2.0)

# Enable gripper encoders (position sensors)
left_gripper_enc=robot.getDevice("gripper_left_finger_joint_sensor")
right_gripper_enc=robot.getDevice("gripper_right_finger_joint_sensor")
left_gripper_enc.enable(timestep)
right_gripper_enc.enable(timestep)
# Enable Camera
camera = robot.getDevice('camera')
camera.enable(timestep)
camera.recognitionEnable(timestep)

# Enable GPS and compass localization
gps = robot.getDevice("gps")
gps.enable(timestep)
compass = robot.getDevice("compass")
compass.enable(timestep)

# Enable LiDAR
lidar = robot.getDevice('Hokuyo URG-04LX-UG01')
lidar.enable(timestep)
lidar.enablePointCloud()

# Enable display
display = robot.getDevice("display")

# Odometry
pose_x     = -4.79645360826846
pose_y     = -0.0005497518444407955
tracker     = 0.0005497518444407955
pose_theta = 0
pose_theta2 = 0

vL = 0
vR = 0

lidar_sensor_readings = [] # List to hold sensor readings
lidar_offsets = np.linspace(-LIDAR_ANGLE_RANGE/2., +LIDAR_ANGLE_RANGE/2., LIDAR_ANGLE_BINS)
lidar_offsets = lidar_offsets[83:len(lidar_offsets)-83] # Only keep lidar readings not blocked by robot chassis

map = None

print("done init")



#Drake Morley Init sensors
sensors = ( #position sensors
"arm_1_joint_sensor",
"arm_2_joint_sensor",
"arm_3_joint_sensor",
"arm_4_joint_sensor",
"arm_5_joint_sensor",
"arm_6_joint_sensor",
"arm_7_joint_sensor",
"gripper_left_finger_joint_sensor",
"gripper_right_finger_joint_sensor",
"head_1_joint_sensor",
"head_2_joint_sensor",
"torso_lift_joint_sensor",
"wheel_left_joint_sensor",
"wheel_right_joint_sensor")
keyboard = robot.getKeyboard()
keyboard.enable(timestep)


#setUpchains
#Drake Morley
#on initilization read the urdf file to creata  series of chains to do kenimatics with
#---------------------------------------------------------------------------------------
base_elements=["base_link", "base_link_Torso_joint", "Torso", "torso_lift_joint", "torso_lift_link", "torso_lift_link_TIAGo front arm_11367_joint", "TIAGo front arm_11367"]

my_chain = Chain.from_urdf_file("robot_urdf.urdf", last_link_vector=[0.004, 0,-0.1741], base_elements=["base_link", "base_link_Torso_joint", "Torso", "torso_lift_joint", "torso_lift_link", "torso_lift_link_TIAGo front arm_11367_joint", "TIAGo front arm_11367"])
print(my_chain.links)

ken_names = ("head_2_joint", "head_1_joint", "arm_1_joint",
              "arm_2_joint",  "arm_3_joint",  "arm_4_joint",      "arm_5_joint",
              "arm_6_joint",  "arm_7_joint",  "wheel_left_joint", "wheel_right_joint")
              
              
for id in range(len(my_chain.links)):
    linkedObj = my_chain.links[id]
    if linkedObj.name not in ken_names:
        print("disabling " + linkedObj.name)
        my_chain.active_links_mask[id] = False
        
motors = []
for linkObj in my_chain.links:
    if linkObj.name in ken_names:
        motor = robot.getDevice(linkObj.name)
        motor.setVelocity(1)
        positionSensor = motor.getPositionSensor()
        positionSensor.enable(timestep)
        motors.append(motor)
print(motors)
print(my_chain.links)
#---------------------------------------------------------------------------------------
#END of setup for kenimatics
# --------------------------------------------------------------------------------------
# Helper Functions





#kenimatic calculator ------------------------------------------------------------------
#Drake Morley
#using 
#pip install git+https://github.com/alters-mit/ikpy.git#egg=ikpy
#refferenced from https://gist.github.com/ItsMichal/4a8fcb330d04f2ccba582286344dd9a7

#Pass a target in the form [x(forward/backwards),y(side to side),z(up/down)] 

#RETURNS: NONE

#to kenimaticsToTarget to calculate and set the position of where the motors need to go
#the motors then get there desired postion set to that
#no collission detection inate however if a series of points were passed
#IE: from [0,0,0] to [1,0,1] one could pass
#[1,0,0] wait for it to complete and then pass [1,0,1] so that way the machine
#moved in a L motion as opposed to straight to it 

#does not natively use the torso_lift to work feel free to roughly throw it up and down as needed

#pos [0,0,0] is locolized at the base of the torso box

def kenimaticsToTarget(target):
    initial_position = [0,0,0,0] + [m.getPositionSensor().getValue() for m in motors] + [0,0,0,0]
    ikResults = my_chain.inverse_kinematics(target, initial_position=initial_position,  target_orientation = [0,0,1], orientation_mode="Y")
    for res in range(len(ikResults)):
    
    
        if my_chain.links[res].name in ken_names:
            robot.getDevice(my_chain.links[res].name).setPosition(ikResults[res])
    print(ikResults)
    return(ikResults)
##END OF FUNCTION (kenimaticsToTarget)-------------------------------------------------
    
    
    
    

#kenimatic helper --------------------------------------------
#Drake Morley

# takes in: target in form [x(forward/backwards),y(side to side),z(up/down)] see above

#returns: NONE

# helper for kenimaticsToTarget. If you want to put waypoints I recomend doing it here or in main
# calling .inverse_kinematics at every step will lag the computer
def kenimaticHelper(target1):
    currentplan = kenimaticsToTarget(target1)
##END OF FUNCTION (kenimatic helper)-------------------------------------------------





#Localization-------------------------------------------------------------------
#Drake Morley

#takes in: velocity left, velocity right, pose of x,
#tracker(IE:pose of y before being flipped) pose of theta

#returns: updated pose of x, updated pose of y, updated pose of theta
#and tracker value for future calculations

# as of current theta is tied to compass after certain errors due to the jerkyness of manual
# mode. Is possible to tie y and x to it as well if errors get out of hand
def doOdomatry(vL, vR, pose_x, tracker, pose_theta):
    new_vL = vL
    vL = vL/MAX_SPEED * MAX_SPEED_MS
    new_vR = vR
    vR = vR/MAX_SPEED * MAX_SPEED_MS
    
    r = (MAX_SPEED_MS)/MAX_SPEED
    
    fSpeed = (vR/r) * (r/2) + (vL/r) * (r/2)
    rSpeed = (vR/r) * (r/AXLE_LENGTH) - (vL/r) * (r/AXLE_LENGTH)
    
    timestepsec = timestep/1000 #seconds

    i_theta_change = (timestepsec * rSpeed)
    i_x_change = timestepsec * (fSpeed * math.cos(pose_theta))
    i_y_change = timestepsec * (fSpeed * math.sin(pose_theta))
    
    # update step
    pose_x += i_x_change
    tracker += i_y_change
    pose_theta -= i_theta_change/2
    pose_y= -tracker
    if pose_theta < 0:
        pose_theta = pose_theta+6.28319
    
    n = compass.getValues()
    rad = ((math.atan2(n[1], n[0])-1.5708))#-1.5708)
    if rad < 0:
        rad = rad+6.28319
    gpsPose_y = gps.getValues()[1]
    gpsPose_x = gps.getValues()[0]
    
    
    if (abs(rad - pose_theta) > .05):#the odomatry works but due to me wanting to jam buttons
        pose_theta=rad               #in manual mode this has been added
                                     #feel free to comment out but be gental
    # print(gpsPose_x, gpsPose_y, rad)
    # print("pose",pose_x,pose_y,pose_theta)
    return pose_x,pose_y,pose_theta,tracker
##END OF FUNCTION (doOdomatry)-------------------------------------------------







# Main Loop
robot_parts["torso_lift_joint"].setPosition(0.1)
gripper_status="open"

while robot.step(timestep) != -1:
    target = [.7,0,.5]
    overBasketTarget = [.3,0,.5] #these cords will take the arm over the basket
    

    key = keyboard.getKey()
    
    if key == keyboard.LEFT :
        vL = -MAX_SPEED
        vR = MAX_SPEED
    elif key == keyboard.RIGHT:
        vL = MAX_SPEED
        vR = -MAX_SPEED
    elif key == keyboard.UP:
        vL = MAX_SPEED
        vR = MAX_SPEED
    elif key == keyboard.DOWN:
        vL = -MAX_SPEED
        vR = -MAX_SPEED
    elif key == ord(' '):
        vL = 0
        vR = 0
    elif key == ord('E'):
        kenimaticHelper([0.3,target[1],target[2]])
    elif key == ord('D'):
        kenimaticHelper(target)
    elif key == ord('C'):
        if(gripper_status=="open"):
            # Close gripper, note that this takes multiple time steps...
            robot_parts["gripper_left_finger_joint"].setPosition(0)
            robot_parts["gripper_right_finger_joint"].setPosition(0)
            if right_gripper_enc.getValue()<=0.005:
                gripper_status="closed"
        else:
            # Open gripper
            robot_parts["gripper_left_finger_joint"].setPosition(0.045)
            robot_parts["gripper_right_finger_joint"].setPosition(0.045)
            if left_gripper_enc.getValue()>=0.044:
                gripper_status="open"
    else: # slow down
        vL *= 0.75
        vR *= 0.75
    
    Tpose_x,Tpose_y,Tpose_theta,tracker = doOdomatry(vL,vR,pose_x,tracker,pose_theta)
    pose_x,pose_y,pose_theta = Tpose_x,Tpose_y,Tpose_theta
    
    robot_parts["wheel_left_joint"].setVelocity(vL)
    robot_parts["wheel_right_joint"].setVelocity(vR)
