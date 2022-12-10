"""grocery controller."""

# Nov 2, 2022

from controller import Robot, Motor, Camera, RangeFinder, Lidar, Keyboard
import math
import numpy as np
from matplotlib import pyplot as plt
import random

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
pose_x     = 0
pose_y     = 0
pose_theta = 0

vL = 0
vR = 0

lidar_sensor_readings = [] # List to hold sensor readings
lidar_offsets = np.linspace(-LIDAR_ANGLE_RANGE/2., +LIDAR_ANGLE_RANGE/2., LIDAR_ANGLE_BINS)
lidar_offsets = lidar_offsets[83:len(lidar_offsets)-83] # Only keep lidar readings not blocked by robot chassis

map = None



# ------------------------------------------------------------------
keyboard = robot.getKeyboard()
keyboard.enable(timestep)

GRID_RESOLUTION = 600
map = np.empty((600,600)) 
mapMask = np.zeros((600,600))
OFFSET_X = 30/2
OFFSET_Y = 16.1/2
step_size = 10

class Node:

    def __init__(self, pt, parent=None):
        self.point = pt # n-Dimensional point
        self.parent = parent # Parent node
        self.path_from_parent = []
        
def state_is_valid(point):
        if mapMask[int(point[0])][int(point[1])] == 1:
            return False
        return True

def get_nearest_vertex(node_list, q_point):

    #print(len(node_list))
    bestNode = Node(None, parent=None)
    bestDistance = math.inf
    for nodes in node_list:
        #print(nodes.point)
        sum = 0.0
        for i in range(len(q_point)):
            #print(nodes.point[i])
            sum+= (nodes.point[i] - q_point[i])**2
        if(sum < bestDistance):
            #print(sum)
            bestDistance = sum
            bestNode = nodes
            #print("change node")
    return bestNode;

def distance(from_point, to_point):
    distance = 0.0
    for i in range(len(from_point)):
        distance += (from_point[i] - to_point[i])**2
    distance = distance**(.5)
    return distance

def steer(from_point, to_point, delta_q):

    dist = distance(from_point, to_point)
    if(dist > delta_q):
        ratio = (delta_q/dist)
        for i in range(len(to_point)):
            to_point[i] = ((1-ratio)*from_point[i]) + (ratio * to_point[i])
            
    # TODO Use the np.linspace function to get 10 points along the path from "from_point" to "to_point"
    path = np.linspace(from_point,to_point,10)
    return path

def check_path_valid(path, state_is_valid):

    for point in path:
        if not state_is_valid(point):
            return False
    return True


def rrt(state_is_valid, starting_point, goal_point, k, delta_q):

    node_list = []
    node_list.append(Node(starting_point, parent=None)) # Add Node at starting point with no parent
    nodes = 1;
    while(nodes <= k):
        newPoint = [random.randint(0, 599), random.randint(0, 599)]
        
        if goal_point is not None:
            if random.random() < 0.05: 
                newPoint = goal_point
        
        parent = get_nearest_vertex(node_list, newPoint)
        path = steer(parent.point, newPoint, delta_q)
        validPoint = check_path_valid(path, state_is_valid)
        
        if validPoint:
            newNode = Node(path[-1], parent)
            newNode.path_from_parent = path
            node_list.append(newNode)
            nodes+=1
            if goal_point is not None:
                distToGoal = distance(newNode.point, goal_point)
                if distToGoal <= 1:
                    break;
    return node_list

if 1 == 1:
    map = np.load("map.npy")
        # plt.imshow(np.fliplr(map))
        # plt.show()
        
    
        # Part 2.2: Compute an approximation of the “configuration space”
     
    print(len(map[1]))
    print(len(map[0]))
    for x in range(0,len(map[0])):
        for y in range(0,len(map[1])):
            if map[x][y] == 1:
                for row in range(y-4,y+4):
                    for col in range(x-4,x+4):
                        if row in range(0, 600):
                            if col in range(0,600):
                                mapMask[row][col] = 1
    map = mapMask
    map = np.rot90(map)
    map = np.flipud(map)
    #map = np.fliplr(map)
    pose_y = -gps.getValues()[1]
    pose_x = -gps.getValues()[0]
    plt.imshow(map)
    plt.show()
    
    nodeStart = [50, 550]
    nodeEnd = [100,100]
    path = rrt(state_is_valid, nodeStart, nodeEnd, 5000, 10)[-1]

    waypoints = []
    parent = path
    count = 0
    while parent is not None:
        x = ((parent.point[1]/20)-OFFSET_X)
        y = (((parent.point[0])/20)-OFFSET_Y)
        waypoints.append((x,y))
        parent = parent.parent
    print ('waypoints',waypoints)
    #plt.imshow(mapMask)
    #plt.show()
    # Part 2.4: Turn paths into waypoints and save on disk as path.npy and visualize it
    np.save("path.npy", waypoints)

    

  

#print(path)
map = np.empty((600,600))    
# Define the grid resolution
GRID_RESOLUTION = 0.1

# Define the size of the floor


# Define the starting position of the robot
ROBOT_START_X = 0
ROBOT_START_Y = -5

mode = 'manual'

waypoints = []

if mode == 'driving':
    # Part 3.1: Load path from disk and visualize it
    
    waypoints = [] # Replace with code to load your path
    waypoints = np.load("path.npy")  
    waypointIndex = 1
    goal= abs(waypoints[1])
    path = waypoints
    state = 0
state = 0
# Helper Functions


gripper_status="closed"



# Main Loop
print('huh?')
while robot.step(timestep) != -1:
    vL = 0
    vR = 0
    pose_y = gps.getValues()[1]
    pose_x = gps.getValues()[0]

    n = compass.getValues()
    rad = ((math.atan2(n[0], n[1])))#-1.5708)
    pose_theta = rad
    rotation_matrix = np.array([[math.cos(pose_theta), -math.sin(pose_theta)],[math.sin(pose_theta),  math.cos(pose_theta)]])
    lidar_sensor_readings = lidar.getRangeImage()
    lidar_sensor_readings = lidar_sensor_readings[83:len(lidar_sensor_readings)-83]
    for i, rho in enumerate(lidar_sensor_readings):
        alpha = lidar_offsets[i]

        if rho > LIDAR_SENSOR_MAX_RANGE:
            continue

        # The Webots coordinate system doesn't match the robot-centric axes we're used to
        rx = -math.cos(alpha)*rho
        ry = math.sin(alpha)*rho


        # Convert detection from robot coordinates into world coordinates
        wx =  math.cos(pose_theta)*rx - math.sin(pose_theta)*ry + pose_x + OFFSET_X
        wy =  +(math.sin(pose_theta)*rx + math.cos(pose_theta)*ry) + pose_y + OFFSET_Y
        #print('x',wx,'y',wy)


        if rho < LIDAR_SENSOR_MAX_RANGE:
            # Part 1.3: visualize map gray values.
            testerx, testery = int(wx*20),int(wy*20)
            if testerx >= 600:
                testerx = 599
            if testerx < 0:
                testerx = 0
            if testery >= 600:
                testery=599
            if testery < 0:
                testery = 0
                
            g = map[testerx][testery]
            if g>1:
                g=1
            map[testerx][testery]+=.005
            color = (g*256**2+g*256+g)*255
            display.setColor(int(color))
            display.drawPixel(int(wx*30),360-int(wy*30))
    
    if mode=='manual':
        key = keyboard.getKey()
        if key == keyboard.LEFT :
            vL = -MAX_SPEED
            vR = MAX_SPEED
        elif key == keyboard.RIGHT :
            vL = MAX_SPEED
            vR = -MAX_SPEED
        elif key == keyboard.UP :
            vL = MAX_SPEED
            vR = MAX_SPEED
        elif key == keyboard.DOWN :
            vL = -MAX_SPEED
            vR = -MAX_SPEED
        elif key == ord('S'):
            mappingMap = map > 0.9
            mappingMap = mappingMap * 1
            print(mappingMap)
            np.save("map.npy",mappingMap)
            print("Map file saved")
        else: # slow down
            vL *= 0.75
            vR *= 0.75
            
    display.setColor(int(0xFF0000))
    display.drawPixel(int(pose_x*20+OFFSET_X),int(pose_y*20+OFFSET_Y))
    
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
    
    if mode == 'driving':
        atObject = False
        PossError = math.sqrt((goal[0] - pose_x )**2 + ( goal[1] - pose_y)**2)
        print(pose_x)
        print(pose_y)
        print(PossError)
        print(goal)

        bearError = (math.atan2(goal[1]-pose_y,goal[0]-pose_x)) - (pose_theta + math.pi)
        print(bearError)

        if bearError < -3.1415: 
            bearError += 6.283 
        if bearError > 6.283:
            bearError -= 6.283
        # Heading error:
        gain =.25
        
        if(abs(PossError)<gain):
            waypointIndex = waypointIndex + 1
            print(waypointIndex)
            if(waypointIndex >= len(waypoints)):
                robot_parts[MOTOR_LEFT].setVelocity(0)
                robot_parts[MOTOR_RIGHT].setVelocity(0)
                exit(0)
            else:
                goal = abs(waypoints[waypointIndex])
            
        
        pass
        
        # ##STEP 2.2: Feedback Controller
    
        dX = PossError
        dTheta = 70 * bearError
      
        pass
        
        
        # ##STEP 1: Inverse Kinematics Equations (vL and vR as a function dX and dTheta)
        # ##Note that vL and vR in code is phi_l and phi_r on the slides/lecture
        vL = (2  * dX - dTheta * AXLE_LENGTH) / 2
        vR = (2 * dX + dTheta * AXLE_LENGTH) / 2 
        
        pass
        
        # ##STEP 2.3: Proportional velocities
        max_val = max(abs(vL), abs(vR), MAX_SPEED)
        vL += max_val
        vR += max_val
        maxVar = max(vL, vR)
        
        vL = (vL / maxVar - 0.5) * 1
        vR = (vR / maxVar - 0.5) * 1
    
    
       
        
        pass
    
        # ##STEP 2.4: Clamp wheel speeds
        if abs(vL) > MAX_SPEED:
            
            vL =  MAX_SPEED
        if abs(vR) > MAX_SPEED:
            
            vR =  MAX_SPEED
            
        if atObject == True:
            firstObject = Camera.getRecognitionObjects()[0]
            id = firstObject.get_id()
            position = firstObject.get_position()
    robot_parts["wheel_left_joint"].setVelocity(vL)
    robot_parts["wheel_right_joint"].setVelocity(vR)