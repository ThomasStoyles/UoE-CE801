import rclpy
import math

from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf2_ros import TransformRegistration
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
#Connect to simulator
#export TURTLEBOT3_MODEL=BURGER
#ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

#Connect to robot
# In turtlefile ssh nameofrobot
# ros2 launch turtlebot3_bringup robot.launch.py
# In new terminal ssh nameofrobot

mynode_ = None
pub_ = None
regions_ = {
}
twstmsg_ = None

# near: 0.3 to 0.5
# medium: 0.2, 0.5, 0.7
# far: 0.5 to 0.7
distances = {
    'near': [0.3, 0.5],
    'medium': [0.2, 0.5, 0.7],
    'far': [0.5, 0.7]
}

# change to negatives on the robots
#slow, medium and fast
linear_speeds = {
    'slow': .1,
    'medium': .2,
    'fast': .4
}
# Left, right and stright 
angular_speeds = {
    'left': 0.3,
    'straight': 0.0,
    'right': -0.3

}
# first index in dict is front_min, second index in dict is front_right, third index is back_right, the values within them will be the linear/speed, angular/direction
rules = {
    "near":{
        "near":{
            "near": ('slow', 'left'),
            "medium": ('slow', 'left'),
            "far": ('slow', 'left'),
        },
        "medium":{
            "near": ('slow', 'left'),
            "medium": ('slow', 'left'),
            "far": ('slow', 'left'),
        },
        "far":{
            "near": ('slow', 'left'),
            "medium": ('slow', 'left'),
            "far": ('slow', 'left'),
        }
    },
    "medium":{
        "near":{
            "near": ('medium', 'left'),
            "medium": ('slow', 'left'),
            "far": ('slow', 'left'),
        },
        "medium":{
            "near": ('medium', 'right'),
            "medium": ('fast', 'straight'),
            "far": ('medium', 'left'),
        },
        "far":{
            "near": ('medium', 'right'),
            "medium": ('medium', 'right'),
            "far": ('fast', 'right'),
        }
    },
    "far":{
        "near":{
            "near": ('medium', 'left'),
            "medium": ('slow', 'left'),
            "far": ('slow', 'left'),
        },
        "medium":{
            "near": ('medium', 'right'),
            "medium": ('fast', 'straight'),
            "far": ('medium', 'left'),
        },
        "far":{
            "near": ('medium', 'right'),
            "medium": ('medium', 'right'),
            "far": ('fast', 'right'),
        }
    }
}

# main function attached to timer callback
def timer_callback():
    global pub_, twstmsg_
    if ( twstmsg_ != None ):
        pub_.publish(twstmsg_)


def clbk_laser(msg):
    global regions_, twstmsg_
   
    regions_ = {
        #LIDAR readings are anti-clockwise based on the graph shown within the lab
        'back_right':  find_nearest(msg.ranges[210:270]),
        'front_right': find_nearest(msg.ranges[270:330]),
        'right_front': find_nearest(msg.ranges[330:359]),
        'left_front': find_nearest(msg.ranges[0:30])
    }    
    twstmsg_= calculate_movement()

   
# Find nearest none zero value in the list
def find_nearest(list):
    x_list = filter(lambda item: item > 0.0, list)  
    return min(min(x_list, default=10), 10)

def calculate_movement():
    # These are the sensors that detect distances around the robot
    global regions_, angular_speeds, linear_speeds, distances

    # Here we set up zones for different distances
    front_distance = {'near': 0.0, 'medium': 0.0, 'far': 0.0}
    back_distance = {'near': 0.0, 'medium': 0.0, 'far': 0.0}
    front = {'near': 0.0, 'medium': 0.0, 'far': 0.0}

    # These print statements help us check the sensor readings
    print('front_right', regions_['front_right'])
    print('back_right', regions_['back_right'])
    print('front', min(regions_['right_front'], regions_['left_front']))
    print('')

    # defines regions again
    front_right = regions_['front_right']
    back_right = regions_['back_right']
    front_min = min(regions_['left_front'], regions_['right_front'])

    # Here membership function for 'front_right' distance
    '''
    1st memebrship of left falling and near = 1
    2nd membership of right falling and far = 1 
    checks all that could be 100% 1/2
    3rd checking the cross over between left side and middle
    else checking middle and right 
    '''
    if front_right < distances['near'][0]:
        front_distance['near'] = 1.0
    elif front_right > distances['far'][1]:
        front_distance['far'] = 1.0
    elif front_right < distances['medium'][1]:
        front_distance['near'] = (distances['near'][1] - front_right) / (distances['near'][1] - distances['near'][0])
        front_distance['medium'] = (front_right - distances['medium'][0]) / (distances['medium'][1] - distances['medium'][0])
    else:
        front_distance['medium'] = (distances['medium'][1] - front_right) / (distances['medium'][2] - distances['medium'][1])
        front_distance['far'] = (front_right - distances['far'][0]) / (distances['far'][1] - distances['far'][0])

    # Calculate membership function for 'back_right' distance
    if back_right < distances['near'][0]:
        back_distance['near'] = 1.0
    elif back_right > distances['far'][1]:
        back_distance['far'] = 1.0
    elif back_right < distances['medium'][1]:
        back_distance['near'] = (distances['near'][1] - back_right) / (distances['near'][1] - distances['near'][0])
        back_distance['medium'] = (back_right - distances['medium'][0]) / (distances['medium'][1] - distances['medium'][0])
    else:
        back_distance['medium'] = (distances['medium'][1] - back_right) / (distances['medium'][2] - distances['medium'][1])
        back_distance['far'] = (back_right - distances['far'][0]) / (distances['far'][1] - distances['far'][0])

    # Calculate membership function for 'front' distance
    if front_min < distances['near'][0]:
        front['near'] = 1.0
    elif front_min > distances['far'][1]:
        front['far'] = 1.0
    elif front_min < distances['medium'][1]:
        front['near'] = (distances['near'][1] - front_min) / (distances['near'][1] - distances['near'][0])
        front['medium'] = (front_min - distances['medium'][0]) / (distances['medium'][1] - distances['medium'][0])
    else:
        front['medium'] = (distances['medium'][1] - front_min) / (distances['medium'][2] - distances['medium'][1])
        front['far'] = (front_min - distances['far'][0]) / (distances['far'][1] - distances['far'][0])


    # calculate the robots movement based on the fuzzy logic
    sum_linear = 0.0
    sum_angular = 0.0
    sum_weights = 0.0

    # sum of the linear and angular memberships and calulates speeds provided they arent 0 
    for x in front_distance:
        for y in back_distance:
            for i in front:
                activation = min(front[i], front_distance[x], back_distance[y])
                if activation > 0:
                    sum_weights += activation
                    linear, angular = rules[i][x][y]
                    sum_linear += activation * linear_speeds[linear]
                    sum_angular += activation * angular_speeds[angular]

    msg = Twist()
    if sum_weights != 0:
        msg.linear.x = sum_linear / sum_weights
        msg.angular.z = sum_angular / sum_weights

    return msg



#used to stop the robot
def stop():
    global pub_
    msg = Twist()
    msg.angular.z = 0.0
    msg.linear.x = 0.0
    pub_.publish(msg)


def main():
    global pub_, mynode_

    rclpy.init()
    mynode_ = rclpy.create_node('reading_laser')

    # define qos profile (the subscriber default 'reliability' is not compatible with robot publisher 'best effort')
    qos = QoSProfile(
        depth=10,
        reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
    )

    # publisher for twist velocity messages (default qos depth 10)
    pub_ = mynode_.create_publisher(Twist, '/cmd_vel', 10)

    # subscribe to laser topic (with our qos)
    sub = mynode_.create_subscription(LaserScan, '/scan', clbk_laser, qos)

    # Configure timer
    timer_period = 0.2  # seconds
    timer = mynode_.create_timer(timer_period, timer_callback)

    # Run and handle keyboard interrupt (ctrl-c)
    try:
        rclpy.spin(mynode_)
    except KeyboardInterrupt:
        stop()  # stop the robot
    except:
        stop()  # stop the robot
    finally:
        # Clean up
        mynode_.destroy_timer(timer)
        mynode_.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()