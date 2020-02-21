# from rws2020_msgs.msg import MakeAPlay
# from rws2020_lib.utils import movePlayer, randomizePlayerPose, getDistanceAndAngleToTarget

#!/usr/bin/env python
import random

import math

import rospy
import tf
from geometry_msgs.msg import Transform, Quaternion
import numpy as np


def getDistanceAndAngleToTarget(tf_listener, my_name, target_name,
                                time=rospy.Time(0), max_time_to_wait=1.0):
    try:
        tf_listener.waitForTransform(my_name, target_name, time, rospy.Duration(max_time_to_wait))
        (trans, rot) = tf_listener.lookupTransform(my_name, target_name, time)
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, tf.Exception):
        rospy.logwarn(my_name + ': Could not get transform from ' + my_name + ' to ' + target_name)
        return None, None

    # compute distance and angle
    x, y = trans[0], trans[1]
    distance = math.sqrt(x ** 2 + y ** 2)
    angle = math.atan2(y, x)
    return distance, angle


def randomizePlayerPose(transform, arena_radius=8):
    """
    Randomizes the initial pose of a player. Based on the code by MGomes.
    :param transform: a geometry_msgs.msg.Transform() which will have the values of x,y and yaw randomized.
    :param arena_radius: the radius of the arena inside which the player can be positioned.
    """
    initial_r = arena_radius * random.random()
    initial_theta = 2 * math.pi * random.random()
    initial_x = initial_r * math.cos(initial_theta)
    initial_y = initial_r * math.sin(initial_theta)
    initial_rotation = 2 * math.pi * random.random()
    transform.translation.x = initial_x
    transform.translation.y = initial_y
    q = tf.transformations.quaternion_from_euler(0, 0, initial_rotation)
    transform.rotation = Quaternion(q[0], q[1], q[2], q[3])


def movePlayer(tf_broadcaster, player_name, transform_now, vel, angle, max_vel):
    """
    Moves a player given its currrent pose, a velocity, and angle, and a maximum velocity
    :param tf_broadcaster: Used to publish the new pose of the player
    :param player_name:  string with the name of the player (must coincide with the name of the tf frame_id)
    :param transform_now: a geometry_msgs.msg.Transform() containing the current pose. This variable is updated with
                          the new player pose
    :param vel: velocity of displacement to take in x axis
    :param angle: angle to turn, limited by max_angle (pi/30)
    :param max_vel: maximum velocity or displacement based on the selected animal
    """
    max_angle = math.pi / 30

    if angle > max_angle:
        angle = max_angle
    elif angle < -max_angle:
        angle = -max_angle

    if vel > max_vel:
        vel = max_vel

    T1 = transform_now

    T2 = Transform()
    T2.rotation = tf.transformations.quaternion_from_euler(0, 0, angle)
    T2.translation.x = vel
    matrix_trans = tf.transformations.translation_matrix((T2.translation.x,
                                                          T2.translation.y,
                                                          T2.translation.z))

    matrix_rot = tf.transformations.quaternion_matrix((T2.rotation[0],
                                                       T2.rotation[1],
                                                       T2.rotation[2],
                                                       T2.rotation[3]))
    matrixT2 = np.matmul(matrix_trans, matrix_rot)

    matrix_trans = tf.transformations.translation_matrix((T1.translation.x,
                                                          T1.translation.y,
                                                          T1.translation.z))

    matrix_rot = tf.transformations.quaternion_matrix((T1.rotation.x,
                                                       T1.rotation.y,
                                                       T1.rotation.z,
                                                       T1.rotation.w))
    matrixT1 = np.matmul(matrix_trans, matrix_rot)

    matrix_new_transform = np.matmul(matrixT1, matrixT2)

    quat = tf.transformations.quaternion_from_matrix(matrix_new_transform)
    trans = tf.transformations.translation_from_matrix(matrix_new_transform)

    T1.rotation = Quaternion(quat[0], quat[1], quat[2], quat[3])
    T1.translation.x = trans[0]
    T1.translation.y = trans[1]
    T1.translation.z = trans[2]

    tf_broadcaster.sendTransform(trans, quat, rospy.Time.now(), player_name, "world")

class Player:

    def __init__(self, player_name):
        self.player_name = player_name

        rospy.logwarn('I am ' + self.player_name)

        red_team = rospy.get_param('/red_team')
        green_team = rospy.get_param('/green_team')
        blue_team = rospy.get_param('/blue_team')

        print('red_team = ' + str(red_team))
        print('green_team = ' + str(green_team))
        print('blue_team = ' + str(blue_team))

        if self.player_name in red_tea:
            self.my_team = 'red'
            self.prey = 'green'
            self.hunter = 'blue'
        elif self.player_name in green_team:
            self.my_team='green'
            self.prey_team = 'blue'
            self.hunter_team = 'red'
        elif self.player_name in blue_team:
            self.my_team = 'blue'
            self.pray_team = 'red'
            self.hunter_team = 'green'
        else
            rospy.logger('My name is not in my team. I want to play')
            exit(0)

        rospy.logwarn('I am ' + self.player_name + ' and I an on team ' + self.my_team + '. ' + self.pray_team + ' players are all going to die')

        self.br = tf.TransformBroadcaster()
        self.transform = Transform()
        randomizePlayerPose(self.transform)

        rospy.Subscriber("make_a_play", MakeAPlay, self.makeAPlayCallBack)  # Subscribe make a play msg

    def makeAPlayCallBack(self, msg):

        max_vel, max_angle = msg.turtle, math.pi / 30

        if msg.green_alive:  # PURSUIT MODE: Follow any green player (only if there is at least one green alive)
            target = msg.green_alive[0]  # select the first alive green player (I am hunting green)
            distance, angle = getDistanceAndAngleToTarget(self.listener,
                                                          self.player_name, target)

            if angle is None:
                angle = 0
            vel = max_vel  # full throttle
            rospy.loginfo(self.player_name + ': Hunting ' + str(target) + '(' + str(distance) + ' away)')
        else:  # what else to do? Lets just move towards the center
            target = 'world'
            distance, angle = getDistanceAndAngleToTarget(self.listener, self.player_name, target)
            vel = max_vel  # full throttle
            rospy.loginfo(self.player_name + ': Moving to the center of the arena.')
            rospy.loginfo('I am ' + str(distance) + ' from ' + target)

        # Actually move the player
        movePlayer(self.br, self.player_name, self.transform, vel, angle, max_vel)


def main():
    print("Hello player node!")

    rospy.init_node('fcurado', anonymous=False)

    player = Player('fcurado')

    rospy.Subscriber("chatter", String, callback)
    rospy.spin()


if __name__ == "__main__":
    main()
