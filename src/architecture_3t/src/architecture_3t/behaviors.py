#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import abc
import importlib
import math

from geometry_msgs.msg import Twist, Vector3
from std_srvs.srv import Empty
from turtlesim.msg import Pose
import rospy
import tf2_ros


class Behavior(abc.ABC):
    state = "undefined"

    def __init__(self):
        self.name = self.__class__.__name__

    @abc.abstractmethod
    def setup(self):
        """Enable function."""
        pass

    @abc.abstractmethod
    def do_once(self):
        pass


class Idle(Behavior):
    def setup(self):
        self.state = "running"

    def do_once(self):
        pass


class Reset(Behavior):
    def setup(self):
        self.turtle_reset = rospy.ServiceProxy(f"/reset", Empty)
        self.state = "running"

    def do_once(self):
        if self.state == "running":
            try:
                self.turtle_reset()
            except rospy.ServiceException:
                self.state = "fail"
                rospy.logwarn(
                    f"Behavior {self.name} failed, turtlesim is not running."
                )
            self.state = "done"


class Draw(Behavior):
    def setup(self, tick_time, turtle_name):
        self.turtle_vel_pub = rospy.Publisher(
            f"/{turtle_name}/cmd_vel",
            Twist,
            queue_size=128,
        )

        self.tick_num = 0
        self.tick_time = tick_time
        self.distance_traveled = 0
        self.angle_traveled = 0

        self.turtle_pos = None
        self.num_pos_similar = 0
        rospy.Subscriber(f"/{turtle_name}/pos", Pose, self.pos_cb)

        self.state = "running"

    def pos_cb(self, msg):
        """Updates pose info with sensor data and generates failure if turtle
        does not move.
        """

        if self.state not in ["done", "fail"] and msg == self.turtle_pos:
            self.num_pos_similar += 1
            if self.num_pos_similar > 60:
                self.state = "fail"
                rospy.logwarn(
                    f"Behavior {self.name} failed, turtle does not move."
                )
        else:
            self.num_pos_similar = 0
            self.turtle_pos = msg


class Line(Draw):
    def setup(self, tick_time, turtle_name):
        super().setup(tick_time, turtle_name)
        self.vx = 1.0

    def do_once(self):
        if self.state == "running":
            if self.distance_traveled <= 1:
                cmd_vel = Twist(
                    linear = Vector3(
                        x = self.vx,
                    ),
                )
                self.turtle_vel_pub.publish(cmd_vel)
                self.distance_traveled += self.vx * self.tick_time
            else:
                self.state = "done"


class Circle(Draw):
    def setup(self, tick_time, turtle_name):
        super().setup(tick_time, turtle_name)
        self.vx = 1.0
        self.wz = 0.5
        self.r = self.vx / self.wz

    def do_once(self):
        if self.state == "running":
            if self.distance_traveled <= 2 * math.pi * self.r:
                cmd_vel = Twist(
                    linear = Vector3(
                        x = self.vx,
                        y = 0.0,
                        z = 0.0,
                    ),
                    angular = Vector3(
                        x = 0.0,
                        y = 0.0,
                        z = self.wz,
                    ),
                )
                self.turtle_vel_pub.publish(cmd_vel)
                self.distance_traveled += self.vx * self.tick_time
                self.angle_traveled += self.wz * self.tick_time
            else:
                self.state = "done"

