#! /usr/bin/env python3
import rospy
import enum
from real_robot_sim.srv import FindWall, FindWallResponse
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import time


class FindWallService:
    def __init__(self, mode):
        rospy.loginfo("Init node find_wall")
        self._mode = mode
        self.laser = {
            "front_laser": 0.0,
            "right_laser": 0.0,
            "left_laser": 0.0,
        }

        self.find_wall_server = rospy.Service('/find_wall', FindWall, self.__find_callback)
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.__laser_callback)
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        self.move = Twist()
        rospy.on_shutdown(self.__clean_shutdown)
        self.rate = rospy.Rate(10)
        rospy.spin()

    def __laser_callback(self, scan):
        # Callback for the laser of the LiDAR
        laser_range = scan.ranges
        if self._mode == 0:  # 180 beams
            self.laser["front_laser"] = min(laser_range[85:105])
            self.laser["right_laser"] = min(laser_range[40:60])
            self.laser["left_laser"] = min(laser_range[130:150])
        elif self._mode == 1:  # 720 beams
            self.laser["front_laser"] = min(laser_range[4 * 85:4 * 105])
            self.laser["right_laser"] = min(laser_range[4 * 40:4 * 60])
            self.laser["left_laser"] = min(laser_range[4 * 130:4 * 150])

    def __find_callback(self, request):
        rospy.loginfo("Finding the wall!!")
        flags = {
            "find_wall" : False,
            "parallel_wall" : False,
            "close_wall" : False
        }

        self.move.linear.x = 0.0
        self.move.angular.z = 0.0

        self.vel_pub.publish(self.move)

        while not flags["parallel_wall"]:
            # Turn to find the wall
            if not flags["find_wall"]:
                if self.laser["right_laser"] > self.laser["left_laser"]:    # Right
                    rospy.loginfo("Right!!")
                    self.move.linear.x = 0.0
                    self.move.angular.z = 0.2

                elif self.laser["left_laser"] > self.laser["right_laser"]: # Left
                    rospy.loginfo("Left!!")
                    self.move.linear.x = 0.0
                    self.move.angular.z = -0.2

            while not flags["find_wall"]:
                if self.laser["right_laser"] > self.laser["front_laser"] and self.laser["left_laser"] > self.laser["front_laser"]:
                    flags["find_wall"] = True

                self.vel_pub.publish(self.move)
                time.sleep(0.1)
                    
            # Move to stay closer to the wall
            if flags["find_wall"] and not flags["close_wall"]:
                rospy.loginfo("Front!!")
                self.move.linear.x = 0.05
                self.move.angular.z = 0.0
                while self.laser["front_laser"] > 0.3:
                    self.vel_pub.publish(self.move)
                    time.sleep(0.1)

                flags["close_wall"] = True

            if flags["close_wall"]:
                self.move.linear.x = 0.0
                self.move.angular.z = 0.2

            if (self.laser["front_laser"] > 0.5) and (0.4 > self.laser["right_laser"]):
                flags["parallel_wall"] = True

            self.vel_pub.publish(self.move)
            time.sleep(0.1)

        self.__clean_shutdown()

        response = FindWallResponse()
        response.wallfound = flags["parallel_wall"]
        return response

    def __clean_shutdown(self):
        # Stop the robot before shutdown the node
        rospy.loginfo("Stopping the robot")
        self.move.linear.x = 0.0
        self.move.angular.z = 0.0
        self.vel_pub.publish(self.move)


if __name__ == "__main__":
    rospy.init_node("find_wall_service", anonymous=True)
    # Select the type of robot, simulation or real
    class simType (enum.Enum):
        Sim = 0
        Real = 1

    try:
        find_wall = FindWallService(mode=simType.Sim)
    except rospy.ROSInterruptException:
        rospy.signal_shutdown("")

