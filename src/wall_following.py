#! /usr/bin/env python3
import rospy
import time
import enum
import actionlib
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from real_robot_sim.srv import FindWall, FindWallRequest
from real_robot_sim.msg import OdomRecordAction, OdomRecordGoal, OdomRecordResult


class RobotControl:
    def __init__(self, mode):
        rospy.loginfo("**************************")
        rospy.loginfo("!!Init the configuration!!")
        self.move = Twist()
        self.laser = {
            "front_laser": 0.0,
            "right_laser": 0.0,
            "left_laser": 0.0
        }
        self._rate = rospy.Rate(10)
        self._mode = mode
        self.current_distance = 0.0
        rospy.on_shutdown(self._clean_shutdown)

        # Subscribers and publishers
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self._laser_callback)
        self.action_client = actionlib.SimpleActionClient('/record_odom', OdomRecordAction)

    def _laser_callback(self, scan):
        #           90
        #           |
        #
        #           V
        #  135 --   o   -- 45
        #
        #           |
        #         180|0
        laser_range = scan.ranges
        if self._mode == 0:     # 180 beams
            self.laser["front_laser"] = min(laser_range[85:105])
            self.laser["right_laser"] = min(laser_range[40:60])
            self.laser["left_laser"] = min(laser_range[130:150])
        elif self._mode == 1:   # 720 beams
            self.laser["front_laser"] = min(laser_range[4 * 85:4 * 105])
            self.laser["right_laser"] = min(laser_range[4 * 40:4 * 60])
            self.laser["left_laser"] = min(laser_range[4 * 130:4 * 150])
        # minpos = laser_range.index(min(laser_range))
        # print (f"The maximum is at position: {minpos + 1}" )

    def _clean_shutdown(self):
        rospy.loginfo("Stoping the robot")
        self.move.linear.x = 0.0
        self.move.angular.z = 0.0
        self.vel_pub.publish(self.move)

    def move_it(self):
        rospy.loginfo("Move it!!")
        self._clean_shutdown()
        # Wait for the action server
        self.action_client.wait_for_server()

        goal = OdomRecordGoal()

        self.action_client.send_goal(goal, feedback_cb=self._feedback_callback)
        state_result = self.action_client.get_state()

        while not rospy.is_shutdown():
            # + angular : left
            # - angular : right
            self.move.linear.x = 0.08
            if 0.3 >= self.laser["right_laser"] >= 0.2 and 0.51 < self.laser["front_laser"]:
                self.move.angular.z = 0.0
                rospy.logdebug("Forward!!")
            elif 0.5 > self.laser["front_laser"]:
                self.move.angular.z = 0.35
                rospy.logdebug("Corner!!")
            elif self.laser["right_laser"] > 0.3:
                self.move.angular.z = -0.085
                rospy.logdebug("Turn to right!!")
            elif self.laser["right_laser"] < 0.2:
                self.move.angular.z = 0.085
                rospy.logdebug("Turn to left!!")

            self.vel_pub.publish(self.move)
            state_result = self.action_client.get_state()
            rospy.logdebug(state_result)
            self._rate.sleep()

            if state_result > 2:
                break

        self._clean_shutdown()

        rospy.loginfo("[Result] State: " + str(state_result))
        if state_result == 3:
            rospy.logerr("Something went wrong in the Server Side")
        if state_result == 4:
            rospy.logwarn("There is a warning in the Server Side")

    def _feedback_callback(self, feedback):
        self.current_distance = feedback.current_total
        rospy.loginfo("Current distance: " + str(feedback.current_total))


if __name__ == "__main__":
    # Create a node 
    rospy.init_node("real_robot_control", anonymous=True, log_level=rospy.INFO)

    class simType (enum.Enum):
        Sim = 0
        Real = 1

    rospy.wait_for_service("/find_wall")
    wall_client = rospy.ServiceProxy('/find_wall', FindWall)

    # Wait for 5 seconds to call the service
    time.sleep(5)
    wall_client_objet = FindWallRequest()

    result = wall_client(wall_client_objet)
    robot_control = RobotControl(mode=simType.Sim)

    try:
        if result:
            robot_control.move_it()
    except rospy.ROSInterruptException:
        rospy.signal_shutdown("")
