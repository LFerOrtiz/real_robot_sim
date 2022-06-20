#! /usr/bin/env python3
import rospy
import actionlib
from real_robot_sim.msg import OdomRecordAction, OdomRecordFeedback, OdomRecordResult
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from tf.transformations import euler_from_quaternion
from math import sqrt, fabs


class OdomRecord:
    def __init__(self):
        # Create messages that are used to publish feedback/result
        self._feedback = OdomRecordFeedback()
        self._result = OdomRecordResult()

        self._server_action = actionlib.SimpleActionServer("/record_odom", OdomRecordAction, self._odom_record_callback, False)
        self._server_action.start()

        # Values to record
        self._odom_values = {
            "x": 0.0,
            "y": 0.0,
            "theta": 0.0
        }

    def _odom_record_callback(self, odom):
        odom = rospy.Subscriber('/odom', Odometry, self._odom_callback, queue_size=1)

        rate = rospy.Rate(1)
        success = False
        self._result.list_of_odoms = []
        self._feedback.current_total = 0.0
        limit_init_point = {
            "high_x": 0.0,
            "low_x": 0.0,
            "high_y": 0.0,
            "low_y": 0.0,
        }

        init_point = Point()
        i = 0
        while not success:
            rospy.logdebug("Running!")
            if self._server_action.is_preempt_requested():
                self._server_action.set_preempted()
                success = False
                break

            current_odom = Point()
            current_odom.x = self._odom_values["x"]
            current_odom.y = self._odom_values["y"]
            current_odom.z = self._odom_values["theta"]

            self._result.list_of_odoms.append(current_odom)

            if i < 1:
                self._feedback.current_total = 0.0
            elif i == 1:
                init_point = current_odom
                limit_init_point["high_x"] = round(init_point.x + (fabs(init_point.x) * 0.3), 3)
                limit_init_point["low_x"] = round(init_point.x - (fabs(init_point.x) * 0.3), 3)
                limit_init_point["high_y"] = round(init_point.y + (fabs(init_point.y) * 0.3), 3)
                limit_init_point["low_y"] = round(init_point.y - (fabs(init_point.y) * 0.3), 3)

            else:
                # Calculates the distance traveled
                self._feedback.current_total += sqrt(
                    (self._result.list_of_odoms[i].x - self._result.list_of_odoms[i - 1].x) ** 2 + (
                            self._result.list_of_odoms[i].y - self._result.list_of_odoms[i - 1].y) ** 2)

            if (limit_init_point["high_x"] > round(self._result.list_of_odoms[i].x, 3) > limit_init_point["low_x"]) and\
                    (limit_init_point["high_y"] > round(self._result.list_of_odoms[i].y, 3) > limit_init_point["low_y"]) \
                    and i > 4:
                success = True

            i += 1
            self._server_action.publish_feedback(self._feedback)
            rate.sleep()

        if success:
            rospy.loginfo("Success getting odometry of the circuit")
            self._server_action.set_succeeded(self._result)

    def _odom_callback(self, odom):
        # Odometry callback
        self._odom_values["x"] = odom.pose.pose.position.x
        self._odom_values["y"] = odom.pose.pose.position.y

        orientation_q = odom.pose.pose.orientation
        (_, _, self._odom_values["theta"]) = euler_from_quaternion([orientation_q.x,
                                                                    orientation_q.y,
                                                                    orientation_q.z,
                                                                    orientation_q.w])


if __name__ == "__main__":
    rospy.init_node("record_odom_action_server", anonymous=True, log_level=rospy.INFO)
    OdomRecord()
    rospy.spin()
