#! /usr/bin/env python3

import rospy
from real_robot_sim.srv import FindWall, FindWallRequest

if __name__ == "__main__":
    rospy.init_node("find_wall_client", anonymous=True)

    rospy.wait_for_service("/find_wall")
    wall_client = rospy.ServiceProxy('/find_wall', FindWall)

    wall_client_objet = FindWallRequest()
    
    result = wall_client(wall_client_objet)
    rospy.loginfo(result)