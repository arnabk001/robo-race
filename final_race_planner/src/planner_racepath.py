#!/usr/bin/env python

import rospy
import numpy as np
from assignment5.srv import *
import Utils
from nav_msgs.srv import GetMap

PLANNER_SERVICE_TOPIC = '/planner_node/get_car_plan'

# Testing pose sets:
START = [2500, (1236-580), 0.0]
good_wp = np.array([[2600,560, 0.0], 
		   [2400,525, 0.0],
		   [1800,715, 0.0],
		   [1845,825, 0.0],
		   [1465,1160, 0.0],
		   [1150,725, 0.0],
		   [565,535, 0.0],
		   [600,400, 0.0]])

good_wp = np.array([[2510.0, 605.0, 0.0],
		[2535.0, 640.0, 0.0],
		[2562.0, 633.0, 0.0],
		[2585.0, 600.0, 0.0],
		[2600.0, 560.0, 0.0],
		[2550.0, 525.0, 0.0],
		[2400.0, 525.0, 0.0],
		[2075.0, 625.0, 0.0],
		[1800.0, 715.0, 0.0],
		[1800.0, 775.0, 0.0], 
		[1845.0, 825.0, 0.0],
		[1745.0, 1025.0, 0.0],
		[1600.0, 1125.0, 0.0],
		[1465.0,1160.0,0.0],
		[1290.0, 1035.0, 0.0],
		[1150.0,725.0,0.0],
		[1150.0, 625.0, 0.0],
		[655.0, 360.0, 0.0],
		[600.0,400.0,0.0],
		[565.0,535.0,0.0]])

# ('mapheight = ', 1236, 'mapwidth = ', 2792)
mapheight = 1236
good_wp[:,1] = mapheight - good_wp[:,1]

if __name__ == '__main__':

    rospy.init_node('planner_test', anonymous=True)

    map_service_name = rospy.get_param("~static_map", "static_map")
    print("Getting map from service: ", map_service_name)
    rospy.wait_for_service(map_service_name)
    map_info = rospy.ServiceProxy(map_service_name, GetMap)().map.info

    rospy.wait_for_service(PLANNER_SERVICE_TOPIC)
    get_plan = []
    plan_file = [] # in x,y,theta format

    for i in range(len(good_wp)):
	if i == 0:
		source = START
		target = good_wp[i]
	else:
		source = target
		target = good_wp[i]

	plan = rospy.ServiceProxy(PLANNER_SERVICE_TOPIC, GetPlan)

	resp = plan(Utils.map_to_world(source, map_info), Utils.map_to_world(target, map_info))
	# print("resp shape = ", np.array(resp).shape)
	print("plan generated for source = ", source, " and target = ", target)
	# print("plan length ", len(resp.plan))

	# raw_input("Press Enter to continue...")
	get_plan.append(resp.plan) # get plan is in pose/orientation format
	
	for item in resp.plan:
	    item_list = [item.position.x,item.position.y,Utils.quaternion_to_angle(item.orientation)]
    	    plan_file.append(item_list)

    print("plan generation complete")
    # print("generated path:\n", get_plan)
    
    plan_file = np.array(plan_file)
    np.save("plan_file", plan_file)
    # PUB_TOPIC = '/planner_node/car_plan'
    

 
    

