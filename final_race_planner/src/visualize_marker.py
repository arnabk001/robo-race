#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA 
import Utils
from nav_msgs.srv import GetMap

STARTPOINT= [2500.0,580.0,0.0]

GOODWAYPOINTS=[	[2510.0, 605.0, 0.0],
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
		[565.0,535.0,0.0],
		[600.0,400.0,0.0]]

BADWAYPOINTS = [[2280.0,800.0,0.0],[1685.0,970.0,0.0],[1390.0,910.0,0.0],[680.0,420.0,0.0],[725.0,635.0,0.0]]


GWCOLOUR = [0.0,0.0,1.0,1.0]
BWCOLOUR = [1.0,0.0,0.0,1.0]
SWCOLOUR = [0.0,1.0,0.0,1.0]
def visualize_markers():
    rospy.init_node('marker_publisher', anonymous=True)
    marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
    map_service_name = rospy.get_param("~static_map", "static_map")
    print("Getting map from service: ", map_service_name)
    rospy.wait_for_service(map_service_name)
    map_info = rospy.ServiceProxy(map_service_name, GetMap)().map.info
    rate = rospy.Rate(1)  # 1 Hz
    mapheight = 1236
    while not rospy.is_shutdown():
	
	# Publishing GoodWayPoints
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.pose.position.x =0.0
	marker.pose.position.y = 0.0
	marker.pose.position.z = 0.0
	marker.pose.orientation.w = 1.0
        marker.scale.x = 0.2
        marker.scale.y = 0.2
	for gwp in GOODWAYPOINTS:
		gwp_world = Utils.map_to_world([gwp[0],mapheight - gwp[1],gwp[2]], map_info)
                
        	marker.points.append(Point(x=gwp_world[0],y=gwp_world[1],z=gwp_world[2]))  # Add your points here
		marker.colors.append(ColorRGBA(GWCOLOUR[0],GWCOLOUR[1],GWCOLOUR[2],GWCOLOUR[3]))

	for bwp in BADWAYPOINTS:
		bwp_world = Utils.map_to_world([bwp[0],mapheight - bwp[1],bwp[2]], map_info)
                
        	marker.points.append(Point(x=bwp_world[0],y=bwp_world[1],z=bwp_world[2]))  # Add your points here
		marker.colors.append(ColorRGBA(BWCOLOUR[0],BWCOLOUR[1],BWCOLOUR[2],BWCOLOUR[3]))

	sp_world = Utils.map_to_world([STARTPOINT[0],mapheight - STARTPOINT[1],STARTPOINT[2]], map_info)
	marker.points.append(Point(x=sp_world[0],y=sp_world[1],z=sp_world[2]))  # Add your points here
	marker.colors.append(ColorRGBA(SWCOLOUR[0],SWCOLOUR[1],SWCOLOUR[2],SWCOLOUR[3]))
	
	marker.text = "Points"
        marker_pub.publish(marker)
        rate.sleep()

if __name__ == '__main__':
    try:
        visualize_markers()
    except rospy.ROSInterruptException:
        pass

