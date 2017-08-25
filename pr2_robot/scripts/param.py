#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64
from std_msgs.msg import Int32
from std_msgs.msg import String
from pr2_robot.srv import *
import yaml
from rospy_message_converter import message_converter

# Helper function to create a yaml friendly dictionary from ROS messages
def make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose):
    yaml_dict = {}
    yaml_dict["test_scene_num"] = test_scene_num.data
    yaml_dict["arm_name"]  = arm_name.data
    yaml_dict["object_name"] = object_name.data
    yaml_dict["pick_pose"] = message_converter.convert_ros_message_to_dictionary(pick_pose)
    yaml_dict["place_pose"] = message_converter.convert_ros_message_to_dictionary(place_pose)
    return yaml_dict

# Helper function to output to yaml file
def send_to_yaml(yaml_filename, dict_list):
    data_dict = {"object_list": dict_list}
    with open(yaml_filename, 'w') as outfile:
        yaml.dump(data_dict, outfile, default_flow_style=False)



# get parameters
object_list_param = rospy.get_param('/object_list')
drop_box_param = rospy.get_param('/dropbox')

print 'Raw parameters:'
print object_list_param
print drop_box_param

group_to_place = {}
group_to_arm = {}
for param in drop_box_param:
    group_to_place[ param['group'] ] = param['position']
    group_to_arm[ param['group'] ] = param['name']

pick_places = []
pick_arms = []
pick_names = []
for param in object_list_param:
    pick_places.append( group_to_place[ param['group'] ] )
    pick_arms.append( group_to_arm[ param['group'] ] )
    pick_names.append( param['name'] )

print 'Processed parameters'
print 'pick_places: ', pick_places
print 'pick_arms: ', pick_arms
print 'pick_names: ', pick_names

# Initialize variables
test_scene_num = Int32()
object_name = String()
arm_name = String()
pick_pose = Pose()
place_pose = Pose()

dict_list = []
for i in xrange(len(pick_places)):
    # Process ROS msgs
    test_scene_num.data = 1
    object_name.data = pick_names[i]
    arm_name.data = pick_arms[i]
    place_pose.position.x = pick_places[i][0]
    place_pose.position.y = pick_places[i][1]
    place_pose.position.z = pick_places[i][2]
    place_pose.orientation.x = 0
    place_pose.orientation.y = 0
    place_pose.orientation.z = 0
    place_pose.orientation.w = 0
    pick_pose.position.x = 1 #pick_places[i][0]
    pick_pose.position.y = 1 #pick_places[i][1]
    pick_pose.position.z = 1 #pick_places[i][2]
    pick_pose.orientation.x = 0
    pick_pose.orientation.y = 0
    pick_pose.orientation.z = 0
    pick_pose.orientation.w = 0

    
    yaml_dict = make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose)
    dict_list.append(yaml_dict)

print dict_list
send_to_yaml("output.yaml", dict_list)
