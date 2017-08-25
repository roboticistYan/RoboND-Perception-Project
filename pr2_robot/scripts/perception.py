#!/usr/bin/env python

# Import modules
import numpy as np
import sklearn
from sklearn.preprocessing import LabelEncoder
import pickle
from sensor_stick.srv import GetNormals
from sensor_stick.features import compute_color_histograms
from sensor_stick.features import compute_normal_histograms
from visualization_msgs.msg import Marker
from sensor_stick.marker_tools import *
from sensor_stick.msg import DetectedObjectsArray
from sensor_stick.msg import DetectedObject
from sensor_stick.pcl_helper import *

import rospy
import tf
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64
from std_msgs.msg import Int32
from std_msgs.msg import String
from pr2_robot.srv import *
from rospy_message_converter import message_converter
import yaml


# Helper function to get surface normals
def get_normals(cloud):
    get_normals_prox = rospy.ServiceProxy('/feature_extractor/get_normals', GetNormals)
    return get_normals_prox(cloud).cluster

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

# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):

# Exercise-2 TODOs:

    # TODO: Convert ROS msg to PCL data
    cloud = ros_to_pcl(pcl_msg)
    
    # TODO: Statistical Outlier Filtering
    outlier_filter = cloud.make_statistical_outlier_filter()
    outlier_filter.set_mean_k(200) # number of neighboring points
    outlier_filter.set_std_dev_mul_thresh(1.0) # scale factor for std. deviation
    cloud_filtered = outlier_filter.filter()

    # TODO: Voxel Grid Downsampling
    #vox = cloud.make_voxel_grid_filter()
    vox = cloud_filtered.make_voxel_grid_filter()
    LEAF_SIZE = 0.01
    vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)
    cloud_filtered = vox.filter()

    # TODO: PassThrough Filter
    passthrough = cloud_filtered.make_passthrough_filter()
    passthrough.set_filter_field_name('z')
    passthrough.set_filter_limits(0.6, 0.75)
    cloud_filtered = passthrough.filter()

    # TODO: RANSAC Plane Segmentation
    seg = cloud_filtered.make_segmenter()
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)
    seg.set_distance_threshold(0.02)    # maximum dist.

    # TODO: Extract inliers and outliers
    inliers, coefficients = seg.segment()
    extracted_objects = cloud_filtered.extract(inliers, negative=True)
    extracted_table = cloud_filtered.extract(inliers, negative=False)

    # TODO: Euclidean Clustering
    white_cloud = XYZRGB_to_XYZ( extracted_objects )
    tree = white_cloud.make_kdtree()
    ec = white_cloud.make_EuclideanClusterExtraction()
    ec.set_ClusterTolerance(0.01)
    ec.set_MinClusterSize(100)
    ec.set_MaxClusterSize(500)
    ec.set_SearchMethod(tree)
    cluster_indices = ec.Extract()

    # TODO: Create Cluster-Mask Point Cloud to visualize each cluster separately
    cluster_color = get_color_list(len(cluster_indices))
    color_cluster_point_list = []
    for j, indices in enumerate(cluster_indices):
        for i, indice in enumerate(indices):
            color_cluster_point_list.append([white_cloud[indice][0],
                                             white_cloud[indice][1],
                                             white_cloud[indice][2],
                                             rgb_to_float(cluster_color[j])])
    cluster_cloud = pcl.PointCloud_PointXYZRGB()
    cluster_cloud.from_list(color_cluster_point_list)

    # TODO: Convert PCL data to ROS messages
    #ros_filtered_cloud = pcl_to_ros(cloud_filtered) # Use this to visaulize fitered result
    ros_cluster_cloud = pcl_to_ros(cluster_cloud)
    ros_table_cloud = pcl_to_ros(extracted_table)

    # TODO: Publish ROS messages
    #pcl_objects_pub.publish(ros_filtered_cloud)
    pcl_objects_pub.publish(ros_cluster_cloud)
    pcl_table_pub.publish(ros_table_cloud)

# Exercise-3 TODOs:

    # Classify the clusters! (loop through each detected cluster one at a time)
    detected_objects_labels = []
    detected_objects = []

    for index, pts_list in enumerate(cluster_indices):
        # Grab the points for the cluster
        pcl_cluster = extracted_objects.extract(pts_list)

        # Convert pcl to ros 
        ros_cluster = pcl_to_ros(pcl_cluster)

        # Compute the associated feature vector
        chists = compute_color_histograms( ros_cluster, using_hsv=True)
        normals = get_normals(ros_cluster)
        nhists = compute_normal_histograms(normals)
        feature = np.concatenate((chists, nhists))

        # Make the prediction
        prediction = clf.predict(scaler.transform(feature.reshape(1,-1)))
        label = encoder.inverse_transform(prediction)[0]
        detected_objects_labels.append(label)

        # Publish a label into RViz
        label_pos = list(white_cloud[pts_list[0]])
        label_pos[2] += .4
        object_markers_pub.publish(make_label(label,label_pos, index))

        # Add the detected object to the list of detected objects.
        do = DetectedObject()
        do.label = label
        do.cloud = ros_cluster
        detected_objects.append(do)

    rospy.loginfo('Detected {} objects: {}'.format(len(detected_objects_labels), detected_objects_labels))

    # Publish the list of detected objects
    detected_objects_pub.publish(detected_objects)

    # Suggested location for where to invoke your pr2_mover() function within pcl_callback()
    # Could add some logic to determine whether or not your object detections are robust
    # before calling pr2_mover()
    try:
        pr2_mover(detected_objects)
    except rospy.ROSInterruptException:
        pass

# function to load parameters and request PickPlace service
def pr2_mover(object_list):

    # Calculate Centroids
    labels = []
    centroids = []

    for obj in object_list:
        labels.append( obj.label)
        point_arr = ros_to_pcl( obj.cloud).to_array()
        centroids.append(np.mean(point_arr, axis=0)[:3])

    #print labels
    print len(labels)
    print labels[0]
    print type(centroids[0][0])
    print centroids[0]

    # TODO: Initialize variables
    test_scene_num = Int32()
    object_name = String()
    arm_name = String()
    pick_pose = Pose()
    place_pose = Pose()

    # TODO: Get/Read parameters
    object_list_param = rospy.get_param('/object_list')
    drop_box_param = rospy.get_param('/dropbox')

    # TODO: Parse parameters into individual variables

    # Map group to arm/place
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

    # TODO: Rotate PR2 in place to capture side tables for the collision map

    # TODO: Loop through the pick list
    dict_list = []
    for i in xrange(len(pick_places)):
        # Process ROS msgs
        test_scene_num.data = 1
        object_name.data = pick_names[i]

        # TODO: Get the PointCloud for a given object and obtain it's centroid
        matched = False
        for j in xrange(len(labels)):
            if pick_names[i] == labels[j]:
                pick_pose.position.x = np.asscalar(centroids[j][0]) #pick_places[i][0]
                pick_pose.position.y = np.asscalar(centroids[j][1]) #pick_places[i][1]
                pick_pose.position.z = np.asscalar(centroids[j][2]) #pick_places[i][2]
                pick_pose.orientation.x = 0
                pick_pose.orientation.y = 0
                pick_pose.orientation.z = 0
                pick_pose.orientation.w = 0
                matched = True
                break
        if not matched:
            print "Failed to detect obejct {}".format(pick_names[i])
            pick_pose.position.x = 1 #centroids[j][0] #pick_places[i][0]
            pick_pose.position.y = 1 #centroids[j][1] #pick_places[i][1]
            pick_pose.position.z = 1 #centroids[j][2] #pick_places[i][2]
            pick_pose.orientation.x = 0
            pick_pose.orientation.y = 0
            pick_pose.orientation.z = 0
            pick_pose.orientation.w = 0

        # TODO: Create 'place_pose' for the object
        place_pose.position.x = pick_places[i][0]
        place_pose.position.y = pick_places[i][1]
        place_pose.position.z = pick_places[i][2]
        place_pose.orientation.x = 0
        place_pose.orientation.y = 0
        place_pose.orientation.z = 0
        place_pose.orientation.w = 0

        # TODO: Assign the arm to be used for pick_place
        arm_name.data = pick_arms[i]

        # TODO: Create a list of dictionaries (made with make_yaml_dict()) for later output to yaml format
        yaml_dict = make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose)
        dict_list.append(yaml_dict)


        # Wait for 'pick_place_routine' service to come up
        #rospy.wait_for_service('pick_place_routine')

        #try:
        #    pick_place_routine = rospy.ServiceProxy('pick_place_routine', PickPlace)

            # TODO: Insert your message variables to be sent as a service request
            #resp = pick_place_routine(TEST_SCENE_NUM, OBJECT_NAME, WHICH_ARM, PICK_POSE, PLACE_POSE)

            #print ("Response: ",resp.success)

        #except rospy.ServiceException, e:
        #    print "Service call failed: %s"%e

    # TODO: Output your request parameters into output yaml file
    print dict_list
    send_to_yaml("output.yaml", dict_list )



if __name__ == '__main__':

    # TODO: ROS node initialization
    rospy.init_node('object_recognizer', anonymous=True)

    # TODO: Create Subscribers
    sub = rospy.Subscriber('/pr2/world/points',pc2.PointCloud2,pcl_callback,queue_size=1)

    # TODO: Create Publishers
    pcl_table_pub = rospy.Publisher('/pr2/pcl_table',PointCloud2,queue_size=1)
    pcl_objects_pub = rospy.Publisher('/pr2/pcl_objects',PointCloud2,queue_size=1)
    object_markers_pub = rospy.Publisher('/object_markers',Marker,queue_size=1)
    detected_objects_pub = rospy.Publisher('/detected_objects',DetectedObjectsArray, queue_size=1)

    # TODO: Load Model From disk
    model = pickle.load(open('model_1.sav','rb'))
    clf = model['classifier']
    encoder = LabelEncoder()
    encoder.classes_ = model['classes']
    scaler = model['scaler']

    # Initialize color_list
    get_color_list.color_list = []

    # TODO: Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()
