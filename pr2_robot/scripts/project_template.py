#!/usr/bin/env python

# Import modules
import numpy as np
import sklearn
from sklearn.preprocessing import LabelEncoder
import pickle
# from RoboND-Perception-Project.srv import GetNormals
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

print('import finished')


# Helper function to get surface
def get_normals(cloud):
    get_normals_prox = rospy.ServiceProxy('/feature_extractor/get_normals', GetNormals)
    return get_normals_prox(cloud).cluster


print('start make_yaml_dict function define')


# Helper function to create a yaml friendly dictionary from ROS messages
def make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose):
    yaml_dict = {}
    yaml_dict["test_scene_num"] = test_scene_num.data
    yaml_dict["arm_name"] = arm_name.data
    yaml_dict["object_name"] = object_name.data
    yaml_dict["pick_pose"] = message_converter.convert_ros_message_to_dictionary(pick_pose)
    yaml_dict["place_pose"] = message_converter.convert_ros_message_to_dictionary(place_pose)
    return yaml_dict


def find_centroid(object_list, item):
    centroid = None
    for _object in object_list:
        if _object.label == item:
            points_arr = ros_to_pcl(_object.cloud).to_array()
            centroid = np.mean(points_arr, axis=0)[:3]

# Helper function to output to yaml file
def send_to_yaml(yaml_filename, dict_list):
    data_dict = {"object_list": dict_list}
    with open(yaml_filename, 'w') as outfile:
        yaml.dump(data_dict, outfile, default_flow_style=False)


print('start pcl_callback step')


# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):
    print('call back function run ...')
    # Exercise-2 TODOs:
    # TODO: Convert ROS msg to PCL data
    cloud = ros_to_pcl(pcl_msg)
    # TODO: Statistical Outlier Filtering
    print('statistical outlier filtering')
    outlier_filter = cloud.make_statistical_outlier_filter()
    outlier_filter.set_mean_k(10)
    x = 0.01
    outlier_filter.set_std_dev_mul_thresh(x)
    cloud_after_statistical_filter = outlier_filter.filter()
    print('start voxel grid downsampling')
    # TODO: Voxel Grid Downsampling
    vox = cloud_after_statistical_filter.make_voxel_grid_filter()
    LEAF_SIZE = 0.01
    vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)
    cloud_after_downsampling = vox.filter()
    filename = 'cloud_after_downsampling.pcd'
    pcl.save(cloud_after_downsampling, filename)

    # TODO: PassThrough Filter
    print('start passthrough fitler run')
    # y axis
    cloud = cloud_after_downsampling.make_passthrough_filter()
    filter_axis = 'x'
    cloud.set_filter_field_name(filter_axis)
    axis_min = 0.35
    axis_max = 0.8
    cloud.set_filter_limits(axis_min, axis_max)
    cloud_after_x_axis_limit = cloud.filter()
    print('start passthrough filter in x axis')
    # z axis
    cloud = cloud_after_x_axis_limit.make_passthrough_filter()
    filter_axis = 'z'
    cloud.set_filter_field_name(filter_axis)
    axis_min = 0.6
    axis_max = 0.9
    cloud.set_filter_limits(axis_min, axis_max)
    cloud_after_x_z_limit = cloud.filter()
    filename = 'cloud_after_x_z_limit.pcd'
    pcl.save(cloud_after_x_z_limit, filename)

    print('start ransac plane segmentation')
    # TODO: RANSAC Plane Segmentation
    seg = cloud_after_x_z_limit.make_segmenter()
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)

    max_dis = 0.01
    seg.set_distance_threshold(max_dis)

    print('Extract inliers and outliers')
    # TODO: Extract inliers and outliers
    inliers, coefficients = seg.segment()

    extracted_inliers = cloud_after_x_z_limit.extract(inliers, negative=False)
    extracted_outliers = cloud_after_x_z_limit.extract(inliers, negative=True)

    filename = 'extracted_inliers.pcd'
    pcl.save(extracted_inliers, filename)

    filename = 'extracted_outliers.pcd'
    pcl.save(extracted_outliers, filename)
    # print('publish ros_cloud_objects to ros publisher')
    # TODO: Publish messages to ros pcl_object_pub publisher.
    # ros_cloud_objects = pcl_to_ros(extracted_outliers)
    # pcl_objects_pub.publish(ros_cloud_objects)

    print('Eucliean clustering')
    # TODO: Euclidean Clustering
    # ros_cloud_objects = pcl_to_ros(extracted_outliers)
    # pcl_objects_pub.publish(ros_cloud_objects)
    white_cloud = XYZRGB_to_XYZ(extracted_outliers)
    tree = white_cloud.make_kdtree()
    # Create a cluster extraction object
    ec = white_cloud.make_EuclideanClusterExtraction()
    # Set tolerances for distance threshold
    # as well as minimum and maximum cluster size (in points)
    # NOTE: These are poor choices of clustering parameters
    # Your task is to experiment and find values that work for segmenting objects.
    ec.set_ClusterTolerance(0.03)
    ec.set_MinClusterSize(11)
    ec.set_MaxClusterSize(45000)
    # Search the k-d tree for clusters
    ec.set_SearchMethod(tree)
    # Extract indices for each of the discovered clusters
    cluster_indices = ec.Extract()


    print('create cluster-mask point cloud')
    # TODO: Create Cluster-Mask Point Cloud to visualize each cluster separately
    # Assign a color corresponding to each segmented object in scene
    color_cluster_point_list = []
    cluster_color = get_color_list(len(cluster_indices))

    for j, indices in enumerate(cluster_indices):
        for i, indice in enumerate(indices):
            color_cluster_point_list.append([white_cloud[indice][0],
                                             white_cloud[indice][1],
                                             white_cloud[indice][2],
                                             rgb_to_float(cluster_color[j])])

    # Create new cloud containing all clusters, each with unique color
    cluster_cloud = pcl.PointCloud_PointXYZRGB()
    cluster_cloud.from_list(color_cluster_point_list)
    # TODO: Convert PCL data to ROS messages
    ros_cluster_cloud = pcl_to_ros(cluster_cloud)
    print('publish pcl_objects_pub messages')
    # TODO: Publish ROS messages
    pcl_objects_pub.publish(ros_cluster_cloud)

    # Exercise-3 TODOs:

    # Classify the clusters! (loop through each detected cluster one at a time)
    detected_objects = []
    detected_objects_labels = []
    object_list = []
    # Grab the points for the cluster
    print('grab the points for the cluster')
    for index, pts_list in enumerate(cluster_indices):
        pcl_cluster = extracted_outliers.extract(pts_list)
        ros_cluster = pcl_to_ros(pcl_cluster)
        print('compute associated feature vector')
        # Compute the associated feature vector
        chists = compute_color_histograms(ros_cluster, using_hsv=True)
        normals_ = get_normals(ros_cluster)
        nhists = compute_normal_histograms(normals_)
        features = np.concatenate((chists, nhists))
        # Make the prediction
        print('make prediction with model/n')
        prediction = clf.predict(scaler.transform(features.reshape(1,-1)))
        label = encoder.inverse_transform(prediction)[0]
        detected_objects_labels.append(label)
        # Publish the list of detected objects
        # Publish a label into RViz
        label_pos = list(white_cloud[pts_list[0]])
        label_pos[2] += .4
        object_markers_pub.publish(make_label(label,label_pos,index))
        print('add detected object to the list of detected objects/n')
        # Add the detected object to the list of detected objects.
        do = DetectedObject()
        do.label = label
        do.cloud = ros_cluster
        detected_objects.append(do)
    rospy.loginfo('Detected {} objects: {}'.format(len(detected_objects_labels), detected_objects_labels))

    # Publish the list of detected objects
    # This is the output you'll need to complete the upcoming project!
    detected_objects_pub.publish(detected_objects)
    # Suggested location for where to invoke your pr2_mover() function within pcl_callback()
    # Could add some logic to determine whether or not your object detections are robust
    # before calling pr2_mover()
    print('try call pr2_mover(detected_objects')
    try:
        pr2_mover(detected_objects)
    except rospy.ROSInterruptException:
        pass

print('start_pr2_mover')

# function to load parameters and request PickPlace service
def pr2_mover(object_list):
    # TODO: Initialize variables
    # resp = pick_place_routine(TEST_SCENE_NUM, OBJECT_NAME, WHICH_ARM, PICK_POSE, PLACE_POSE)
    TEST_SCENE_NUM = Int32()
    OBJECT_NAME = String()
    WHICH_ARM = String()
    PICK_POSE = Pose()
    PLACE_POSE = Pose()

    # TODO: Get/Read parameters
    pick_list = []

    # rospy.get_param return a dictionary include {'name' :  , 'group':    }
    object_list_param = rospy.get_param('/object_list')
    # TODO: Parse parameters into individual variables
    for i in range(0, len(object_list_param)):
        object_name = object_list_param[i]['name']
        object_group = object_list_param[i]['group']
        pick_list.append((object_name, object_group))

    # TODO: Rotate PR2 in place to capture side tables for the collision map
    print('rotate PR2 in palce to capture side tables for the collision map')
    box_param = rospy.get_param('/dropbox')
    # red box in left side
    red_box_pos = box_param[0]['position']
    # green box in right side
    green_box_pos = box_param[1]['position']

    dict_list = []

    for item, group in pick_list:
        TEST_SCENE_NUM.data = 3

        centroids = find_centroid(object_list, item)

        OBJECT_NAME.data = item

        if group == 'green':
            WHICH_ARM.data = 'right'
            PLACE_POSE.position.x = green_box_pos[0]
            PLACE_POSE.position.y = green_box_pos[1]
            PLACE_POSE.position.z = green_box_pos[2]

        else:
            WHICH_ARM.data = 'left'
            PLACE_POSE.position.x = red_box_pos[0]
            PLACE_POSE.position.y = red_box_pos[1]
            PLACE_POSE.position.z = red_box_pos[2]

        if centroids is not None:
            PICK_POSE.position.x = np.asscalar(centroids[0])
            PICK_POSE.position.y = np.asscalar(centroids[1])
            PICK_POSE.position.z = np.asscalar(centroids[2])

        else:
            PICK_POSE.position.x = 0
            PICK_POSE.position.y = 0
            PICK_POSE.position.z = 0
        print('make yaml_dict')
        yaml_dict = make_yaml_dict(TEST_SCENE_NUM, WHICH_ARM, OBJECT_NAME, PICK_POSE, PLACE_POSE)
        dict_list.append(yaml_dict)
    print('send to yaml')
    send_to_yaml("output_lm3.yaml", dict_list)

    # Wait for 'pick_place_routine' service to come up
    rospy.wait_for_service('pick_place_routine')

    try:
        pick_place_routine = rospy.ServiceProxy('pick_place_routine', PickPlace)

        # TODO: Insert your message variables to be sent as a service request
        resp = pick_place_routine(TEST_SCENE_NUM, OBJECT_NAME, WHICH_ARM, PICK_POSE, PLACE_POSE)

        print("Response: ", resp.success)
        print("response get pick_place_routine and response success")
    except rospy.ServiceException, e:
        print('rospy serviece Exception')
        print
        "Service call failed: %s" % e


# TODO: Output your request parameters into output yaml file

if __name__ == '__main__':
    print('start main')
    # TODO: ROS node initialization
    rospy.init_node('clustering_lm', anonymous=True)
    # TODO: Create Subscribers
    print('create a subscriber')
    pcl_sub = rospy.Subscriber("/pr2/world/points", pc2.PointCloud2, pcl_callback, queue_size=1)
    # TODO: Create Publishers
    print('create publishers')
    pcl_objects_pub = rospy.Publisher("/pcl_objects", PointCloud2, queue_size=1)
    pcl_table_pub = rospy.Publisher("/pcl_table", PointCloud2, queue_size=1)
    pcl_lm_pub = rospy.Publisher("/pcl_lm", PointCloud2, queue_size=1)
    no_noise_pub = rospy.Publisher("/no_noise_cloud", PointCloud2, queue_size=1)
    objects_pub = rospy.Publisher("/objects_cloud", PointCloud2, queue_size=1)
    clusters_pub = rospy.Publisher("/clusters_cloud", PointCloud2, queue_size=1)
    object_markers_pub = rospy.Publisher("/object_markers", Marker, queue_size=1)
    detected_objects_pub = rospy.Publisher("/detected_objects", DetectedObjectsArray, queue_size=1)
    print('after define publihser')
    # load model from disk in vm
    print('load model with pickle')
    model = pickle.load(open('/home/robond/catkin_ws/src/RoboND-Perception-Project/pr2_robot/scripts/model.sav', 'rb'))
    clf = model['classifier']
    encoder = LabelEncoder()
    encoder.classes_ = model['classes']
    scaler = model['scaler']

    # Initialize color_list
    get_color_list.color_list = []
    print('get color list color ')
    # TODO: Spin while node is not shutdown
    while not rospy.is_shutdown():
        print('rospy.spin')
        rospy.spin()
        print('rospy.spin() run')
