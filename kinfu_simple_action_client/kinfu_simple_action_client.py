#! /usr/bin/env python

import rospy
# from __future__ import print_function

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
import actionlib_tutorials.msg
from kinfu_msgs.msg import RequestAction,KinfuTsdfRequest,KinfuRequestHeader
from sensor_msgs.msg import PointCloud2


def mesh_request(reset = False):
    rospy.loginfo("Meshing function now")
    # variable to keep track of some timing to benchmark
    times={}
    times['start_time'] = rospy.get_rostime()
    # () to the constructor.
    if not kin_fu.action_client:
        # create the action client if we need to
        kin_fu.action_client = actionlib.SimpleActionClient(kin_fu.request_action_key, RequestAction)

    # Waits until the action server has started up and started
    # listening for goals.
    kin_fu.action_client.wait_for_server()

    # Creates a goal to send to the action server.
    kin_fu.seeds['mesh'] += 1
    this_tsdf_request = KinfuTsdfRequest()
    this_tsdf_request.request_reset = reset
    this_tsdf_request.tsdf_header.request_type = 2 # a mesh
    this_tsdf_request.tsdf_header.request_id = kin_fu.seeds['mesh']

    fake_goal_wrapper = Generic()  # unclear why this was required.  the auto generated serialize function expecte a goal with a top level attribute called request
    fake_goal_wrapper.request = this_tsdf_request
    # goal = this_tsdf_request
    # goal = RequestAction(request=this_tsdf_request)

    # Sends the goal to the action server.
    kin_fu.action_client.send_goal(fake_goal_wrapper)

    # Waits for the server to finish performing the action.
    kin_fu.action_client.wait_for_result()
    times['post_extract_time'] = rospy.get_rostime()
    rospy.loginfo("extracted...")
    # Prints out the result of executing the action
    result = kin_fu.action_client.get_result()
    times['pose_retrieve_time'] = rospy.get_rostime()
    rospy.loginfo("retrieved...")

    publish_point_cloud(result.mesh)
    times['post_publish_time'] =rospy.get_rostime()
    rospy.loginfo("published...")

    export_ply_file(result.mesh)
    times['post_export_time'] = rospy.get_rostime()

    timing={
        'extraction': (times['post_extract_time']-times['start_time']).to_nsec()/1000000,
        'retrieval': (times['pose_retrieve_time']-times['post_extract_time']).to_nsec()/1000000,
        'publishing': (times['post_publish_time']-times['pose_retrieve_time']).to_nsec()/1000000,
        # 'export': (times['post_export_time']-times['post_publish_time']).to_nsec()/1000000,
    }
    print "Timing summary:"
    for item in timing.keys():
        print "\t" + item + ": " + str(timing[item]) + " milliseconds"


def publish_point_cloud(mesh_polygon):
    if not kin_fu.publishers['point_cloud']:
        kin_fu.publishers['point_cloud'] = rospy.Publisher(
            '/kinfu/extract/pointcloud',
            PointCloud2,
            queue_size=5)
        kin_fu.publishers['point_cloud'].publish(mesh_polygon.cloud)


def export_ply_file(mesh):
    print "polygons:", str(len(mesh.polygons)),
    print "points:", str(mesh.cloud.width)


def KinFuConsumer_Factory():
    this_object = Generic()
    this_object.request_action_key = "/kinfu_output/actions/request"
    this_object.action_client = None
    this_object.seeds= {'mesh':0}
    this_object.publishers = {'point_cloud':None}
    return this_object

if __name__ == '__main__':
    class Generic:
        pass

    try:
        # this is the name of the action based interface
        kin_fu = KinFuConsumer_Factory()
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('ros_kinfo_simple_action_client')
        # set a rate
        rate_val = .05
        rospy.loginfo("FYI: Mesh request rate is every: " + str(1/rate_val) + " second(s)")
        rate = rospy.Rate(rate_val) # once every ten (10) seconds

        while not rospy.is_shutdown():
            # result = fibonacci_client()
            result = mesh_request(True)
            #delay and then make another pass
            rate.sleep()
    except rospy.ROSInterruptException:
        print "program interrupted before completion"
    except:
        raise