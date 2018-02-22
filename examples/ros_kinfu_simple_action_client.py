#! /usr/bin/env python
import rospy, actionlib

# import message that we'll need to interact with the action server
from kinfu_msgs.msg import RequestAction, KinfuTsdfRequest, KinfuRequestHeader
# we need this definition in order to be able to publish the pointcloud
from sensor_msgs.msg import PointCloud2


def mesh_request(reset=False):
    # Creates a client (if not yet ecists) to make a request to our action
    # server to extract the current mesh

    rospy.loginfo("Making request to extract mesh from KinFu now...")

    # variable to keep track of some timing to benchmark
    times = {'start_time': rospy.get_rostime()}

    # create the action client if it doesn't already exists
    if not kin_fu.action_client:
        # create the action client if we need to
        kin_fu.action_client = actionlib.SimpleActionClient(kin_fu.request_action_key, RequestAction)

    # Waits until the action server has started up and started
    kin_fu.action_client.wait_for_server()

    # increment our request counter
    kin_fu.seeds['mesh'] += 1
    # Creates a goal to send to the action server.
    this_tsdf_request = KinfuTsdfRequest()
    this_tsdf_request.request_reset = reset  # boolean indicating that we wish the integration volume to be reset
    this_tsdf_request.tsdf_header.request_type = 2  # a mesh
    this_tsdf_request.tsdf_header.request_id = kin_fu.seeds['mesh']

    fake_goal_wrapper = Generic()
    # unclear why this was required.  The auto generated serialize function
    # expectes a goal with a top level attribute matching the message
    # type (in this case: request) but if you create the 'RequestAction' message
    # it has an extra layer of properties that causes the serialization of the
    # message to fail.  so we create a fake wrapper and add our request message
    # directly to it so that we can use the client
    fake_goal_wrapper.request = this_tsdf_request

    # Sends the goal to the action server.
    kin_fu.action_client.send_goal(fake_goal_wrapper)

    # Waits for the server to finish performing the action.
    kin_fu.action_client.wait_for_result()
    times['post_extract_time'] = rospy.get_rostime()
    rospy.loginfo("extracted...")

    # Retrieve the result
    action_result = kin_fu.action_client.get_result()
    times['pose_retrieve_time'] = rospy.get_rostime()
    rospy.loginfo("retrieved...")

    # Now publish the cloud part of the mesh
    publish_point_cloud(action_result.mesh)
    times['post_publish_time'] = rospy.get_rostime()
    rospy.loginfo("published...")

    # Now export the mesh
    export_ply_file(action_result.mesh)
    times['post_export_time'] = rospy.get_rostime()

    timing = {
        'extraction': (times['post_extract_time'] - times['start_time']).to_nsec() / 1000000,
        'retrieval': (times['pose_retrieve_time'] - times['post_extract_time']).to_nsec() / 1000000,
        'publishing': (times['post_publish_time'] - times['pose_retrieve_time']).to_nsec() / 1000000,
        # 'export': (times['post_export_time']-times['post_publish_time']).to_nsec()/1000000,
    }
    rospt.loginfo("Extraction timing summary")
    for item in timing.keys():
        print "\t\t\t" + item + ": " + str(timing[item]) + " milliseconds"


def publish_point_cloud(mesh_polygon):
    if not kin_fu.publishers['point_cloud']:
        kin_fu.publishers['point_cloud'] = rospy.Publisher(
            kin_fu.point_cloud_publishing_topic_name,
            PointCloud2,
            queue_size=5)
        kin_fu.publishers['point_cloud'].publish(mesh_polygon.cloud)


def export_ply_file(mesh):
    print "polygons:", str(len(mesh.polygons)),
    print "points:", str(mesh.cloud.width)


def KinFuConsumer_Factory():
    this_consumer = Generic()
    this_consumer.request_action_key = "/kinfu_output/actions/request"
    this_consumer.action_client = None
    this_consumer.seeds = {'mesh': 0}
    this_consumer.publishers = {'point_cloud': None}
    this_consumer.point_cloud_publishing_topic_name = '/kinfu/extract/pointcloud'
    return this_consumer


if __name__ == '__main__':
    class Generic:
        pass


    try:
        # this is the name of our simple interface to ros_kinfu
        kin_fu = KinFuConsumer_Factory()

        # Initializes a rospy node
        rospy.init_node('ros_kinfu_simple_action_client')

        # set a rate
        rate_val = .05
        rospy.loginfo("FYI: Mesh request rate is every: " + str(1 / rate_val) + " second(s)")
        rate = rospy.Rate(rate_val)  # once every ten (10) seconds

        # loop until we are killed
        while not rospy.is_shutdown():
            # result = fibonacci_client()
            result = mesh_request(True)
            # delay and then make another pass
            rate.sleep()
    except rospy.ROSInterruptException:
        print "program interrupted before completion"
    except:
        raise
