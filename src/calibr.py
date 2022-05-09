import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
import tf2_ros
import tf2_geometry_msgs
import geometry_msgs.msg


rospy.init_node('calibr',anonymous=True)
#初始化tf2
tf_buffer = tf2_ros.Buffer(rospy.Duration(2))
tf2_ros.TransformListener(tf_buffer)
step=0

rate = rospy.Rate(20)

def get_transformation(source_frame, target_frame,
                       tf_cache_duration=2.0):


    # get the tf at first available time
    try:
        transformation = tf_buffer.lookup_transform(target_frame,
                source_frame, rospy.Time(0), rospy.Duration(2))
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException):
        rospy.logerr('Unable to find the transformation from %s to %s'
                     % source_frame, target_frame)
    return transformation


def transform_pose(transformation, pose):
    tfpose = \
        tf2_geometry_msgs.do_transform_pose(pose,transformation)
    return tfpose

def callback(data):
    global orb_distance,step

    world_to_map = get_transformation("world", "map")
    tfpose=transform_pose(world_to_map,data)
    orb_distance=tfpose.pose.position.z
    step=1


def calibration(data):
    global K,step
    real_distance=data.data*0.1
    K=real_distance/orb_distance
    step=2



while step==0:
    rospy.Subscriber('/orb_slam3_ros/camera',PoseStamped,callback,queue_size=1)


while step==1:
    rospy.Subscriber("/tof",Float32,calibration,queue_size=2)

print(K)
