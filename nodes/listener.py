import rospy
import tf2_ros

from robot import HelloRobot


class Listener:

    def __init__(self):

        # try:
        #     rospy.init_node('tf_listener_node')
        # except rospy.exceptions.ROSException:
        #     print('node already initialized')
        
        self.tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(self.tfBuffer)

        self.rate = rospy.Rate(1.0) 

    def start(self, parent_frame, child_frame):

        try:
            while not rospy.is_shutdown():
                try:
                    trans = self.tfBuffer.lookup_transform(child_frame, parent_frame, rospy.Time())

                    print(trans)
                    return trans
                
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                    rospy.logwarn("TF lookup failed!")

                self.rate.sleep()
                
        except rospy.ROSInterruptException:
            pass

if __name__=="__main__":
    listener = Listener()
    listener.start("base_link", "camera_color_frame")


        