#!/usr/bin/env python

import rospy
import tf2_ros

if __name__ == '__main__':
    print("starting1")
    # rospy.init_node('tf_listener_node')
    tfBuffer = tf2_ros.Buffer()
    # listener = tf.TransformListener()
    listener = tf2_ros.TransformListener(tfBuffer)

    print("starting2")
    # Wait for the tf data to become available
    # rospy.sleep(1.0)
    print("starting3")

    rate = rospy.Rate(1.0)  # Update rate in Hz
    

    try:
        print("starting4")
        while not rospy.is_shutdown():
            print("starting5")
            try:
                # Get the list of available frames
                trans = tfBuffer.lookup_transform("base_link", "camera_color_frame", rospy.Time())
                # frames = listener.getFrameStrings()
                # print("Available frames:")
                # print(frames)
                # print("")

                print(trans)
                exit()

            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.logwarn("TF lookup failed!")

            rate.sleep()

    except rospy.ROSInterruptException:
        pass
