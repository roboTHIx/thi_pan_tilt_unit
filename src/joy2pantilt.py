 #! /usr/bin/env python

import rospy


#from std_msgs.msg import Float32MultiArray
#from sensor_msgs.msg import Joy




#def joy_callback(data):
#    # axis 3 and 4 are pan and tilt
#    pan  = data.axes[3] 
#    tilt = data.axes[4]





def talker():
 #   pantilt_pub = rospy.Publisher('/pan_tilt/position', Float32MultiArray, queue_size=10)
 #   joy_sub     = rospy.Subscriber('/joy', Joy, joy_callback)
    rospy.init_node('talker', anonymous=True)

    rate = rospy.Rate(1) # 10hz
    while not rospy.is_shutdown():
        # print("1")
        # pantilt_msg = Float32MultiArray()
        # pantilt_msg.data = [0.0, 0.0]

        # rospy.loginfo(pantilt_msg)
        # pantilt_pub.publish(pantilt_msg)
        rate.sleep()




if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass