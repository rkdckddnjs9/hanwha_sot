#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Int32

def keyboard_input():
    rospy.init_node('keyboard_input_node', anonymous=True)
    pub = rospy.Publisher('/arion/mtt/core/follow_target_id', Int32, queue_size=10)
    rate = rospy.Rate(10)  # 10hz

    while not rospy.is_shutdown():
        try:
            # Python 2.x의 경우 raw_input()을 사용하세요.
            tracking_id = int(input("Enter tracking ID: "))
            rospy.loginfo("Received tracking ID: %d", tracking_id)

            # ROS 토픽에 메시지 발행
            pub.publish(Int32(tracking_id))
        except KeyboardInterrupt:
            break
        except Exception as e:
            rospy.logerr("Error: %s", e)

        rate.sleep()

if __name__ == '__main__':
    try:
        keyboard_input()
    except rospy.ROSInterruptException:
        pass