#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def keyboard_input():
    rospy.init_node('keyboard_input_node', anonymous=True)
    pub = rospy.Publisher('tracking_id', String, queue_size=10)
    rate = rospy.Rate(10)  # 10hz

    while not rospy.is_shutdown():
        try:
            # Python 2.x의 경우 raw_input()을 사용하세요.
            tracking_id = input("Enter tracking ID: ")
            rospy.loginfo("Received tracking ID: %s", tracking_id)

            # ROS 토픽에 메시지 발행
            pub.publish(tracking_id)
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

# ======== keep going pub
# #!/usr/bin/env python
# import rospy
# import threading
# from std_msgs.msg import String

# class KeyboardInputNode:
#     def __init__(self):
#         self.tracking_id = None
#         self.lock = threading.Lock()

#     def start(self):
#         rospy.init_node('keyboard_input_node', anonymous=True)
#         self.pub = rospy.Publisher('tracking_id', String, queue_size=10)
#         input_thread = threading.Thread(target=self.get_input)
#         input_thread.start()

#         rate = rospy.Rate(10)  # 10hz
#         while not rospy.is_shutdown():
#             with self.lock:
#                 if self.tracking_id is not None:
#                     self.pub.publish(self.tracking_id)
#             rate.sleep()

#     def get_input(self):
#         while not rospy.is_shutdown():
#             try:
#                 tracking_id_input = input("Enter tracking ID: ")
#                 with self.lock:
#                     self.tracking_id = tracking_id_input
#             except KeyboardInterrupt:
#                 break
#             except Exception as e:
#                 rospy.logerr("Error: %s", e)

# if __name__ == '__main__':
#     try:
#         node = KeyboardInputNode()
#         node.start()
#     except rospy.ROSInterruptException:
#         pass