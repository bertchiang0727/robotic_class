import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import Int32  # 假設完成訊號是 Int32 值，完成為 1

a1 = [0.0, 0.0, 0.0]
a2 = [0.0, 0.0, 1.0]
a3 = [0.0, 0.0, 2.0]
b1 = [1.0, 0.0, 0.0]
b2 = [1.0, 0.0, 1.0]
b3 = [1.0, 0.0, 2.0]
c1 = [2.0, 0.0, 0.0]
c2 = [2.0, 0.0, 1.0]
c3 = [2.0, 0.0, 2.0]

def send_point(point, pub):
    msg = Point()
    msg.x, msg.y, msg.z = point
    pub.publish(msg)
    rospy.loginfo(f"➡️ 發送點位: {point}")

def wait_for_done():
    while not rospy.is_shutdown():
        msg = rospy.wait_for_message('/point_done', Int32)
        if msg.data == 1:
            rospy.loginfo("✅ 完成點位移動")
            break
        rospy.sleep(0.1)  # 避免佔用過多 CPU

def first_state_1(pub):
    path = [b1, a1, c1, a1]
    for point in path:
        send_point(point, pub)
        wait_for_done()

if __name__ == '__main__':
    rospy.init_node('point_sender_with_wait')
    pub = rospy.Publisher('/target_point', Point, queue_size=10)
    rospy.sleep(1.0)  # 等待建立連線
    first_state_1(pub)