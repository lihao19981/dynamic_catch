
import rospy


def talker():
	rospy.init_node("pytalker",anonymous=True)
	#pub = rospy.Publisher("gps_info",gps,queue_size=10) #Publisher 函数第一个参数是话题名称，第二个参数 数据类型，现在就是我们定义的msg 最后一个是缓冲区的大小
	#queue_size:None 不建议，会设置成为阻塞式同步收发模式
	#queue_size:0 不建议，会设置为无限缓冲区模式，危险
	#queue_size:10 or more  一般情况下设为10，queue_size太大会导致数据延时不同步
	#更新频率是1hz
	rate = rospy.Rate(1)
	x = 1.0
	y = 2.0
	state = "working"
	while not rospy.is_shutdown():
		rospy.loginfo('talker:gps:x=%f,y=%f')
		#pub.publish(gps(state,x,y))
		x = 2*x
		y = 2*y
		rate.sleep()
if __name__ = "__main__":
	talker()

