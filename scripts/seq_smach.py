#!/usr/bin/env python

import rospy
import bluetooth
from std_msgs.msg import Int16MultiArray, Int32
import random


class SeqSmach():
	"""docstring for SeqSmach"""
	def __init__(self):
		rospy.init_node('seq_smach')
		rospy.Subscriber("animation_udp/rtc_feedback", Int16MultiArray, self.rtc_callback, queue_size=1)
		rospy.Timer(rospy.Duration(1), self.plotting_callback)
		self.mid_pub = rospy.Publisher("animation_udp/motion_msg", Int32, queue_size=1)
		self.vid_pub = rospy.Publisher("animation_sm/bluetooth_vmsg", Int32, queue_size=1)

		self.motion_queue = [1,2,3,4,5]
		self.video_queue = [1000, 2004, 1000, 1005, 1007]
		# self.motion_queue = [4,5]
		# self.video_queue = [1005, 1007]
		
		self.trans_video = 1000 #video for transition 
		self.midx = 0
		self.vidx = 0
		self.motion_msg = 0
		self.video_msg = 0
		self.track_ani = 0
		self.in_ani = 0
		self.ani_finished = 0
		self.prev_ani_finished = 0
		self.trans_time = 3.0
		self.t_trans_offset = 0
		self.trans_begun = 0
		self.now = rospy.get_time()
		rospy.loginfo("sequential smach is init")

	def rtc_callback(self,msg):
		self.track_ani = msg.data[0]
		self.in_ani = msg.data[1]
		self.ani_finished = msg.data[2]

	def make_decision(self):
		if self.track_ani is 0:
			self.midx = 0
			self.vidx = 0

		if self.track_ani is 1:
			if self.ani_finished is 1 and self.prev_ani_finished is not 1 and self.trans_begun is 0:
				self.trans_begun = 1
				self.t_trans_offset = rospy.get_time()

			self.now = rospy.get_time()
			if self.trans_begun is 1 and self.now > (self.t_trans_offset + self.trans_time):
				self.midx = min(self.midx+1, len(self.motion_queue)-1)
				self.vidx = min(self.vidx+1, len(self.video_queue)-1)
				self.trans_begun = 0

			self.motion_msg = self.motion_queue[self.midx]
			if self.in_ani:
				self.video_msg = self.video_queue[self.vidx]
			else:
				self.video_msg = self.trans_video
		else:
			self.motion_msg = self.motion_queue[0]
			self.video_msg = self.trans_video

		self.mid_pub.publish(self.motion_msg)
		self.vid_pub.publish(self.video_msg)

		self.prev_ani_finished = self.ani_finished

	def decision_loop(self):
		while not rospy.is_shutdown():
			self.make_decision()
			rospy.sleep(1/200.0)

	def plotting_callback(self, event):
		rospy.loginfo("[seq smach]: motion id:{}, video id:{}".format(self.motion_msg, self.video_msg))
		rospy.loginfo("[seq smach]: trans_begun:{}, waitingflag is:{}".format(self.trans_begun,self.now > (self.t_trans_offset + self.trans_time)))

if __name__ == '__main__':
	ss = SeqSmach()
	ss.decision_loop()
	rospy.spin()
