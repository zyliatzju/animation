#!/usr/bin/env python

import rospy
import bluetooth
from std_msgs.msg import Int16MultiArray, Int32
import smach 
import smach_ros
import random

class Utils():
	"""docstring for SeqSmach"""
	def __init__(self):
		rospy.init_node('ani_smach')
		rospy.Subscriber("animation_udp/rtc_feedback", Int16MultiArray, self.rtc_callback, queue_size=1)
		self.mid_pub = rospy.Publisher("animation_udp/motion_msg", Int32, queue_size=1)
		self.vid_pub = rospy.Publisher("animation_sm/bluetooth_vmsg", Int32, queue_size=1)
		self.video_trans = [11001000,11001002]
		self.rate = 1/300.0
		self.track_ani = 0
		self.in_ani = 0
		self.ani_finished = 0
		self.prev_ani_finished = 0
		self.trans_time = 3.0
		self.t_trans_offset = 0
		self.trans_begun = 0
		rospy.loginfo("animation smach is init")

	def rtc_callback(self,msg):
		self.track_ani = msg.data[0]
		self.in_ani = msg.data[1]
		self.ani_finished = msg.data[2]

	def make_decision_on_motion(self):
		if self.track_ani is 1:
			if self.ani_finished is 1 and self.prev_ani_finished is not 1 and self.trans_begun is 0:
				self.trans_begun = 1
				self.t_trans_offset = rospy.get_time()

			now = rospy.get_time()
			if self.trans_begun is 1 and now > (self.t_trans_offset + self.trans_time):
				self.trans_begun = 0
				return "new_motion"
			else:
				return "in_motion"
		else:
			return "wait_transit"

	def make_decision_on_emotion(self):
		if self.track_ani is 1:
			if self.in_ani:
				return "begin_emotion"
			else:
				return "in_transit"
		else:
			return "in_transit"


	# def make_decision_2(self):
	# 	if self.track_ani is 0:
	# 		self.midx = 0
	# 		self.vidx = 0

	# 	if self.track_ani is 1:
	# 		if self.ani_finished is 1 and self.prev_ani_finished is not 1 and self.trans_begun is 0:
	# 			self.trans_begun = 1
	# 			self.t_trans_offset = rospy.get_time()

	# 		now = rospy.get_time()
	# 		if self.trans_begun is 1 and now > (self.t_trans_offset + self.trans_time):
	# 			self.midx = min(self.midx+1, len(self.motion_queue))
	# 			self.vidx = min(self.vidx+1, len(self.video_queue))
	# 			self.trans_begun = 0

	# 		self.motion_msg = self.motion_queue[self.midx]
	# 		if self.in_ani: #animation 
	# 			self.video_msg = self.video_queue[self.vidx]
	# 		else: #transition 
	# 			self.video_msg = self.trans_video
	# 	else:
	# 		self.motion_msg = self.motion_queue[0]
	# 		self.video_msg = self.trans_video

	# 	self.mid_pub.publish(self.motion_msg)
	# 	self.vid_pub.publish(self.video_msg)

	# 	self.prev_ani_finished = self.ani_finished

##############################################################################################
##### State Machine ##########################################################################
##############################################################################################

class HappyState(smach.State):
	def  __init__(self):
		smach.State.__init__(self, 
			outcomes=['return_to_InAni'],
			input_keys=['holding_in'],
			output_keys=['status_out'])
		self.motion_queue = [6,13]
		self.video_queue = [11001003,11001001]

	def execute(self, userdata):
		mid = random.randomint(0,len(self.motion_queue))
		vid = random.randomint(0,len(self.video_queue))
		trans_id = random.randomint(0,len(utils.video_trans))

		while not rospy.is_shutdown():
			curr_state = utils.make_decision_on_motion()
			if curr_state == "in_motion":
				utils.mid_pub.publish(self.motion_queue[mid])
				curr_vstate = utils.make_decision_on_emotion()
				if curr_vstate == "begin_emotion":
					utils.vid_pub.publish(self.video_queue[vid])
				else:
					utils.vid_pub.publish(utils.video_trans[trans_id])
			elif curr_state == "wait_transit":
				if userdata.holding_in == 1:
					utils.mid_pub.publish(self.motion_queue[mid])
				else:
					userdata.status_out = curr_state
					return 'return_to_InAni'
			elif curr_state == "new_motion":
				userdata.status_out = curr_state
				return 'return_to_InAni'

			rospy.sleep(utils.rate)

class SadState(smach.State):
	def  __init__(self):
		smach.State.__init__(self, 
			outcomes=['return_to_InAni'],
			input_keys=['holding_in'],
			output_keys=['status_out'])
		self.motion_queue = [6,13]
		self.video_queue = [11001003,11001001]

	def execute(self, userdata):
		mid = random.randomint(0,len(self.motion_queue))
		vid = random.randomint(0,len(self.video_queue))
		trans_id = random.randomint(0,len(utils.video_trans))

		while not rospy.is_shutdown():
			curr_state = utils.make_decision_on_motion()
			if curr_state == "in_motion":
				utils.mid_pub.publish(self.motion_queue[mid])
				curr_vstate = utils.make_decision_on_emotion()
				if curr_vstate == "begin_emotion":
					utils.vid_pub.publish(self.video_queue[vid])
				else:
					utils.vid_pub.publish(utils.video_trans[trans_id])
			elif curr_state == "wait_transit":
				if userdata.holding_in == 1:
					utils.mid_pub.publish(self.motion_queue[mid])
				else:
					userdata.status_out = curr_state
					return 'return_to_InAni'
			elif curr_state == "new_motion":
				userdata.status_out = curr_state
				return 'return_to_InAni'

			rospy.sleep(utils.rate)

class LookaroundState(smach.State):
	def  __init__(self):
		smach.State.__init__(self, 
			outcomes=['return_to_InAni'],
			input_keys=['holding_in'],
			output_keys=['status_out'])
		self.motion_queue = [6,13]
		self.video_queue = [11001003,11001001]

	def execute(self, userdata):
		mid = random.randomint(0,len(self.motion_queue))
		vid = random.randomint(0,len(self.video_queue))
		trans_id = random.randomint(0,len(utils.video_trans))

		while not rospy.is_shutdown():
			curr_state = utils.make_decision_on_motion()
			if curr_state == "in_motion":
				utils.mid_pub.publish(self.motion_queue[mid])
				curr_vstate = utils.make_decision_on_emotion()
				if curr_vstate == "begin_emotion":
					utils.vid_pub.publish(self.video_queue[vid])
				else:
					utils.vid_pub.publish(utils.video_trans[trans_id])
			elif curr_state == "wait_transit":
				if userdata.holding_in == 1:
					utils.mid_pub.publish(self.motion_queue[mid])
				else:
					userdata.status_out = curr_state
					return 'return_to_InAni'
			elif curr_state == "new_motion":
				userdata.status_out = curr_state
				return 'return_to_InAni'

			rospy.sleep(utils.rate)

class AngryState(smach.State):
	def  __init__(self):
		smach.State.__init__(self, 
			outcomes=['return_to_InAni'],
			input_keys=['holding_in'],
			output_keys=['status_out'])
		self.motion_queue = [6,13]
		self.video_queue = [11001003,11001001]

	def execute(self, userdata):
		mid = random.randomint(0,len(self.motion_queue))
		vid = random.randomint(0,len(self.video_queue))
		trans_id = random.randomint(0,len(utils.video_trans))

		while not rospy.is_shutdown():
			curr_state = utils.make_decision_on_motion()
			if curr_state == "in_motion":
				utils.mid_pub.publish(self.motion_queue[mid])
				curr_vstate = utils.make_decision_on_emotion()
				if curr_vstate == "begin_emotion":
					utils.vid_pub.publish(self.video_queue[vid])
				else:
					utils.vid_pub.publish(utils.video_trans[trans_id])
			elif curr_state == "wait_transit":
				if userdata.holding_in == 1:
					utils.mid_pub.publish(self.motion_queue[mid])
				else:
					userdata.status_out = curr_state
					return 'return_to_InAni'
			elif curr_state == "new_motion":
				userdata.status_out = curr_state
				return 'return_to_InAni'

			rospy.sleep(utils.rate)

class ConcernedState(smach.State):
	def  __init__(self):
		smach.State.__init__(self, 
			outcomes=['return_to_InAni'],
			input_keys=['holding_in'],
			output_keys=['status_out'])
		self.motion_queue = [6,13]
		self.video_queue = [11001003,11001001]

	def execute(self, userdata):
		mid = random.randomint(0,len(self.motion_queue))
		vid = random.randomint(0,len(self.video_queue))
		trans_id = random.randomint(0,len(utils.video_trans))

		while not rospy.is_shutdown():
			curr_state = utils.make_decision_on_motion()
			if curr_state == "in_motion":
				utils.mid_pub.publish(self.motion_queue[mid])
				curr_vstate = utils.make_decision_on_emotion()
				if curr_vstate == "begin_emotion":
					utils.vid_pub.publish(self.video_queue[vid])
				else:
					utils.vid_pub.publish(utils.video_trans[trans_id])
			elif curr_state == "wait_transit":
				if userdata.holding_in == 1:
					utils.mid_pub.publish(self.motion_queue[mid])
				else:
					userdata.status_out = curr_state
					return 'return_to_InAni'
			elif curr_state == "new_motion":
				userdata.status_out = curr_state
				return 'return_to_InAni'

			rospy.sleep(utils.rate)


class InAniState(smach.State):
	def __init__(self):
		smach.State.__init__(self, 
			outcomes=['happy','concerned','angry','lookaround','sad'],
			input_keys=['status_in'],
			output_keys=['holding_out'])
		self.ani_seq = ['lookaround','concerned','happy']
		self.seq_id = 0

	def execute(self, userdata):
		if userdata.status_in == "wait_transit":
			userdata.holding_out = 1
			self.seq_id = 0
		elif userdata.status_in == "new_motion":
			userdata.holding_out = 0
			self.seq_id = min(self.seq_id+1, len(self.ani_seq))
		
		return self.ani_seq[self.seq_id]



if __name__ == '__main__':
	global utils
	utils = Utils()
	sm = smach.StateMachine()
	sm.userdata.holding = 1
	sm.userdata.status = "wait_transit"

	with sm:
		smach.StateMachine.add('InAniState', InAniState(), 
			transitions={'happy':'HappyState','concerned':'ConcernedState','angry':'AngryState',
			'lookaround':'LookaroundState','sad':'SadState'},
			remapping={'status_in':'status','holding_out':'holding'})
		smach.StateMachine.add('HappyState', HappyState(),
			transitions={'return_to_InAni':'InAniState'},
			remapping={'holding_in':'holding','status_out':'status'})
		smach.StateMachine.add('SadState', SadState(),
			transitions={'return_to_InAni':'InAniState'},
			remapping={'holding_in':'holding','status_out':'status'})
		smach.StateMachine.add('LookaroundState', LookaroundState(),
			transitions={'return_to_InAni':'InAniState'},
			remapping={'holding_in':'holding','status_out':'status'})
		smach.StateMachine.add('AngryState', AngryState(),
			transitions={'return_to_InAni':'InAniState'},
			remapping={'holding_in':'holding','status_out':'status'})
		smach.StateMachine.add('ConcernedState', ConcernedState(),
			transitions={'return_to_InAni':'InAniState'},
			remapping={'holding_in':'holding','status_out':'status'})

	sm.execute()
	rospy.loginfo("[ani_smach]: smach init")
	rospy.spin()
