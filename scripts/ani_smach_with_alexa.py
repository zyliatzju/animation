#!/usr/bin/env python

import rospy
import bluetooth
from std_msgs.msg import Int16MultiArray, Int32, String
import smach 
import smach_ros
import random

class Utils():
	"""docstring for SeqSmach"""
	def __init__(self):
		rospy.init_node('ani_smach')
		rospy.Subscriber("alexa_listener_commands", String, self.alexa_callback, queue_size=10)
		rospy.Subscriber("animation_udp/rtc_feedback", Int16MultiArray, self.rtc_callback, queue_size=1)
		self.mid_pub = rospy.Publisher("animation_udp/motion_msg", Int32, queue_size=1)
		self.vid_pub = rospy.Publisher("animation_sm/bluetooth_vmsg", Int32, queue_size=1)
		rospy.Timer(rospy.Duration(1), self.plotting_callback)
		self.video_trans = [1000] 
		self.speech_cmd_queue = ["lookaround"]
		self.use_speech = rospy.get_param('use_speech',True)
		self.rate = 1/300.0
		self.track_ani = 0
		self.in_ani = 0
		self.ani_finished = 0
		self.prev_ani_finished = 0
		self.trans_time = 3.0
		self.t_trans_offset = 0
		self.trans_begun = 0
		self.pub_motion_msg = 0
		self.pub_video_msg = 0
		self.now = rospy.get_time()

		rospy.loginfo("animation smach is init")
		rospy.loginfo("use_speech set to {}".format(self.use_speech))

	def alexa_callback(self, msg):
		self.speech_cmd_queue.append(msg.data)
		rospy.loginfo("speech_cmd_queue updated to {}".format(self.speech_cmd_queue))

	def rtc_callback(self,msg):
		self.track_ani = msg.data[0]
		self.in_ani = msg.data[1]
		self.ani_finished = msg.data[2]

	def make_decision_on_motion(self):
		if self.track_ani is 1:
			if self.ani_finished is 1 and self.prev_ani_finished is not 1 and self.trans_begun is 0:
				self.trans_begun = 1
				self.t_trans_offset = rospy.get_time()
			self.prev_ani_finished = self.ani_finished
			
			self.now = rospy.get_time()
			if self.trans_begun is 1 and self.now > (self.t_trans_offset + self.trans_time):
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

	def plotting_callback(self, event):
		rospy.loginfo("[seq smach]: published motion msg is {}, video_msg is {}".format(self.pub_motion_msg, self.pub_video_msg))
		rospy.loginfo("[seq smach]: trans_begun:{}, waitingflag is:{}".format(self.trans_begun,self.now > (self.t_trans_offset + self.trans_time)))


##############################################################################################
##### State Machine ##########################################################################
##############################################################################################

class HappyState(smach.State):
	def  __init__(self):
		smach.State.__init__(self, 
			outcomes=['return_to_InAni'],
			input_keys=['holding_in'],
			output_keys=['status_out'])
		self.motion_queue = [2]
		self.video_queue = [2004]

	def execute(self, userdata):
		mid = random.randint(0,len(self.motion_queue)-1)
		vid = random.randint(0,len(self.video_queue)-1)
		trans_id = random.randint(0,len(utils.video_trans)-1)

		while not rospy.is_shutdown():
			curr_state = utils.make_decision_on_motion()
			curr_vstate = utils.make_decision_on_emotion()
			if curr_state == "in_motion":
				utils.pub_motion_msg = self.motion_queue[mid]
				utils.mid_pub.publish(self.motion_queue[mid])
				if curr_vstate == "begin_emotion":
					utils.pub_video_msg = self.video_queue[vid]
					utils.vid_pub.publish(self.video_queue[vid])
				else:
					utils.pub_video_msg = utils.video_trans[trans_id]
					utils.vid_pub.publish(utils.video_trans[trans_id])
			elif curr_state == "wait_transit":
				utils.pub_video_msg = utils.video_trans[trans_id]
				utils.vid_pub.publish(utils.video_trans[trans_id])
				if userdata.holding_in == 1:
					utils.pub_motion_msg = self.motion_queue[mid]
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
		self.motion_queue = [3]
		self.video_queue = [1000]

	def execute(self, userdata):
		mid = random.randint(0,len(self.motion_queue)-1)
		vid = random.randint(0,len(self.video_queue)-1)
		trans_id = random.randint(0,len(utils.video_trans)-1)

		while not rospy.is_shutdown():
			curr_state = utils.make_decision_on_motion()
			curr_vstate = utils.make_decision_on_emotion()
			if curr_state == "in_motion":
				utils.pub_motion_msg = self.motion_queue[mid]
				utils.mid_pub.publish(self.motion_queue[mid])
				if curr_vstate == "begin_emotion":
					utils.pub_video_msg = self.video_queue[vid]
					utils.vid_pub.publish(self.video_queue[vid])
				else:
					utils.pub_video_msg = utils.video_trans[trans_id]
					utils.vid_pub.publish(utils.video_trans[trans_id])
			elif curr_state == "wait_transit":
				utils.pub_video_msg = utils.video_trans[trans_id]
				utils.vid_pub.publish(utils.video_trans[trans_id])
				if userdata.holding_in == 1:
					utils.pub_motion_msg = self.motion_queue[mid]
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
		self.motion_queue = [1]
		self.video_queue = [1000]

	def execute(self, userdata):
		mid = random.randint(0,len(self.motion_queue)-1)
		vid = random.randint(0,len(self.video_queue)-1)
		trans_id = random.randint(0,len(utils.video_trans)-1)

		while not rospy.is_shutdown():
			curr_state = utils.make_decision_on_motion()
			curr_vstate = utils.make_decision_on_emotion()
			if curr_state == "in_motion":
				utils.pub_motion_msg = self.motion_queue[mid]
				utils.mid_pub.publish(self.motion_queue[mid])
				if curr_vstate == "begin_emotion":
					utils.pub_video_msg = self.video_queue[vid]
					utils.vid_pub.publish(self.video_queue[vid])
				else:
					utils.pub_video_msg = utils.video_trans[trans_id]
					utils.vid_pub.publish(utils.video_trans[trans_id])
			elif curr_state == "wait_transit":
				utils.pub_video_msg = utils.video_trans[trans_id]
				utils.vid_pub.publish(utils.video_trans[trans_id])
				if userdata.holding_in == 1:
					utils.pub_motion_msg = self.motion_queue[mid]
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
		self.motion_queue = [3]
		self.video_queue = [1000]

	def execute(self, userdata):
		mid = random.randint(0,len(self.motion_queue)-1)
		vid = random.randint(0,len(self.video_queue)-1)
		trans_id = random.randint(0,len(utils.video_trans)-1)

		while not rospy.is_shutdown():
			curr_state = utils.make_decision_on_motion()
			curr_vstate = utils.make_decision_on_emotion()
			if curr_state == "in_motion":
				utils.pub_motion_msg = self.motion_queue[mid]
				utils.mid_pub.publish(self.motion_queue[mid])
				if curr_vstate == "begin_emotion":
					utils.pub_video_msg = self.video_queue[vid]
					utils.vid_pub.publish(self.video_queue[vid])
				else:
					utils.pub_video_msg = utils.video_trans[trans_id]
					utils.vid_pub.publish(utils.video_trans[trans_id])
			elif curr_state == "wait_transit":
				utils.pub_video_msg = utils.video_trans[trans_id]
				utils.vid_pub.publish(utils.video_trans[trans_id])
				if userdata.holding_in == 1:
					utils.pub_motion_msg = self.motion_queue[mid]
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
		self.motion_queue = [3]
		self.video_queue = [1000]

	def execute(self, userdata):
		mid = random.randint(0,len(self.motion_queue)-1)
		vid = random.randint(0,len(self.video_queue)-1)
		trans_id = random.randint(0,len(utils.video_trans)-1)

		while not rospy.is_shutdown():
			curr_state = utils.make_decision_on_motion()
			curr_vstate = utils.make_decision_on_emotion()
			if curr_state == "in_motion":
				utils.pub_motion_msg = self.motion_queue[mid]
				utils.mid_pub.publish(self.motion_queue[mid])
				if curr_vstate == "begin_emotion":
					utils.pub_video_msg = self.video_queue[vid]
					utils.vid_pub.publish(self.video_queue[vid])
				else:
					utils.pub_video_msg = utils.video_trans[trans_id]
					utils.vid_pub.publish(utils.video_trans[trans_id])
			elif curr_state == "wait_transit":
				utils.pub_video_msg = utils.video_trans[trans_id]
				utils.vid_pub.publish(utils.video_trans[trans_id])
				if userdata.holding_in == 1:
					utils.pub_motion_msg = self.motion_queue[mid]
					utils.mid_pub.publish(self.motion_queue[mid])
				else:
					userdata.status_out = curr_state
					return 'return_to_InAni'
			elif curr_state == "new_motion":
				userdata.status_out = curr_state
				return 'return_to_InAni'

			rospy.sleep(utils.rate)


class DanceState(smach.State):
	def  __init__(self):
		smach.State.__init__(self, 
			outcomes=['return_to_InAni'],
			input_keys=['holding_in'],
			output_keys=['status_out'])
		self.motion_queue = [2]
		self.video_queue = [2004]

	def execute(self, userdata):
		mid = random.randint(0,len(self.motion_queue)-1)
		vid = random.randint(0,len(self.video_queue)-1)
		trans_id = random.randint(0,len(utils.video_trans)-1)

		while not rospy.is_shutdown():
			curr_state = utils.make_decision_on_motion()
			curr_vstate = utils.make_decision_on_emotion()
			if curr_state == "in_motion":
				utils.pub_motion_msg = self.motion_queue[mid]
				utils.mid_pub.publish(self.motion_queue[mid])
				if curr_vstate == "begin_emotion":
					utils.pub_video_msg = self.video_queue[vid]
					utils.vid_pub.publish(self.video_queue[vid])
				else:
					utils.pub_video_msg = utils.video_trans[trans_id]
					utils.vid_pub.publish(utils.video_trans[trans_id])
			elif curr_state == "wait_transit":
				utils.pub_video_msg = utils.video_trans[trans_id]
				utils.vid_pub.publish(utils.video_trans[trans_id])
				if userdata.holding_in == 1:
					utils.pub_motion_msg = self.motion_queue[mid]
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
			outcomes=['happy','concerned','angry','lookaround','sad','dance','end'],
			input_keys=['status_in'],
			output_keys=['holding_out','end_of_queue_out'])
		self.ani_seq = ['lookaround','happy','sad','lookaround']
		self.seq_id = 0

	def execute(self, userdata):
		if utils.use_speech == True:
			queue = utils.speech_cmd_queue
		else:
			queue = self.ani_seq

		rospy.loginfo("queue to play is {}".format(queue))

		if userdata.status_in == "wait_transit":
			userdata.holding_out = 1
			self.seq_id = 0
		elif userdata.status_in == "new_motion":
			userdata.holding_out = 0
			self.seq_id = self.seq_id+1 
			# if seq_id is larger than last one + 1
			if self.seq_id >= len(queue):
				self.seq_id = len(queue)
				while not rospy.is_shutdown():
					if utils.use_speech:
						queue = utils.speech_cmd_queue
						if self.seq_id < len(queue):
							rospy.loginfo("speech queue is enlarged") 
							break
					if utils.track_ani == 0:
						userdata.holding_out = 1
						self.seq_id = 0
						rospy.loginfo("[seq smach]: not tracking ani, reset")
						break

					rospy.loginfo("[seq smach]: queue is finished, waiting")
					rospy.sleep(1)
			else:
				self.seq_id = min(self.seq_id, len(queue)-1)

		return queue[self.seq_id]



if __name__ == '__main__':
	global utils
	utils = Utils()
	sm = smach.StateMachine(outcomes=['end'])
	sm.userdata.holding = 1
	sm.userdata.status = "wait_transit"

	with sm:
		smach.StateMachine.add('InAniState', InAniState(), 
			transitions={'happy':'HappyState','concerned':'ConcernedState','angry':'AngryState',
			'lookaround':'LookaroundState','sad':'SadState','dance':'DanceState','end':'end'},
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
		smach.StateMachine.add('DanceState', DanceState(),
			transitions={'return_to_InAni':'InAniState'},
			remapping={'holding_in':'holding','status_out':'status'})


	sm.execute()
	rospy.loginfo("[ani_smach]: smach init")
	rospy.spin()
