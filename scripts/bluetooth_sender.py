#!/usr/bin/env python

import rospy
import bluetooth
from std_msgs.msg import Int32

class BlueSender():
	"""docstring for BlueSender"""
	def __init__(self):
		rospy.init_node('bt_sender')
		rospy.Subscriber("animation_sm/bluetooth_vmsg", Int32, self.vmsg_callback)
		self.rec_vid = 0
		self.prev_vid = 0
		self.faddr = "24:4C:E3:63:1B:CA"
		self.uuid = "94f39d29-7d6d-437d-973b-fba39e49d4ee"
		self.Rate = rospy.Rate(1/3.0)
		
		self.init_sock()
		rospy.loginfo("bluetooth sender is init")

	def init_sock(self):
		self.find_device()
		rospy.sleep(5)
		self.sock = self.call_service()
		rospy.loginfo("begin to handshake")
		rospy.sleep(3)
		while not rospy.is_shutdown():
			self.sock.send(str(0))
			rospy.loginfo("msg sent: {}".format(0))
			data = self.sock.recv(1024)
			rospy.loginfo("data recv: {}".format(data))
			if data == '0':
				rospy.loginfo("ready")
				break		
			self.Rate.sleep()


	def find_device(self):
		found = 0;

		rospy.loginfo("performing inquiry...")

		while not rospy.is_shutdown():

			nearby_devices = bluetooth.discover_devices(
			        duration=5, lookup_names=True, flush_cache=True, lookup_class=False)

			rospy.loginfo("found %d devices" % len(nearby_devices))

			for addr, name in nearby_devices:
			    try:
			        rospy.loginfo("  %s - %s" % (addr, name))
			    except UnicodeEncodeError:
			        rospy.loginfo("  %s - %s" % (addr, name.encode('utf-8', 'replace')))

			    if self.faddr == addr:
			    	rospy.loginfo("{} found".format(self.faddr))
			    	found = 1

			if not found:
				rospy.loginfo("{} not found".format(self.faddr))
			else:
				break

			self.Rate.sleep()


	def call_service(self):
		r = rospy.Rate(1/3.0)
		rospy.loginfo("connecting to service {}...".format(self.uuid))
		while not rospy.is_shutdown():
			service_matches = bluetooth.find_service(uuid = self.uuid,address = self.faddr)
			if len(service_matches) == 0:
				rospy.loginfo("couldn't find the service = {}, try again".format(uuid))
			else:
				rospy.loginfo("service found")
				break

			r.sleep()

		first_match = service_matches[0]
		port = first_match["port"]
		name = first_match["name"]
		host = first_match["host"]

		rospy.loginfo("connecting to {} on {} on port {}".format(name,host, port))
		sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
		sock.connect((host,port))

		return sock 

	def sender_test(self):
		for msg in self.video_queue:
			self.sock.send(msg)
			rospy.loginfo("msg sent: {}".format(msg))
			while not rospy.is_shutdown():
				data = self.sock.recv(1024)
				rospy.loginfo("data recv: {}".format(data))
				if data == '0':
					rospy.loginfo("received feedback, allow to send the next code")
					break
				rospy.sleep(1)

			self.Rate.sleep()	


	def vmsg_callback(self,msg):
		self.rec_vid = msg.data


	def sender_loop(self):
		status = 0
		while not rospy.is_shutdown():
			if self.rec_vid != self.prev_vid or status == 0:
				status = 0
				self.sock.send(str(self.rec_vid))
				rospy.loginfo("vmsg sent: {}".format(self.rec_vid))
				data = self.sock.recv(1024)
				rospy.loginfo("data recv: {}".format(data))
				if data == '0':
					rospy.loginfo("feedback received, status ok")
					status = 1

			self.prev_vid = self.rec_vid
			rospy.sleep(1/300.0)

	
if __name__ == '__main__':
	bs = BlueSender()
	bs.sender_loop()
	rospy.spin()
