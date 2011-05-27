#!/usr/bin/env python

import roslib; roslib.load_manifest('stereo_webcam')

import cv
import rospy
import message_filters

from cv_bridge       import CvBridge
from sensor_msgs.msg import Image, CameraInfo

pub_image = rospy.Publisher('image_sync', Image)

def CreateImageSub(i):
	topic      = 'camera{0}/image'.format(i)
	subscriber = message_filters.Subscriber(topic, Image)
	return subscriber

def ImageCallback(*msg_in):
	br   = CvBridge()
	num  = len(msg_in)
	imgs = map(lambda msg: br.imgmsg_to_cv(msg), msg_in)

	cols, rows = cv.GetSize(imgs[0])
	tiles = cv.CreateMat(rows, cols * num, cv.CV_8UC3)

	# Tile the individual images to form a single output.
	for i in range(0, num):
		tile = cv.GetSubRect(tiles, (i * cols, 0, cols, rows))
		cv.Copy(imgs[i], tile)

	msg_out = br.cv_to_imgmsg(tiles)
	msg_out.header.stamp    = msg_in[0].header.stamp
	msg_out.header.frame_id = msg_in[0].header.frame_id
	pub_image.publish(msg_out)

def main():
	rospy.init_node('test_node')

	cams = rospy.get_param('~cameras', 1)
	subs = map(CreateImageSub, range(0, cams))
	sync = message_filters.TimeSynchronizer(subs, 10)
	sync.registerCallback(ImageCallback)

	rospy.spin()

if __name__ == '__main__':
	main()
