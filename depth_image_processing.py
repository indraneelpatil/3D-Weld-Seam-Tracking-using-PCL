#!/usr/bin/env python
# license removed for brevity

import roslib
import sys
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float32
import time

def depth_callback(data):
			global pub1,pub2,rate,bridge_object
		
        		# We select bgr8 because its the OpneCV encoding by default
        		cv_image = bridge_object.imgmsg_to_cv2(data ,desired_encoding = '16UC1')
			#cv_image = self.bridge_object.imgmsg_to_cv2(data ,desired_encoding = 'bgr8')
			#except CvBridgeError as e:
			#	print(e)
		
	

			depth_array = np.array(cv_image, dtype=np.float32)
			depth_array[np.isnan(depth_array)] = 0
			cv2.normalize(depth_array, depth_array, 0, 1, cv2.NORM_MINMAX)
			depth_array2 = depth_array*255
			#target = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
	
			#print(depth_array.shape)
			blur = cv2.GaussianBlur(depth_array2 , (9,9) , 0)
			ret, thresh = cv2.threshold(blur, 3, 255, cv2.THRESH_BINARY_INV)		
			#ret, thresh = cv2.threshold(blur, 1, 255, cv2.THRESH_BINARY_INV)
			kernel = np.ones((9,9), np.uint8)
			img_erosion = cv2.erode(thresh, kernel, iterations=1)
			img_dilation = cv2.dilate(img_erosion, kernel, iterations=1)	 				
			cvuint8 = cv2.convertScaleAbs(img_dilation)
			#cvuint8 = cv2.bitwise_not(cvuint8)
			#frame_centre = [212,256]
			
			image, contours, hier = cv2.findContours(cvuint8, cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
			# Find the index of the largest contour
			areas = [cv2.contourArea(c) for c in contours]
			max_index = np.argmax(areas)
			cnt=contours[max_index]
			#cv2.drawContours(cvuint8, contours, -1, (0,255,0), 3)

			if cv2.contourArea(cnt)>5000:
				#print cv2.contourArea(cnt)
				#time.sleep(1)
	    			x, y, w, h = cv2.boundingRect(cnt)
	    			# draw a rectangle to visualize the bounding rect
				cv2.rectangle(cvuint8, (x, y), (x+w, y+h), (255, 255, 255), 2)
				centre= (((x+x+w)/2),((y+y+h)/2))
				k = ((x+x+w)/2)
				l = ((y+y+h)/2)
					
				er_y=float(((x+x+w)/2)-212)*(-1)
				er_z=float(((y+y+h)/2)-256)*(-1)
				print centre
				px = depth_array[k,l]
				#print(px)
		
				print('********************************')
				#rospy.loginfo(er_x)
				
				pub1.publish(er_y)
				#pub2.publish(er_z)
				
			
			rate.sleep()
			cv2.imshow("depth",cvuint8)
	    		cv2.waitKey(1)
				
			
			
		
		#except CvBridgeError as e:
		#	print(e)
				
				
		


def listener():
		global pub1,pub2, rate,bridge_object,pub3
    		rospy.init_node('zed_depth', anonymous=True)
    		bridge_object = CvBridge()
		pub1 = rospy.Publisher('er_y' , Float32, queue_size = 10)
		pub2 = rospy.Publisher('er_z' , Float32, queue_size = 10)
    		rospy.Subscriber("/kinect2/sd/image_depth",Image ,depth_callback)
		
		
		rate = rospy.Rate(100)
    		
if __name__ == '__main__':
	try:
			detector = listener()
			rospy.spin()

	except rospy.ROSInterruptException:
			rospy.loginfo("Detector node terminated.")
			cv2.destroyAllWindows()
	
