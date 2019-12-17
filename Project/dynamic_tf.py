
import rospy
import tf
import tf.msg
import geometry_msgs.msg
import math
from tf import TransformListener
import numpy as np

median_array = np.zeros((10), dtype=np.float64)


class DynamicTFBroadcaster:
	def __init__(self):
		self.pub_tf = rospy.Publisher("/denso_cube_tf", tf.msg.tfMessage, queue_size=10)
		self.tf = TransformListener()
		self.t = geometry_msgs.msg.TransformStamped()
		self.translation_array = np.zeros((3), dtype=np.float64)
		self.mainloop()
	
	""" def median(self,array):
			norma = np.sqrt(np.power(array, 2))
			median_array.pop(0)
			median_array.append(norma)
			median_array.sort()
			print(array)
			median = median_array(6)
			return median """

	def mainloop(self):
		values_array = np.zeros((7,4), dtype=np.float64)

		i = 0
		while not rospy.is_shutdown():
			# Run this loop at about 10Hz
			rospy.sleep(0.5)
			
			if self.tf.frameExists("denso") and self.tf.frameExists("ar_marker_4"):
					#t = self.tf.getLatestCommonTime("world", "ar_marker_7")
					position, quaternion = self.tf.lookupTransform("denso", "ar_marker_4", rospy.Time(0))
					self.t.header.frame_id = "denso"
					self.t.header.stamp = rospy.Time.now()
					self.t.child_frame_id = "ar_marker_4"
					self.t.transform.translation.x = position[0]
					self.t.transform.translation.y = position[1]
					self.t.transform.translation.z = position[2]

					
				
					self.t.transform.rotation.x = quaternion[0]
					self.t.transform.rotation.y = quaternion[1]
					self.t.transform.rotation.z = quaternion[2]
					self.t.transform.rotation.w = quaternion[3]
					tfm = tf.msg.tfMessage([self.t])
					self.pub_tf.publish(tfm)

					values_array[0][i] = position[0]
					values_array[1][i] = position[1]
					values_array[2][i] = position[2]
					values_array[3][i] = quaternion[0]
					values_array[4][i] = quaternion[1]
					values_array[5][i] = quaternion[2]
					values_array[6][i] = quaternion[3]

					#translation_array = [position[0], position[1], position[2]
	
					if(i == 3):
						i = 0
						translation_x = self.mean(values_array[0]) 
						translation_y = self.mean(values_array[1])
						translation_z = self.mean(values_array[2])

						#rotation_x = self.madian(values_array[3])
						#rotation_y = self.median(values_array[4])
						#rotation_z = self.median(values_array[5])
						#rotation_w = self.median(values_array[6])

						print("Translation x: %.3f " % (translation_x))
						print("Translation y: %.3f " % (translation_y))
						print("Translation z: %.3f " % (translation_z))

						#print("Rotation x: %.3f " % (rotation_x))
						#print("Rotation y: %.3f " % (rotation_y))
						#print("Rotation z: %.3f " % (rotation_z))
						#print("Rotation w: %.3f " % (rotation_w))
					else:
						i = i + 1

					#print(values_array[0])

					rospy.loginfo(tfm)
	def mean(self, array):
		mean = array[0] + array[1] + array[2] + array[3]
		return mean/4

		


if __name__ == '__main__':
	rospy.init_node('denso_cube_tf_broadcaster')
	tfb = DynamicTFBroadcaster()
	rospy.spin()
