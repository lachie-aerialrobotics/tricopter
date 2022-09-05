#!/usr/bin/env python
import rospy
import tf2_ros
from tricopter.srv import AlignTracking

class ViconMonitor:
    def __init__(self):
        self.tfBuffer = tf2_ros.Buffer()
        tfListener = tf2_ros.TransformListener(self.tfBuffer)

    def align_tracking(self, req):
        vicon_tf = self.tfBuffer.lookup_transform('map', 'hexacopter', time=rospy.Time.now(), timeout=rospy.Duration(5))
        vision_tf = self.tfBuffer.lookup_transform('map', 'base_link', time=rospy.Time.now(), timeout=rospy.Duration(5))
        rospy.loginfo("Vicon recentered!")
        response = AlignTracking()
        return response

def main():
    rospy.init_node('vicon_tf_server')
    #init tf listener
    
    monitor = ViconMonitor()
    align_tracking = rospy.Service('align_tracking', AlignTracking, monitor.align_tracking)

    rospy.spin()

if __name__ == '__main__':
    main()