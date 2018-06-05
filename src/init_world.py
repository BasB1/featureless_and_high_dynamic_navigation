#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf
import sys

class InitWorld():
    def __init__(self):
        rospy.init_node('init_world', anonymous=True)
        self.GetInitPosUWB()
        while not rospy.is_shutdown():
            self.PublishInitPos()
        
    def GetInitPosUWB(self):
        listener = tf.TransformListener()
        while not rospy.is_shutdown():
            try:
                self.world_frame_id = rospy.get_param('/pozyx_node_trilateration/world_frame_id', 'world')
                self.tag_frame_id = rospy.get_param('/pozyx_node_trilateration/tag_frame_id', 'pozyx_tag')
                (self.trans, self.rot) = listener.lookupTransform(self.world_frame_id, self.tag_frame_id, rospy.Time(0))
                rospy.loginfo("Found initial position")
                break
            except: 
                rospy.logerr(sys.exc_info()[1])
                rospy.logerr("Could not initialize robot position to world frame... trying every second.")
                rospy.sleep(1)
                
    def PublishInitPos(self):
        br = tf.TransformBroadcaster()
        rospy.loginfo("Setting initial position to world frame")
        while not rospy.is_shutdown():
            try:
                br.sendTransform((self.trans[0], self.trans[1], 0),
                     tf.transformations.quaternion_from_euler(0, 0, 0),
                     rospy.Time.now(),
                     "init_pos",
                     "world")
                br.sendTransform((0, 0, 0),
                     tf.transformations.quaternion_from_euler(0, 0, 0),
                     rospy.Time.now(),
                     "odom",
                     "init_pos")
            except:
                rospy.logerr(sys.exc_info()[1])

if __name__ == '__main__':
    init = InitWorld()
