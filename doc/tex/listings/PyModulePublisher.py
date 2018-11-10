def publishTopic(self, topic, message_type, message):
    rate = rospy.Rate(1)
    exec('pub = rospy.Publisher(\''+topic+'\', '+message_type+', queue_size=10)')
    rate.sleep()
    pub.publish(message)
    rate.sleep()