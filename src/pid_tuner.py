#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
import matplotlib.pyplot as plt

class PIDTuner:
    def __init__(self):
        rospy.init_node("pid_tuner", anonymous=True)
        # Setup subscribers
        _ = rospy.Subscriber("/pid_err", Float64, self.pid_err_callback)
        self.errs = []

    def pid_err_callback(self, msg):
        self.errs.append(msg.data)
        plt.plot(self.errs, 'r')
        plt.pause(0.05)
        plt.draw()

if __name__ == "__main__":
    try:
        pid_tuner = PIDTuner()
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
