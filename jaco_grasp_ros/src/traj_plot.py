#!/usr/bin/env python 

import rospy
from control_msgs.msg import FollowJointTrajectoryFeedback
import matplotlib.pyplot as plt
from std_msgs.msg import Int8
from datetime import datetime
# from IPython import embed


class TrajSubscriber: 

    def __init__(self): 
        self.traj_sub = rospy.Subscriber("/j2s7s300_driver/trajectory_controller/state", 
                                        FollowJointTrajectoryFeedback, self.traj_callback)
        
        self.fin_sub = rospy.Subscriber("/trajectory_finished", 
                                        Int8, self.fin_callback)
        
        # # 0 = before execution
        # # 1 = during execution
        # # 2 = after execution
        self.executing = Int8()
        self.executing.data = 0

        self.seq = []   # will be used as time axis for plot (seq parameter from msg)
        self.desired = [[] for i in range(7)]
        self.actual = [[] for i in range(7)]
        self.error = [[] for i in range(7)]

    def traj_callback(self, msg): 
        # print('Callback executed!')

        if self.executing.data == 1:
            # rospy.loginfo("Saving trajectory data...")
            self.seq.append(msg.header.seq)
            for i in range(7):
                self.desired[i].append(msg.desired.positions[i])
                self.actual[i].append(msg.actual.positions[i])
                self.error[i].append(msg.error.positions[i])


    def fin_callback(self, msg):
        # if execution is finished, 
        # for each of the 7 joints, plot desired vs actual position

        self.executing.data = msg.data

        # only want this after execution is done
        if msg.data == 2:
            rospy.loginfo("Plotting trajectory")
            fig = plt.figure("Joint Trajectories")
            for i in range(7):
                # fig0 = plt.figure(f"Joint {i+1}")
                plt.subplot(7, 2, 2*i+1)
                plt.plot(self.seq, self.desired[i], 'g')
                plt.plot(self.seq, self.actual[i], 'b')
                # plt.plot(self.seq, self.desired[i], 'g', label=f"desired j{i}")
                # plt.plot(self.seq, self.actual[i], 'b', label=f"actual j{i}")
                plt.legend()

                # fig1 = plt.figure(f"Error for joint {i+1}")
                plt.subplot(7, 2, 2*i+2)
                plt.plot(self.seq, self.error[i], 'r', label="error")
            plt.legend()

            file_name = "joint_trajectories " + str(datetime.now()) + ".png"
            plt.savefig(file_name)
            plt.show()

def main(): 
    # create a subscriber instance 
    sub = TrajSubscriber() 
    
    # initializing the subscriber node 
    rospy.init_node('listener', anonymous=True) 
    rospy.spin() 

if __name__ == '__main__': 
    try: 
        main() 
    except rospy.ROSInterruptException: 
        pass
