#!/usr/bin/env python 

import rospy
from control_msgs.msg import FollowJointTrajectoryFeedback
import matplotlib.pyplot as plt
from std_msgs.msg import Int8
# from IPython import embed


class TrajSubscriber: 

    def __init__(self): 
        print("Initializing the instance!") 
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

        # now simply display what 
        # you've received from the topic 
        # rospy.loginfo(rospy.get_caller_id() + "The velocities are %s", 
        #             FollowJointTrajectoryFeedback) 

        if self.executing.data == 1:
            rospy.loginfo("Saving trajectory data...")
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
            # print(f"self.seq: ", self.seq)
            # print(f"self.desired: ", self.desired)
            for i in range(7):
                fig0 = plt.figure(f"Joint {i+1}")
                plt.subplot(7, 2, 2*i+1)
                plt.plot(self.seq, self.desired[i], 'g', label="desired")
                plt.plot(self.seq, self.actual[i], 'b', label="actual")
                plt.legend()

                # fig1 = plt.figure(f"Error for joint {i+1}")
                plt.subplot(7, 2, 2*i+2)
                plt.plot(self.seq, self.error[i], 'r', label="error")
            plt.legend()
            plt.savefig("joint_trajectories.png")
            plt.show()

def main(): 
    # create a subscriber instance 
    sub = TrajSubscriber() 
    
    # follow it up with a no-brainer sequence check 
    print('Currently in the main function...') 
    
    # initializing the subscriber node 
    rospy.init_node('listener', anonymous=True) 
    rospy.spin() 

if __name__ == '__main__': 
    try: 
        main() 
    except rospy.ROSInterruptException: 
        pass
