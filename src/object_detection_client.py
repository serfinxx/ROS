#! /usr/bin/env python

import rospy
import actionlib


class obstacle_detectoin_client:

    def feedback_callback(self, feedback_data):
        self.distance_travelled = feedback_data.current_distance_travelled
        print('FEEDBACK: current distance travelled: {:.2f} m'.format(
            self.distance_travelled))

    def __init__(self):
        self.action_complete = False
        self.distance_travelled = 0.0

        rospy.init_node('od_client')

        self.rate = rospy.Rate(10)

        self.goal = SearchGoal()

        self.client = actionlib.SimpleActionClient('/od_server',
                                                   SearchAction)
        self.client.wait_for_server()

        rospy.on_shutdown(self.shutdown_ops)

    def shutdown_ops(self):
        if not self.action_complete:
            rospy.logwarn("Received a shutdown request. Cancelling Goal...")
            self.client.cancel_goal()
            rospy.logwarn("Goal Cancelled")

    def print_result(self):

        results = self.client.get_result()
        print("RESULT: ".format())

    def send_goal(self, velocity, distance):
        self.goal.fwd_velocity = velocity
        self.goal.approach_distance = distance

        # send the goal to the action server:
        self.client.send_goal(self.goal, feedback_cb=self.feedback_callback)

    def main(self):
        self.send_goal(velocity=0.1, distance=0.1)

        while self.client.get_state() < 2:
            print("STATE: Current state code is {}".format(
                self.client.get_state()))

            self.rate.sleep()

        self.action_complete = True
        print("RESULT: Action State = {}".format(self.client.get_state()))
        self.print_result()


if __name__ == '__main__':
    odc_object = obstacle_detectoin_client()
    try:
        odc_object.main()
    except rospy.ROSInterruptException:
        pass
