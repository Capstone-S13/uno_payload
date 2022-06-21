#!/usr/bin/env python3

from payload_interfaces.msg import PayLoadMode
from payload_interfaces.msg import PayLoadActionAction
from payload_interfaces.msg import PayLoadActionGoal
from payload_interfaces.msg import PayLoadActionResult
from payload_interfaces.msg import PayLoadActionFeedback

import rospy
import actionlib
from std_msgs.msg import UInt32

class PayloadActionServer():

    #_feedback = PayLoadAction.msg.

    def __init__(self):
        #super().__init__('payload_service_node')
        self.payload_init()
        self.cmd_pub = rospy.Publisher("/uno_payload/cmd", UInt32, queue_size=10)

        self.action_name = 'payload_action'
        self._as = actionlib.SimpleActionServer(
            self.action_name,
            PayLoadActionAction,
            execute_cb=self.execute_callback,
            auto_start = False
        )
        self._as.start()


    def execute_callback(self, goal_handle):
        pusher_action_string = ""
        
        # pusher action
        if (goal_handle.pusher_action.operation == PayLoadMode.PUSHER_OUT):
            pusher_action_string = "Pusher Out"
        elif (goal_handle.pusher_action.operation == PayLoadMode.PUSHER_IN):
            pusher_action_string = "Pusher In"
        elif (goal_handle.pusher_action.operation == PayLoadMode.PUSHER_MOVING):
            pusher_action_string = "Pusher Moving"
        elif (goal_handle.pusher_action.operation == PayLoadMode.PUSHER_IDLE):
            pusher_action_string = "Pusher Idling"
        elif (goal_handle.pusher_action.operation == PayLoadMode.ERROR):
            pusher_action_string = "What?! Why do send it to perform an error?!"


        rospy.loginfo(f"Executing action: \
            Pusher Action:{pusher_action_string}")

        pusher_cmd = UInt32()
        pusher_cmd.data = goal_handle.pusher_action.operation

        self._feedback = PayLoadActionFeedback()

        # Payload Command
        rospy.loginfo(f"attempting to set {pusher_action_string}")
        try:

            self.cmd_pub.publish(pusher_cmd)
            try:
                status = rospy.wait_for_message("/uno_payload/status", UInt32, timeout=10.0)
            except rospy.ROSException as e:
                rospy.logerr("Payload status not updated!")
                rospy.logerr(e)

            if status.data == goal_handle.pusher_action.operation:
                self._feedback.pusher_state.operation =\
                    goal_handle.pusher_action.operation
            else:
                self._feedback.pusher_state.operation = PayLoadMode.ERROR
        except Exception as e:
            rospy.loginfo(f"error: {e}")
            rospy.loginfo(f"failed to complete action: \
                {pusher_action_string}")
            self._feedback.pusher_state.operation = PayLoadMode.ERROR

        self._feedback.success = False
        self._as.publish_feedback(self._feedback)

        self._result = PayLoadActionResult()
        self._result.pusher_state = self._feedback.pusher_state

        success = False
        if(self._feedback.pusher_state.operation == goal_handle.pusher_action.operation):
            self._result.success = True
            
            rospy.loginfo('%s: Succeeded' % self.action_name)
            self._as.set_succeeded(self._result)

    def payload_init(self):
        rospy.loginfo('action server is up!')

def main(args=None):
    rospy.init_node('payload_service_node')
    server = PayloadActionServer()
    rospy.spin()

if __name__ == '__main__':
    main()