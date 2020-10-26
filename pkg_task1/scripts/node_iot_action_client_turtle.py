#!/usr/bin/env python

# ROS Node - Action Client - IoT ROS Bridge

import rospy
import actionlib

from pkg_ros_iot_bridge.msg import msgRosIotAction      # Message Class that is used by ROS Actions internally
from pkg_ros_iot_bridge.msg import msgRosIotGoal        # Message Class that is used for Goal Messages
from pkg_ros_iot_bridge.msg import msgRosIotResult      # Message Class that is used for Result Messages
from pkg_task1.msg import msgTurtleAction       # Message Class that is used by ROS Actions internally
from pkg_task1.msg import msgTurtleGoal         # Message Class that is used for Goal messages
from pkg_task1.msg import msgTurtleResult

class RosIotBridgeActionClient:

    # Constructor
    def __init__(self):

        # Initialize Action Client
        self._ac = actionlib.ActionClient('/action_ros_iot',
                                          msgRosIotAction)
        
        # Dictionary to Store all the goal handels
        self._goal_handles = {}

        # Store the MQTT Topic on which to Publish in a variable
        param_config_iot = rospy.get_param('config_pyiot')
        self._config_mqtt_pub_topic = param_config_iot['mqtt']['topic_pub']

        # Wait for Action Server that will use the action - '/action_iot_ros' to start
        self._ac.wait_for_server()
        rospy.loginfo("Action server up, we can send goals.")

    
    # This function will be called when there is a change of state in the Action Client State Machine
    def on_transition(self, goal_handle):
        
        # from on_goal() to on_transition(). goal_handle generated by send_goal() is used here.
        
        result = msgRosIotResult()

        index = 0
        for i in self._goal_handles:
            if self._goal_handles[i] == goal_handle:
                index = i
                break

        rospy.loginfo("Transition Callback. Client Goal Handle #: " + str(index))
        rospy.loginfo("Comm. State: " + str(goal_handle.get_comm_state()) )
        rospy.loginfo("Goal Status: " + str(goal_handle.get_goal_status()) )
        
        # Comm State - Monitors the State Machine of the Client which is different from Server's
        # Comm State = 2 -> Active
        # Comm State = 3 -> Wating for Result
        # Comm State = 7 -> Done
        
        # if (Comm State == ACTIVE)
        if goal_handle.get_comm_state() == 2:
            rospy.loginfo(str(index) + ": Goal just went active.")
        
        # if (Comm State == DONE)
        if goal_handle.get_comm_state() == 7:
            rospy.loginfo(str(index) + ": Goal is DONE")
            rospy.loginfo(goal_handle.get_terminal_state())
            
            # get_result() gets the result produced by the Action Server
            result = goal_handle.get_result()
            rospy.loginfo(result.flag_success)

            if (result.flag_success == True):
                rospy.loginfo("Goal successfully completed. Client Goal Handle #: " + str(index))
            else:
                rospy.loginfo("Goal failed. Client Goal Handle #: " + str(index))


    # This function is used to send Goals to Action Server
    def send_goal(self, arg_protocol, arg_mode, arg_topic, arg_message):
        # Create a Goal Message object
        goal = msgRosIotGoal()

        goal.protocol = arg_protocol
        goal.mode = arg_mode
        goal.topic = arg_topic
        goal.message = arg_message

        rospy.loginfo("Send goal.")
        
        # self.on_transition - It is a function pointer to a function which will be called when 
        #                       there is a change of state in the Action Client State Machine
        goal_handle = self._ac.send_goal(goal,
                                         self.on_transition,
                                         None)

        return goal_handle


class SimpleActionClientTurtle:

    # Constructor
    def __init__(self):
        self._ac = actionlib.SimpleActionClient('/action_turtle',
                                                msgTurtleAction)
        self._ac.wait_for_server()
        rospy.loginfo("Action server is up, we can send new goals!")

        param_config_iot = rospy.get_param('config_pyiot')
        self._config_mqtt_server_url = param_config_iot['mqtt']['server_url']
        self._config_mqtt_server_port = param_config_iot['mqtt']['server_port']
        self._config_mqtt_sub_topic = param_config_iot['mqtt']['topic_sub']
        self._config_mqtt_pub_topic = param_config_iot['mqtt']['topic_pub']
        self._config_mqtt_qos = param_config_iot['mqtt']['qos']
        self._config_mqtt_sub_cb_ros_topic = param_config_iot['mqtt']['sub_cb_ros_topic']
        self._config_google_apps_spread_sheet_id = param_config_iot['google_apps']['spread_sheet_id']

        self.action_client = RosIotBridgeActionClient()

    # Function to send Goals to Action Servers
    def send_goal(self, arg_dis, arg_angle):
        
        # Create Goal message for Simple Action Server
        goal = msgTurtleGoal(distance=arg_dis, angle=arg_angle)
        
        '''
            * done_cb is set to the function pointer of the function which should be called once 
                the Goal is processed by the Simple Action Server.

            * feedback_cb is set to the function pointer of the function which should be called while
                the goal is being processed by the Simple Action Server.
        ''' 
        self._ac.send_goal(goal, done_cb=self.done_callback,
                           feedback_cb=self.feedback_callback)        
        rospy.loginfo("Goal has been sent.")
    
    # Function print result on Goal completion
    def done_callback(self, status, result):
        message = "(" + str(result.final_x) + "," + str(result.final_y) + "," + str(result.final_theta) + ")"
        rospy.loginfo("Status is : " + str(status))
        rospy.loginfo("Result is : " + str(result))
        goal_handle1 = self.action_client.send_goal("mqtt", "pub", self.action_client._config_mqtt_pub_topic, message)
        self.action_client._goal_handles['1'] = goal_handle1

    # Function to print feedback while Goal is being processed
    def feedback_callback(self, feedback):
        rospy.loginfo(feedback)



# Main
def main():
    rospy.init_node('node_ros_iot_bridge_action_client')

    action_client = RosIotBridgeActionClient()

    goal_handle1 = action_client.send_goal("mqtt", "pub", action_client._config_mqtt_pub_topic, "Hello from Action Client!")
    action_client._goal_handles['1'] = goal_handle1
    rospy.loginfo("Goal #1 Sent")

    goal_handle2 = action_client.send_goal("mqtt", "sub", "eyrc/vb/mqtt/myTopic", "NA")
    action_client._goal_handles['2'] = goal_handle2
    rospy.loginfo("Goal #2 Sent")


    for i in range(7):
        if i == 0:
            obj_client = SimpleActionClientTurtle()
            obj_client.send_goal(2, 0)
            rospy.sleep(5)
        else:
            obj_client = SimpleActionClientTurtle()
            obj_client.send_goal(2, 60)
            rospy.sleep(5)
    rospy.spin()

'''
    # 2. Create a object for Simple Action Client.
    obj_client = SimpleActionClientTurtle()

    # 3. Send Goals to Draw a Square
    obj_client.send_goal(2, 0)
    rospy.sleep(5)
    
    obj_client.send_goal(2, 60)
    #goal_reached(pub_client, param_config_iot)
    rospy.sleep(5)

    obj_client.send_goal(2, 60)
    #goal_reached(pub_client, param_config_iot)
    rospy.sleep(5)

    obj_client.send_goal(2, 60)
    #goal_reached(pub_client, param_config_iot)
    rospy.sleep(5)
    
    obj_client.send_goal(2, 60)
    #goal_reached(pub_client, param_config_iot)
    rospy.sleep(5)

    obj_client.send_goal(2, 60)
    #goal_reached(pub_client, param_config_iot)
    rospy.sleep(5)
    
    obj_client.send_goal(2, 60)
    #goal_reached(pub_client, param_config_iot)

    # rospy.sleep(1.0)
    # goal_handle1.cancel()'''


if __name__ == '__main__':
    main()
