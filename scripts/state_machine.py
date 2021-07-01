#!/usr/bin/env python

import rospy
import smach
import smach_ros

from std_msgs.msg import String, Float32MultiArray
from abb_robot_msgs.srv import SetIOSignal, SetIOSignalRequest, TriggerWithResultCode
from perception.msg import PerceptionData
from geometry_msgs.msg import PoseArray

prev_state = 'P1'

screws_list = []
back_cover_cut_path_list = []
front_cover_cut_path_list = []
motherboard_list = []
keyboard_list = []
fan_list = []
cpu_list = []
ports_cut_path_list = []
screws_cut_path_list = []
cd_rom_list = []
hard_disk_list = []
flipping_points_list = []


# define state Capturing_image
next_state = {'P1': {'screws detected': 'P2'},
              'P2': {'cutting done': 'P3'},
              'P3': {'flipping done': 'P4'},
              'P4': {'cover only': 'P5'},
              'P4': {'keyboard and cover': 'P8'},
              'P5': {'cutting done': 'P6'},
              'P6': {'screws detected': 'P7'},
              'P6': {'no screws': 'P9'},
              'P7': {'cutting done': 'P9'},
              'P8': {'cutting done': 'P11'},
              'P9': {'pick done': 'P12'},
              'P9': {'pick failed': 'P10'},
              'P10': {'flipping done': 'P1'},
              'P11': {'pick failed': 'P10'},
              'P12': {'screws detected': 'P13'},
              'P13': {'screws success': 'P15'},
              'P13': {'screws failed': 'P14'},
              'P14': {'cutting done': 'P15'},
              'P15': {'pick done': 'P16'},
              'P16': {'pick done': 'P17'},
              'P17': {'pick done': 'P18'},
              'P18': {'pick done': 'P21'},
              'P18': {'pick failed': 'P19'},
              'P19': {'cutting done': 'P20'},
              'P20': {'pick failed': 'P22'},
              'P20': {'pick done': 'P21'},
              'P21': {'reset': 'P1'},
              'P22': {'continue': 'P1'}}


passed_argument = {'P2':screws_cut_path_list,
                   'P3':flipping_points_list,
                   'P5':front_cover_cut_path_list,
                   'P7':screws_cut_path_list,
                   'P8':keyboard_list,
                   'P9':front_cover_cut_path_list, #TODO change it to center of cover for picking
                   'P10':flipping_points_list,
                   'P11':keyboard_list, #TODO change it to center of keyboard for picking
                   'P13':screws_list,
                   'P14':screws_cut_path_list,
                   'P15':hard_disk_list,
                   'P16':fan_list,
                   'P17':cd_rom_list,
                   'P18': motherboard_list,
                   'P19': ports_cut_path_list,
                   'P20': motherboard_list} #TODO account for P18

class Capturing_image(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Capture Done', 'Camera Disconnected'], input_keys=[
        ], output_keys=['captured_objects'])

        self.capture_publisher = rospy.Publisher(
            "/capture_state", String, queue_size=0)
        

    def execute(self, userdata):
        global screws_list
        global back_cover_cut_path_list
        global front_cover_cut_path_list
        global motherboard_list
        global keyboard_list
        global fan_list
        global cpu_list
        global ports_cut_path_list
        global screws_cut_path_list
        global cd_rom_list
        global hard_disk_list
        global flipping_points_list

        rospy.sleep(2)
        capture_msg = String()
        capture_msg.data = "capture"
        self.capture_publisher.publish(capture_msg)
        received_msg = rospy.wait_for_message("/found_components", String)
        if received_msg.data == "Camera Disconnected":
            return 'Camera Disconnected'
        else:
            components_msg  = rospy.wait_for_message("/perception_data", PerceptionData)
            
            screws_list = components_msg.screws.data
            back_cover_cut_path_list = components_msg.back_cover_cut_path.data
            front_cover_cut_path_list = components_msg.front_cover_cut_path.data
            motherboard_list = components_msg.motherboard.data
            keyboard_list = components_msg.keyboard.data
            fan_list = components_msg.fan.data
            ports_cut_path_list = components_msg.ports_cut_path
            screws_cut_path_list = components_msg.screws_cut_path
            cd_rom_list = components_msg.cd_rom.data
            hard_disk_list = components_msg.hard_disk.data
            flipping_points_list = components_msg.flipping_points.data
            
            
            if len(screws_list) > 0:
                userdata.captured_objects = 'screws detected'
            elif len(front_cover_cut_path_list) > 0:
                if len(keyboard_list) > 0:
                    userdata.captured_objects = 'keyboard and cover'
                else:
                    userdata.captured_objects = 'cover only'
            
            return 'Capture Done'


# define state Processing
class Processing(smach.State):
    def __init__(self):
        smach.State.__init__(
            self, outcomes=['Cover', 'Screws', 'Loose Components', 'Need Data','Flip','Process Termination'],
            input_keys=['captured_objects'], output_keys = ['operation_parts'])

        self.operation_publisher = rospy.Publisher(
            "/operation", String, queue_size=1)

        self.capture_list = list(['P1', 'P4', 'P6', 'P12'])
        self.cut_list = list(['P2', 'P5', 'P7', 'P8', 'P14', 'P19'])
        self.screw_list = list(['P13'])
        self.grip_list = list(['P9', 'P11', 'P15', 'P16', 'P17', 'P18', 'P20'])
        self.flip_list = list(['P3', 'P10'])
        self.wait_list = list(['P21', 'P22'])

    def execute(self, userdata):
        global prev_state
        # rospy.sleep(5)
        # operation_msg = String()
        
        # if not(userdata.captured_objects == 'cutting done' 
        # or userdata.captured_objects == 'screws success'
        # or userdata.captured_objects == 'screws failed'
        # or userdata.captured_objects == 'pick done'
        # or userdata.captured_objects == 'pick failed'
        # or userdata.captured_objects == 'flipping done'):
            # if len(screws_list) > 0:
            #     userdata.captured_objects = 'screws detected'
            # elif len(front_cover_cut_path_list) > 0:
            #     if len(keyboard_list) > 0:
            #         userdata.captured_objects = 'keyboard and cover'
            #     else:
            #         userdata.captured_objects = 'cover only'
                    
                
        
        # if userdata.captured_objects == "cover":
        #     operation_msg.data = "Cutting"
        #     self.operation_publisher.publish(operation_msg)
        #     return 'Cover'

        # elif userdata.captured_objects == "screws":
        #     operation_msg.data = "Screw Loosening"
        #     self.operation_publisher.publish(operation_msg)
        #     return 'Screws'

        # elif userdata.captured_objects == "loose components":
        #     operation_msg.data = "Gripping"
        #     self.operation_publisher.publish(operation_msg)
        #     return 'Loose Components'
        # elif userdata.captured_objects is None:
        #     return 'Need Data'
        next_operation = next_state[prev_state][userdata.captured_objects]
        if next_operation in passed_argument.keys():
            userdata.operation_parts = passed_argument[next_operation]
        passed_argument[next_operation] = []
        if next_operation in self.capture_list:
            prev_state = next_operation
            return 'Need Data'
        elif next_operation in self.cut_list:
            prev_state = next_operation
            return 'Cover'
        elif next_operation in self.grip_list:
            prev_state = next_operation
            return 'Loose Components'
        elif next_operation in self.screw_list:
            prev_state = next_operation
            return 'Screws'
        elif next_operation in self.flip_list:
            prev_state = next_operation
            return 'Flip'
        elif next_operation in self.wait_list:
            prev_state = next_operation
            return 'Process Termination'


# define state Cutting
class Cutting(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=[
                             'Cutting Done', 'Collision'],
                             input_keys = ['operation_parts'],output_keys=['prev_state','captured_objects'])
        self.cutting_px_pub = rospy.Publisher("/cutting_path", Float32MultiArray, queue_size=1)
        self.cutting_xyz_pub = rospy.Publisher("/cut_xyz", PoseArray, queue_size=1)
        

    def execute(self, userdata):
        rospy.sleep(2)
        
        cutting_px_msg = Float32MultiArray()
        cutting_px_msg.data = userdata.operation_parts
        self.cutting_px_pub.publish(cutting_px_msg)
        
        received_xyz = rospy.wait_for_message('/px_to_xyz', PoseArray)
        self.cutting_xyz_pub.publish(received_xyz)
        
        received_msg = rospy.wait_for_message('/done', String)
        if received_msg.data == "Done":
            userdata.captured_objects = 'cutting done'
            return 'Cutting Done'
        elif received_msg.data == "cutting failed":
            userdata.prev_state = 'cutting'
            return 'Collision'

# define state Screw_loosening


class Screw_loosening(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=[
                             'Screw Loosening Done', 'Collision'],
                             input_keys = ['operation_parts'], output_keys=['prev_state','captured_objects'])
        self.screw_px_pub = rospy.Publisher("/cutting_path", Float32MultiArray, queue_size=1)
        self.screw_xyz_pub = rospy.Publisher("/screw_xyz", PoseArray, queue_size=1)

    def execute(self, userdata):
        rospy.sleep(2)
        
        screw_px_msg = Float32MultiArray()
        screw_px_msg.data = userdata.operation_parts
        self.screw_px_pub.publish(screw_px_msg)
        
        received_xyz = rospy.wait_for_message('/px_to_xyz', PoseArray)
        self.screw_xyz_pub.publish(received_xyz)
        
        received_msg = rospy.wait_for_message('/done', String)
        if received_msg.data == "Done":
            userdata.captured_objects = 'screws success'
            return 'Screw Loosening Done'
        elif received_msg.data == "screw failed":
            userdata.prev_state = 'screw'
            return 'Collision'

# define state Gripping


class Gripping(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=[
                             'Gripping Done', 'Gripping Failed'],
                             input_keys = ['operation_parts'], output_keys=['captured_objects'])
        self.grip_px_pub = rospy.Publisher("/cutting_path", Float32MultiArray, queue_size=1)
        self.grip_xyz_pub = rospy.Publisher("/grip_xyz", PoseArray, queue_size=1)

    def execute(self, userdata):
        rospy.sleep(2)
        
        grip_px_msg = Float32MultiArray()
        grip_px_msg.data = userdata.operation_parts
        self.grip_px_pub.publish(grip_px_msg)
        
        received_xyz = rospy.wait_for_message('/px_to_xyz', PoseArray)
        self.grip_xyz_pub.publish(received_xyz)
        
        received_msg = rospy.wait_for_message('/done', String)
        if received_msg.data == "Gripping Done":
            userdata.captured_objects = 'pick done'
            return 'Gripping Done'
        else:
            return 'Gripping Failed'

#define state flipping
        
class Flipping(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=[
                             'Flipping Done'],
                             input_keys=['operation_parts'], output_keys=['captured_objects'])
        
        self.flip_px_pub = rospy.Publisher("/cutting_path", Float32MultiArray, queue_size=1)
        self.flip_xyz_pub = rospy.Publisher("/flip_xyz", PoseArray, queue_size=1)

    def execute(self, userdata):
        rospy.sleep(2)
        
        flip_px_msg = Float32MultiArray()
        flip_px_msg.data = userdata.operation_parts
        self.flip_px_pub.publish(flip_px_msg)
        
        received_xyz = rospy.wait_for_message('/px_to_xyz', PoseArray)
        self.flip_xyz_pub.publish(received_xyz)
        
        userdata.captured_objects = 'flipping done'
        return 'Flipping Done'

# define state Connection Error


class Connection_Error(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Error Solved'])
        self.connection_error_publisher = rospy.Publisher(
            "/connection_error_handled", String, queue_size=0)

    def execute(self, userdata):
        rospy.sleep(2)
        print("Please Connect The Camera Then Press Any Button")
        camera_connected = String()
        camera_connected.data = raw_input()
        self.connection_error_publisher.publish(camera_connected)
        return 'Error Solved'
    
# define state Process Failed


class Wait_For_Operator(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Reset/Continue'])

    def execute(self, userdata):
        print("Replace Laptop the Press any Key to Reset/Continue")
        button_prssed = raw_input()
        return 'Reset/Continue'

# define state Gripping Failed


class Gripper_Failed(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Error Solved'])

    def execute(self, userdata):
        rospy.sleep(1)
        return 'Error Solved'

# define state Collision Error


class Collision_Error(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=[
                             'prev_state == Cutting', 'prev_state == Screw Loosening'], input_keys=['prev_state'])
        self.cutting_tool = rospy.ServiceProxy(
            "/rws/set_io_signal", SetIOSignal)
        self.motors_on = rospy.ServiceProxy(
            "/rws/set_motors_on", TriggerWithResultCode)

    def change_tool_status(self, status=0):
        self.cutting_tool.wait_for_service()
        cutting_tool_status = SetIOSignalRequest()
        cutting_tool_status.signal = "CuttingTool"
        cutting_tool_status.value = str(status)
        response = self.cutting_tool(cutting_tool_status)
        return response

    def execute(self, userdata):
        self.change_tool_status(status=0)
        rospy.sleep(6)
        self.change_tool_status(status=1)
        self.motors_on.wait_for_service()
        response = self.motors_on()

        if userdata.prev_state == 'cutting':
            return 'prev_state == Cutting'
        elif userdata.prev_state == 'screw':
            return 'prev_state == Screw Loosening'


def main():
    rospy.init_node('system_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=[])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('Capturing Image', Capturing_image(),
                               transitions={'Capture Done': 'Processing', 'Camera Disconnected': 'Connection Error'})
        smach.StateMachine.add('Processing', Processing(),
                               transitions={'Cover': 'Cutting', 'Screws': 'Screw Loosening', 'Loose Components': 'Gripping', 'Need Data': 'Capturing Image', 'Flip':'Flipping','Process Termination':'Wait For Operator'})
        smach.StateMachine.add('Cutting', Cutting(),
                               transitions={'Cutting Done': 'Processing', 'Collision': 'Collision Error'})
        smach.StateMachine.add('Screw Loosening', Screw_loosening(),
                               transitions={'Screw Loosening Done': 'Processing','Collision': 'Collision Error'})
        smach.StateMachine.add('Gripping', Gripping(),
                               transitions={'Gripping Done': 'Processing', 'Gripping Failed': 'Gripping Error'})
        smach.StateMachine.add('Flipping', Flipping(),
                               transitions={'Flipping Done': 'Processing'})
        smach.StateMachine.add('Connection Error', Connection_Error(),
                               transitions={'Error Solved': 'Capturing Image'})
        smach.StateMachine.add('Wait For Operator', Wait_For_Operator(),
                               transitions={'Reset/Continue': 'Processing'})
        smach.StateMachine.add('Gripping Error', Gripper_Failed(),
                               transitions={'Error Solved': 'Processing'})
        smach.StateMachine.add('Collision Error', Collision_Error(),
                               transitions={'prev_state == Cutting': 'Cutting', 'prev_state == Screw Loosening': 'Screw Loosening'})

    # Execute SMACH plan
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()
    outcome = sm.execute()
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
