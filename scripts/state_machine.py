#!/usr/bin/env python

import rospy
import smach
import smach_ros

from std_msgs.msg import String, Float32MultiArray
from abb_robot_msgs.srv import SetIOSignal, SetIOSignalRequest, TriggerWithResultCode
from cut_contour.robot_helpers import MotionServices
from perception.msg import PerceptionData
from geometry_msgs.msg import PoseArray, Pose
from copy import deepcopy


prev_state = 'P4'
# prev_state = 'P4'

gripping_tool  = '' #'suction cup' or 'gripper'


components_xyz = {'screws_list': [],
                   'back_cover_cut_path_list': [],
                   'front_cover_cut_path_list': [],
                   'front_cover_list': [],
                   'motherboard_list': [],
                   'keyboard_list': [],
                   'keyboard_cut_path_list': [],
                   'fan_screws_list': [],
                   'fan_list': [],
                   'cpu_list': [],
                   'ports_cut_path_list': [],
                   'screws_cut_path_list': [],
                   'cd_rom_list': [],
                   'cd_rom_cut_path_list': [],
                   'hard_disk_list': [],
                   'flipping_points_list': [],
                   'screws_near_cpu_cut_path_list': [],
                   'screws_near_cpu_list': []}


# define state Capturing_image
next_state = {'P1': {#'screws detected': 'P2'},
                     #'screws detected': 'P3'}, #TODO: remove this line and uncomment above
                     #'fan screws detected': 'P16'},#TODO: remove this line and uncomment above
                     'screws detected': 'P13'}, #TODO: remove this line and uncomment above
              'P2': {'cutting done': 'P3',
                     'screws success': 'P3'}, 
              
              'P3': {'flipping done': 'P4'},
              
              'P4': {'cover only': 'P5',
                     #'keyboard and cover': 'P8'},
                     'keyboard and cover': 'P11'},
                     #'cdrom': 'P17'}, 
              
              'P5': {'cutting done': 'P6'},
              
              'P6': {'screws detected': 'P7',
                     'no screws': 'P9'},
              
              'P7': {'cutting done': 'P9'},
              'P8': {'cutting done': 'P11'},
              
              'P9': {'pick done': 'P12',
                     'pick failed': 'P10'},
              
              'P10': {'flipping done': 'P1'},
              'P11': {'pick failed': 'P10',
                      'pick done': 'P6'}, # TODO: Tell Joe there wasn't a pick done key here???!!!
              
              'P12': {'screws detected': 'P13',
                      'cpu screws': 'P24'},
              
              'P13': {#'screws success': 'P15',
                      #'screws failed': 'P14'},
                      'screws success': 'P19',#TODO: remove this line and uncomment above
                      'screws failed': 'P19'},#TODO: remove this line and uncomment above
              
              'P14': {'cutting done': 'P15'},
              'P15': {'pick done': 'P15.5'},
              'P15.5': {'screws success': 'P16'},
              'P16': {'pick done': 'P23'},
              'P17': {'pick done': 'P18'},
              
              'P18': {'pick done': 'P21',
                      'pick failed': 'P19'},
              
              'P19': {'cutting done': 'P20'},
              
              'P20': {'pick failed': 'P22',
                      'pick done': 'P21'},
              
              'P21': {'reset': 'P1'},
              'P22': {'continue': 'P1'},
              'P23': {'cutting done':'P17'},
              'P24': {'flipping done': 'P25'},
              'P25': {'cpu screws': 'P26'},
              'P26': {'cutting done': 'P27'},
              'P27': {'flipping done': 'P28'},
              'P28': {'pick done': 'P29'},
              'P29': {'screws detected': 'P13'}}

sorting_boxes = {'P9': 'plastic box',
                 'P11': 'plastic box',
                 'P15': 'hard disk box',
                 'P16': 'fan box',
                 'P17': 'CD ROM box',
                 'P18': 'motherboard box',
                 'P20': 'motherboard box'}


# passed_argument = {#'P2':screws_cut_path_list,
#                    'P2':screws_list,
#                    'P3':flipping_points_list,
#                    'P5':front_cover_cut_path_list,
#                    'P7':screws_cut_path_list,
#                    'P8':keyboard_list,
#                    'P9':front_cover_cut_path_list, #TODO change it to center of cover for picking
#                    'P10':flipping_points_list,
#                    'P11':keyboard_list, #TODO change it to center of keyboard for picking
#                    'P13':screws_list,
#                    'P14':screws_cut_path_list,
#                    'P15':hard_disk_list,
#                    'P16':fan_list,
#                    'P17':cd_rom_list,
#                    'P18': motherboard_list,
#                    'P19': ports_cut_path_list,
#                    'P20': motherboard_list} #TODO account for P18

def xyz_list_to_pose_array(xyz_list):
    pose_array = PoseArray()
    for i in range(0, len(xyz_list), 3):  # x, y, z for each point
        pose = Pose()
        pose.position.x = xyz_list[i]
        pose.position.y = xyz_list[i + 1]
        pose.position.z = xyz_list[i + 2]
        pose.orientation.x = 0
        pose.orientation.y = 0
        pose.orientation.z = 0
        pose.orientation.w = 1
        pose_array.poses.append(pose)
    return pose_array

class Camera():
    def __init__(self):
        self.camera_ms = MotionServices('camera')
    
    def move_camera_to_capture(self):
        self.camera_ms.move_group.set_pose_reference_frame('base_link')
        self.camera_ms.move_group.set_named_target('clamping_mechanism_horizontal_pose')
        self.camera_ms.move_group.set_planning_time(0.1)
        self.camera_ms.move_group.set_num_planning_attempts(5)
        self.camera_ms.move_group.go()
        rospy.sleep(1)
    

class Capturing_image(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Capture Done', 'Camera Disconnected'], input_keys=[
        ], output_keys=['captured_objects'])

        self.capture_publisher = rospy.Publisher(
            "/capture_state", String, queue_size=0)
        
        self.camera_obj = Camera()
        

    def execute(self, userdata):
        global components_xyz

        rospy.sleep(2)
        
        self.camera_obj.move_camera_to_capture()
        self.camera_obj.camera_ms.change_tool_status('YELLOW_LED', status=1)
        self.camera_obj.camera_ms.change_tool_status('RED_LED', status=0)
        self.camera_obj.camera_ms.change_tool_status('GREEN_LED', status=0)
        
        self.camera_obj.camera_ms.change_tool_status('Clamp_Off', status = 0)
        rospy.sleep(1)
        self.camera_obj.camera_ms.change_tool_status('Clamp_On', status = 1)
        rospy.sleep(1)
        capture_msg = String()

        if prev_state == 'P25':
            capture_msg.data = "capture cpu screws"
        else:
            capture_msg.data = "capture"
            
        self.capture_publisher.publish(capture_msg)
        received_msg = rospy.wait_for_message("/found_components", String)
        if received_msg.data == "Camera Disconnected":
            return 'Camera Disconnected'
        else:
            components_msg  = rospy.wait_for_message("/perception_data", PerceptionData)
            
            
            components_xyz['screws_list'] =  components_msg.screws.data
            components_xyz['back_cover_cut_path_list'] =  components_msg.back_cover_cut_path.data
            components_xyz['front_cover_cut_path_list'] =  components_msg.front_cover_cut_path.data
            print("front_cover_cut_path_list = ", components_xyz['front_cover_cut_path_list'])
            components_xyz['front_cover_list'] =  components_msg.front_cover.data
            components_xyz['motherboard_list'] =  components_msg.motherboard.data
            components_xyz['keyboard_list'] =  components_msg.keyboard.data
            components_xyz['keyboard_cut_path_list'] =  components_msg.keyboard_cut_path.data
            components_xyz['fan_list'] =  components_msg.fan.data
            components_xyz['fan_screws_list'] =  components_msg.fan_screws.data
            components_xyz['ports_cut_path_list'] =  components_msg.ports_cut_path
            components_xyz['screws_cut_path_list'] =  components_msg.screws_cut_path
            components_xyz['cd_rom_list'] =  components_msg.cd_rom.data
            components_xyz['cd_rom_cut_path_list'] =  components_msg.cd_rom_cut_path
            components_xyz['hard_disk_list'] =  components_msg.hard_disk.data
            components_xyz['flipping_points_list'] =  components_msg.flipping_points
            components_xyz['cpu_list'] =  components_msg.cpu.data
            components_xyz['screws_near_cpu_cut_path_list'] =  components_msg.screws_near_cpu_cut_path
            components_xyz['screws_near_cpu_list'] =  components_msg.screws_near_cpu.data
            
            if len(components_xyz['fan_list']) > 0 and (prev_state == 'P1'): #TODO: Remove this if condition
                userdata.captured_objects = 'fan screws detected'
            elif len(components_xyz['fan_screws_list']) > 0 and (prev_state == 'P1'): #TODO: Remove this if condition
                userdata.captured_objects = 'fan screws detected'
            elif len(components_xyz['cd_rom_cut_path_list']) > 0 and (prev_state == 'P4'): #TODO: Remove this if condition
                userdata.captured_objects = 'cdrom'
            # elif len(components_xyz['screws_near_cpu_list']) > 0 and not(prev_state == 'P29'):
            #     userdata.captured_objects = 'cpu screws'
            elif len(components_xyz['screws_list']) > 0 and not(prev_state == 'P4'):
                userdata.captured_objects = 'screws detected'
            elif len(components_xyz['front_cover_cut_path_list']) > 0:
                if len(components_xyz['keyboard_list']) > 0:
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

        self.capture_list = list(['P1', 'P4', 'P6', 'P12', 'P25', 'P29'])
        self.cut_list = list(['P5', 'P7', 'P8', 'P14', 'P19', 'P23', 'P26'])
        self.screw_list = list(['P2', 'P13', 'P15.5'])
        self.grip_list = list(['P9', 'P11', 'P15', 'P16', 'P17', 'P18', 'P20', 'P28'])
        self.flip_list = list(['P3', 'P10', 'P24', 'P27'])
        self.wait_list = list(['P21', 'P22'])
        # self.suction_list = list(['P9','P15','P16','P17']) #TODO: Removed Fan from suction, review it
        self.suction_list = list(['P9','P15','P17'])
        
        self.camera_obj = Camera()

    def execute(self, userdata):
        global prev_state
        global gripping_tool
        
        passed_argument = {#'P2': components_xyz['screws_cut_path_list'],
                   'P2': components_xyz['screws_list'],
                   'P3': components_xyz['flipping_points_list'],
                   'P5': components_xyz['front_cover_cut_path_list'],
                   'P7': components_xyz['screws_cut_path_list'],
                   'P8': components_xyz['keyboard_cut_path_list'],
                   'P9': components_xyz['front_cover_list'], 
                   'P10': components_xyz['flipping_points_list'],
                   'P11': components_xyz['keyboard_list'], 
                   'P13': components_xyz['screws_list'],
                   'P14': components_xyz['screws_cut_path_list'],
                   'P15': components_xyz['hard_disk_list'],
                   'P15.5': components_xyz['fan_screws_list'],
                   'P16': components_xyz['fan_list'],
                   'P17': components_xyz['cd_rom_list'],
                   'P18': components_xyz['motherboard_list'],
                   'P19': components_xyz['ports_cut_path_list'],
                   'P20': components_xyz['motherboard_list'],  #TODO account for P18
                   'P23': components_xyz['cd_rom_cut_path_list'],
                   'P24': components_xyz['flipping_points_list'],
                   'P26': components_xyz['screws_near_cpu_cut_path_list'],
                   'P27': components_xyz['flipping_points_list'],
                   'P28': components_xyz['cpu_list']}

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
        #print("Prev_State_1 =", prev_state )
        
        self.camera_obj.camera_ms.change_tool_status('YELLOW_LED', status=1)
        self.camera_obj.camera_ms.change_tool_status('RED_LED', status=0)
        self.camera_obj.camera_ms.change_tool_status('GREEN_LED', status=0)
        
        next_operation = next_state[prev_state][userdata.captured_objects]
        if next_operation in passed_argument.keys():
            userdata.operation_parts = passed_argument[next_operation]
         #   print("First Operation Parts =", passed_argument[next_operation] )
        passed_argument[next_operation] = []
        #print("Second Operation Parts =", passed_argument[next_operation] )
        if next_operation in self.capture_list:
            prev_state = next_operation
            return 'Need Data'
        elif next_operation in self.cut_list:        # rospy.sleep(5)
            # rospy.sleep(5)
            # rospy.sleep(5)
            # rospy.sleep(5)
    
            prev_state = next_operation
            return 'Cover'
        elif next_operation in self.grip_list:
            if next_operation in self.suction_list:
                gripping_tool = 'suction cup'
            else:
                gripping_tool = 'gripper'
            #print("gripping_tool =", gripping_tool  )
            
            rospy.set_param('sorting_destination', sorting_boxes[next_operation])  #set sorting destination to be read by gripping node  
            prev_state = next_operation
            return 'Loose Components'
        elif next_operation in self.screw_list:
            prev_state = next_operation
         #   print("Prev_State_2 =", prev_state )
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
        
        self.loop_list = ['P7','P14','P19', 'P23', 'P26']
        
        self.camera_obj = Camera()

    def execute(self, userdata):
        rospy.sleep(2)
        
        self.camera_obj.camera_ms.change_tool_status('YELLOW_LED', status=0)
        self.camera_obj.camera_ms.change_tool_status('RED_LED', status=0)
        self.camera_obj.camera_ms.change_tool_status('GREEN_LED', status=1)
        
        #in case of a list of contours, send one by one and wait for their completion
        if prev_state in self.loop_list:
            for loop_counter in range(0, len(userdata.operation_parts)):
                if prev_state == 'P26':
                    cutting_data_pose_array = userdata.operation_parts[loop_counter].data
                else:
                    cutting_data_xyz_list = userdata.operation_parts[loop_counter].data
                    cutting_data_pose_array = xyz_list_to_pose_array(cutting_data_xyz_list)

                self.cutting_xyz_pub.publish(cutting_data_pose_array)

                received_msg = rospy.wait_for_message('/done', String)
                if received_msg.data == "cutting failed":
                    userdata.prev_state = 'cutting'
                    return 'Collision'
            return 'Cutting Done'
        
        #in case of a single contour send it and wait for completion
        else:
            cutting_data_xyz_list = userdata.operation_parts
            cutting_data_pose_array = xyz_list_to_pose_array(cutting_data_xyz_list)
            self.cutting_xyz_pub.publish(cutting_data_pose_array)
            
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
        
        self.camera_obj = Camera()

    def execute(self, userdata):
        rospy.sleep(5)
        
        self.camera_obj.camera_ms.change_tool_status('YELLOW_LED', status=0)
        self.camera_obj.camera_ms.change_tool_status('RED_LED', status=0)
        self.camera_obj.camera_ms.change_tool_status('GREEN_LED', status=1)
        
        screw_data_xyz_list = userdata.operation_parts
        screw_data_pose_array = xyz_list_to_pose_array(screw_data_xyz_list)
        self.screw_xyz_pub.publish(screw_data_pose_array)
        
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
        self.suction_xyz_pub = rospy.Publisher("/suction_xyz", PoseArray, queue_size=1)
        
        self.camera_obj = Camera()

    def execute(self, userdata):
        rospy.sleep(2)
        
        self.camera_obj.camera_ms.change_tool_status('YELLOW_LED', status=0)
        self.camera_obj.camera_ms.change_tool_status('RED_LED', status=0)
        self.camera_obj.camera_ms.change_tool_status('GREEN_LED', status=1)
        
        if not(len(userdata.operation_parts)>0):
            return 'Gripping Done'
        
        grip_data_xyz_list = userdata.operation_parts
        grip_data_pose_array = xyz_list_to_pose_array(grip_data_xyz_list)
        if gripping_tool == 'suction cup':
            self.suction_xyz_pub.publish(grip_data_pose_array)
            #print("published2")
        else:
            self.grip_xyz_pub.publish(grip_data_pose_array)
        
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
        
        self.camera_obj = Camera()

    def execute(self, userdata):
        rospy.sleep(2)
        
        self.camera_obj.camera_ms.change_tool_status('YELLOW_LED', status=0)
        self.camera_obj.camera_ms.change_tool_status('RED_LED', status=0)
        self.camera_obj.camera_ms.change_tool_status('GREEN_LED', status=1)
        
        # flip_px_msg = Float32MultiArray()
        # flip_px_msg.data = userdata.operation_parts
        # self.flip_px_pub.publish(flip_px_msg)
        
        # received_xyz = rospy.wait_for_message('/px_to_xyz', PoseArray)
        received_xyz = userdata.operation_parts
        self.flip_xyz_pub.publish(received_xyz)
        
        received_msg = rospy.wait_for_message('/done', String)
        
        userdata.captured_objects = 'flipping done'
        return 'Flipping Done'

# define state Connection Error


class Connection_Error(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Error Solved'])
        self.connection_error_publisher = rospy.Publisher(
            "/connection_error_handled", String, queue_size=0)
        
        self.camera_obj = Camera()

    def execute(self, userdata):
        rospy.sleep(2)
        
        self.camera_obj.camera_ms.change_tool_status('YELLOW_LED', status=0)
        self.camera_obj.camera_ms.change_tool_status('RED_LED', status=1)
        self.camera_obj.camera_ms.change_tool_status('GREEN_LED', status=0)
        
        print("Please Connect The Camera Then Press Any Button")
        camera_connected = String()
        camera_connected.data = raw_input()
        self.connection_error_publisher.publish(camera_connected)
        return 'Error Solved'
    
# define state Process Failed


class Wait_For_Operator(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Reset/Continue'])
        self.camera_obj = Camera()

    def execute(self, userdata):
        
        self.camera_obj.camera_ms.change_tool_status('YELLOW_LED', status=0)
        self.camera_obj.camera_ms.change_tool_status('RED_LED', status=1)
        self.camera_obj.camera_ms.change_tool_status('GREEN_LED', status=0)
        
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
        
        self.camera_obj = Camera()

    def change_tool_status(self, status=0):
        self.cutting_tool.wait_for_service()
        cutting_tool_status = SetIOSignalRequest()
        cutting_tool_status.signal = "CuttingTool"
        cutting_tool_status.value = str(status)
        response = self.cutting_tool(cutting_tool_status)
        return response

    def execute(self, userdata):
        
        self.camera_obj.camera_ms.change_tool_status('YELLOW_LED', status=0)
        self.camera_obj.camera_ms.change_tool_status('RED_LED', status=1)
        self.camera_obj.camera_ms.change_tool_status('GREEN_LED', status=0)
        
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
