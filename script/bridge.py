import base64
import time
import roslibpy
import socketio
import json
import os
import subprocess
from threading import Lock
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
#from std_msgs.msg import String
#from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from engineio.payload import Payload

Payload.max_decode_packets = 2048

verbose = 0 #0: Low message, 1: Enable

lock = Lock()

sio = socketio.Client()
sio.connect('ws://127.0.0.1:5556', namespaces=['/rosbridge'])
# sio.connect('ws://45.76.210.253:5556', namespaces=['/rosbridge'])
# sio.connect('ws://172.20.10.6:5556', namespaces=['/rosbridge'])
# sio.connect('ws://192.168.242.106:5556', namespaces=['/rosbridge'])

print('SID:', sio.sid)

host = 'localhost'
port = 9090
client = roslibpy.Ros(host, port)
client.run()

subscriber = {}
c_subscriber = None
start_time = -1

# Topic const str
cmd_vel_str = '/cmd_vel'
turtle1_cmd_vel_str = '/turtle1/cmd_vel'
move_base_simple_str = '/move_base_simple/goal'
map_str = '/map'
ros2_camera_str = '/camera/image/compressed'
ros1_camera_str = '/raspicam_node/image/compressed'
amcl_pose_str = '/amcl_pose'
goal_pose_str = '/goal_pose'
custom_pose_str = '/custom_pose'
# End const str

topic_type = {
    cmd_vel_str: 'geometry_msgs/msg/Twist',
    turtle1_cmd_vel_str: 'geometry_msgs/Twist',
    move_base_simple_str: 'geometry_msgs/PoseStamped',
    map_str: 'nav_msgs/OccupancyGrid',
    ros1_camera_str: 'sensor_msgs/CompressedImage',
    amcl_pose_str: 'geometry_msgs/msg/PoseWithCovarianceStamped',
    goal_pose_str: 'geometry_msgs/PoseStamped',
    custom_pose_str: 'geometry_msgs/msg/PoseWithCovarianceStamped'
}

class CameraQos(Node):

    def __init__(self, topic, qos_profile):
        if not rclpy.ok():
            rclpy.init()
        super().__init__('camera_qos')
        self.topic = topic
        self.sub = self.create_subscription(
            CompressedImage, topic, self.c_callback, qos_profile)

    def c_callback(self, message):
        global start_time
        
        sec = str(message.header.stamp.sec)
        nanosec = str(message.header.stamp.nanosec)
        frame_id = '"' + str(message.header.frame_id) + '"'
        format = '"' + str(message.format) + '"'
        data = '"' + str(base64.b64encode(message.data))[2:-1] + '"'
        
        msg = '{"header": {"stamp": {"sec": ' + sec + ', "nanosec": ' + nanosec + '}, "frame_id": ' + frame_id + '}, "format": ' + format +', "data": ' + data + '}'
        jmsg = json.loads(msg)
        
        jmsg['topic'] = self.topic
        #print('jmsg =', jmsg)

        if -1 == start_time:
            sio.emit('bridge', jmsg, namespace='/rosbridge')
            start_time = time.time()
        else:
            sio.emit('bridge', jmsg, namespace='/rosbridge')    
            end_time = time.time()
            fps = 1 / (end_time - start_time)
            print(f"FPS = {fps}")
            start_time = time.time()

class Talker():
    def __init__(self, topic, topic_type):
        self.topic=topic
        self.topic_type=topic_type

    def talk(self,data):
        talker = roslibpy.Topic(client, self.topic, self.topic_type)
        talker.publish(roslibpy.Message(data))
        talker.unadvertise()

class RosbridgeNamespace(socketio.ClientNamespace):
    def __init__(self, namespace):
        super().__init__(namespace)
        if not rclpy.ok():
            rclpy.init()
        self.is_lock = False
        self.is_task_run = False
        #self.ini_nav()
        
    def ini_nav(self, x = 2.0, y = 0.5, z = 1.0, w = 0.0):
        
        ix = float(x) if x is not None else None
        iy = float(y) if y is not None else None
        iz = float(z) if z is not None else None
        iw = float(w) if w is not None else None
        lock.acquire()
        self.navigator = BasicNavigator()

        # Set our demo's initial pose
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()    
        #initial_pose.pose.position.x = 3.45
        #initial_pose.pose.position.y = 2.15
        #initial_pose.pose.orientation.z = 1.0
        
        #initial_pose.pose.position.x = 2.0
        #initial_pose.pose.position.y = 0.5
        if ix is not None:
            initial_pose.pose.position.x = ix
        if iy is not None:
            initial_pose.pose.position.y = iy
        if iz is not None:
            initial_pose.pose.orientation.z = iz
        if iw is not None:
            initial_pose.pose.orientation.w = iw
        self.navigator.setInitialPose(initial_pose)
        lock.release()
        
    def auto_nav(self, dest_x = -2.0, dest_y = -0.5, dest_z = None, dest_w = 1.0):
        #if None == self.navigator:
        #    self.ini_nav()
        
        # Avoid double click
        if self.is_lock:
            return
        # Change goal while task is NOT complete
        #elif not self.navigator.isTaskComplete() and self.goal_pose:
        elif self.is_task_run:
            self.goal_pose.pose.position.x = dest_x
            self.goal_pose.pose.position.y = dest_y
            self.navigator.goToPose(self.goal_pose)
            return
        
        self.is_lock = True    
        self.navigator.waitUntilNav2Active()

        # Go to our demos first goal pose
        self.goal_pose = PoseStamped()
        self.goal_pose.header.frame_id = 'map'
        self.goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        #self.goal_pose.pose.position.x = -2.0
        #self.goal_pose.pose.position.y = -0.5
        if dest_x is not None:
            self.goal_pose.pose.position.x = dest_x
        if dest_y is not None:
            self.goal_pose.pose.position.y = dest_y
        if dest_z is not None:
            self.goal_pose.pose.orientation.z = dest_z
        if dest_w is not None:
            self.goal_pose.pose.orientation.w = dest_w                 

        # sanity check a valid path exists
        # path = self.navigator.getPath(initial_pose, self.goal_pose)

        self.navigator.goToPose(self.goal_pose)

        i = 0
        self.is_lock = False
        self.is_task_run = True
        while not self.navigator.isTaskComplete():
            ################################################
            #
            # Implement some code here for your application!
            #
            ################################################

            # Do something with the feedback
            i = i + 1
            feedback = self.navigator.getFeedback()
            if feedback and i % 5 == 0:
                print('Estimated time of arrival: ' + '{0:.0f}'.format(
                      Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                      + ' seconds.')

                # Some navigation timeout to demo cancellation
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                    self.navigator.cancelTask()

        # Do something depending on the return code
        self.is_task_run = False
        self.is_lock = True
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Goal succeeded!')
        elif result == TaskResult.CANCELED:
            print('Goal was canceled!')
        elif result == TaskResult.FAILED:
            print('Goal failed!')
        else:
            print('Goal has an invalid return status!')
        self.is_lock = False
            
    #def shutdown_nav(self):
    #    self.navigator.lifecycleShutdown()
    #    self.navigator = None
        #exit(0)
        
    def on_connect(self):
        print(f"connect, sid={sio.sid}")

    def on_disconnect(self):
        print(f"disconnect")

    def on_server(self, data):
        global subscriber
        global c_subscriber

        #self.lock = Lock()
        print(f"收到server傳送的訊息: {data}")
        data_dict = json.loads(data)
        #print(f"收到server傳送的訊息: {data_dict}")
        topic = data_dict.get('topic', None)
        service = data_dict.get('service', None)
        ope = data_dict.get('op', None)
        data_id = data_dict.get('id', None)
        tp_type = topic_type.get(topic, None) if topic else None
        
        if tp_type is not None and ope == 'publish':
            talker=Talker(topic, tp_type)
            lock.acquire()
            talker.talk(data_dict['msg'])
            lock.release()
        elif tp_type is not None and ope == 'subscribe':
            lock.acquire()
            subscriber[topic] = roslibpy.Topic(client, topic, tp_type)
            subscriber[topic].subscribe(fp_callback(topic))
            lock.release()
        elif tp_type is not None and ope == 'unsubscribe':
            unsub = subscriber.get(topic, None)
            if unsub:
                lock.acquire()
                unsub.unsubscribe()
                lock.release()
        elif topic == ros2_camera_str  and ope == 'subscribe':
            if None == c_subscriber:
                lock.acquire()
                custom_qos_profile = QoSProfile(
                    depth=1,
                    reliability=QoSReliabilityPolicy.BEST_EFFORT)
                c_subscriber = CameraQos(topic, custom_qos_profile)
                lock.release()
                print("Start spin")
                rclpy.spin(c_subscriber)
                print("End spin")
            #if None == c_subscriber:
            #    c_subscriber = roslibpy.Topic(client, '/camera/image/compressed', 'sensor_msgs/msg/CompressedImage')
            #c_subscriber.subscribe(camera_callback)        
        elif topic == ros2_camera_str and ope == 'unsubscribe':
            if c_subscriber:
                lock.acquire()
                c_subscriber.destroy_subscription(c_subscriber.sub)
                c_subscriber.destroy_node()
                rclpy.shutdown()
                #rclpy.init() # re-init rclpy
                c_subscriber = None
                lock.release()
            #if c_subscriber: 
            #    c_subscriber.unsubscribe()
        elif data_id == 'publish:/ini_nav' and ope == 'publish':
            msg_dict = data_dict['msg']
            x, y, z, w = msg_dict.get('ix', None), msg_dict.get('iy', None), msg_dict.get('iz', None), msg_dict.get('iw', None)
            self.ini_nav(x, y, z, w)
        elif data_id == 'publish:/auto_nav' and ope == 'publish':
            msg_dict = data_dict['msg']
            x, y, z, w = msg_dict.get('x', None), msg_dict.get('y', None), msg_dict.get('z', None), msg_dict.get('w', None)
            self.auto_nav(x, y, z, w)
        elif data_id == 'publish:/save_map' and ope == 'publish':
            msg_dict = data_dict['msg']
            save_map_path = msg_dict['path']
            print("Path :", save_map_path)
            lock.acquire()
            result = subprocess.run("ros2 run nav2_map_server map_saver_cli -f " + save_map_path, capture_output=True, text=True, shell=True)
            lock.release() 
            print(result.stdout)
            print(result.stderr)
        elif ope == 'call_service' and service is not None and data_id is not None:
            msg_dict = data_dict.get('args', None)
            lock.acquire()
            result = call_service(service, data_id, msg_dict)
            lock.release()
            emit_response(service, result)
            print(result)
        #elif data_dict['iresultd'] == 'publish:/shutdown_nav' and data_dict['op']=='publish':
        #    self.shutdown_nav()

sio.register_namespace(RosbridgeNamespace('/rosbridge'))

def emit_response(service, result):
    response = {
        'header': {'frame_id': service[1:]},
        'op': "service_response",
        'service': service,
    }
    values = []
    out = {'out':'null'}
    r = False
    if result:
        ack = result.get('ack', None)
        try:
            if ack is None or ack !='FAIL':
                r = True
            out['out'] = json.dumps(result)
        except:
            out['out'] = str(result)
    
    values.append(out)
    response['values'] = values
    response['result'] = r
    sio.emit('bridge', response, namespace='/rosbridge')

def fp_callback(topic_name):
    def callback(message):
        message['topic'] = topic_name
        sio.emit('bridge', message, namespace='/rosbridge')
        if 1 == verbose:
            print(f"Callback, SID={sio.sid}")
        else:
            print("Callback")
    return callback
    
def call_service(service_name, service_type, params=None):
    service = roslibpy.Service(client, service_name, service_type)
    request = roslibpy.ServiceRequest(params)
    result = None
    try:
        result = dict(service.call(request, timeout=10))
    except Exception as e:
        print(f"Error:{e}")
    return result
    
