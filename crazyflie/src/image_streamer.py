import socket,os,struct, time
import numpy as np
import cv2
import yaml
from ament_index_python.packages import get_package_share_directory


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo

from rcl_interfaces.msg import ParameterDescriptor, ParameterType


class ImageNode(rclpy.node.Node):
    def __init__(self):
        super().__init__("image_node")

        # declare topic names
        self.declare_parameter(
            name="image_topic",
            value="/camera/image",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Image topic to publish to.",
            ),
        )

        self.declare_parameter(
            name="camera_info_topic",
            value="/camera/camera_info",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Camera info topic to subscribe to.",
            ),
        )

        # update topic names from config
        image_topic = (
            self.get_parameter("image_topic").get_parameter_value().string_value
        )
        self.get_logger().info(f"Image topic: {image_topic}")

        info_topic = (
            self.get_parameter("camera_info_topic").get_parameter_value().string_value
        )
        self.get_logger().info(f"Image info topic: {info_topic}")

        
        # load camera parameters from yaml
        # TODO: could possibly be done in the same way as with the other parameters
        config_path = os.path.join(
                    get_package_share_directory('crazyflie'),
                    'config',
                    'camera_config.yaml'
                )

        # create messages and publishers
        self.image_mgs = Image()
        self.camera_info_msg = self._construct_from_yaml(config_path)
        self.image_publisher = self.create_publisher(Image, image_topic, 10)
        self.info_publisher = self.create_publisher(CameraInfo, info_topic, 10)

        # TODO: just for testing, needs to be moved
        self.info_publisher.publish(self.camera_info_msg)

        # set up connection to AI Deck
        deck_ip = "192.168.4.1"
        deck_port = '5000'
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client_socket.connect((deck_ip, deck_port))
        self.image = None
        self.rx_buffer = bytearray()

        # set up timers for callbacks
        timer_period = 0.5
        self.rx_timer = self.create_timer(timer_period, self.receive_callback)
        self.tx_timer = self.create_timer(timer_period, self.publish_callback)


    def _construct_from_yaml(self, path):
        camera_info = CameraInfo()
        with open(path) as f:
            config = yaml.load(f, Loader=yaml.FullLoader)

        camera_info.header.frame_id = config['camera_name']
        camera_info.header.stamp = self.get_clock().now().to_msg()
        camera_info.width = int(config['image_width'])
        camera_info.height = int(config['image_height'])
        camera_info.distortion_model = config['distortion_model']
        camera_info.d = config['distortion_coefficients']['data']
        camera_info.k = config['camera_matrix']['data']
        camera_info.r = config['rectification_matrix']['data']
        camera_info.p = config['projection_matrix']['data']
        return camera_info

    def _rx_bytes(self, size):
        data = bytearray()
        while len(data) < size:
            data.extend(self.client_socket.recv(size-len(data)))
        return data

    def receive_callback(self):
        # first get the info
        packetInfoRaw = self._rx_bytes(4)
        [length, routing, function] = struct.unpack('<HBB', packetInfoRaw)

        # receive the header
        imgHeader = self._rx_bytes(length - 2)
        [magic, width, height, depth, format, size] = struct.unpack('<BHHBBI', imgHeader)

        # if magic is correct, get new image
        if magic == 0xBC:
            imgStream = bytearray()

            while len(imgStream) < size:
                packetInfoRaw = self._rx_bytes(4)
                [length, dst, src] = struct.unpack('<HBB', packetInfoRaw)
                #print("Chunk size is {} ({:02X}->{:02X})".format(length, src, dst))
                chunk = self._rx_bytes(length - 2)
                imgStream.extend(chunk)

            raw_img = np.frombuffer(imgStream, dtype=np.uint8)
            raw_img.shape = (width, height)
            self.image = cv2.cvtColor(raw_img, cv2.COLOR_BayerBG2RGBA)

        else: # otherwise set image to None again
            self.image = None

    def publish_callback(self):
        if self.image is not None:
            self.image_mgs.header.frame_id = self.camera_info_msg.header.frame_id
            self.image_mgs.header.stamp = self.get_clock().now().to_msg()
            self.camera_info_msg.header.stamp = self.image_mgs.header.stamp

            self.image_mgs.height = self.camera_info_msg.height
            self.image_mgs.width = self.camera_info_msg.width
            self.image_mgs.encoding = 'rgba8'
            self.image_mgs.step = self.image.step
            self.image_mgs.is_bigendian = 0 # TODO: implement automatic check depending on system
            self.image_mgs.data = self.image.data

            self.image_publisher.publish(self.image_mgs)
            self.info_publisher.publish(self.camera_info_msg)
            self.image = None
            
            

def main(args=None):
    rclpy.init(args=args)
    node = ImageNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()