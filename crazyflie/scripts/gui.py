#!/usr/bin/env python3

import math
import threading
from pathlib import Path

import rclpy
from geometry_msgs.msg import Pose, Twist
from rcl_interfaces.msg import Log
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from nicegui import Client, app, events, ui, ui_run


class NiceGuiNode(Node):

    def __init__(self) -> None:
        super().__init__('nicegui')

        # find all crazyflies
        self.cfnames = []
        for srv_name, srv_types in self.get_service_names_and_types():
            if 'crazyflie_interfaces/srv/StartTrajectory' in srv_types:
                # remove '/' and '/start_trajectory'
                cfname = srv_name[1:-17]
                if cfname != 'all':
                    self.cfnames.append(cfname)

        print(self.cfnames)
        self.cfs = []

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)




        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 1)
        self.sub_log = self.create_subscription(Log, 'rosout', self.on_rosout, 1)

        self.logs = dict()

        with Client.auto_index_client:
            # with ui.row().classes('w-full h-full no-wrap'):
            #     self.log = ui.log().classes('w-full h-full no-wrap')



            
            with ui.row().classes('items-stretch'):
            #     with ui.card().classes('w-44 text-center items-center'):
            #         ui.label('Data').classes('text-2xl')
            #         ui.label('linear velocity').classes('text-xs mb-[-1.8em]')
            #         slider_props = 'readonly selection-color=transparent'
            #         self.linear = ui.slider(min=-1, max=1, step=0.05, value=0).props(slider_props)
            #         ui.label('angular velocity').classes('text-xs mb-[-1.8em]')
            #         self.angular = ui.slider(min=-1, max=1, step=0.05, value=0).props(slider_props)
            #         ui.label('position').classes('text-xs mb-[-1.4em]')
            #         self.position = ui.label('---')
                with ui.card().classes('w-full h-full'):
                    ui.label('Visualization').classes('text-2xl')
                    with ui.scene(800, 600, on_click=self.on_vis_click) as scene:
                        for name in self.cfnames:
                            robot = scene.stl('/urdf/cf2_assembly.stl').scale(1.0).material('#ff0000').with_name(name)
                            self.cfs.append(robot)
                        # with scene.group() as self.robot_3d:
                        #     prism = [[-0.5, -0.5], [0.5, -0.5], [0.75, 0], [0.5, 0.5], [-0.5, 0.5]]
                        #     self.robot_object = scene.extrusion(prism, 0.4).material('#4488ff', 0.5)
                            
            with ui.row().classes('w-full h-full'):
                with ui.tabs().classes('w-full') as tabs:
                    self.tabs = []
                    for name in ["all"] + self.cfnames:
                        self.tabs.append(ui.tab(name))
                with ui.tab_panels(tabs, value=self.tabs[0]).classes('w-full') as self.tabpanels:
                    for name, tab in zip(["all"] + self.cfnames, self.tabs):
                        with ui.tab_panel(tab):
                            self.logs[name] = ui.log().classes('w-full h-full no-wrap')
                            ui.label("Battery Voltage: ")
                            ui.button("Reboot")

            # Call on_timer function
            self.timer = self.create_timer(0.1, self.on_timer)

    def send_speed(self, x: float, y: float) -> None:
        msg = Twist()
        msg.linear.x = x
        msg.angular.z = -y
        self.linear.value = x
        self.angular.value = y
        self.cmd_vel_publisher.publish(msg)

    def on_rosout(self, msg: Log) -> None:
        if msg.name in self.logs:
            self.logs[msg.name].push(msg.msg)

    def on_timer(self) -> None:
        for name, robot in zip(self.cfnames, self.cfs):
            t = self.tf_buffer.lookup_transform(
                            "world",
                            name,
                            rclpy.time.Time())
            pos = t.transform.translation
            robot.move(pos.x, pos.y, pos.z)

    def on_vis_click(self, e: events.SceneClickEventArguments):
        hit = e.hits[0]
        name = hit.object_name or hit.object_id
        ui.notify(f'You clicked on the {name}')
        if name == 'ground':
            self.tabpanels.value = 'all'
        else:
            self.tabpanels.value = name



    # def handle_pose(self, msg: Pose) -> None:
    #     self.position.text = f'x: {msg.position.x:.2f}, y: {msg.position.y:.2f}'
    #     self.robot_3d.move(msg.position.x, msg.position.y)
    #     self.robot_3d.rotate(0, 0, 2 * math.atan2(msg.orientation.z, msg.orientation.w))


def main() -> None:
    # NOTE: This function is defined as the ROS entry point in setup.py, but it's empty to enable NiceGUI auto-reloading
    pass


def ros_main() -> None:
    rclpy.init()
    node = NiceGuiNode()
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass


app.add_static_files("/urdf", "/home/whoenig/projects/crazyflie/crazyswarm2/src/crazyswarm2/crazyflie/urdf/")
app.on_startup(lambda: threading.Thread(target=ros_main).start())
ui_run.APP_IMPORT_STRING = f'{__name__}:app'  # ROS2 uses a non-standard module name, so we need to specify it here
ui.run(uvicorn_reload_dirs=str(Path(__file__).parent.resolve()), favicon='🤖')
