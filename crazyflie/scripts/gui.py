#!/usr/bin/env python3

import math
import threading
import time
from pathlib import Path
from functools import partial

import rclpy
from geometry_msgs.msg import Pose, Twist
from rcl_interfaces.msg import Log
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from crazyflie_interfaces.msg import Status
import rowan

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

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 1)
        self.sub_log = self.create_subscription(Log, 'rosout', self.on_rosout, rclpy.qos.QoSProfile(
                        depth=1000,
                        durability=rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL))

        self.logs = dict()
        self.supervisor_labels = dict()
        self.battery_labels = dict()
        self.radio_labels = dict()
        self.robotmodels = dict()
        # set of robot names that had a status recently
        self.watchdog = dict()

        with Client.auto_index_client:

            with ui.row().classes('items-stretch'):
                with ui.card().classes('w-full h-full'):
                    ui.label('Visualization').classes('text-2xl')
                    with ui.scene(800, 400, on_click=self.on_vis_click) as scene:
                        for name in self.cfnames:
                            robot = scene.stl('/urdf/cf2_assembly.stl').scale(1.0).material('#ff0000').with_name(name)
                            self.robotmodels[name] = robot
                            self.watchdog[name] = time.time()
                    scene.camera.x = 0
                    scene.camera.y = -1
                    scene.camera.z = 2
                    scene.camera.look_at_x = 0  
                    scene.camera.look_at_y = 0
                    scene.camera.look_at_z = 0

            with ui.row().classes('w-full h-lvh'):
                with ui.tabs().classes('w-full') as tabs:
                    self.tabs = []
                    for name in ["all"] + self.cfnames:
                        self.tabs.append(ui.tab(name))
                with ui.tab_panels(tabs, value=self.tabs[0], on_change=self.on_tab_change).classes('w-full') as self.tabpanels:
                    for name, tab in zip(["all"] + self.cfnames, self.tabs):
                        with ui.tab_panel(tab):
                            self.logs[name] = ui.log().classes('w-full h-96 no-wrap')
                            self.supervisor_labels[name] = ui.label("")
                            self.battery_labels[name] = ui.label("")
                            self.radio_labels[name] = ui.label("")
                            ui.button("Reboot", on_click=partial(self.on_reboot, name=name))

            for name in self.cfnames:
                self.create_subscription(Status, name + '/status', partial(self.on_status, name=name), 1)

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
        # filter by crazyflie and add to the correct log
        if msg.name == "crazyflie_server":
            if msg.msg.startswith("["):
                idx = msg.msg.find("]")
                name = msg.msg[1:idx]
                # if it was an "all" category, add only to CFs
                if name == 'all':
                    for logname ,log in self.logs.items():
                        if logname != "all":
                            log.push(msg.msg)
                elif name in self.logs:
                    self.logs[name].push(msg.msg[idx+2:])

        # add all possible messages to the 'all' tab
        self.logs['all'].push(msg.msg)

    def on_timer(self) -> None:
        for name, robotmodel in self.robotmodels.items():
            t = self.tf_buffer.lookup_transform(
                            "world",
                            name,
                            rclpy.time.Time())
            pos = t.transform.translation
            robotmodel.move(pos.x, pos.y, pos.z)
            robotmodel.rotate(*rowan.to_euler([
                t.transform.rotation.w,
                t.transform.rotation.x,
                t.transform.rotation.y,
                t.transform.rotation.z], "xyz"))

            # no update for a while -> mark red
            if time.time() - self.watchdog[name] > 2.0:
                robotmodel.material('#ff0000')

    def on_vis_click(self, e: events.SceneClickEventArguments):
        hit = e.hits[0]
        name = hit.object_name or hit.object_id
        ui.notify(f'You clicked on the {name}')
        if name == 'ground':
            self.tabpanels.value = 'all'
        else:
            self.tabpanels.value = name

    def on_status(self, msg, name) -> None:
        status_ok = True
        supervisor_text = ""
        if msg.supervisor_info & Status.SUPERVISOR_INFO_CAN_BE_ARMED:
            supervisor_text += "can be armed; "
        if msg.supervisor_info & Status.SUPERVISOR_INFO_IS_ARMED:
            supervisor_text += "is armed; "
        if msg.supervisor_info & Status.SUPERVISOR_INFO_AUTO_ARM:
            supervisor_text += "auto-arm; "
        if msg.supervisor_info & Status.SUPERVISOR_INFO_CAN_FLY:
            supervisor_text += "can fly; "
        if msg.supervisor_info & Status.SUPERVISOR_INFO_IS_FLYING:
            supervisor_text += "is flying; "
        if msg.supervisor_info & Status.SUPERVISOR_INFO_IS_TUMBLED:
            supervisor_text += "is tumpled; "
            status_ok = False
        if msg.supervisor_info & Status.SUPERVISOR_INFO_IS_LOCKED:
            supervisor_text += "is locked; "
            status_ok = False
        self.supervisor_labels[name].set_text(supervisor_text)

        battery_text = f'{msg.battery_voltage:.2f} V'
        if msg.battery_voltage < 3.8:
            status_ok = False
        if msg.pm_state == Status.PM_STATE_BATTERY:
            battery_text += " (on battery)"
        elif msg.pm_state == Status.PM_STATE_CHARGING:
            battery_text += " (charging)"
        elif msg.pm_state == Status.PM_STATE_CHARGED:
            battery_text += " (charged)"
        elif msg.pm_state == Status.PM_STATE_LOW_POWER:
            battery_text += " (low power)"
            status_ok = False
        elif msg.pm_state == Status.PM_STATE_SHUTDOWN:
            battery_text += " (shutdown)"
            status_ok = False
        self.battery_labels[name].set_text(battery_text)

        radio_text = f'{msg.rssi} dBm; Unicast: {msg.num_rx_unicast} / {msg.num_tx_unicast}; Broadcast: {msg.num_rx_broadcast} / {msg.num_tx_broadcast}'
        self.radio_labels[name].set_text(radio_text)

        if status_ok:
            self.robotmodels[name].material('#00ff00')
        else:
            self.robotmodels[name].material('#ff0000')

        self.watchdog[name] = time.time()


    def on_tab_change(self, arg):
        for name, robotmodel in self.robotmodels.items():
            if name != arg.value:
                robotmodel.scale(1)
        if arg.value in self.robotmodels:
            self.robotmodels[arg.value].scale(2)

    def on_reboot(self, name) -> None:
        ui.notify(f'Reboot not implemented, yet')


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
