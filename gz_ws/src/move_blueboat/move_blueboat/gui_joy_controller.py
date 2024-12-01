# Copyright 2024, Markus Buchholz

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import Float64
import threading
import time
import tkinter as tk

class BlueBoatGUIController(Node):

    def __init__(self):
        super().__init__('blue_boat_gui_controller')

        self.callback_group = ReentrantCallbackGroup()
        
        self.motor_port_publisher = self.create_publisher(
            Float64,
            '/model/blueboat/joint/motor_port_joint/cmd_thrust',
            10,
            callback_group=self.callback_group
        )
        
        self.motor_stbd_publisher = self.create_publisher(
            Float64,
            '/model/blueboat/joint/motor_stbd_joint/cmd_thrust',
            10,
            callback_group=self.callback_group
        )

        self.port_thrust = 0.0
        self.stbd_thrust = 0.0

        self.lock = threading.Lock()

        self.get_logger().info('BlueBoatGUIController initialized')
        
        self.running_event = threading.Event()
        self.running_event.set()

        self.update_thread = threading.Thread(target=self.update_thrust)
        self.update_thread.start()

        self.holding = False

    def update_thrust(self):
        while self.running_event.is_set():
            with self.lock:
                self.motor_port_publisher.publish(Float64(data=self.port_thrust))
                self.motor_stbd_publisher.publish(Float64(data=self.stbd_thrust))
            time.sleep(0.1)

    def change_thrust(self, port_increment, stbd_increment):
        self.holding = True
        while self.holding:
            with self.lock:
                self.port_thrust = max(min(self.port_thrust + port_increment, 15.0), -15.0)
                self.stbd_thrust = max(min(self.stbd_thrust + stbd_increment, 15.0), -15.0)
                self.get_logger().info(f'port thrust: {self.port_thrust}, stbd thrust: {self.stbd_thrust}')
            time.sleep(0.1)

    def increase_thrust(self):
        threading.Thread(target=self.change_thrust, args=(1.0, 1.0)).start()

    def decrease_thrust(self):
        threading.Thread(target=self.change_thrust, args=(-1.0, -1.0)).start()

    def turn_right(self):
        threading.Thread(target=self.change_thrust, args=(1.0, -1.0)).start()

    def turn_left(self):
        threading.Thread(target=self.change_thrust, args=(-1.0, 1.0)).start()

    def stop_holding(self, event=None):
        self.holding = False

    def destroy_node(self):
        self.running_event.clear()
        self.update_thread.join()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    blue_boat_gui_controller = BlueBoatGUIController()

    def on_closing():
        blue_boat_gui_controller.destroy_node()
        rclpy.shutdown()
        root.destroy()

    root = tk.Tk()
    root.title("Blue Boat Controller")

    frame = tk.Frame(root)
    frame.pack()

    btn_up = tk.Button(frame, text="↑", command=blue_boat_gui_controller.increase_thrust, height=5, width=10)
    btn_up.grid(row=0, column=1)
    btn_up.bind("<ButtonRelease-1>", blue_boat_gui_controller.stop_holding)

    btn_left = tk.Button(frame, text="←", command=blue_boat_gui_controller.turn_left, height=5, width=10)
    btn_left.grid(row=1, column=0)
    btn_left.bind("<ButtonRelease-1>", blue_boat_gui_controller.stop_holding)

    btn_down = tk.Button(frame, text="↓", command=blue_boat_gui_controller.decrease_thrust, height=5, width=10)
    btn_down.grid(row=1, column=1)
    btn_down.bind("<ButtonRelease-1>", blue_boat_gui_controller.stop_holding)

    btn_right = tk.Button(frame, text="→", command=blue_boat_gui_controller.turn_right, height=5, width=10)
    btn_right.grid(row=1, column=2)
    btn_right.bind("<ButtonRelease-1>", blue_boat_gui_controller.stop_holding)

    root.protocol("WM_DELETE_WINDOW", on_closing)

    rclpy_thread = threading.Thread(target=rclpy.spin, args=(blue_boat_gui_controller,))
    rclpy_thread.start()

    root.mainloop()

    rclpy_thread.join()

if __name__ == '__main__':
    main()
