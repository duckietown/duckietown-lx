# Program to make a simple
# login screen
import rospy
from duckietown.dtros import DTROS, NodeType, TopicType
from std_msgs.msg import String

import tkinter as tk
import os

root = tk.Tk()

p1 = tk.PhotoImage(file="/root/.icons/duckietown.png")

# Setting icon of master window
root.iconphoto(False, p1)

# setting the windows size
root.geometry("250x150")
root.title("VLF control tool")

DB_NAME = os.getenv("VEHICLE_NAME")

inter_node = DTROS(node_name="InteractionNode", node_type=NodeType.DIAGNOSTICS)

pub = rospy.Publisher(f"/{DB_NAME}/vlf_node/action", String, queue_size=1, dt_topic_type=TopicType.GENERIC)


def submit_stop():
    msg = String()
    msg.data = "stop"
    pub.publish(msg)


def submit_go():
    msg = String()
    msg.data = "go"
    pub.publish(msg)


sub_go_btn = tk.Button(root, text="   Go   ", command=submit_go)
sub_stop_btn = tk.Button(root, text=" Stop ", command=submit_stop)

sub_go_btn.grid(row=0, column=0)
sub_stop_btn.grid(row=1, column=0)

# performing an infinite loop
# for the window to display
root.mainloop()
