# Program to make a simple
# login screen
import rospy
from duckietown.dtros import DTROS, NodeType, TopicType
from std_msgs.msg import String

import tkinter as tk
import os

root=tk.Tk()

p1 = tk.PhotoImage(file = '/root/.icons/duckietown.png')
 
# Setting icon of master window
root.iconphoto(False, p1)

# setting the windows size
root.geometry("250x150")
root.title("PID parameters tool")
# declaring string variable
# for storing name and password
v_var=tk.DoubleVar()
ref_var=tk.DoubleVar()
# kp_var=tk.DoubleVar()
# ki_var=tk.DoubleVar()
# kd_var=tk.DoubleVar()


DB_NAME = os.getenv('VEHICLE_NAME')

inter_node = DTROS(
        node_name="InteractionNode",
        node_type=NodeType.DIAGNOSTICS
    )

parameters_pub = rospy.Publisher(
        f'/{DB_NAME}/PID_parameters',
        String,
        queue_size=1,
        dt_topic_type=TopicType.GENERIC
    )

def submit_stop():
    PID_param_str = "STOP"

    print(PID_param_str)

    msg = String()
    msg.data = PID_param_str

    parameters_pub.publish(msg)

def submit():
    v_val=v_var.get()
    ref=ref_var.get()
    # Kp_val=kp_var.get()
    # Ki_val=ki_var.get()
    # Kd_val=kd_var.get()

    # PID_param_str = f"{ref};{v_val};{Kp_val};{Ki_val};{Kd_val}"
    PID_param_str = f"{ref};{v_val}"

    print(PID_param_str)

    msg = String()
    msg.data = PID_param_str

    parameters_pub.publish(msg)

ref = tk.Label(root, text = 'ref', font=('calibre',10, 'bold'))
ref_entry = tk.Entry(root,textvariable = ref_var, font=('calibre',10,'normal'))

v = tk.Label(root, text = 'v_0', font=('calibre',10, 'bold'))
v_entry = tk.Entry(root,textvariable = v_var, font=('calibre',10,'normal'))

# kp = tk.Label(root, text = 'Kp', font=('calibre',10, 'bold'))
# kp_entry = tk.Entry(root,textvariable = kp_var, font=('calibre',10,'normal'))

# ki = tk.Label(root, text = 'Ki', font = ('calibre',10,'bold'))
# ki_entry=tk.Entry(root, textvariable = ki_var, font = ('calibre',10,'normal'))

# kd = tk.Label(root, text = 'Kd', font = ('calibre',10,'bold'))
# kd_entry=tk.Entry(root, textvariable = kd_var, font = ('calibre',10,'normal'))


sub_btn=tk.Button(root,text = 'Send Commands', command = submit)
sub_stop_btn=tk.Button(root,text = 'Stop', command = submit_stop)

# placing the label and entry in
# the required position using grid
# method
ref.grid(row=0,column=0)
ref_entry.grid(row=0,column=1)
v.grid(row=1,column=0)
v_entry.grid(row=1,column=1)
# kp.grid(row=2,column=0)
# kp_entry.grid(row=2,column=1)
# ki.grid(row=3,column=0)
# ki_entry.grid(row=3,column=1)
# kd.grid(row=4,column=0)
# kd_entry.grid(row=4,column=1)
sub_btn.grid(row=2,column=0)
sub_stop_btn.grid(row=3,column=0)

# performing an infinite loop
# for the window to display
root.mainloop()
