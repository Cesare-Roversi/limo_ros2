#!/usr/bin/env python3
import os
import threading
import customtkinter
from plansys2_msgs.srv import GetProblemInstances, GetStates
from plansys2_msgs.msg import Param
from std_msgs.msg import Empty

from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from limo_planner_interfaces.action import Message

class ControllerGUI(Node):
    def __init__(self):
        super().__init__('controllerGUI')

        self._action_client = ActionClient(self, Message, 'message')
        self.progressbar = []
        self.progressbar_label = []
        self.statelabel = None

        self.subscription = self.create_subscription(
            Empty,
            'problem_expert/update_notify',
            self.update_screen_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.cli_instances = self.create_client(GetProblemInstances, 'problem_expert/get_problem_instances')
        self.cli_predicates = self.create_client(GetStates, 'problem_expert/get_problem_predicates')

    def update_screen_callback(self, msg):
        self.get_logger().info('UPDATING SCREEN')
        while not self.cli_instances.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = GetProblemInstances.Request()
        while not self.cli_predicates.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req2 = GetStates.Request()
        self.future=self.cli_instances.call_async(self.req)
        self.future.add_done_callback(self.get_instances_callback)
    def get_instances_callback(self, future):
        self.instances = future.result().instances
        self.future2=self.cli_predicates.call_async(self.req2)
        self.future2.add_done_callback(self.get_predicates_callback)
    def get_predicates_callback(self, future2):
        self.states = future2.result().states

        string_to_display="True Instances: \n"
        for instance in self.instances:
            string_to_display+="("+str(instance.name)+" : "+str(instance.type)+")\n"
        string_to_display+="True Predicates: \n"
        for state in self.states:
            string_to_display+="("+str(state.name)
            for param in state.parameters:
                string_to_display+=" "+str(param.name)
            string_to_display+=")\n"
        worldState.delete("0.0", "end")
        worldState.insert("0.0", string_to_display)

    def send_goal(self, order, request):
        for widget in progressbarsFrame.winfo_children():
            widget.destroy()
        controllerFrame.pack_forget()
        progressbarsFrame.pack(pady=0, padx=0, fill="both", expand=True)
        self.state_label=customtkinter.CTkLabel(master=progressbarsFrame, text="goal sent", font=("Arial", 15))
        self.state_label.pack(pady=5, padx=10)
        goal_msg = Message.Goal()
        goal_msg.operation = order
        goal_msg.request = request

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        if(order==3):
            root.destroy()
            rclpy.shutdown()
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        progressbarsFrame.pack_forget()
        controllerFrame.pack(pady=0, padx=0, fill="both", expand=True)
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.success))

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.state_label.configure(text="STATE:" + feedback.state)
        if(feedback.state=="EXECUTING"):
            while len(feedback.feedback) < len(self.progressbar):#eventuale rimozione progressbar in eccesso
                self.progressbar_label[len(self.progressbar)-1].destroy()
                self.progressbar[len(self.progressbar)-1].destroy()
                self.progressbar_label.pop(len(self.progressbar)-1)
                self.progressbar.pop(len(self.progressbar)-1)
            for i in range (min(len(self.progressbar), len(feedback.feedback))):#aggiornamento progressbar esistenti
                if((feedback.feedback[i].completion!=self.progressbar[i].get()) | (feedback.feedback[i].action_full_name!=self.progressbar_label[i].cget("text"))):
                    self.progressbar_label[i].configure(text=feedback.feedback[i].action_full_name)
                self.progressbar[i].set(feedback.feedback[i].completion)
            for i in range(len(self.progressbar), len(feedback.feedback)):#aggiunta eventuali nuove progressbar
                self.progressbar_label.append(customtkinter.CTkLabel(master=progressbarsFrame, text=feedback.feedback[i].action_full_name, font=("Arial", 15)))
                self.progressbar_label[i].pack(pady=5, padx=10)
                self.progressbar.append(customtkinter.CTkProgressBar(master=progressbarsFrame, orientation="horizontal"))
                self.progressbar[i].pack(pady=10, padx=10)
                self.progressbar[i].set(0.0)
                self.progressbar[i].set(feedback.feedback[i].completion)

    
def main(args=None):
    rclpy.init(args=args)
    global NODE
    NODE = ControllerGUI()
    customtkinter.set_appearance_mode("System")
    customtkinter.set_default_color_theme("dark-blue")
    global root
    root = customtkinter.CTk()
    root.geometry("450x700")
    def check_if_connected_list():
        print("Checking if connected...")
        NODE.send_goal(0, entry.get())
    def plan_patrol():
        print("Planning patrol...")
        NODE.send_goal(1, entry.get())
    def set_goal():
        print("Setting goal...")
        NODE.send_goal(2, entry.get())
    def close_window():
        print("Closing window...")
        NODE.send_goal(3, entry.get())
        #TODO: solve a problem with the shutdown of the node

    frame = customtkinter.CTkFrame(master=root)
    frame.pack(pady=20, padx=60, fill="both", expand=True)

    worldState_label = customtkinter.CTkLabel(master=frame, text="World State", font=("Arial", 20))
    worldState_label.pack(pady=10, padx=10)
    global worldState
    worldState = customtkinter.CTkTextbox(master=frame, width=700, height=200)
    worldState.pack(pady=10, padx=10)
    worldState.insert("0.0","▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒")

    controller_label = customtkinter.CTkLabel(master=frame, text="Controller", font=("Arial", 20))
    controller_label.pack(pady=10, padx=10)

    controllerORprogressbarsFrame = customtkinter.CTkFrame(master=frame)
    controllerORprogressbarsFrame.pack(pady=0, padx=0, fill="both", expand=True)
    global controllerFrame
    controllerFrame = customtkinter.CTkFrame(master=controllerORprogressbarsFrame)
    controllerFrame.pack(pady=0, padx=0, fill="both", expand=True)
    button1 = customtkinter.CTkButton(master=controllerFrame, text="Check if connected", command=check_if_connected_list)
    button1.pack(pady=10, padx=10)
    label1 = customtkinter.CTkLabel(master=controllerFrame, text="Inserisci i waypoint separati da virgola (es: wp1, wp2, wp3)\n--------------------", font=("Arial", 15))
    label1.pack(pady=2, padx=10)
    button2 = customtkinter.CTkButton(master=controllerFrame, text="Plan Patrol", command=plan_patrol)
    button2.pack(pady=10, padx=10)
    label2 = customtkinter.CTkLabel(master=controllerFrame, text="Inserisci i waypoint separati da virgola (es: wp1, wp2, wp3)\n--------------------", font=("Arial", 15))
    label2.pack(pady=2, padx=10)
    button3 = customtkinter.CTkButton(master=controllerFrame, text="Set Goal", command=set_goal)
    button3.pack(pady=10, padx=10)
    label3 = customtkinter.CTkLabel(master=controllerFrame, text="Inserisci i goal (virgola = and, '>' = goal successivi in coda)\n--------------------", font=("Arial", 15))
    label3.pack(pady=2, padx=10)
    button4 = customtkinter.CTkButton(master=controllerFrame, text="Close", command=close_window)
    button4.pack(pady=10, padx=10)

    entry = customtkinter.CTkEntry(master=controllerFrame, placeholder_text="Enter message before clicking buttons")
    entry.pack(pady=10, padx=10)
    entry.configure(width=400)

    global progressbarsFrame
    progressbarsFrame = customtkinter.CTkScrollableFrame(master=controllerORprogressbarsFrame)
    progressbarsFrame.pack_forget()
    NODE.update_screen_callback(Empty());
    thread_spin = threading.Thread(target=rclpy.spin, args=(NODE,))
    thread_spin.start()
    root.mainloop()

    NODE.destroy_node()
    rclpy.shutdown()
    thread_spin.join()

if __name__ == '__main__':
    main()