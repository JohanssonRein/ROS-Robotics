# (c) robotics.snowcron.com
# Use: MIT license

from teleop_gui import robot_controller
import rclpy

import tkinter as tk
from PIL import Image, ImageTk
import threading, os



class App(threading.Thread):

    def __init__(self, controller):
        self.controller = controller
        self.arrLidarButtons = []
        self.arrOdomValues = []

        threading.Thread.__init__(self)
        self.start()

#    def callback(self):
#        self.root.quit()
#
#    def quit(self):
#        self.root.destroy()

    def after(self):
        for i, dRange in enumerate(self.controller.arrLidarRanges):
            if(i == 4):
                continue
            if(dRange < 999999.9):
                self.arrLidarButtons[i]["text"] = "%.2f" % (dRange)

        self.arrOdomValues[0]["text"] = "%.2f" % (self.controller.current_x)
        self.arrOdomValues[1]["text"] = "%.2f" % (self.controller.current_roll)
        self.arrOdomValues[2]["text"] = "%.2f" % (self.controller.current_y)
        self.arrOdomValues[3]["text"] = "%.2f" % (self.controller.current_pitch)
        self.arrOdomValues[4]["text"] = "%.2f" % (self.controller.current_z)
        self.arrOdomValues[5]["text"] = "%.2f" % (self.controller.current_yaw)

#        img = ImageTk.PhotoImage(image=self.controller.cv_image)
#        self.camera_label.configure(image=img)

        self.root.after(500, self.after)

    def run(self):
        self.root = tk.Tk()
        #frame_top = tk.Frame(master=self.root)#, width=200, height=100, bg="red")
        frame_row_1 = tk.Frame(master=self.root)
        frame_div_1 = tk.Frame(master=self.root, bg="gray", height=5)
        frame_row_2 = tk.Frame(master=self.root)#, width=200, height=100, bg="red")
        frame_div_2 = tk.Frame(master=self.root, bg="gray", height=5)
        frame_row_3 = tk.Frame(master=self.root)
        
        self.root.title("Controller GUI")
        self.root.resizable(width=False, height=False)
        dInit = 0.0
        for i in range(3):
            for j in range(3):
                frame = tk.Frame(
                    master=frame_row_1,
                    relief=tk.RAISED,
                    borderwidth=1
                )
                frame.grid(row=i, column=j, padx=5, pady=5)
                strText = f"{dInit}"
                if(i == 1 and j == 1):
                    strText = "STOP"
                button = tk.Button(master=frame, text=strText, width=10,
                    command = lambda m=(i,j) : self.processNavCommand(m))
                button.pack() #padx=5, pady=5)
                self.arrLidarButtons.append(button)
                
        frame_row_1.pack(fill=tk.BOTH)#side=tk.LEFT)

        frame_div_1.pack(fill="x")

        # ---
        self.arrOdomLabels = [ ["x:", "roll:"], ["y:", "pitch:"], ["z:", "yaw:"] ]
        for nRow in range(3):
            for nCol in range(2):
                frame = tk.Frame(
                    master=frame_row_2,
                    relief=tk.RAISED,
                    borderwidth=1
                )
                frame.grid(row=nRow, column=nCol, padx=5, pady=5)
                label = tk.Label(master=frame, text=self.arrOdomLabels[nRow][nCol],
                    width=4)
                label.pack(padx=5, pady=5, side=tk.LEFT)
                # ---
                value = tk.Label(master=frame, text=f"{dInit}", width=8, bg="white",
                    anchor="w")
                value.pack(padx=5, pady=5, side=tk.LEFT)
                self.arrOdomValues.append(value)

        frame_row_2.pack(fill=tk.BOTH)#, side=tk.LEFT)

        frame_div_2.pack(fill="x")

        #img = ImageTk.PhotoImage(image=self.controller.cv_image)
        self.camera_label = tk.Label(master=frame_row_3)
        self.camera_label.pack()
        
        frame_row_3.pack(fill=tk.BOTH)

        # ---
        #frame_top.pack()

        self.root.after(500, self.after)
        self.root.mainloop()
        print("Shutting down", flush=True)
        self.controller.destroy_node()
        rclpy.shutdown()        
        
        os._exit(1)

    def processNavCommand(self, nav):
        if(nav[0] == 1 and nav[1] == 1):  # Switch mode
            if(self.controller.bManualControl):
                self.controller.bManualControl = False
                self.arrLidarButtons[4]["text"] = "STOP"
            else:
                self.controller.bManualControl = True
                self.arrLidarButtons[4]["text"] = "START"
        elif(self.controller.bManualControl):
            if(nav[0] == 0 and nav[1] == 1):    # Forward
                self.controller.follow_wall(1)
            elif(nav[0] == 1 and nav[1] == 0):  # Left
                self.controller.follow_wall(0)
            elif(nav[0] == 1 and nav[1] == 2):  # Right
                self.controller.follow_wall(2)
            else:
                pass


def main(args=None):
    rclpy.init(args=args)
    controller = robot_controller.Controller()
    
    app = App(controller)

    controller.app = app

    rclpy.spin(controller)
        
#    controller.destroy_node()
#    rclpy.shutdown()

    #os._exit(1)

    #exit()
    #app.join()
    #app.quit()
    #app.terminate()
    
 
if __name__ == '__main__':
    main()

