import tkinter as tk
import math
import serial
import testmotor
import serial.tools.list_ports

junct132 = [11,1,0,0.1,6.28,1]
junct232 = [12,1,0,0.1,6.28,1]
junct332 = [13,1,0,0.1,6.28,1]
junct432 = [14,1,0,0.1,6.28,1]
junct532 = [15,1,0,0.1,6.28,1]
junct632 = [16,1,0,0.1,6.28,1]
pivot32 =  [21,1,0,0.1,6.28,1]
wheelMotorRB32 = [22,1,0,0.1,6.28,1]
wheelMotorLB32 = [23,1,0,0.1,6.28,1]
wheelMotorRF32 = [24,1,0,0.1,6.28,1]
wheelMotorLF32 = [25,1,0,0.1,6.28,1]
frontBinLift32 = [31,1,0,0.1,6.28,1]
backBinLift32 = [32,1,0,0.1,6.28,1]
frontBinLift32 = [31,1,0,0.1,6.28,1]
backBinLift32 = [32,1,0,0.1,6.28,1]
frontBinLiftDual32 = [33,1,0,0.1,6.28,1]
backBinLiftDual32 = [34,1,0,0.1,6.28,1]


chassisLift4 = [26,1,0,0.1,6.28,1]
pivot4 = [21,1,0,0.1,6.28,1]
wheelMotorRB4 = [22,1,0,0.1,6.28,1]
wheelMotorLB4 = [23,1,0,0.1,6.28,1]
wheelMotorRF4 = [24,1,0,0.1,6.28,1]
wheelMotorLF4 = [25,1,0,0.1,6.28,1]
frontBinLift4 = [31,1,0,0.1,6.28,1]
backBinLift4 = [32,1,0,0.1,6.28,1]
shoulderMotor4 = [11,1,0,0.1,6.28,1]
elbowMotor4 = [12,1,0,0.1,6.28,1]
endEffectorYaw4 = [13,1,0,0.1,6.28,1]
endEffectorExtend4 = [14,1,0,0.1,6.28,1]

loop_prd = 10

def setmotorparams(datvals):

     for x in range(len(datvals)):
          curr = datvals[x]
          if x == 0:
               change_box_contents(canIDEntry, curr)
          if x == 1:
               change_box_contents(kPEntry, curr)
          if x == 2:
               change_box_contents(kIEntry, curr)
          if x == 3:
               change_box_contents(kDEntry, curr)
          if x == 4:
               change_box_contents(magEntry, curr)
          if x == 5:
               change_box_contents(freqEntry, curr)

def close_window(_master):
    _master.quit()
    _master.destroy()

class MotorUI():

     def __init__(self):
          self.master = tk.Tk()
          self.master.resizable()
          self.master.grid_rowconfigure(20, minsize = 20)
          self.master.protocol("WM_DELETE_WINDOW", lambda: close_window(self.master))
          self.master.geometry("800x800")
          self.master.title("Motor Wizard")
          self.spinstatus = False
          self.commstatus = False
          self.Nucleo = None
          self.motor = None
          self.mag = 0.0
          self.freq = 0.0
          self.time = 0.0
          self.position_goal = 0.0
          self.motor_position = 0.0
          self.motor_velocity = 0.0
          self.motor_torque_reference = 0.0
          self.motor_torque_current = 0.0
          self.motor_select = False

     def serialloop(self):
          self.master.after(loop_prd, self.serialloop)
          self.time = self.time+loop_prd/1000
          if(self.Nucleo):
               if self.Nucleo.in_waiting:
                    content = self.Nucleo.readline()
                    # print(content)
                    values = content.strip().decode('ascii').split(',')
                    if(len(values)==6):
                         id,angle,vel,torque_ref,torque_cur,checksum_dumb = [float(s) for s in values]
                         rec_checksum = id+angle+vel+torque_ref+torque_cur
                         if(abs(rec_checksum-checksum_dumb)<0.1):
                              self.motor_position = angle
                              self.motor_velocity = vel
                              self.motor_torque_reference = torque_ref
                              self.motor_torque_current = torque_cur
          if self.spinstatus:
               self.position_goal = self.mag*math.sin(self.freq*2.0*math.pi*self.time)
               numrot = math.floor(abs(self.position_goal)/(math.pi*2.0))
               if(self.position_goal>0):
                    dir = 1
               else:
                    dir = -1
               p_des = self.position_goal -  (dir*math.pi*2.0*numrot)
               if(dir==-1):
                    dir = 0
               
               if self.motor:
                    self.motor.sendCMD(p_des,dir,numrot,0.0,0.0)

          if selector.get()!= "NONE" and selector.get()!="Select an available COM Port":
               COMPORT = selector.get().split("-")[0]
               #print("COMPORT IS "+COMPORT)
               #print("PORT"+ portEntry.get())
               length = len(portEntry.get())
               if COMPORT != portEntry.get():
                    portEntry.delete(0,length)
                    portEntry.insert(0,COMPORT)
                    change_label_contents(commlabel, "Comm Status: Click Connect")
          else:
               change_label_contents(commlabel, "Comm Status: INVALID SELECTION")

               

     def connect_nucleo(self, NUCLEO_COM):
          try:
               self.Nucleo = serial.Serial(NUCLEO_COM, timeout = .001)
               self.Nucleo.baudrate = 921600
               print("Connected")
               self.setcomms(True)
               change_label_contents(commlabel, "Comm Status: ON")
               change_label_contents(commlabel2, "Current Comm Port: " + portEntry.get() )
          except:
               print("Fail")
               self.setcomms(False)
               change_label_contents(commlabel, "Comm Status: Failed to Connect")
               change_label_contents(commlabel2, "Current Comm Port: None")
               
     def disconnect_nucleo(self):
          self.setcomms(False)
          change_label_contents(commlabel, "Comm Status: OFF")
          change_label_contents(commlabel2, "Current Comm Port: None"),
          self.Nucleo.close()
          self.Nucleo = None

     def setupSpin(self, CAN_ID, kp, kd, ki, mag, freq):
          self.motor = None
          self.createMotor(CAN_ID)
          self.enableMotor()
          self.sendMotorGains(kp,kd,ki)
          self.setTestValues(mag,freq)
          self.updatePosition()
          self.time = 0.0
          prop = self.motor_position/self.mag
          prop = min(max(-1.0, prop), 1.0)
          self.time = math.asin(prop)/(self.freq*2.0*math.pi)


     def updatePosition(self):
          updated = False
          if(self.Nucleo):
               while(not updated):
                    if self.Nucleo.in_waiting:
                         values = self.Nucleo.readline().strip().decode('ascii').split(',')
                         if(len(values)==6):
                              id,angle,vel,torque_ref,torque_cur,checksum_dumb = [float(s) for s in values]
                              rec_checksum = id+angle+vel+torque_ref+torque_cur
                              if(abs(rec_checksum-checksum_dumb)<0.1):
                                   self.motor_position = angle
                                   self.motor_velocity = vel
                                   self.motor_torque_reference = torque_ref
                                   self.motor_torque_current = torque_cur
                                   updated = True

     def stopSpin(self, CAN_ID):
          self.motor = None
          self.createMotor(CAN_ID)
          self.disableMotor()
          self.motor = None
     def createMotor(self, CAN_ID):
          self.motor = testmotor.testmotor(self.Nucleo,int(CAN_ID))
     def enableMotor(self):
          if self.motor:
               self.motor.sendEnable()
     def disableMotor(self):
          if self.motor:
               self.motor.sendDisable()
     def sendMotorGains(self,kp,kd,ki):
          if self.motor:
               self.motor.sendGains(kp,kd,ki,0)
     def setTestValues(self, mag, freq):
          self.mag = mag
          self.freq = freq
     def setstatus(self, status):
          self.spinstatus = status
     
     def setcomms(self, status):
          self.commstatus = status
     
     def spincheck(self, canID, kP, kD, kI, mag, freq):
          print(self.commstatus)
          if self.commstatus:
               self.setstatus(True)
               change_label_contents(spinstatuslabel, "Spin Status: ON")
               print("check")
               print(canID)
               self.setupSpin(canID, kP, kD, kI, mag, freq)
          else:
               change_label_contents(spinstatuslabel, "Spin Status: NOT CONNECTED")




          

     


motorui = MotorUI()


ROW_NUMBER = 0






ROW_NUMBER += 1
vel_ramp_label = tk.Label(motorui.master, text="Comm Input", font=('Helvetica', 13, 'bold'))
vel_ramp_label.grid(row=ROW_NUMBER, column=0, padx=10, pady=10)


ROW_NUMBER +=1


comm_connect_button = tk.Button(motorui.master,text="Connect",command=lambda: [motorui.connect_nucleo(portEntry.get())])
comm_connect_button.grid(row=ROW_NUMBER, column=0, padx=10, pady=10)



comm_disconnect_button = tk.Button(motorui.master,text="Disconnect",command=lambda: [ motorui.disconnect_nucleo()])
comm_disconnect_button.grid(row=ROW_NUMBER, column=1, padx=10, pady=10)



ROW_NUMBER += 1
portlabel = tk.Label(motorui.master, text="Comm Port:")
portlabel.grid(row=ROW_NUMBER, column=0)
portEntry = tk.Entry(motorui.master, width=6)
portEntry.grid(row=ROW_NUMBER, column=1)
portEntry.insert(0,"COM")


commlabel = tk.Label(motorui.master, text="Comm Status: OFF")
commlabel.grid(row=ROW_NUMBER, column=2)

commlabel2 = tk.Label(motorui.master, text="Current Comm Port: COM")
commlabel2.grid(row=ROW_NUMBER, column=3)



ROW_NUMBER += 1

OPTIONS = ["NONE"]


available_ports = serial.tools.list_ports.comports()

for port in available_ports:
     OPTIONS.append(port)

print(OPTIONS)

selector = tk.StringVar(motorui.master)
selector.set("Select an available COM Port")

testlabel = tk.OptionMenu(motorui.master,selector,*OPTIONS)
testlabel.grid(row=ROW_NUMBER,column=1)


def change_box_contents(_entry, _content):
    _entry.delete(0, "end")
    _entry.insert(0, _content)



def change_label_contents(label, content):
    label.config(text = content)



def setlift(self):

     return



def setpivot(self):
     return

def setWMRB(self):
     return

def setWMLB(self):
     return

def setWMRF(self):
     return

def setWMLF(self):
     return

def spinmotor(self):
     return


def stopmotor(self):

     return


ROW_NUMBER += 1




ROW_NUMBER += 1
vel_ramp_label = tk.Label(motorui.master, text="v3.2 Motor Presets", font=('Helvetica', 13, 'bold'))
vel_ramp_label.grid(row=ROW_NUMBER, column=0, padx=20, pady=20)

ROW_NUMBER += 1


enable_button = tk.Button(motorui.master,text="RB Wheel",command=lambda: setmotorparams(wheelMotorRB32))
enable_button.grid(row=ROW_NUMBER, column=0, padx=10, pady=5)



enable_button = tk.Button(motorui.master,text="LB Wheel",command=lambda: setmotorparams(wheelMotorLB32))
enable_button.grid(row=ROW_NUMBER, column=1, padx=10, pady=5)



enable_button = tk.Button(motorui.master,text="RF Wheel",command=lambda: setmotorparams(wheelMotorRF32))
enable_button.grid(row=ROW_NUMBER, column=2, padx=10, pady=5)



enable_button = tk.Button(motorui.master,text="LF Wheel",command=lambda: setmotorparams(wheelMotorLF32))
enable_button.grid(row=ROW_NUMBER, column=3, padx=10, pady=5)


ROW_NUMBER +=1

enable_button = tk.Button(motorui.master,text="Front Binlift",command=lambda: setmotorparams(frontBinLift32))
enable_button.grid(row=ROW_NUMBER, column=0, padx=10, pady=5)



enable_button = tk.Button(motorui.master,text="Back Binlift",command=lambda: setmotorparams(backBinLift32))
enable_button.grid(row=ROW_NUMBER, column=1, padx=10, pady=5)


enable_button = tk.Button(motorui.master,text="Front Binlift Dual Motor",command=lambda: setmotorparams(frontBinLiftDual32))
enable_button.grid(row=ROW_NUMBER, column=2, padx=10, pady=5)



enable_button = tk.Button(motorui.master,text="Back Binlift Dual Motor",command=lambda: setmotorparams(backBinLiftDual32))
enable_button.grid(row=ROW_NUMBER, column=3, padx=10, pady=5)


ROW_NUMBER +=1


enable_button = tk.Button(motorui.master,text="Arm J1",command=lambda: setmotorparams(junct132))
enable_button.grid(row=ROW_NUMBER, column=0, padx=10, pady=5)



enable_button = tk.Button(motorui.master,text="Arm J2",command=lambda: setmotorparams(junct232))
enable_button.grid(row=ROW_NUMBER, column=1, padx=10, pady=5)



enable_button = tk.Button(motorui.master,text="Arm J3",command=lambda: setmotorparams(junct332))
enable_button.grid(row=ROW_NUMBER, column=2, padx=10, pady=5)



enable_button = tk.Button(motorui.master,text="Arm J4",command=lambda: setmotorparams(junct432))
enable_button.grid(row=ROW_NUMBER, column=3, padx=10, pady=5)




ROW_NUMBER +=1


enable_button = tk.Button(motorui.master,text="Arm J5",command=lambda: setmotorparams(junct532))
enable_button.grid(row=ROW_NUMBER, column=0, padx=10, pady=5)



enable_button = tk.Button(motorui.master,text="Arm J6",command=lambda: setmotorparams(junct632))
enable_button.grid(row=ROW_NUMBER, column=1, padx=10, pady=5)


enable_button = tk.Button(motorui.master,text="Chassis Pivot",command=lambda: setmotorparams(pivot32))
enable_button.grid(row=ROW_NUMBER, column=2, padx=10, pady=5)


ROW_NUMBER += 1
vel_ramp_label = tk.Label(motorui.master, text="v4.0 Motor Presets", font=('Helvetica', 13, 'bold'))
vel_ramp_label.grid(row=ROW_NUMBER, column=0, padx=20, pady=20)

ROW_NUMBER += 1
enable_button = tk.Button(motorui.master,text="Chassis Lift",command=lambda: setmotorparams(chassisLift4))
enable_button.grid(row=ROW_NUMBER, column=0, padx=10, pady=5)


enable_button = tk.Button(motorui.master,text="Chassis Pivot",command=lambda: setmotorparams(pivot4))
enable_button.grid(row=ROW_NUMBER, column=1, padx=10, pady=5)



enable_button = tk.Button(motorui.master,text="RB Wheel",command=lambda: setmotorparams(wheelMotorRB4))
enable_button.grid(row=ROW_NUMBER, column=2, padx=10, pady=5)



enable_button = tk.Button(motorui.master,text="LB Wheel",command=lambda: setmotorparams(wheelMotorLB4))
enable_button.grid(row=ROW_NUMBER, column=3, padx=10, pady=5)



ROW_NUMBER +=1

enable_button = tk.Button(motorui.master,text="RF Wheel",command=lambda: setmotorparams(wheelMotorRF4))
enable_button.grid(row=ROW_NUMBER, column=0, padx=10, pady=5)



enable_button = tk.Button(motorui.master,text="LF Wheel",command=lambda: setmotorparams(wheelMotorLF4))
enable_button.grid(row=ROW_NUMBER, column=1, padx=10, pady=5)



enable_button = tk.Button(motorui.master,text="Front Binlift",command=lambda: setmotorparams(frontBinLift4))
enable_button.grid(row=ROW_NUMBER, column=2, padx=10, pady=5)



enable_button = tk.Button(motorui.master,text="Back Binlift",command=lambda: setmotorparams(backBinLift4))
enable_button.grid(row=ROW_NUMBER, column=3, padx=10, pady=5)


ROW_NUMBER +=1

enable_button = tk.Button(motorui.master,text="Shoulder Motor",command=lambda: setmotorparams(shoulderMotor4))
enable_button.grid(row=ROW_NUMBER, column=0, padx=10, pady=5)



enable_button = tk.Button(motorui.master,text="Elbow Motor",command=lambda: setmotorparams(elbowMotor4))
enable_button.grid(row=ROW_NUMBER, column=1, padx=10, pady=5)



enable_button = tk.Button(motorui.master,text="End Effector Yaw",command=lambda: setmotorparams(endEffectorYaw4))
enable_button.grid(row=ROW_NUMBER, column=2, padx=10, pady=5)



enable_button = tk.Button(motorui.master,text="End Effector Extend",command=lambda: setmotorparams(endEffectorExtend4))
enable_button.grid(row=ROW_NUMBER, column=3, padx=10, pady=5)



ROW_NUMBER += 1
vel_ramp_label = tk.Label(motorui.master, text="Motor Params", font=('Helvetica', 13, 'bold'))
vel_ramp_label.grid(row=ROW_NUMBER, column=0, padx=10, pady=10)


ROW_NUMBER += 1
canIDlabel = tk.Label(motorui.master, text="CAN ID:")
canIDlabel.grid(row=ROW_NUMBER, column=0)
canIDEntry = tk.Entry(motorui.master, width=6)
canIDEntry.grid(row=ROW_NUMBER, column=1)

ROW_NUMBER += 1
kPlabel = tk.Label(motorui.master, text="Kp:")
kPlabel.grid(row=ROW_NUMBER, column=0)
kPEntry = tk.Entry(motorui.master, width=6)
kPEntry.grid(row=ROW_NUMBER, column=1)

ROW_NUMBER += 1
kIlabel = tk.Label(motorui.master, text="Ki:")
kIlabel.grid(row=ROW_NUMBER, column=0)
kIEntry = tk.Entry(motorui.master, width=6)
kIEntry.grid(row=ROW_NUMBER, column=1)

ROW_NUMBER += 1
kDlabel = tk.Label(motorui.master, text="Kd:")
kDEntry = tk.Entry(motorui.master, width=6)
kDEntry.grid(row=ROW_NUMBER, column=1)



ROW_NUMBER += 1
maglabel = tk.Label(motorui.master, text="Mag (rad):")
maglabel.grid(row=ROW_NUMBER, column=0)
magEntry = tk.Entry(motorui.master, width=6)
magEntry.grid(row=ROW_NUMBER, column=1)



ROW_NUMBER += 1
freqlabel = tk.Label(motorui.master, text="Freq (hz):")
freqlabel.grid(row=ROW_NUMBER, column=0)
freqEntry = tk.Entry(motorui.master, width=6)
freqEntry.grid(row=ROW_NUMBER, column=1)





ROW_NUMBER += 1
enable_button = tk.Button(motorui.master,text="Spin",command=lambda: [motorui.spincheck(float(canIDEntry.get()),float(kPEntry.get()), float(kDEntry.get()), float(kIEntry.get()),float(magEntry.get()), float(freqEntry.get()))])
enable_button.grid(row=ROW_NUMBER, column=0, padx=20, pady=20)


enable2_button = tk.Button(motorui.master,text="Stop",command=lambda: [motorui.setstatus(False), change_label_contents(spinstatuslabel, "Spin Status: OFF"),motorui.disableMotor()])

enable2_button.grid(row=ROW_NUMBER, column=1, padx=20, pady=20)

spinstatuslabel = tk.Label(motorui.master, text="Spin Status: OFF")
spinstatuslabel.grid(row=ROW_NUMBER, column=2)



motorui.master.after(1000, motorui.serialloop)
tk.mainloop()