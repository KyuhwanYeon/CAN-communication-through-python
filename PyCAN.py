#%% Receive CAN and draw the animation plot
# Author: Kyuhwan
# Project: Ego-vehicle speed prediction using long short-term memory based recurrent neural network
# Date: 2018-12-07
# Description: This script could be used as reference for how to receive the CAN signal through the CANoe with timer interrupt
#              Also, draw the animation plot

# import the library
import can
import struct
import numpy as np
import time
import copy
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from apscheduler.schedulers.background import BackgroundScheduler
#%% CAN configuration
# Set ID
ID1 = 0x5B1
ID2 = 0x5B2
ID3 = 0x5B3
ID4 = 0x5B4
ID5 = 0x5C0
WhlSpdID = 0x386
# Set can interfase, using the vector, CANoe 
bus1 = can.interface.Bus(bustype='vector', app_name='CANoe', can_filters=[{"can_id": ID1, "can_mask": 0xFFF, "extended": False}], channel=0, bitrate=500000, rx_queue_size=10000, receive_own_messages=False)
bus2 = can.interface.Bus(bustype='vector', app_name='CANoe', can_filters=[{"can_id": ID2, "can_mask": 0xFFF, "extended": False}], channel=0, bitrate=500000, rx_queue_size=10000, receive_own_messages=False)
bus3 = can.interface.Bus(bustype='vector', app_name='CANoe', can_filters=[{"can_id": ID3, "can_mask": 0xFFF, "extended": False}], channel=0, bitrate=500000, rx_queue_size=10000, receive_own_messages=False)
bus4 = can.interface.Bus(bustype='vector', app_name='CANoe', can_filters=[{"can_id": ID4, "can_mask": 0xFFF, "extended": False}], channel=0, bitrate=500000, rx_queue_size=10000, receive_own_messages=False)
bus5 = can.interface.Bus(bustype='vector', app_name='CANoe', can_filters=[{"can_id": ID5, "can_mask": 0xFFF, "extended": False}], channel=0, bitrate=500000, rx_queue_size=10000, receive_own_messages=False)
bus_Measured = can.interface.Bus(bustype='vector', app_name='CANoe', can_filters=[{"can_id": WhlSpdID, "can_mask": 0xFFF, "extended": False}], channel=0, bitrate=500000, rx_queue_size=10000, receive_own_messages=False)

PrdSpdArr = np.zeros(15) 
PreSpdArr = np.zeros(15)
plt.close('all')
fig = plt.figure(figsize=(10,5))
ax1 = fig.add_subplot(1,1,1)   
ax1.grid()  
text_template = 'Calculation time = %.5fs'
CalTime_text=ax1.text(0.55,0.05,'',transform = ax1.transAxes,fontsize = 18)
line, = ax1.plot([], [], color="r",lw = 0.2,marker = '*', markersize = 4)
global CalTimeArr 
CalTimeArr = []
TempCalculationTime = 0
#%% Function for animation plot
def animate(i):
    global TimeArr
    global SpdArr
    global count
    global PredictedSpdArr
    global PrdTimeArr
    global CalculationTime
   
    if count >29:       
        ax1.plot(TimeArr, SpdArr,'--',lw = 0.4,color="black" ,zorder=10,label="True" if count == 30 else "")
        plotPrdTimeArr = []
        plotPrdTimeArr.extend([TimeArr[-1]])
        plotPrdTimeArr.extend(PrdTimeArr)   
        plotPrdSpdArr = []
        plotPrdSpdArr.extend([SpdArr[-1]])
        plotPrdSpdArr.extend(PredictedSpdArr)   
        ax1.set_xlim([TimeArr[-20],TimeArr[-1]+20])
        ax1.set_ylim([0,80])
        

        line.set_data(plotPrdTimeArr, plotPrdSpdArr)
    ax1.set_title('Ego-vehicle speed prediction',fontsize = 18)
    ax1.set_xlabel('Time [sec]', fontsize = 18)
    ax1.set_ylabel('Speed [km/h]',fontsize = 18)
    ax1.xaxis.set_tick_params(labelsize=14)
    ax1.yaxis.set_tick_params(labelsize=14)

    CalTime_text.set_text(text_template % CalculationTime)
    CalTimeArr.append(CalculationTime)
    np.savetxt('CalTimeArr.txt', CalTimeArr)

    print ('Mean of Cal TimeArr : ',np.mean(CalTimeArr))
    print ('Max of Cal TimeArr : ',np.max(CalTimeArr))
    

    return line, CalTime_text
#%% Function for receive CAN
def ReadCAN():
    global TimeArr
    global SpdArr
    global count
    global PredictedSpdArr
    global PrdTimeArr
    global CalculationTime

    # Receive massage
    recvMsg1 = bus1.recv(timeout=None)
    recvMsg2 = bus2.recv(timeout=None)
    recvMsg3 = bus3.recv(timeout=None)
    recvMsg4 = bus4.recv(timeout=None)
    recvMsg5 = bus5.recv(timeout=None)
    # Unpack the massage
    # Please read the struct document on the internet and CAN signal layout of yours
    UnpackedMsg1 = struct.unpack('<HHHH',recvMsg1.data)
    UnpackedMsg2 = struct.unpack('<HHHH',recvMsg2.data)
    UnpackedMsg3 = struct.unpack('<HHHH',recvMsg3.data)
    UnpackedMsg4 = struct.unpack('<HHHH',recvMsg4.data)
    UnpackedMsg5 = struct.unpack('<II',recvMsg5.data)
    idx = 0
    for idx, val in enumerate(UnpackedMsg1):
        PrdSpdArr[idx] = val*0.03125
    for idx, val in enumerate(UnpackedMsg2):
        PrdSpdArr[idx+4] = val*0.03125             
    for idx, val in enumerate(UnpackedMsg3):
        PrdSpdArr[idx+8] = val*0.03125             
    for idx, val in enumerate(UnpackedMsg4[0:3]):
        PrdSpdArr[idx+12] = val*0.03125             
    VehSpd = UnpackedMsg4[3]*0.03125     
    CalculationTime = UnpackedMsg5[0]*0.0001

    TimeArr.append(count)
    SpdArr.append(VehSpd)
    PredictedSpdArr =  copy.copy(PrdSpdArr)
    for i in range(15):
        PrdTimeArr[i] = count +i +1
        
    count += 1
#%% Main
if __name__ == '__main__':
    global TimeArr
    TimeArr = []
    global SpdArr
    SpdArr = []
    global count
    count = 0
    global PredictedSpdArr
    PredictedSpdArr = []    
    global PrdTimeArr
    PrdTimeArr = np.zeros(15)   
    global CalculationTime
    CalculationTime = 0 
     # Read the CAN signal every seconds
    sched = BackgroundScheduler()
    sched.start()
    sched.add_job(ReadCAN,'interval',seconds = 1)    
    # Draw the animation plot
    ani = animation.FuncAnimation(fig, animate, interval=20)
    plt.show()


