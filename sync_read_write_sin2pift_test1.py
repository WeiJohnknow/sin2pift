import time
import numpy as np
from dynamixel_sdk import *                    # Uses Dynamixel SDK library

# Control table address
ADDR_PRO_TORQUE_ENABLE      = 64               # Control table address is different in Dynamixel model
ADDR_PRO_GOAL_POSITION      = 116
ADDR_PRO_PRESENT_POSITION   = 132

# Data Byte Length
LEN_PRO_GOAL_POSITION       = 4
LEN_PRO_PRESENT_POSITION    = 4

# Protocol version
PROTOCOL_VERSION            = 2.0               # See which protocol version is used in the Dynamixel

# Default setting
DXL1_ID                     = 2                 # Dynamixel#1 ID : 1
DXL2_ID                     = 3              # Dynamixel#1 ID : 2
BAUDRATE                    = 3000000             # Dynamixel default baudrate : 57600
DEVICENAME                  = 'COM9'    # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque
DXL_MOVING_STATUS_THRESHOLD = 20                # Dynamixel moving status threshold



portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)


# Open port
portHandler.openPort()
# Set port baudrate
portHandler.setBaudRate(BAUDRATE)   
#Q=int(input('馬達控制參數如下 : \n1.PWM、velocity、position\n2.velocity、position\n3.PID\n請輸入控制方案為 : '))
#print('控制方案:',Q)
groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, 112, 8)
'''
if Q==1:
    groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, 100, 20)#寫入PWM、Velocity、Position
elif Q==2:
    groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, 112, 8)#寫入Velocity、Position
elif Q==3:
    groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, 112, 8)#寫入PID
'''


#讀取目標位置、速度、電流
groupSyncRead = GroupSyncRead(portHandler, packetHandler, 126, 10)  



 
#%%

# Enable Dynamixel#1 Torque
packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)

# Enable Dynamixel#2 Torque
packetHandler.write1ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)

# Add parameter storage for Dynamixel#1 present position value
groupSyncRead.addParam(DXL1_ID)


# Add parameter storage for Dynamixel#2 present position value
groupSyncRead.addParam(DXL2_ID)

#%%
T=10
dt=0.1
deg=90
t=0
f=1.0/T
pos=[0]
sp=0

data_vel = []
data_pos = []
count = 0
while True:
    
     
    rad = np.deg2rad(deg*(np.sin(2*np.pi*f*t)))
    #將收集的弳度資料收集至pos的list裡，並取用倒數第一個值(最新)與倒數第二個值來相減，即可得到位移量。
    pos.append(rad)    
    rad_l =pos[-1]-pos[-2]          #位移量
    rad_s =rad_l/dt                 #位移量/分割時間=轉速            
    rpm=rad_s*60/(2*np.pi)          #將rad/s換成rad/m之後除以2pi會等於rpm
    vel = int(np.abs(rpm/0.229))    #馬達手冊之轉速
    change_pos = int(2048+deg*(np.sin(2*np.pi*f*t)*4095/360))
    if vel == 0:
        vel =1
    if t >= T:
        break
    t+= dt
    print(vel,change_pos)
    data_vel.append(vel)            #紀錄轉速參數
    data_pos.append(change_pos)     #紀錄位置變化
    
m=len(data_vel)
print('list裡共有',m,'個項目')
#%%
while True:
    new_t = time.time()

    # Allocate goal position value into byte array
    #param_goal_PWM_1= [DXL_LOBYTE(DXL_LOWORD(150)), DXL_HIBYTE(DXL_LOWORD(150)), DXL_LOBYTE(DXL_HIWORD(150)), DXL_HIBYTE(DXL_HIWORD(150))]
    param_goal_position_1 = [DXL_LOBYTE(DXL_LOWORD(data_pos[count])), DXL_HIBYTE(DXL_LOWORD(data_pos[count])), DXL_LOBYTE(DXL_HIWORD(data_pos[count])), DXL_HIBYTE(DXL_HIWORD(data_pos[count]))]
    param_Profile_Velocity_1 = [DXL_LOBYTE(DXL_LOWORD(data_vel[count])), DXL_HIBYTE(DXL_LOWORD(data_vel[count])), DXL_LOBYTE(DXL_HIWORD(data_vel[count])), DXL_HIBYTE(DXL_HIWORD(data_vel[count]))]
    
    #param_goal_PWM_2= [DXL_LOBYTE(DXL_LOWORD(150)), DXL_HIBYTE(DXL_LOWORD(150)), DXL_LOBYTE(DXL_HIWORD(150)), DXL_HIBYTE(DXL_HIWORD(150))]
    param_goal_position_2 = [DXL_LOBYTE(DXL_LOWORD(data_pos[count])), DXL_HIBYTE(DXL_LOWORD(data_pos[count])), DXL_LOBYTE(DXL_HIWORD(data_pos[count])), DXL_HIBYTE(DXL_HIWORD(data_pos[count]))]
    param_Profile_Velocity_2 = [DXL_LOBYTE(DXL_LOWORD(data_vel[count])), DXL_HIBYTE(DXL_LOWORD(data_vel[count])), DXL_LOBYTE(DXL_HIWORD(data_vel[count])), DXL_HIBYTE(DXL_HIWORD(data_vel[count]))]
    
    # Add Dynamixel#1 goal position value to the Syncwrite parameter storage  把設備1想要的目標位置加到封包中
    groupSyncWrite.addParam(DXL1_ID, param_Profile_Velocity_1+param_goal_position_1)
    
    
    # Add Dynamixel#2 goal position value to the Syncwrite parameter storage  把設備2想要的目標位置加到封包中
    groupSyncWrite.addParam(DXL2_ID, param_Profile_Velocity_2+param_goal_position_2)
    
    
    count+=1
    time.sleep(0.1)
    future_t = time.time()
    sp=future_t-new_t               #驗證dt是否符合0.1s

    if (dt-sp)>0:                   #時間補償
        time.sleep(dt-sp)
    
    if m == count:
        break
    # Syncwrite goal position  發送封包
    groupSyncWrite.txPacket()
    # Clear syncwrite parameter storage  清除參數
    groupSyncWrite.clearParam()  

    # Syncread present position
    groupSyncRead.txRxPacket()
    # Get Dynamixel#1 present position value
    dxl1_present_current = groupSyncRead.getData(DXL1_ID, 126, 2) 
    dxl1_present_velocity = groupSyncRead.getData(DXL1_ID, 128, 4)
    dxl1_present_position = groupSyncRead.getData(DXL1_ID, 132, 4)
    
    #print("ID2:",msg1)
    # Get Dynamixel#2 present position value
    dxl2_present_current = groupSyncRead.getData(DXL2_ID, 126, 2)
    dxl2_present_velocity = groupSyncRead.getData(DXL2_ID, 128, 4)
    dxl2_present_position = groupSyncRead.getData(DXL2_ID, 132, 4)
    change_Unit_cur_1=0
    change_Unit_cur_2=0
    change_Unit_vel_1=0
    change_Unit_vel_2=0
    
    if dxl1_present_current>=5 and dxl1_present_current>=5:
        change_Unit_cur_1=65535-(dxl1_present_current-1)
        change_Unit_cur_2=65535-(dxl2_present_current-1)
    elif dxl1_present_current>=5:
        change_Unit_cur_1=dxl1_present_current
        change_Unit_cur_2=dxl2_present_current
    elif dxl1_present_velocity>=38 and dxl1_present_velocity>=38:
        change_Unit_vel_1=4294967286-(dxl1_present_velocity-1)
        change_Unit_vel_2=4294967286-(dxl2_present_velocity-1)
    elif dxl1_present_velocity<=38 and dxl1_present_velocity<=38:
        change_Unit_vel_1=change_Unit_vel_1
        change_Unit_vel_2=change_Unit_vel_2
        
    msg1=change_Unit_cur_1,change_Unit_vel_1,dxl1_present_position
    msg2=change_Unit_cur_2,change_Unit_vel_2,dxl2_present_position
    
    print("ID2:",msg1,';','ID3:',msg2)
count=0    

#%%
# Clear syncread parameter storage
groupSyncRead.clearParam()
time.sleep(2)
# Disable Dynamixel#1 Torque
packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)

# Disable Dynamixel#2 Torque
packetHandler.write1ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)


# Close port
portHandler.closePort()
