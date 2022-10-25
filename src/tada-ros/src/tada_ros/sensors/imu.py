'''
        Read Gyro and Accelerometer by Interfacing Raspberry Pi with MPU6050 using Python
    http://www.electronicwings.com
'''
import smbus #import SMBus module of I2C
from time import sleep          #import
import numpy as np
import time
import subprocess
import csv
import time

#some MPU6050 Registers and their Address
PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
INT_ENABLE   = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H  = 0x43
GYRO_YOUT_H  = 0x45
GYRO_ZOUT_H  = 0x47



m1_pos = 0
m2_pos = 0

def MPU_Init():
    #write to sample rate register
    bus.write_byte_data(Device_Address, SMPLRT_DIV, 7)

    #Write to power management register
    bus.write_byte_data(Device_Address, PWR_MGMT_1, 1)

    #Write to Configuration register
    bus.write_byte_data(Device_Address, CONFIG, 0)

    #Write to Gyro configuration register
    bus.write_byte_data(Device_Address, GYRO_CONFIG, 24)

    #Write to interrupt enable register
    bus.write_byte_data(Device_Address, INT_ENABLE, 1)

def read_raw_data(addr):
    #Accelero and Gyro value are 16-bit
        high = bus.read_byte_data(Device_Address, addr)
        low = bus.read_byte_data(Device_Address, addr+1)
    
        #concatenate higher and lower value
        value = ((high << 8) | low)
        
        #to get signed value from mpu6050
        if(value > 32768):
                value = value - 65536
        return value
    
#def move_command(time):
    #command = str(time)
#     command = "sudo ./red_test 1 " + str(time) + " 200 200 500 500"
    #print(command) #UNCOMMENT LATER!!!!
#     subprocess.Popen(command)
    #subprocess.Popen(['sudo', './red_test', '2', command, '292', '292', '1000', '1000'])
    #return(m1_pos, m2_pos)
    
bus = smbus.SMBus(1)  # or bus = smbus.SMBus(0) for older version boards
Device_Address = 0x68   # MPU6050 device address

MPU_Init()

# Initial variables
state = 0
start = time.time()
start_time = time.time()
initial_itr = 0
swing_time = 0
swing = [0, 0, 0]
avg_swing = [0.7, 0.7, 0.7]
avg_val_swing = 0
initial_itr1 = 0
gyro_thres = 5
accel_thres = 0.5
state=0
swing_time=0
# open the file in the write mode
f = open('/home/pi/TADA_IMU/data,csv', 'w')

# create the csv writer
writer = csv.writer(f)


print (" Reading Data of Gyroscope and Accelerometer")
n=0
while n<30:

    #Read Accelerometer raw value
    acc_x = read_raw_data(ACCEL_XOUT_H)
    acc_y = read_raw_data(ACCEL_YOUT_H)
    acc_z = read_raw_data(ACCEL_ZOUT_H)

    #Read Gyroscope raw value
    gyro_x = read_raw_data(GYRO_XOUT_H)
    gyro_y = read_raw_data(GYRO_YOUT_H)
    gyro_z = read_raw_data(GYRO_ZOUT_H)

    #Full scale range +/- 250 degree/C as per sensitivity scale factor
    Ax = acc_x/16384.0
    Ay = acc_y/16384.0
    Az = acc_z/16384.0

    Gx = gyro_x/131.0
    Gy = gyro_y/131.0
    Gz = gyro_z/131.0
    
#print(Gz)

# write a row to the csv file
    row = [Ax, Ay, Az, Gx, Gy, Gz, state, swing_time, time.time()-start]
    writer.writerow(row)

    # Print raw values of IMU
 #   print("Gx=%.2f" %Gx, u'\u00b0'+ "/s", "\tGy=%.2f" %Gy, u'\u00b0'+ "/s", "\tGz=%.2f" %Gz, u'\u00b0'+ "/s", "\tAx=%.2f g" %Ax, "\tAy=%.2f g" %Ay, "\tAz=%.2f g" %Az)
   # print("Gx=%.2f" %Gx, "Gy=%.2f" %Gy, "Gz=%.2f" %Gz, "Ax=%.2f g" %Ax, "Ay=%.2f g" %Ay, "Az=%.2f g" %Az)
   
    #print("%.2f," %Gz)
    sleep(0.01)

    #gyro_mag = np.linalg.norm([Gx, Gy, Gz])
    #accel_mag = np.linalg.norm([Ax, Ay, Az])
#     print(gyro_mag, accel_mag)
#     print(Gx, Gy, Gz)

# if gyro vel is above threshold, then collect swing data
    if Gz < gyro_thres: # stance
        initial_itr = 0
        # collect the swing time and save the data only once
        if initial_itr == 0:
            state = 0
#             print(swing_time)
            start_time = time.time()
            swing.append(swing_time)
            avg_swing = swing[3:]
            avg_val_swing = np.mean(avg_swing)
            swing_time = 0
            initial_itr = 1
            initial_itr1 = 0
        else: # to ensure that swing is only appended once
            state = 0
    # when not saving data, move the motor at the first itr
    # and collect the swing timeb
    else: # swing
        if (initial_itr1==0):
            avg_swing_command = int(1000 * avg_val_swing)
#             move_command(avg_swing_command)
            # sleep(avg_val_swing+0.3)
            initial_itr1 = 1
        else:
            state = 1
            swing_time = time.time() - start_time     

    #print("avg swing time= " + str(avg_swing_command))
    #print("%d, %.2f" %(state, swing_time))
    n+=1

# close the file
f.close()
 

 
