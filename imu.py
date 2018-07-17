import smbus
import time
import numpy as np
from numpy.linalg import norm
import math

i2c = smbus.SMBus(1)

IMU_ADDRESS             = 0x68
MPU9150_PWR_MGMT_1      = 0x6B
MPU9150_GYRO_CONFIG     = 0x1B
MPU9150_ACCEL_CONFIG    = 0x1C
ACCEL_OUT               = 0x3B

def dataConv(data1,data2):
    value = data1 | (data2 << 8)
    if (value & (1 << 16 -1)):
        value -= (1<<16)
    return value
def readAccel(n=5):
    acc = np.array([[0],[0],[0]])
    for i in range(n):
        data = i2c.read_i2c_block_data(IMU_ADDRESS,ACCEL_OUT,6)
        acc_x = round((dataConv(data[1],data[0])/16384.0)*9.82,3)
        acc_y = round((dataConv(data[3],data[2])/16384.0)*9.82,3)
        acc_z = round((dataConv(data[5],data[4])/16384.0)*9.82,3)
        acc = np.add(acc,np.array([[acc_x],[acc_y],[acc_z]]))
    return np.divide(acc,n)
def imu_calibrate(n=2000):
    bias = np.array([[0],[0],[0]])
    for i in range(n):
        data = i2c.read_i2c_block_data(IMU_ADDRESS,ACCEL_OUT,6)
        accel = readAccel(1)
        bias = np.add(bias,accel)
    return np.divide(bias,n)
def imu_init():
    i2c.write_byte_data(IMU_ADDRESS,MPU9150_PWR_MGMT_1,0x00)
    time.sleep(2)
    i2c.write_byte_data(IMU_ADDRESS,MPU9150_GYRO_CONFIG,0x00)
    time.sleep(0.1)
    i2c.write_byte_data(IMU_ADDRESS,MPU9150_ACCEL_CONFIG,0x00)
    time.sleep(0.1)

t1 = time.time()
t2 = 0
vel = np.array([[0],[0],[0]])
imu_init()
acc_bias = imu_calibrate()
print("bias: {}".format(acc_bias))
time.sleep(2)
last_accel = readAccel()-acc_bias
while (t2<15):
    accel = readAccel()-acc_bias
    dacc = accel+last_accel
    vel = vel+dacc*0.001
    abs_vel = norm(vel)
    x,y,z = accel
    print("Vel: {0:.3f},x: {1:.3f},y: {2:.3f},z: {3:.3f}".format(float(abs_vel),float(x),float(y),float(z)))
    time.sleep(0.001)
    last_accel = accel
    t2 = time.time()-t1
