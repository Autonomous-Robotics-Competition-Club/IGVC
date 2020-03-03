"""
This script initiates a serial connection with a flight controller. The data is 
read for a user-specified amount of time and the IMU and GPS data is saved to a
list. Once the data is finished being read it is written to two files, one for
the IMU data and one for the GPS data.
"""

import time
from pymavlink import mavutil

if __name__=='__main__':

    # Initiate serial connection to flight controller
    master = mavutil.mavlink_connection('/dev/ttyACM0', 115200)

    msg_types = []

    imu_f = open('imu.txt', 'w')
    gps_f = open('gps.txt', 'w')

    imu_list = []
    gps_list = []

    time_start = time.time()

    # Amount of time to collect data in seconds
    time_run = 25
    
    # Continue to read serial messages until time expires
    while time.time() - time_start <= 25:

        # Read message from serial connection
        msg = master.recv_match()
        if not msg:
            continue
        
        # Get the type of message
        msg_type = msg.get_type()

        if msg_type not in msg_types:
            msg_types.append(msg_type)

        if msg_type=='HIGHRES_IMU':
            imu_list.append([time.time() - time_start, msg.xacc, msg.yacc, msg.zacc, msg.xgyro, msg.ygyro, msg.zgyro])

        if msg_type=='GPS_RAW_INT':
            gps_list.append([time.time() - time_start, msg.lat, msg.lon, msg.alt])

    # Once data has been collected print to files
    imu_f.write('Time[s], Xacc[m/s/s], Yacc[m/s/s], Zacc[m/s/s], Xgyro[rad/s], Ygyro[rad/s], Zgyro[rad/s]\n')
    for listitem in imu_list:
        listitem = str(listitem)[1:-1]
        imu_f.write(listitem+'\n')

    gps_f.write('Time[s], Latitude [degE7], Longitude [degE7], Altitude [mm]\n')
    for listitem in gps_list:
        listitem = str(listitem)[1:-1]
        gps_f.write(listitem+'\n')

    imu_f.close()
    gps_f.close()
