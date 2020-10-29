# /*
# ****************************************************************************
# * plottrajectorytracking.py:
# * Illustration of results of trajectory tracking results
# *
# * by Hu.ZH(CrossOcean.ai)
# ****************************************************************************
# */

import sqlite3
import pandas as pd
import matplotlib.pyplot as plt
import math
import numpy as np


## 参数 ##
DBNAME = 'Sun Dec  8 10:37:36 2019.db'
num_thruster = 2
ACTUATOR = 'underactuated'  # underactuated
time_stamp_select = np.array([120, 300])

##


# connect to sqlite database
db_conn = sqlite3.connect(DBNAME)
# create a cursor to execute SQL commands
db_cursor = db_conn.cursor()

#  retrieve controller data from database
db_cursor.execute("SELECT * FROM controller")
controller_rows = db_cursor.fetchall()

#  retrieve estimator data from database
db_cursor.execute("SELECT * FROM estimator")
estimator_rows = db_cursor.fetchall()

# #  retrieve planner data from database
# db_cursor.execute("SELECT * FROM route_planning")
# route_planner_rows = db_cursor.fetchall()

# Save (commit) the changes
db_conn.commit()

# We can also close the connection if we are done with it.
# Just be sure any changes have been committed or they will be lost.
db_conn.close()


# db_conn = sqlite3.connect('../data/wp.db')
# db_cursor = db_conn.cursor()
# db_cursor.execute("SELECT * FROM WP")
# wp_rows = db_cursor.fetchall()
# db_conn.commit()
# db_conn.close()
# wpdata = pd.DataFrame(wp_rows)
# wpdata.columns = ['ID', 'X', 'Y']
# convert lists to dataframe


controllerdata = pd.DataFrame(controller_rows)
namelist = ['ID', 'DATETIME', 'set_x', 'set_y', 'set_theta',
            'set_u', 'set_v', 'set_r', 'tau_x', 'tau_y', 'tau_mz']
for i in range(num_thruster):
    namelist.append('alpha'+str(i+1))
for i in range(num_thruster):
    namelist.append('rpm'+str(i+1))
namelist.append('est_x')
namelist.append('est_y')
namelist.append('est_mz')
controllerdata.columns = namelist
controllerdata['DATETIME'] = controllerdata['DATETIME'].astype(float)


estimatordata = pd.DataFrame(estimator_rows)
estimatordata.columns = ['ID', 'DATETIME',
                         'meas_x', 'meas_y', 'meas_theta',
                         'meas_u', 'meas_v', 'meas_r',
                         'state_x', 'state_y', 'state_theta',
                         'state_u', 'state_v', 'state_r',
                         'perror_x', 'perror_y', 'perror_mz',
                         'verror_x', 'verror_y', 'verror_mz',
                         'curvature', 'speed', 'dspeed']
estimatordata['DATETIME'] = estimatordata['DATETIME'].astype(float)


# routeplannerdata = pd.DataFrame(route_planner_rows)
# routeplannerdata.columns = ['ID', 'DATETIME',
#                             'command_x', 'command_y', 'command_theta',
#                             'wp0_x', 'wp0_y', 'wp1_x', 'wp1_y']

# time stamp
timestamp = min(controllerdata["DATETIME"].loc[0],
                estimatordata["DATETIME"].loc[0])
controllerdata['DATETIME'] = (controllerdata['DATETIME']-timestamp)*86400
estimatordata['DATETIME'] = (estimatordata['DATETIME']-timestamp)*86400


# 选择显示的时间段

estimatordata = estimatordata[
    (estimatordata['DATETIME'] > time_stamp_select[0])
    & (estimatordata['DATETIME'] < time_stamp_select[1])]
controllerdata = controllerdata[
    (controllerdata['DATETIME'] > time_stamp_select[0])
    & (controllerdata['DATETIME'] < time_stamp_select[1])]


# results of 2d trajectory of vessel
plt.figure(2, figsize=(10, 8))
plt.suptitle("Plannar trajectory of GPS", fontsize=14)
plt.plot(estimatordata["meas_y"]-3.509e5,
         estimatordata["meas_x"]-3.4337e6, 'ko', lw=2, markersize=1)
# plt.plot(wpdata['Y'], wpdata['X'],
#          color='tab:gray', lineStyle='-', lw=1)
plt.axis('equal')
plt.xlabel('E (m)')
plt.ylabel('N (m)')


plt.savefig('trajectory.png')

############ time series of 3DoF velocity #############
plt.figure(4, figsize=(15, 10))
plt.suptitle("time series of speed", fontsize=12)
plt.subplot(3, 1, 1)
plt.plot(estimatordata['DATETIME'], estimatordata['state_u'], '-r', lw=2)
plt.plot(controllerdata['DATETIME'], controllerdata['set_u'],
         color='tab:blue', lineStyle='-', lw=2)
plt.ylabel('u (m/s)')
plt.legend(('estimated', 'desired'), loc='upper right')

plt.subplot(3, 1, 2)
plt.plot(estimatordata['DATETIME'],
         estimatordata['state_theta']*180/math.pi, '-r', lw=2)
plt.plot(controllerdata['DATETIME'], controllerdata['set_theta']*180/math.pi,
         color='tab:blue', lineStyle='-', lw=2)
plt.ylabel('theta (deg)')
plt.legend(('estimated', 'desired'), loc='upper right')


plt.subplot(3, 1, 3)
plt.plot(estimatordata['DATETIME'], estimatordata['state_r'], '-r', lw=2)
plt.plot(controllerdata['DATETIME'], controllerdata['set_r'],
         color='tab:blue', lineStyle='-', lw=2)
plt.ylabel('r (rad/s)')
plt.legend(('estimated', 'desired'), loc='upper right')

plt.savefig('time.png')


plt.show()
