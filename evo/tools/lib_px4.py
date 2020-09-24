import numpy as np
import csv
import matplotlib.pyplot as plt
import tf.transformations as tf
import pyulog

# NOTE: Pixhawk uses the Hamilton convention for quaternions, i.e. q = [w, x, y, z]
# tf uses the JPL convention, i.e. q = [x, y, z, w]

# Notes on frames:
# In ROS we use the NWU convention, i.e. x points along the 'front' of the vehicle,
# y to the left, and z up.
# For position coordinates, PX4 uses WND

class px4Log:
	def __init__(self, path_dir, filename):
		self.filename = filename
		self.path_dir = path_dir
		self.ulog = pyulog.ULog(self.path_dir + '/' + self.filename + '/' + self.filename + '.ulg')
		# position of IMU in COM frame
		self.imu_pos = np.array([self.ulog.initial_parameters['EKF2_IMU_POS_X'],
								 self.ulog.initial_parameters['EKF2_IMU_POS_Y'],
								 self.ulog.initial_parameters['EKF2_IMU_POS_Z']])
		# Create empty arrays to avoid exceptions:
		self.lpe = np.zeros([1,10]) 
		self.pos_sp = np.zeros([1,10]) 
		self.att = np.zeros([1,10]) 
		self.att_sp = np.zeros([1,10]) 
		self.att_euler = np.zeros([1,10]) 
		self.att_sp_euler = np.zeros([1,10]) 
		self.tilt_angles = np.zeros([1,10]) 
		self.imu_data = np.zeros([1,10]) 
		self.gyro_data = np.zeros([1,10]) 
		self.wrench_sp = np.zeros([1,10]) 
		self.batt_voltage = np.zeros([1,10]) 
		self.rotor_speeds = np.zeros([2,1,10])
		self.vo_data = np.zeros([1,8])
		self.accel_and_gyro = np.zeros([1,10])
	def readlpe(self):
		lpe = []
		path = self.path_dir + '/' + self.filename + '/' + self.filename + '_vehicle_local_position_0.csv'
		try:
			with open(path,'rb') as csvfile:
				reader = csv.reader(csvfile, delimiter=',')
				i=0
				for row in reader:
					if i>0:
						lpe.append(row)
					i=i+1
			self.lpe = np.array(lpe).astype(float)		# px4 timestamp is in ms
		except:
			print "\x1b[0;30;41mCould not find local position logfile.\x1b[0m"
	def readPosSetpoint(self):
		pos_sp = []
		path = self.path_dir + '/' + self.filename + '/' + self.filename + '_position_setpoint_triplet_0.csv'
		try:
			with open(path,'rb') as csvfile:
				reader = csv.reader(csvfile, delimiter=',')
				i=0
				for row in reader:
					if i>0:
						pos_sp.append(row)
					i=i+1
			self.pos_sp = np.array(pos_sp).astype(float)		# px4 timestamp is in ms
		except:
			print "\x1b[0;30;41mCould not find position setpoint logfile.\x1b[0m"
	def readAttSetpoint(self):
		att_sp = []
		path = self.path_dir + '/' + self.filename + '/' + self.filename + '_vehicle_attitude_setpoint_0.csv'
		try:
			with open(path,'rb') as csvfile:
				reader = csv.reader(csvfile, delimiter=',')
				i=0
				for row in reader:
					if i>0:
						att_sp.append(row)
					i=i+1
			self.att_sp = np.array(att_sp).astype(float)		# px4 timestamp is in ms
			n = np.size(self.att_sp,0)
			self.att_sp_euler = np.zeros([n,6])
			for i in range(0,n):
				self.att_sp_euler[i,0:3] = tf.euler_from_quaternion(np.hstack([self.att_sp[i,6:9],self.att_sp[i,5:6]]))
				self.att_sp_euler[i,3:6] = self.att_sp[i,1:4]
			self.att_sp_euler = np.unwrap(self.att_sp_euler, axis=0)
		except:
			print "\x1b[0;30;41mCould not find attitude setpoint logfile.\x1b[0m"
	def readAttitude(self):
		att = []
		path = self.path_dir + '/' + self.filename + '/' + self.filename + '_vehicle_attitude_0.csv'

		try:
			with open(path,'rb') as csvfile:
				reader = csv.reader(csvfile, delimiter=',')
				i=0
				for row in reader:
					if i>0:
						att.append(row)
					i=i+1
			self.att = np.array(att).astype(float)		# px4 timestamp is in ms
			n = np.size(self.att,0)
			self.att_euler = np.zeros([n,3])
			for i in range(0,n):
				q = np.hstack([self.att[i,5:8],self.att[i,4:5]])
				# q_rot = np.array([0.707,0.707,0,0])
				# q = tf.quaternion_multiply(q_rot,q)
				self.att_euler[i] = tf.euler_from_quaternion(q)
			self.att_euler = np.unwrap(self.att_euler, axis=0)
		except:
			print "\x1b[0;30;41mCould not find attitude logfile.\x1b[0m"
	def readTiltAngles(self):
		tilt_angles = []
		path = self.path_dir + '/' + self.filename + '/' + self.filename + '_actuator_controls_2_0.csv'
		with open(path,'rb') as csvfile:
			reader = csv.reader(csvfile, delimiter=',')
			i=0
			for row in reader:
				if i>0:
					tilt_angles.append(row[1:])
				i=i+1
		self.tilt_angles = np.array(tilt_angles).astype(float)		# px4 timestamp is in us
	def readRotorSpeeds(self):
		rotor_speeds_0 = []
		rotor_speeds_1 = []
		path_0 = self.path_dir + '/' + self.filename + '/' + self.filename + '_actuator_controls_0_0.csv'
		path_1 = self.path_dir + '/' + self.filename + '/' + self.filename + '_actuator_controls_1_0.csv'
		with open(path_0,'rb') as csvfile:
			reader = csv.reader(csvfile, delimiter=',')
			i=0
			for row in reader:
				if i>0:
					rotor_speeds_0.append(row[1:])
				i=i+1
		with open(path_1,'rb') as csvfile:
			reader = csv.reader(csvfile, delimiter=',')
			i=0
			for row in reader:
				if i>0:
					rotor_speeds_1.append(row[1:])
				i=i+1
		self.rotor_speeds = np.array([np.array(rotor_speeds_0).astype(float), np.array(rotor_speeds_1).astype(float)])		# px4 timestamp is in us
	def readIMU_0(self):
		imu_data = []
		path = self.path_dir + '/' + self.filename + '/' + self.filename + '_sensor_accel_0.csv'
		with open(path,'rb') as csvfile:
			reader = csv.reader(csvfile, delimiter=',')
			i=0
			for row in reader:
				if i>0:
					imu_data.append(row)
				i=i+1
		self.imu_data = np.array(imu_data).astype(float)		# px4 timestamp is in us
	def readIMU_gyro(self):
		gyro_data = []
		path = self.path_dir + '/' + self.filename + '/' + self.filename + '_sensor_gyro_0.csv'
		with open(path,'rb') as csvfile:
			reader = csv.reader(csvfile, delimiter=',')
			i=0
			for row in reader:
				if i>0:
					gyro_data.append(row)
				i=i+1
		self.gyro_data = np.array(gyro_data).astype(float)		# px4 timestamp is in us

	def readIMU_combined(self):
		accel_and_gyro_data = []
		path = self.path_dir + '/' + self.filename + '/' + self.filename + '_sensor_combined_0.csv'
		with open(path, 'rb') as csvfile:
			reader = csv.reader(csvfile, delimiter=',')
			i = 0
			for row in reader:
				if i > 0:
					accel_and_gyro_data.append(row)
				i = i + 1
		self.accel_and_gyro = np.array(accel_and_gyro_data).astype(float)  # px4 timestamp is in us

	def readVisualOdometry(self):
		vo_data = []
		path = self.path_dir + '/' + self.filename + '/' + self.filename + '_vehicle_visual_odometry_0.csv'
		with open(path,'rb') as csvfile:
			reader = csv.reader(csvfile, delimiter=',')
			i=0
			for row in reader:
				if i>0:
					vo_data.append(row[:8])
				i=i+1
		self.vo_data = np.array(vo_data).astype(float)		# px4 timestamp is in us
	def readBatteryVoltage(self):
		batt_voltage = []
		path = self.path_dir + '/' + self.filename + '/' + self.filename + '_battery_status_0.csv'
		with open(path,'rb') as csvfile:
			reader = csv.reader(csvfile, delimiter=',')
			i=0
			for row in reader:
				if i>0:
					batt_voltage.append(row)
				i=i+1
		self.batt_voltage = np.array(batt_voltage).astype(float)		# px4 timestamp is in us
	def readWrenchSetpoint(self):
		wrench_sp = []
		path = self.path_dir + '/' + self.filename + '/' + self.filename + '_wrench_setpoint_0.csv'
		with open(path,'rb') as csvfile:
			reader = csv.reader(csvfile, delimiter=',')
			i=0
			for row in reader:
				if i>0:
					wrench_sp.append(row)
				i=i+1
		self.wrench_sp = np.array(wrench_sp).astype(float)		# px4 timestamp is in us
	def plotTiltAngles(self):
		# t0 = self.tilt_angles[0,0]
		plt.plot(self.tilt_angles[:,0], self.tilt_angles[:,1:7])
		plt.xlabel('t [s]')
		plt.ylabel('angle [rad]')
		plt.title('Tilt angles')
		#plt.legend(('x_{MSF}','y_{MSF}','z_{MSF}','x_{lpe}','y_{lpe}','z_{lpe}'))
		plt.grid()
	def plotRotorSpeeds(self):
		# t0 = self.tilt_angles[0,0]
		plt.subplot(211)
		plt.plot(self.rotor_speeds[0][:,0], self.rotor_speeds[0][:,1:7])
		plt.xlabel('t [s]')
		plt.ylabel('Rotor speed normalized [1]')
		plt.title('Rotor speeds 0')
		#plt.legend(('x_{MSF}','y_{MSF}','z_{MSF}','x_{lpe}','y_{lpe}','z_{lpe}'))
		plt.grid()
		plt.subplot(212)
		plt.plot(self.rotor_speeds[1][:,0], self.rotor_speeds[1][:,1:7])
		plt.xlabel('t [s]')
		plt.ylabel('Rotor speed normalized [1]')
		plt.title('Rotor speeds 1')
		plt.grid()
	def plotPosSetpoint(self):
		# t0 = self.pos_sp[0,0]
		plt.plot(self.pos_sp[:,0], self.pos_sp[:,44:47])
		#plt.legend(('x_{MSF}','y_{MSF}','z_{MSF}','x_{lpe}','y_{lpe}','z_{lpe}'))
		plt.grid()
	def plotPosSetpointAndTiltAngles(self):
		plt.subplot(2,1,1)
		# t0 = self.pos_sp[0,0]
		plt.plot(self.pos_sp[:,0], self.pos_sp[:,44:47])
		#plt.legend(('x_{MSF}','y_{MSF}','z_{MSF}','x_{lpe}','y_{lpe}','z_{lpe}'))
		plt.grid()
		plt.subplot(2,1,2)
		plt.plot(self.tilt_angles[:,0], self.tilt_angles[:,2:8])
		plt.grid()
	def plotBatteryVoltage(self):
		plt.plot(self.batt_voltage[:,0]/1e6,self.batt_voltage[:,1:3])
		plt.legend(['V_raw','V_filtered'])
		plt.title("Voltage raw and filtered")
		plt.grid()
