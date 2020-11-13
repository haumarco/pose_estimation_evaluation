#!/usr/bin/python3

import numpy as np
import sys
import yaml

import os
import traceback

def read_estimation_robot(test_number, est_type):
	# read estimation from robot

	data = np.genfromtxt("./" + test_number + "/"+ est_type + "_data" + test_number + ".txt", delimiter = ",", skip_header=1)

	data = data[:, [0, 2, 4, 7]] # est_time, stamp_time, d, phi
	#print(data[0,0]- data[1,1])

	t_est = data[:,0] / 1e9
	delay = data[:,0] - data[:,1]
	d = data[:,2]
	phi_est = data[:,3]

	return(t_est, phi_est)

###

def read_truth_localization(test_number):

	with open("./"+test_number+"/autobot06.yaml", 'r') as stream:


		data = yaml.safe_load(stream)


		timestart = data['begin_time_stamp']
		data_points = len(data['trajectory_data'])
		
		x = np.zeros((data_points,))
		y= np.zeros((data_points,))
		R = np.zeros((3,3, data_points))            
		phi = np.zeros((3, data_points))

		dx = 999.999*np.ones((data_points, ))
		dy = 999.999*np.ones((data_points, ))
		dr = 999.999*np.ones((data_points, ))
		dphi = 999.999*np.ones((data_points, ))

		final_trajectory = []
		save_time_stamps = np.zeros((0,1))
		#print(data['trajectory_data'].items())
		for idx, [time, traj] in enumerate(data['trajectory_data'].items()):
			x[idx] = np.array(traj[0])
			y[idx] = np.array(traj[1])
			
			save_time_stamps = np.append(save_time_stamps, time)
			#print(time)
			
			R[:,:,idx] = np.reshape(np.array(traj[3:]), (3,3))
			phi[:,idx] = np.array([np.arctan2(-R[1,2,idx],R[2,2,idx]), 
								np.arctan2(R[0,2,idx],np.sqrt(R[0,0,idx]**2 + R[0,1,idx]**2)),
								np.arctan2(-R[0,1,idx], R[0,0,idx])])
			
			z = phi[2,idx]
			#print(timestart.keys())
			points = np.array([x[idx], y[idx]])
			final_trajectory.append([points, z])
		final_array = final_trajectory
		


	x_true = np.zeros((0,1))
	y_true = np.zeros((0,1))
	alpha_true = np.zeros((0,1))
	t_true = np.zeros((0,1))

	for entry in range(0, len(final_array)):
		x =  (final_array[entry][0][0] )   #-2.2
		y = final_array[entry][0][1]  #+ 0.8
		alpha = final_array[entry][1]

		x_true = np.append(x_true, x)
		y_true = np.append(y_true, y)
		alpha_true = np.append(alpha_true, alpha)
		t_true = np.append(t_true, save_time_stamps[entry])
		#print("%s %s %s %s %s" %(x,y,alpha, entry, save_time_stamps[entry]))
		

	#copy from notebook
	return(t_true, timestart, x_true, y_true, alpha_true)


def write_to_file(test_number, est_type, mean, stdev):
	f = open("./evaluation_results_3x3.txt", "a")
	f.write(str(test_number)+ "," + str(est_type) + "," + str(mean)  + "," + str(stdev)+"\n")
	f.close()
	return


def evaluate_estimation(phi_est, t_est, t_all, t_0, x, y, alpha):
	a=0.6
	b=1.185
	phi_true = np.zeros((0,1))
	t_true = np.zeros((0,1))


	for i in range(len(t_all)):
	#while i < len(t_true):
		#sector 1:
		if x[i] > a and x[i] < b and y[i] < a+0.1:
			#print("1-")
			if abs(alpha[i]) > np.pi/2:
				alpha[i] -= np.sign(alpha[i]) * np.pi
			#	print("1")

		#sector 2:
		elif x[i] > b - 0.1 and y[i] > a and y[i] < b:
			alpha[i] -= np.sign(alpha[i]) * np.pi/2
			#print("2")
		#sector 3
		elif x[i] > a and x[i] < b and y[i] > b -0.1:
			if abs(alpha[i]) > np.pi/2:
				alpha[i] -= np.sign(alpha[i]) * np.pi
				#print("3")
		#sector 4
		elif x[i] < a + 0.1 and y[i] > a and y[i] < b:
			alpha[i] -= np.sign(alpha[i]) * np.pi/2
			#print("4 %s %s %s" %(x[i], y[i], alpha[i]))

		else:
			#print("curve%s %s" %(x[i], y[i]))
			continue


		phi_true = np.append(phi_true, alpha[i])
		t_true = np.append(t_true, float(t_all[i]) + t_0)


	#print(phi_true)
	#print(t_true)



	u = 0
	i = 0
	phi_comp = np.zeros((0,1))
	while i < len(t_true):
		if t_true[i] < t_est[u]:
			phi_comp = np.append(phi_comp, (phi_est[u] - phi_est[u-1]) / (t_est[u] - t_est[u-1]) * (t_true[i] - t_est[u-1]) + phi_est[u-1])
			p = (phi_est[u] - phi_est[u-1]) / (t_est[u] - t_est[u-1]) * (t_true[i] - t_est[u-1]) + phi_est[u-1]
			#print(t_true[i], t_est[u])
			#print("vor %s interpl %s nach %s" %(phi_est[u-1],p,phi_est[u]))
			i += 1
		
		else:
			u += 1

	#print(phi_comp)

	error = phi_comp - phi_true
	#print(phi_true)

	mean = np.mean(error)
	stdev = np.std(error)
	print(mean, stdev)
	return(mean, stdev)


if __name__ ==	"__main__":

	test_number = sys.argv[1]

	for w in (["SF", "cam"]):
		print(w)

		t_est, phi_est = read_estimation_robot(test_number, w)

		t_all, t_0, x, y, alpha = read_truth_localization(test_number)

		mean, stdev = evaluate_estimation(phi_est, t_est, t_all, t_0, x, y, alpha)

		write_to_file(test_number, w, mean, stdev)