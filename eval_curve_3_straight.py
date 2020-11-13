#!/usr/bin/python3

import numpy as np
import sys
import yaml
import matplotlib.pyplot as plt

import os
import traceback



def read_estimation_robot(test_number, est_type):
	# read estimation from robot

	data = np.genfromtxt("./" + test_number + "/"+ est_type + "_data" + test_number + ".txt", delimiter = ",", skip_header=1)

	data = data[:, [0, 2, 4, 7]] # est_time, stamp_time, d, phi
	#print(data[0,0]- data[1,1])

	t_est = data[:,1] / 1e9
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
	f = open("./evaluation_results.txt", "a")
	f.write(str(test_number)+ "," + est_type + "," + str(mean)  + "," + str(stdev)+"\n")

	f.close()
	return


def evaluate_estimation(phi_est, t_est, t_all, t_0, x, y, alpha, est_type):

	a=0.6
	b=2.1 #1.185
	phi_true = np.zeros((0,1))
	t_true = np.zeros((0,1))
	t_plot = np.zeros((0,1))
	mean_stdev = np.zeros((0,3))
	k =0


	for i in range(len(t_all)):

		if x[i] > 1.6 and y[i] > a and y[i] < b:
			alpha[i] -= np.sign(alpha[i]) * np.pi/2

		
			phi_true = np.append(phi_true, alpha[i])
			t_true = np.append(t_true, float(t_all[i]) + t_0)
			t_plot = np.append(t_plot, float(t_all[i]))

	i = 1
	print(len(t_true))
	while i < len(t_true)-2:
		phi_true[i] = 0.25 * phi_true[i-1] + 0.5 * phi_true[i] + 0.25 *phi_true[i+1]
		#phi_true[i] = (phi_true[i-1] +phi_true[i] + phi_true[i+1])/3
		i+=1
	#error = phi_true
	#plt.plot(t_true -t_true[0], phi_true, xlabel = "a")
	# plt.set_xlabel('time [s]')
	# plt.set_ylabel('phi_localization [rad]')
	#plt.savefig("./phi_true_push-line.png")
	#print(len(phi_true))


	u=0
	phi_comp = np.zeros((0,1))
	i =0
	while i < len(t_true):
		if i < len(t_true) -1:

			while abs(phi_true[i+1] - phi_true[i]) > 0.1:
				print("delete", t_true[i+1]-t_true[0], phi_true[i+1])
				t_true = np.delete(t_true, i+1)
				phi_true = np.delete(phi_true, i+1)


			if t_true[i+1] - t_true[i] > 0.5:
				t_true = np.insert(t_true, i+1, 0.5 * (t_true[i+1] + t_true[i]))
				phi_true = np.insert(phi_true, i+1, 0.5 * (phi_true[i+1] + phi_true[i]))
	
		if t_true[i] < t_est[u]:
			phi_comp = np.append(phi_comp, (phi_est[u] - phi_est[u-1]) / (t_est[u] - t_est[u-1]) * (t_true[i] - t_est[u-1]) + phi_est[u-1])
			p = (phi_est[u] - phi_est[u-1]) / (t_est[u] - t_est[u-1]) * (t_true[i] - t_est[u-1]) + phi_est[u-1]
			i += 1
		
		else:
			u += 1

	error = phi_comp - phi_true
	
	mean = np.mean(error)
	stdev = np.std(error)

	print(mean, stdev)
	return(mean, stdev)


if __name__ ==	"__main__":


	test_number = sys.argv[1]

	for w in (["SF", "cam"]):
	#for w in (["SF"]):
		print(w)

		t_est, phi_est = read_estimation_robot(test_number, w)
		#t_est, phi_est =0,0
		t_all, t_0, x, y, alpha = read_truth_localization(test_number)
		#t_all, t_0, x, y, alpha = 0,0,0,0,0
		mean, stdev = evaluate_estimation(phi_est, t_est, t_all, t_0, x, y, alpha, w)

		write_to_file(test_number, w, mean, stdev)