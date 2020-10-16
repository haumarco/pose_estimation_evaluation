#!/usr/bin/python3

import numpy as np
import sys

def read_estimation_robot(test_number):
	# read estimation from robot

	data = np.genfromtxt("./" + test_number + "/SF_data" + test_number + ".txt", delimiter = ",", skip_header=1)

	data = data[:, [0, 2, 4, 7]] # est_time, stamp_time, d, phi
	#print(data[0,0]- data[1,1])

	t_est = data[:,0]
	delay = data[:,0] - data[:,1]
	d = data[:,2]
	phi_est = data[:,3]

	return(t_est, phi_est)

###

def read_truth_localization(test_number):

	#copy from notebook
	return


def write_to_file(test_number, mean, stdev):
	f = open("/evaluation_results", "a")
	f.write(test_number + ", " + mean  + ", " + stdev)
	f.close()
	return


def evaluate_estimation(phi_est, t_true, x, y, alpha):
	a=0.6
	b=1.185
	phi_true = np.array(())
	

	for i in range(len(t_true)):

		#sector 1:
		if x[i] > a and x[i] < b and y[i] < a+0.1:
			if abs(alpha[i]) > np.pi/2:
				alpha[i] += np.pi

		#sector 2:
		elif x[i] > b - 0.1 and y[i] > a and y[i] < b:
			alpha[i] -= np.sign(alpha[i]) * np.pi/2

		#sector 3:
		elif x[i] > a and x[i] < b and y[i] > b -0.1:
			if abs(alpha[i]) > np.pi/2:
				alpha[i] += np.pi

		elif x[i] < a + 0.1 and y[i] > a and y[i] < b:
			alpha[i] -= np.sign(alpha[i]) * np.pi/2

		else:
			print("NOT IN LANE!")

		phi_true[i] = alpha[i]



	u = 0
	i = 0
	while i < len(t_true):
		if t_true[i] < t_est[u]:
			phi_comp = (phi_est[u] - phi_est[u-1]) / (t_est[u] - t_est[u-1]) * (t_true[i] - t_est[u-1]) + phi_est[u-1]
			i += 1
		
		else:
			u += 1



	error = phi_true - phi_comp

	mean = np.mean(error)
	stdev = np.std(error)

	return(mean, stdev)


if __name__ ==	"__main__":

	test_number = sys.argv[1]

	t_est, phi_est = read_estimation_robot(test_number)

	t_true, x, y, alpha = read_truth_localization(test_number)

	mean, stdev = evaluate_estimation(phi_est, t_true, x, y, alpha)

	write_to_file(mean, stdev)