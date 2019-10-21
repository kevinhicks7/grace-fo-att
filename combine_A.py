import numpy as np
import spiceypy.spiceypy as sp
import opt_comb


def combine(filename,n_points):
	#Read File
	with open(filename,'r') as header_and_data:
		#read data but ignore header
		for curline in header_and_data:
			if curline.startswith('# End of YAML header'):
				break
		data = header_and_data.readlines()[0:]

	#Parse Data
	time  = []
	time_frac = []
	ID = []
	qSF = []
	confid = []
	for i in range (0,len(data)):
		#get rid of newline and then convert string to array
		data[i] = data[i].rstrip().split(' ')
		time.append(int(data[i][0]))
		time_frac.append(int(data[i][1]))
		ID.append(int(data[i][3]))
		qSF.append([float(i) for i in data[i][5:9]])
		confid.append(int(data[i][11]))

	#Rotate and Optimally Combine Quaternion at Every Time
	#Rotation from spacecraft to common frame
	Q_SF2C = [[-0.184652303391424, 0.6877863330342120, 0.6765142470974430, 0.1875685485836400],
			  [0.2482955018618801, -0.0400731149707713, 0.8516159983601500, 0.4598844208589170],
			  [-0.4627545612843981, 0.8531198681517500, -0.0427367676938312, -0.2371039334602200]]

	q_opt = []
	comb_type = []
	time_return = []
	time_frac_return = []
	time_last = 0
	comb_type = []
	for i in range(0,int(len(data)),3):
		q_valid = []
		combination = []

		#check to make sure same time stamp and ignore other fractional timesteps
		if time[i] != time_last and time[i] == time[i+1] and time[i] == time[i+2]:
			time_return.append(time[i])
			time_frac_return.append(time_frac[i])
			#extract quaternions with low enough confidence and
			for k in range(0,3):
				if confid[i+k] <= 5:
					#print(i+k-1)
					#print(ID[i+k-1])
					#print(qSF[i+k])
					qC = sp.qxq(qSF[i+k],Q_SF2C[ID[i+k]-1]) #-1 to match 0 based indexing
					q_valid.append(qC)
					combination.append(int(ID[i+k]))


			#format combination into string in ascending order
			combination.sort()
			#print(combination)
			#combination[combination != 0]
			comb_type.append(''.join(str(x) for x in combination))


			'''
			if confid[i+1] <= 5:
				q2C = sp.qxq(qSF[i+1],Q_SF2C[1])
				q_valid.append(q2C)
				combination = int(ID[i+1])
			if confid[i+2] <= 5:
				q3C = sp.qxq(qSF[i+2],Q_SF2C[2])
				q_valid.append(q3C)
				combination = int(ID[i+2])
			comb_type.append(combination)
			'''
			#find optimal combination of quaternions
			if len(q_valid) == 1:
				q_opt.append(q_valid[0])
			elif len(q_valid) == 2:
				q_opt.append(opt_comb.opt_comb2(q_valid[0],q_valid[1],combination))
			elif len(q_valid) == 0:
				print('No valid star tracker readings')
			else:
				#try with only 1-3 combination
				#q_opt.append(opt_comb.opt_comb2(q_valid[0],q_valid[2],'13'))
				q_opt.append(opt_comb.opt_comb3(q_valid[0],q_valid[1],q_valid[2]))

		time_last = time[i]
	return [time_return,time_frac_return,comb_type,q_opt]
