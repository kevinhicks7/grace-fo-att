import spiceypy.spiceypy as sp
import numpy as np
def opt_comb2(q1,q2,comb):
	#need inverse of quaternion 1
	q1_inv = [q1[0],-q1[1],-q1[2],-q1[3]] #norm is 1, so don't have to divide

	#approximate difference of two quaternions
	delta = sp.qxq(q1_inv,q2) #first element should be 1

	if comb == '12':
		M = [[0.340346230008916, 0.144498947041826, -0.090223089414205],
	       	   [0.185800204446449, 0.228953355830318, 0.185797865375266],
	      	   [-0.113213672795652, 0.183245159249732, 0.430405211880520]]
	elif comb == '13':
		M = [[0.340346230008916, -0.144498947041826, -0.090223089414205],
	       	   [-0.185800204446449, 0.228953355830318, -0.185797865375266],
	      	   [-0.113213672795652, -0.183245159249732, 0.430405211880520]]
	else:
		M = [[0.340346230008916, -0.144498947041826, -0.090223089414205],
	       	   [-0.185800204446449, 0.228953355830318, -0.185797865375266],
	      	   [-0.113213672795652, -0.183245159249732, 0.430405211880520]]

	Mxdelta = np.matmul(M,delta[1:])

	#convert to quaternion
	delta_q = [((4-Mxdelta[0]**2 - Mxdelta[1]**2 - Mxdelta[2]**2)**.5)/2,Mxdelta[0]/2,delta[1]/2,delta[2]/2]
	#delta_q_inv = [delta_q[0],-delta_q[1],-delta_q[2],-delta_q[3]]
	R_opt = sp.mxmt(sp.q2m(q1),sp.q2m(delta_q))
	q_opt = sp.m2q(R_opt)

	return np.ndarray.tolist(q_opt)

def opt_comb3(q1,q2,q3):
	q1_inv = [q1[0],-q1[1],-q1[2],-q1[3]] #norm is 1, so don't have to divide
	delta12 = sp.qxq(q1_inv,q2)
	delta13 = sp.qxq(q1_inv,q3)

	M12 = [[0.340346699807362, 0.144500456301639, -0.090220503250359],
       	   [0.185801401219710, 0.228954283314325, 0.185798544898709],
       	   [-0.113209244528658, 0.183245297546861, 0.430401082295432]]
	M13 = [[0.340346230008916, -0.144498947041826, -0.090223089414205],
       	   [-0.185800204446449, 0.228953355830318, -0.185797865375266],
      	   [-0.113213672795652, -0.183245159249732, 0.430405211880520]]

	M12xdelta = np.matmul(M12,delta12[1:])
	M13xdelta13 = np.matmul(M13,delta13[1:])

	#add deltas together
	delta_sum = [M12xdelta[i] + M13xdelta13[i] for i in range(0,3)]
	#convert delta to quaternion
	delta_sum_q = [((4-delta_sum[0]**2 - delta_sum[1]**2 - delta_sum[2]**2)**.5)/2,delta_sum[0]/2,delta_sum[1]/2,delta_sum[2]/2]

	#find q_opt by given equation
	R_opt = sp.mxmt(sp.q2m(q1),sp.q2m(delta_sum_q))
	q_opt = sp.m2q(R_opt)

	return np.ndarray.tolist(q_opt)
