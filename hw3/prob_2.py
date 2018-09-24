import numpy as np 
from numpy import linalg as LA
from numpy.linalg import matrix_rank

if __name__ == "__main__":
	alpha = np.array([0.75, 0.75, 1.25])
	beta = np.array([1.0, 1.5, 1.0])

	for i in range(0,len(alpha)):
		F = np.array([[alpha[i], alpha[i]],[beta[i]*(alpha[i]-1), beta[i]*alpha[i]]])
		G = np.array([[alpha[i]],[beta[i]*alpha[i]]])
		H = np.array([1, 1])
		w,v = LA.eig(F)
		print("eigenvalues of F")
		print(w)
		mags = np.absolute(w)
		if mags[0] > 1.0 or mags[1] > 1.0:
			print("These params are not stable")
		else:
			print("these params are stable")

		# determine observability
		max_iters = 5
		j = 1
		cur_obs = H
		O = H
		rank = 0
		while rank < 2 and j < max_iters:
			cur_obs = np.dot(cur_obs,F)
			O = np.vstack((O,cur_obs))
			rank = matrix_rank(O)
			print("rank is " + str(rank))
			print(O);
			j = j+1

		if j < max_iters:
			print("system is observable with " + str(j) + " observations")
		else:
			print("system is not observable")
