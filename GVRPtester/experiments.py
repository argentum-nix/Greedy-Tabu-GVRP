import os


def experiment(instance_num):
	directory = "./instances/"

	#instance numiter lentabu
	bashCommand = './GVRPGreedyTabu AB{0} {1} {2}'

	iterations = [8, 34, 55];
	lenTabu = [3, 8, 13, 21];

	for i in iterations:
		for l in lenTabu:
			os.system(bashCommand.format(instance_num, i, l));


instances = [101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112,
			113, 114, 115, 116, 117, 118, 119, 120, 201, 202, 203, 204, 205, 206,
			207, 208, 209, 210, 211, 212, 213, 214, 215, 216, 217, 218, 219, 220]

for ele in instances:
	experiment(ele)
