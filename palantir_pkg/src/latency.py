import numpy as np
import matplotlib.pyplot as plt
def parseline(line):
	line_list = [x.strip() for x in line.split()]
	print "line Read", line_list
	time_line = [x.strip() for x in line_list[6].split("=")]
	print "time parsed", time_line[1]
	return float(time_line[1])

def main():
	inputFname = "latency_model.txt"
	in_ = file(inputFname, "r")
	line_no = 0
	times = []
	for line in  in_.readlines():
		line_no += 1
		if line_no >= 5:
			timeRead = parseline(line)
			times.append(timeRead)

	print "average latency", np.mean(times)
	plt.plot(times)
	plt.show()

main()