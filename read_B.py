def read(filename,n_points):
	#Read File
	with open(filename,'r') as header_and_data:
		#read data but ignore header
		for curline in header_and_data:
			if curline.startswith('# End of YAML header'):
				break
		data = header_and_data.readlines()[0:]
		#data = [data[i].rstrip().split(' ') for i in data]

	#Parse Data
	time  = []
	ID = []
	q = []
	combination = []
	for i in range (0,len(data)):
		#get rid of newline and then convert string to array
		data[i] = data[i].rstrip().split(' ')
		time.append(int(data[i][0]))
		ID.append(data[i][3])
		q.append([float(i) for i in data[i][3:7]])
		combination.append(data[i][2])
	return [time,combination,q]
