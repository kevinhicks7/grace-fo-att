import matplotlib as mpl
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import spiceypy.spiceypy as sp
import numpy as np


import combine_A
import read_B

#vpn:
#eid password
#push
#ssh kevin@goose.csr.utexas.edu
#Khic201322! ?

#/home/jplftp/data
#check to see if quaternion norm isn't one
#try with only 1 and 3 with only interpolation
#text file: integer, time fraction, x,y,z,theta (want to look at all of data)
#interpolate A instead of B

#mpl.use('Agg')
plt.switch_backend('agg')

f, ((ax1,ax2,ax3, ax4)) = plt.subplots(4,1,sharex=True)

num_points = 1
[T1A,Tf1A,C1A, Q1A] = combine_A.combine('SCA_1A/8-23/SCA1A_2019-08-23_C_04.txt',num_points)
print('Combined A')
[T1B,C1B, Q1B] = read_B.read('SCA_1B/8-23/SCA1B_2019-08-23_C_04.txt',num_points)
print('Read B')

#define a color for each combination type
colorsA = []
for i in range(0,len(Q1A)):
    if C1A[i] == '1':
        colorsA.append('pink')
    elif C1A[i] == '2':
        colorsA.append('black')
    elif C1A[i] == '12':
        colorsA.append('purple')
    elif C1A[i] == '3':
        colorsA.append('yellow')
    elif C1A[i] == '13':
        colorsA.append('orange')
    elif C1A[i] == '23':
        colorsA.append('red')
    elif C1A[i] == '123':
        colorsA.append('blue')

colorsB = []
for i in range(0,len(Q1A)):
    if C1B[i] == '1':
        colorsB.append('black')
    elif C1B[i] == '2':
        colorsB.append('black')
    elif C1B[i] == '3':
        colorsB.append('black')
    elif C1B[i] == '4':
        colorsB.append('black')
    elif C1B[i] == '5':
        colorsB.append('black')
    elif C1B[i] == '6':
        colorsB.append('black')
    elif C1B[i] == '7':
        colorsB.append('black')
    elif C1B[i] == '16':
        colorsB.append('black')
    elif C1B[i] == '17':
        colorsB.append('yellow') #1,IMU
    elif C1B[i] == '18':
        colorsB.append('yellow') #2,IMU
    elif C1B[i] == '19':
        colorsB.append('orange') #1,2,IMU
    elif C1B[i] == '20':
        colorsB.append('yellow') #3,IMU
    elif C1B[i] == '21':
        colorsB.append('red') #1,3,IMU
    elif C1B[i] == '22':
        colorsB.append('purple') #2,3,IMU
    elif C1B[i] == '23':
        colorsB.append('blue') #1,2,3,IMU

num_points = int(len(Q1A)/16) - 1
M1A = []
M1B = []
eul_x = []
eul_y = []
eul_z = []
theta = []
num_err = [0,0,0]
sum_err = [0,0,0]
sum_t_diff = 0
A_bad_q = 0
B_bad_q = 0
#file = open('time_and_angles.txt','w')

for i in range(num_points):


    #check times are same
    if abs(T1A[i]-T1B[i]) != 0:
        print('Time mismatch')
        break
    if (np.linalg.norm(Q1A[i]) - 1) > .01:
        A_bad_q = A_bad_q + 1
    if (np.linalg.norm(Q1B[i]) - 1) > .01:
        B_bad_q = B_bad_q + 1
    #linearly interpolate B quaternions to match A partial times
    for k in range(4):
        Q1B[i][k] = Q1B[i][k] + (Q1B[i+1][k]-Q1B[i][k])*Tf1A[i]/10**6

    #calculate angle differences between attitude measurements
    M1A.append(sp.q2m(Q1A[i]))
    M1B.append(sp.q2m(Q1B[i]))
    M_diff = sp.mtxm(M1A[i],M1B[i])
    eul = sp.m2eul(sp.xpose(M_diff),1,2,3)
    eul_x.append(eul[0]*1000)
    eul_y.append(eul[1]*1000)
    eul_z.append(eul[2]*1000)
    theta.append(np.arccos((np.trace(M_diff) - 1)/2)*1000)

    #write to text file
    #file.write('%d %d %f %f %f %f \n' % (T1A[i],Tf1A[i],eul_x[i],eul_y[i],eul_z[i],theta[i],))

    #percentage of data in error
    for i in range(3):
        if eul[i] > .01:
            num_err[i] +=1
            sum_err[i] += eul[i]
#file.close()
x_std = np.std(eul_x)
x_mean = np.mean(eul_x)
y_std = np.std(eul_y)
y_mean = np.mean(eul_y)
z_std = np.std(eul_z)
z_mean = np.mean(eul_z)
print('X mean: ',x_mean)
print('X std: ',x_std)
print('Y mean: ',y_mean)
print('Y std: ',y_std)
print('Z mean: ',z_mean)
print('Z std: ',z_std)

ax1.scatter([i for i in range(num_points)],eul_x,s = .05,c=colorsA)
ax2.scatter([i for i in range(num_points)],eul_y,s = .05,c=colorsA)
ax3.scatter([i for i in range(num_points)],eul_z,s = .05,c=colorsA)
ax4.scatter([i for i in range(num_points)],theta,s = .05,c=colorsA)


#legend
pink = mpatches.Patch(color='pink',label='1')
black = mpatches.Patch(color='black',label='2')
red = mpatches.Patch(color='red',label='2-3')
yellow = mpatches.Patch(color='yellow',label='3')
orange = mpatches.Patch(color='orange',label='1-3')
purple = mpatches.Patch(color='purple',label='1-2')
blue = mpatches.Patch(color='blue',label='1-2-3')

ax1.legend(handles=[pink,black,red,yellow,orange,purple,blue],loc=1,prop={'size':3})

#Scaling and labeling
sz = 8
vert_shift = .92
ax1.set_title('X',size=sz,y = vert_shift)
ax1.set_ylim((-.667,.667))
ax1.yaxis.set_minor_locator(mpl.ticker.AutoMinorLocator())
ax1.yaxis.grid(True,which='minor',linewidth=.5,linestyle='--')
ax1.yaxis.grid(True,which='major',linewidth=.5,linestyle='--')

ax2.set_title('Y',size=sz,y = vert_shift)
ax2.set_ylim((-1,1))
ax2.yaxis.set_minor_locator(mpl.ticker.AutoMinorLocator())
ax2.yaxis.grid(True,which='minor',linewidth=.5,linestyle='--')
ax2.yaxis.grid(True,which='major',linewidth=.5,linestyle='--')

ax3.set_title('Z',size=sz,y = vert_shift)
ax3.set_ylim((-1,1))
ax3.yaxis.set_minor_locator(mpl.ticker.AutoMinorLocator())
ax3.yaxis.grid(True,which='minor',linewidth=.5,linestyle='--')
ax3.yaxis.grid(True,which='major',linewidth=.5,linestyle='--')

ax4.set_title('Angle',size=sz,y = vert_shift)
ax4.set_ylim((.2,1))
ax4.yaxis.set_minor_locator(mpl.ticker.AutoMinorLocator())
ax4.yaxis.grid(True,which='minor',linewidth=.5,linestyle='--')
ax4.yaxis.grid(True,which='major',linewidth=.5,linestyle='--')


ax1.set_ylabel('mrad',size = sz)
ax2.set_ylabel('mrad',size = sz)
ax3.set_ylabel('mrad',size = sz)
ax4.set_ylabel('mrad',size = sz)

plt.xlabel('Time')
ax1.set_ylabel('mrad')
plt.savefig('AvsB_One_Orbit.png',dpi=1000)
plt.show()
print('Plotted and saved figure.')
