import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import spiceypy.spiceypy as sp
import numpy as np

import combine_A
import read_B

plt.switch_backend('agg')

f, (ax1,ax2,ax3) = plt.subplots(3,1,sharex=True)

num_points = 1
[T1A,Tf1A,C1A, Q1A] = combine_A.combine('SCA_1A/6-26/SCA1A_2019-06-26_C_04.txt',num_points)
print('Combined A')
[T1B,C1B, Q1B] = read_B.read('SCA_1B/6-26/SCA1B_2019-06-26_C_04.txt',num_points)
print('Read B')

#define a color for each combination type
colorsA = []
for i in range(0,len(Q1A)):
    if C1A[i] == '1':
        colorsA.append('yellow')
    elif C1A[i] == '2':
        colorsA.append('yellow')
    elif C1A[i] == '12':
        colorsA.append('red')
    elif C1A[i] == '3':
        colorsA.append('yellow')
    elif C1A[i] == '13':
        colorsA.append('red')
    elif C1A[i] == '23':
        colorsA.append('orange')
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

num_points = int(len(Q1A)/10) - 1
M1A = []
M1B = []
eul_x = []
eul_y = []
eul_z = []
num_err = [0,0,0]
sum_err = [0,0,0]
sum_t_diff = 0
for i in range(num_points):
    #check times are same
    if abs(T1A[i]-T1B[i]) != 0:
        print('Time mismatch')
        break

    #linearly interpolate B quaternions to match A partial times
    for k in range(4):
        Q1B[i][k] = Q1B[i][k] + (Q1B[i+1][k]-Q1B[i][k])*Tf1A[i]/10**6

    #calculate angle differences between attitude measurements
    M1A.append(sp.q2m(Q1A[i]))
    M1B.append(sp.q2m(Q1B[i]))
    M_diff = sp.mtxm(M1A[i],M1B[i])
    eul = sp.m2eul(sp.xpose(M_diff),1,2,3)
    eul_x.append(eul[0])
    eul_y.append(eul[1])
    eul_z.append(eul[2])

    #percentage of data in error
    for i in range(3):
        if eul[i] > .01:
            num_err[i] +=1
            sum_err[i] += eul[i]

percent_err = [num_err[i]/num_points*100 for i in range(3)]
mag_err = [sum_err[i]/num_err[i] for i in range(3)]

print('X % bad data: ',percent_err[0])
print('Y % bad data: ',percent_err[1])
print('Z % bad data: ',percent_err[2])

print('X error average magnitude: ', mag_err[0])
print('Y error average magnitude: ', mag_err[1])
print('Z error average magnitude: ', mag_err[2])

print('Total time difference',sum_t_diff)

ax1.scatter([i for i in range(num_points)],eul_x,s = .05,c=colorsB)
ax2.scatter([i for i in range(num_points)],eul_y,s = .05,c=colorsB)
ax3.scatter([i for i in range(num_points)],eul_z,s = .05,c=colorsB)


#legend
red = mpatches.Patch(color='red',label='1-3')
blue = mpatches.Patch(color='blue',label='1-2-3')
ax1.legend(handles=[red,blue],loc=1,prop={'size':6})

#Titles
range = .001
ax1.set_title('Euler X')
ax1.set_ylim((-range,range))
ax2.set_title('Euler Y')
ax2.set_ylim((-range,range))
ax3.set_title('Euler Z')
ax3.set_ylim((-range,range))

#ax4.set_title('I')
#ax5.set_title('J')
#ax6.set_title('K')
plt.xlabel('Time')
plt.ylabel('Radians')
plt.savefig('AvsB.png',dpi=1000)
print('Plotted and saved figure.')

'''
q_diff = [sp.qxq([Q1A[i][0],-Q1A[i][1],-Q1A[i][2],-Q1A[i][3]],Q1B[i])[0] for i in range(num_points)]
theta_AB = [2*np.arccos(q_diff[i]) for i in range(num_points)]
theta_AB = [2*np.arccos(sp.qxq([Q1A[i][0],-Q1A[i][1],-Q1A[i][2],-Q1A[i][3]],Q1B[i])[0]) for i in range(num_points)]
theta_AB = [abs(theta_AB[i]) % 3.14159 for i in range(num_points)]
ax1.scatter([i for i in range(num_points)],theta_AB,s = .05,c=colorsB)

compute using inverse tan
ax2.scatter([i for i in range(num_points)],[2*np.arctan2(np.linalg.norm(sp.qxq([Q1A[i][0],-Q1A[i][1],-Q1A[i][2],-Q1A[i][3]],Q1B[i])[1:]),sp.qxq([Q1A[i][0],-Q1A[i][1],-Q1A[i][2],-Q1A[i][3]],Q1B[i])[0]) for i in range(num_points)],s = .05,c=colorsB)

plot difference between scalar parts of quaternions
ax3.scatter([i for i in range(num_points)],[abs(Q1A[i][0])-abs(Q1B[i][0]) for i in range(num_points)],s =.05,c=colorsB)
plot difference between i components
ax4.scatter([i for i in range(num_points)],[abs(Q1A[i][1])-abs(Q1B[i][1]) for i in range(num_points)],s =.05,c=colorsB)
plot difference between j components
ax5.scatter([i for i in range(num_points)],[abs(Q1A[i][2])-abs(Q1B[i][2]) for i in range(num_points)],s =.05,c=colorsB)
plot difference between k components
ax6.scatter([i for i in range(num_points)],[abs(Q1A[i][3])-abs(Q1B[i][3]) for i in range(num_points)],s =.05,c=colorsB)
errorcount = 0
for i in range(num_points):
    if theta_AB[i] > .02:
        errorcount += 1

print(errorcount/num_points*100)
'''
