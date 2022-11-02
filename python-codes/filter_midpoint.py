# importing libraries
import math as m
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.animation as animation
import random
from itertools import count
from matplotlib import style
import time



def midpoint(lst):
    if(len(lst)>1):
        #take distance between first two points as reference 
        distance_min = m.sqrt((lst[0][0]-lst[1][0])*(lst[0][0]-lst[1][0]) + (lst[0][1]-lst[1][1])*(lst[0][1]-lst[1][1]))
        point_1 = [lst[0][0],lst[0][1]]
        point_2 = [lst[1][0],lst[1][1]]
        for t in range(len(lst)-1):
            for s in range(t+1,len(lst)):
                distance = m.sqrt((lst[t][0]-lst[s][0])*(lst[t][0]-lst[s][0]) + (lst[t][1]-lst[s][1])*(lst[t][1]-lst[s][1]))
                if (distance<distance_min):
                    distance_min = distance
                    point_1 = [lst[t][0],lst[t][1]]
                    point_2 = [lst[s][0],lst[s][1]]
        midpoint_x = (point_1[0]+point_2[0])/2
        midpoint_y = (point_1[1]+point_2[1])/2
        l = [midpoint_x,midpoint_y]
        return [midpoint_x,midpoint_y]
                    
    else :
        l = [lst[0][0],lst[0][1]]
        return [lst[0][0],lst[0][1]]




var_bool = True  
#variable
PADT1 = []
mm = []
nn = []
cartesian_points=[]
cartesian_points_final=[]
pp = []
all_data = ""
cartesian_points=[]
temp_chaine= ""
temp_chaine2=""
x_coordinate = 0.0
y_coordinate = 0.0
xx = []
yy = []
current = 0 
 
# to run GUI event loop
plt.ion()
 
# here we are creating sub plots
figure, ax = plt.subplots(figsize=(10, 8))

 
# setting title
plt.title("Radar chart  with midpoint filter ", fontsize=20)
# setting x-axis label and y-axis label
plt.xlabel("X-axis")
plt.ylabel("Y-axis")

plt.xlim(-500,500 )
plt.ylim(0,800)



# read and decode file 
file = open('move1_PADT_3.txt','r')
read = file.readlines()
all_data = read[0]
#print(all_data)
all_points = all_data.split("/")
for x in range (0,len(all_points)-1):
        temp_chaine = all_points[x]
        PADT1 =temp_chaine.split("%")
        cartesian_points = []
        for v in range (0,len(PADT1)-1):
                    temp_chaine2 = PADT1[v]
                    z=  temp_chaine2.split(";")
                    #angle = (float(z[2])) *1.0*0.0174533
                    angle = m.radians(90-float(z[2]))
                    distance = float(z[0])
                    x_coordinate = distance*(m.cos(angle))
                    y_coordinate = distance*(m.sin(angle))
                  #  print(x_coordinate)
                  #  print(y_coordinate)
                  #  print("//////////////")
                    cartesian_points.append([x_coordinate,y_coordinate])
                    
        cartesian_points_final.append(cartesian_points)
#A = np.array(cartesian_points)
#print(cartesian_points_final)





l_truth = []
# Loop
for w in range(len(cartesian_points_final)):
        midpoint_list = midpoint(cartesian_points_final[w])
        #midpoint(cartesian_points_final[w])
        
        z = np.array(midpoint_list[0])
        e = np.array(midpoint_list[1])
        if ((z==0)and(e==0)):
            continue 
        time.sleep(0.1)
        plt.scatter(z,e)
        figure.canvas.draw()
        figure.canvas.flush_events()
        l_truth.append([midpoint_list[0],midpoint_list[1]])
       # if((time.time()-current)>2):
       #     plt.clf()
       #     plt.xlim(-300,300 )
       #     plt.ylim(0,400)
print(l_truth)




for t in range(len(l_truth)):
    if (t == (len(l_truth)-1)):
        print("end do nothing")
    else :
        p1,p2 = l_truth[t],l_truth[t+1]
        p1 =[l_truth[t][0],l_truth[t+1][0]]
        p2 =[l_truth[t][1],l_truth[t+1][1]] 
        print("////////////////////////")
        print(p1)
        print(p2)
        plt.plot(p1, p2, marker = 'o')
        plt.show()
        
    






