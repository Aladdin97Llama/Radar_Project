
# importing libraries
import math as m
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.animation as animation
import random
from itertools import count
from matplotlib import style
import time

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
plt.title("Radar chart PADT output no filter", fontsize=20)
# setting x-axis label and y-axis label
plt.xlabel("X-axis")
plt.ylabel("Y-axis")

plt.xlim(-5,5)
plt.ylim(0,8)



# read and decode file 
file = open('2_person_move2_not_same_time.txt','r')
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
                    distance = float(z[0])/100
                    x_coordinate = distance*(m.cos(angle))
                    y_coordinate = distance*(m.sin(angle))
                    #print("angle in degree ")
                    #print(float(z[2]))
                    #print("angle in radian     ")
                    #print(angle)
                    #print("distance  ")
                    #print(distance)
                    #print("x coor   ")
                    #print(x_coordinate)
                    #print("y coor  ")
                    #print(y_coordinate)
                    #print("//////////////////////////")
                    
                    if ((x_coordinate != 0)or(y_coordinate != 0 )):
                            cartesian_points.append([x_coordinate,y_coordinate])
                    else :
                            5+5  
        if(cartesian_points != []):
                cartesian_points_final.append(cartesian_points)
        #cartesian_points_final.append(cartesian_points)
#A = np.array(cartesian_points)
print(cartesian_points_final)






# Loop
for t in range(len(cartesian_points_final)):
        
        for index_i in range(len(cartesian_points_final[t])):
                    #m = cartesian_points_final[t][0]
                    mm.append(cartesian_points_final[t][index_i][0])
                    nn.append(cartesian_points_final[t][index_i][1])
                    #n = cartesian_points_final[t][1]
                    #print("time : ")
                    #print(time.time())
                    #print("x : ")
                    #print(mm)
                    #print("y : ")
                    #print(nn)
                    #print("/////")
                    #plt.suptitle(time.time(), y=1.05, fontsize=18)
                    #if((cartesian_points[t][0]!=0)or (cartesian_points[t][1]!=0)):
                    #current = time.time()
                    z = np.array([nn])
                    e = np.array([mm])
                    # Plotting point using sactter method
                    #plt.scatter(e,z, c='blue')
                    plt.scatter(e,z)
                    #plt.show()
                    # drawing updated values
                    figure.canvas.draw()
                 
                    # This will run the GUI event
                    # loop until all UI events
                    # currently waiting have been processed
                    figure.canvas.flush_events()
        #time.sleep(0.1)
        #print(mm)
        #print(nn)
        #print("----------------")
        #plt.clf()
        #plt.xlim(-300,300)
        #plt.ylim(0,400)
        #time.sleep(0.5)
        mm = []
        nn = []
         
     
   # if((time.time()-current)>2):
   #     plt.clf()
   #     plt.xlim(-300,300 )
   #     plt.ylim(0,400)   


