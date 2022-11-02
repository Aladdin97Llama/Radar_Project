
# importing libraries
import math as m
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.animation as animation
import random
from itertools import count
from matplotlib import style
import time


#variable
cartesian_points=[]
all_data = ""
cartesian_points=[]
temp_chaine= ""
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
plt.title("Radar chart  (most dominant target) ", fontsize=20)
# setting x-axis label and y-axis label
plt.xlabel("X-axis")
plt.ylabel("Y-axis")

plt.xlim(-300,300 )
plt.ylim(0,400)



# read and decode file 
file = open('move1_tadt_.txt','r')
read = file.readlines()
all_data = read[0]
#print(all_data)
all_points = all_data.split("/")
for x in range (0,len(all_points)-1):
    temp_chaine = all_points[x] 
    z=  temp_chaine.split(";")
    #angle = (float(z[2])) *1.0*0.0174533
    angle = m.radians(90-float(z[2]))
    distance = float(z[0])
    x_coordinate = distance*(m.cos(angle))
    y_coordinate = distance*(m.sin(angle))
    cartesian_points.append([x_coordinate,y_coordinate])
A = np.array(cartesian_points)





 
# Loop
for t in range(len(cartesian_points)):
    # creating new Y values
    #  new_y = np.sin(x-0.5*t)
    # updating data values
    #line1.set_xdata(x)
    #line1.set_ydata(new_y)
    #plt.clf()
    #plt.xlim(-300,300 )
    #plt.ylim(0,400)
    m = cartesian_points[t][0]
    n = cartesian_points[t][1]
    print("time : ")
    print(time.time())
    print("x : ")
    print(m)
    print("y : ")
    print(n)
    print("/////")
    #plt.suptitle(time.time(), y=1.05, fontsize=18)
    if((cartesian_points[t][0]!=0)or (cartesian_points[t][1]!=0)):
            current = time.time()
            z = np.array([n])
            e = np.array([m])
            # Plotting point using sactter method
            plt.scatter(e,z)
            #plt.show()
            # drawing updated values
            figure.canvas.draw()
         
            # This will run the GUI event
            # loop until all UI events
            # currently waiting have been processed
            figure.canvas.flush_events()
 
    time.sleep(0.1)
    #if((time.time()-current)>2):
       # plt.clf()
        #plt.xlim(-300,300 )
        #plt.ylim(0,400)
