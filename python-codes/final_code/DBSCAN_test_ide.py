import matplotlib.pyplot as plt
import numpy
import scipy.cluster.hierarchy as hcluster
from pandas import DataFrame
from matplotlib.pyplot import figure 
from sklearn.datasets import make_blobs
import math as m
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.animation as animation
import random
from itertools import count
from matplotlib import style
from sklearn.cluster import DBSCAN 
import time
 
#variable
targets = []     # this list will contain all targets    "each element of this list is a target class"
var_bool = True 
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
#this is 19 colors available to plot  19 targets at same time
color_table = ['blue','green','red','black','cyan','magenta','yellow','skyblue','orange','white','violet','purpel','pink','navy','deeppink','springgreen','peru','olive','beige']






#class
class target :
    
    # attribute   

    # method
    def __init__(self,color):
        #each target have unique color for ploting
        self.target_active_color = color 
        self.target_color = color
        self.path_history = []
        self.speed_history = []
        self.angle_history = [] 
        self.magnitude_history = []
        self.warning_flag = False
        self.alert_flag = False
        self.target_status = False
        self.current_x = 0
        self.current_y = 0
        self.last_frame_number = 0
        self.angle_variance = 0
        self.total_angle_variance = 0 

        
        #slef.color = color_tabel[index]
    def kalman_filter_function ():
        #apply the KF for new measerment
        1+1
    def update_statuts():
        # update angle and speed status
        1+1
    def check_if_points_in_region(list):
        #check if the new measurements in the neighbour of this target
        1+1

        
    def update(self,target_new_data):
        self.target_status = True
        self.current_x =target_new_data[0]
        self.current_y =target_new_data[1]
        self.path_history.append([target_new_data[0],target_new_data[1]])
        if(len(self.angle_history)==0):
            self.angle_history.append(target_new_data[3])
        else :
            self.angle_variance = target_new_data[3]-self.angle_history[-1]
            self.total_angle_variance = self.total_angle_variance + self.angle_variance    # angles here represented like in radar datasheet
            self.angle_history.append(target_new_data[3])
        self.last_frame_number = target_new_data[5] 
        self.speed_history.append(target_new_data[2])
        self.angle_history.append(target_new_data[3])
        self.magnitude_history.append(target_new_data[4])
        
    def delete_old_target(self,current_frame_n):
        if (((current_frame_n - self.last_frame_number)>4)  and (len(self.path_history)>0)):
            delete_index = 0 
            self.target_status = False
            #self.target_color = 'white'
            #change color to white and hide all points
            for delete_index in range(len(self.path_history)):
                x_to_delete = self.path_history[delete_index][0]
                y_to_delete = self.path_history[delete_index][1]
                plt.scatter(x_to_delete,y_to_delete,color='white',s=45)
                #figure.canvas.draw()
                #figure.canvas.flush_events()
            self.path_history = []
            self.current_y = 0
            self.current_x = 0
        

        



# functions define

#************************************************************************************************************
def read_extract_file(filename):
    file = open(filename,'r')
    read = file.readlines()
    all_data = read[0]
    #print(all_data)
    all_points = all_data.split("/")    # '/' is the separation between each frame
    frame_freq = 0 
    for x in range (0,len(all_points)-1):
            temp_chaine = all_points[x]
            PADT1 =temp_chaine.split("%")   # '%' is the separation between measurements from the same frame
            cartesian_points = []
            frame_freq = frame_freq + 1 
            for v in range (0,len(PADT1)-1):
                        temp_chaine2 = PADT1[v]
                        z=  temp_chaine2.split(";")
                        angle = m.radians(90-float(z[2]))
                        angle_from_radar = (float(z[2]))
                        distance = float(z[0])/100
                        speed = float(z[1])
                        magnitude = float(z[3])
                        x_coordinate = distance*(m.cos(angle))
                        y_coordinate = distance*(m.sin(angle))
                        
                        if ((x_coordinate != 0)or(y_coordinate != 0 )):
                                cartesian_points.append([x_coordinate,y_coordinate,speed,angle_from_radar,magnitude,frame_freq])
            if(cartesian_points != []):
                    cartesian_points_final.append(cartesian_points)
    #print("done extract list")

    return cartesian_points_final   # return list of all frames of the radar with all the data needed (position in x,y ,speed,angle,magnitude,number_of_frame



#************************************************************************************************************    
def midpoint(lst):
    if(len(lst)>1):
        #take distance between first two points as reference 
        distance_min = m.sqrt((lst[0][0]-lst[1][0])*(lst[0][0]-lst[1][0]) + (lst[0][1]-lst[1][1])*(lst[0][1]-lst[1][1]))
        point_1 = [lst[0][0],lst[0][1],lst[0][2],lst[0][3],lst[0][4],lst[0][5]]
        point_2 = [lst[1][0],lst[1][1],lst[1][2],lst[1][3],lst[1][4],lst[1][5]]
        for t in range(len(lst)-1):
            for s in range(t+1,len(lst)):
                distance = m.sqrt((lst[t][0]-lst[s][0])*(lst[t][0]-lst[s][0]) + (lst[t][1]-lst[s][1])*(lst[t][1]-lst[s][1]))
                if (distance<distance_min):
                    distance_min = distance
                    point_1 = [lst[t][0],lst[t][1],lst[t][2],lst[t][3],lst[t][4],lst[t][5]]
                    point_2 = [lst[s][0],lst[s][1],lst[s][2],lst[s][3],lst[s][4],lst[s][5]]
        midpoint_x = (point_1[0]+point_2[0])/2
        midpoint_y = (point_1[1]+point_2[1])/2
        midpoint_speed = (point_1[2]+point_2[2])/2
        midpoint_angle= (point_1[3]+point_2[3])/2
        midpoint_magnitude = (point_1[4]+point_2[4])/2
        midpoint_frame_freq = point_1[5]
        
        l = [midpoint_x,midpoint_y,midpoint_speed,midpoint_angle,midpoint_magnitude,midpoint_frame_freq]
        return [midpoint_x,midpoint_y,midpoint_speed,midpoint_angle,midpoint_magnitude,midpoint_frame_freq]
                    
    else :
        l = [lst[0][0],lst[0][1],lst[0][2],lst[0][3],lst[0][4],lst[0][5]]
        return [lst[0][0],lst[0][1],lst[0][2],lst[0][3],lst[0][4],lst[0][5]]



#************************************************************************************************************
def show_clusters(X,cluster):
    df = DataFrame(dict(x=X[:,0],y=X[:,1],label=cluster))
    colors = {-1: 'red', 0:'blue',1:'orange',2:'green',3:'skyblue',4:'black'}
    fig,ax = plt.subplots(figsize=(8,8))
    grouped = df.groupby('label')
    for key,group in grouped :
        group.plot(ax=ax, kind='scatter',x='x', y='y', label=key, color=colors[key])
    plt.show()

#************************************************************************************************************
def clustering():
    frame_number = 0
    for t in range(len(all_frames)):
            #time.sleep(1)
            #print("frame_freq")
            #print(all_frames[t][0][5])
            for target_index_delete in range(len(target_list)):
                target_list[target_index_delete].delete_old_target(all_frames[t][0][5])
            frame_number = frame_number + 1
            #print("////////////////////////////////")
            #print("frame number   " + str(frame_number) + " : ")
            points_to_cluster = []         # this list will take all the measurment (x,y) for one frame
            for index_i in range(len(all_frames[t])):   
                        points_to_cluster.append([cartesian_points_final[t][index_i][0],cartesian_points_final[t][index_i][1]])
            if(len(points_to_cluster)>1):
                cluster_per_frame = cluster_function(points_to_cluster,t)
                for cluster_index_check in range (len(cluster_per_frame)):
                    #check the position of each cluter
                    print("this is the measurement for checking")
                    print(cluster_per_frame[cluster_index_check])
                    check_point(cluster_per_frame[cluster_index_check])
                #ploting_cluster(cluster_per_frame)
            else :
                #print("this is just one measurement don't do clustering")
                #print(points_to_cluster[0])
                points_to_cluster[0].append(cartesian_points_final[t][0][2])
                points_to_cluster[0].append(cartesian_points_final[t][0][3])
                points_to_cluster[0].append(cartesian_points_final[t][0][4])
                points_to_cluster[0].append(cartesian_points_final[t][0][5])
                check_point(points_to_cluster[0])
                
            plot_target()   #plot target
    

#************************************************************************************************************
def cluster_function(points_to_cluster,t):
    midpoint_for_all = []
    #arguments
    #points_to_cluster  : list of (x,y) measurments coming from one frame
    #t index of this frame in the big list that contains all frame
    X = np.array(points_to_cluster)
    dz = DataFrame(dict(x=X[:,0],y=X[:,1]))
    clustering = DBSCAN(eps=1,min_samples=1).fit(X)
    cluster = clustering.labels_  # getting cluster array 
    cluster_number = clustering.n_features_in_  # getting number of clusters
    max_cluster = len(set(cluster))
    #print("-----------")
    #print("number of cluster " + str(max_cluster))
    #print("-----------")
    # associate clusters with measurement
    current_frame_cluster = [[]for i in range(max_cluster)]     #cluster = clustering.n_features_in_
    for k in range(max_cluster):
        for n in range(len(cluster)):                                           #cluster = clustering.labels_
            if (cluster[n]==k):
                current_frame_cluster[k].append(all_frames[t][n])
    # test & print clustter we have            
    for i in range(max_cluster):
        midpoint_result = midpoint(current_frame_cluster[i])
        midpoint_for_all.append(midpoint_result)
        #print("cluster N° " + str(i+1) + " :")
        #print(current_frame_cluster[i])
        #print("")
        #print("midpoint results for this cluster  : ")
        #print(midpoint_result)
        #print("/")
    #return current_frame_cluster
    return midpoint_for_all

#************************************************************************************************************
def ploting_cluster(cluster_per_frame):
    plt.clf()
    plt.xlim(-5,5)
    plt.ylim(0,8)
    colour_list = ['black','red','blue','green','orange','skyblue']
    for index_p in range(len(cluster_per_frame)):
            x_to_plot = np.array(cluster_per_frame[index_p][0])
            y_to_plot = np.array(cluster_per_frame[index_p][1])
            plt.scatter(x_to_plot,y_to_plot,color=colour_list[index_p])
            figure.canvas.draw()
            figure.canvas.flush_events()
            
#************************************************************************************************************
def plot_target():
    plotting_points = []
    for target_index_plot in range(len(target_list)):
        #and ( len(target_list[target_index_plot].path_history) > 1 )
        if(   (target_list[target_index_plot].target_status) and ( len(target_list[target_index_plot].path_history)>1) ):
            #plot this target
            for target_index_plot_one_target in range(len(target_list[target_index_plot].path_history)):
                x_to_plot = np.array(target_list[target_index_plot].path_history[target_index_plot_one_target][0])
                y_to_plot = np.array(target_list[target_index_plot].path_history[target_index_plot_one_target][1])
                plotting_points = plt.scatter(x_to_plot,y_to_plot,color=target_list[target_index_plot].target_color,s=30)
                figure.canvas.draw()
                figure.canvas.flush_events()
                
    

#************************************************************************************************************
            
def check_point (midpoint_cluster):
    match_flag = 0
    matching_target = 0 
    target_in_neighbor = []
    x_to_class = midpoint_cluster[0]
    y_to_class = midpoint_cluster[1]
    list_for_neighbor_target = []
    if (len(target_list)>0):
        #check if this point could be related to an existed target
        #  function
        for target_index_check in range(len(target_list)):
            #print(target_list[target_index_check].target_status)
            #print("******")
            if(    (target_list[target_index_check].target_status )  and (  (x_to_class<=(target_list[target_index_check].current_x+1.5))  and (x_to_class>=(target_list[target_index_check].current_x-1.5))  )        and       (    (y_to_class<=(target_list[target_index_check].current_y+0.5)) and (y_to_class>=(target_list[target_index_check].current_y-1.5))     )   ):#if x and y fit this condition so they are related to this target
                match_flag = match_flag + 1
                matching_target = target_index_check
                list_for_neighbor_target.append(target_list[target_index_check])
                #print("we match the condition")
                #print("//////////////////")


        if (match_flag > 1):
                #print("runing priority algorithm")
                distance_min = m.sqrt((x_to_class-list_for_neighbor_target[0].current_x)*(x_to_class-list_for_neighbor_target[0].current_x) + (y_to_class-list_for_neighbor_target[0].current_y)*(y_to_class-list_for_neighbor_target[0].current_y))                    # min distance
                nearest_target_index = 0 
                for index_priority_target in range (len(list_for_neighbor_target)):
                    distance = m.sqrt((x_to_class-list_for_neighbor_target[index_priority_target].current_x)*(x_to_class-list_for_neighbor_target[index_priority_target].current_x) + (y_to_class-list_for_neighbor_target[index_priority_target].current_y)*(y_to_class-list_for_neighbor_target[index_priority_target].current_y))
                    if (distance<distance_min):
                        distance_min = distance
                        nearest_target_index = index_priority_target
                # add new measurement for the nearest target
                target_list[nearest_target_index].update(midpoint_cluster)
                
                
        elif (match_flag ==1):
            #add point to the only target
            target_list[matching_target].update(midpoint_cluster) 
            #print("add to the only target")
        
            
        else :
            #print("this is new targt")
            for target_index_check in range(len(target_list)):
                if ((target_list[target_index_check].target_status)==False):
                    #print("we find free target")
                    target_list[target_index_check].update(midpoint_cluster)
                    #print("new target updated ")
                    break 
                    
            
            #create new target
    



    
    
#***********************************
#********** main code **************
#***********************************


target1,target2,target3,target4,target5,target6,target7,target8,target9,target10,target11,target12,target13,target14,target15= target(color_table[0]),target(color_table[1]),target(color_table[2]),target(color_table[3]),target(color_table[4]),target(color_table[5]),target(color_table[6]),target(color_table[7]),target(color_table[8]),target(color_table[9]),target(color_table[10]),target(color_table[11]),target(color_table[12]),target(color_table[13]),target(color_table[14])
target_list = [target1,target2,target3,target4,target5,target6,target7,target8,target9,target10,target11,target12,target13,target14,target15]



# read and extract data from file
#save the text file in the same folder where this code is
#filename = '2_person_move1.txt'
filename='3outside.txt'
all_frames = read_extract_file(filename)
plt.ion()                                    # to run GUI event loop
figure, ax = plt.subplots(figsize=(10, 8))   # here we are creating sub plots
plt.title("Radar chart PADT output clustering & multi tracking without filter", fontsize=20) # setting title
plt.xlabel("X-axis  in m")
plt.ylabel("Y-axis  in m")
plt.xlim(-10,10)
plt.ylim(0,30)
#plt.xlim(-5,5)
#plt.ylim(0,10)
     
#while(1):
clustering()
#plot_target()




    
        



    
    
