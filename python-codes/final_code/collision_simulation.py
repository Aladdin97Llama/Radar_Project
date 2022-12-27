from matplotlib.patches import Rectangle
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



from filterpy.kalman import KalmanFilter
from scipy.linalg import block_diag
from filterpy.common import Q_discrete_white_noise
 
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
turning_counter = 0.0 
#this is 19 colors available to plot  19 targets at same time
color_table = ['blue','green','red','black','cyan','magenta','yellow','skyblue','orange','white','violet','purpel','pink','navy','deeppink','springgreen','peru','olive','beige']

target_inside_left_warning_zone = False
target_inside_right_warning_zone = False
target_inside_left_danger_zone = False
target_inside_right_danger_zone = False
target_inside_rear_danger_zone = False
scooter_turning_right_flag = False
scooter_turning_left_flag  = False




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
        self.current_x_kf = 0
        self.current_y_kf = 0 
        self.last_frame_number = 0
        self.angle_variance = 0
        self.total_angle_variance = 0
        self.update_points = []
        self.mu = np.array([0,0])
        self.kf_x = 0
        self.kf_y = 0
        self.kf_update = []
        self.new_measure = np.array([0,0])
        self.y_tolerance = 0
        self.a_coeff = 1.0
        self.b_coeff = 0
        self.x_projection = 0
        self.y_projection =0
        self.collision_coeff = 0.0
        self.warning_text = ""
        self.position_map = "0"
        
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
        self.current_x =target_new_data[0]
        self.current_y =target_new_data[1]
        # Kalman Filter process
        if(len(self.path_history)<1):
            self.tracker = Kalman_filter_init(target_new_data[0],target_new_data[1])
            #self.kf_update.append([target_new_data[0],target_new_data[1]])
            self.current_x_kf = target_new_data[0]
            self.current_x_kf = target_new_data[1]
        else :
            self.tracker.predict()
            self.tracker.update(np.array([target_new_data[0],target_new_data[1]]))
            #print("********************************************************************************************************   ")
            #print("this is x and y value before KF")
            #print([target_new_data[0],target_new_data[1]])
            #print("this is kf update result")
            #print(self.tracker.get_update())
            self.mu,_ = self.tracker.get_update()
            self.kf_update.append([self.mu[0],self.mu[2]])
            #self.current_x = self.mu[0]
            #self.current_y = self.mu[2]
            #print(self.kf_update)
            #print("********************************************************************************************************   ")
        #finish KF process 
        self.target_status = True
        #self.current_x =target_new_data[0]
        #self.current_y =target_new_data[1]
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
        #update the line path of the target
        if (len( self.path_history)>1):
            # line equation Y = a*X + b
            self.a_coeff = ((self.path_history[-1][1]-self.path_history[-2][1])/(self.path_history[-1][0]-self.path_history[-2][0]))  # a=(y1-y2)/(x1-x2)
            self.b_coeff = self.current_y - (self.a_coeff*self.current_x)                                       # b = Y1-a*X1
            self.collision_coeff = (-self.b_coeff)/self.a_coeff 
        self.x_projection = -(self.b_coeff/self.a_coeff)
        self.y_projection = 0
        #print all measurement for this target
        #print(" this is measurement for target with color " + self.target_color)
        #for i in range (len(self.path_history)):
            #print(self.path_history[i])
            


            
    def delete_old_target(self,current_frame_n):
        if (((current_frame_n - self.last_frame_number)>4)  and (len(self.path_history)>0)):
            delete_index = 0 
            self.target_status = False
            plt.clf()
            plt.title("Radar chart PADT output clustering & multi tracking & KF filter", fontsize=20) # setting title
            plt.xlabel("X-axis  in m")
            plt.ylabel("Y-axis  in m")

            plt.xlim(-10,10)
            plt.ylim(-5,30)
            #plt.xlim(-3,3)
            #plt.ylim(0,10)
            plot_aera ()
     
            #change color to white and hide all points
            #for delete_index in range(len(self.path_history)):
                #x_to_delete = self.path_history[delete_index][0]
                #y_to_delete = self.path_history[delete_index][1]
                #plt.scatter(x_to_delete,y_to_delete,color='white',s=45)
                #figure.canvas.draw()
                #figure.canvas.flush_events()
            self.path_history = []
            self.current_y = 0.0
            self.current_x = 0.0
            self.kf_x  = 0.0
            self.kf_y  = 0
            self.current_x_kf = 0.0
            self.current_y_kf = 0.0
            self.mu = np.array([0,0])
            self.kf_update = []
            self.x_projection = 0.0
            self.y_projection =0.0
            self.position_map = "0"
    def check_collision(self):

        #check target path predection 
        #if( (self.collision_coeff>-1)and (self.collision_coeff <1) ):   # when y = 0 we check if x exist in the range of [-1m, 1m]
        #check position of the target
        
        self.position_map = ""
        #check if target in  left warning zone
        if (  (self.current_y>4)  and  (self.current_y<10)   and  (self.current_x<2)   and  (self.current_x>0.5)  ):
            #print(" The " + self.target_color + "  target inside left warning zone  ")
            self.position_map = "3"
        #check if target in  right warning zone
        if (  (self.current_y>4)  and  (self.current_y<10)   and  (self.current_x>-2)   and  (self.current_x<-0.5) ):
            #print(" The " + self.target_color + "  target inside right warning zone  ")
            self.position_map = "4"
        #check if target in  right danger zone
        if (  (self.current_y>0)  and  (self.current_y<4)   and  (self.current_x<2)   and  (self.current_x>0.5)    ):
            #print(" The " + self.target_color + "  target inside left danger zone  ")
            self.position_map = "1"
        #check if target in  left danger zone
        if (  (self.current_y>0)  and  (self.current_y<4)   and  (self.current_x>-2)   and  (self.current_x<-0.5)  ):
            #print(" The " + self.target_color + "  target inside right danger zone  ")
            self.position_map = "2"
        #check if target in rear danger zone
        if (  (self.current_y>0)  and  (self.current_y<4)   and  (self.current_x>-0.5)   and  (self.current_x<0.5) ):
            #print(" The " + self.target_color + "  target inside rear danger zone  ")
            self.position_map = "5"




        # you need to calculate the time to reach     time to reach = distance/relative speed
        #time to reach = distance/relative speed
        #if time_to_reach > 3 :
            #this target could no be a real danger in the present
            #self.collision_time_flag = False
        #else :
            #this target could represent a collision danger if it have a collision path too    (priority for this type of collision over blindspot or rear collision
            #self.collision_time_flag = True
        



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
                        speed = float(z[1])/3.6
                        magnitude = float(z[3])
                        x_coordinate = distance*(m.cos(angle))
                        y_coordinate = distance*(m.sin(angle))
                        
                        if ((x_coordinate != 0)or(y_coordinate != 0 )):
                                cartesian_points.append([x_coordinate,y_coordinate,speed,angle_from_radar,magnitude,frame_freq])
            if(cartesian_points != []):
                    cartesian_points_final.append(cartesian_points)
    print("done extract list")
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
    time_frame = 0 
    for t in range(len(all_frames)):
            #time.sleep(1)
            #print("frame_freq")
            #print(all_frames[t][0][5])
            for target_index_delete in range(len(target_list)):
                target_list[target_index_delete].delete_old_target(all_frames[t][0][5])
            frame_number = all_frames[t][0][5]
            time_frame = frame_number*0.2          #radar sample at each 0.2s (arduino code for extract data)

            
            print("////////////////////////////////")
            print("************************************************************************************************************")
            print("     frame number    " + str(frame_number) + " : ")
            print( " current time   : "  + str(format(time_frame, ".2f")) + "  secondes")
            print("************************************************************************************************************")
            print("")
            #print("points collected  : ")
            #print(all_frames[t])

            
            points_to_cluster = []         # this list will take all the measurment (x,y) for one frame
            for index_i in range(len(all_frames[t])):   
                        points_to_cluster.append([cartesian_points_final[t][index_i][0],cartesian_points_final[t][index_i][1]])
            if(len(points_to_cluster)>1):
                cluster_per_frame = cluster_function(points_to_cluster,t)
                #print("those points are for filtering     " + str(len(cluster_per_frame)))
                for cluster_index_check in range (len(cluster_per_frame)):
                    #print(cluster_per_frame[cluster_index_check])
                    #print("************************************")
                    #check the position of each cluter
                    #print("this is the measurement for checking")
                    #print(cluster_per_frame[cluster_index_check])
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
            plot_target(t)   #plot target
    

#************************************************************************************************************
def cluster_function(points_to_cluster,t):
    midpoint_for_all = []
    #arguments
    #points_to_cluster  : list of (x,y) measurments coming from one frame
    #t index of this frame in the big list that contains all frame
    #print("point to cluster")
    #print(points_to_cluster)
    X = np.array(points_to_cluster)
    dz = DataFrame(dict(x=X[:,0],y=X[:,1]))
    clustering = DBSCAN(eps=2,min_samples=1).fit(X)
#eps : Epsilon is the radius of the circle to be created around each data point to check the density # in this case eps = 2m it's good for cars clustering but not for single person (because width of one person is about 0.5m)
#min_samples : is the minimum number of data points required inside that circle for that data point to be classified as a Core point # sould be equal to 1 because we except one single measurement
    
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
def plot_target(t):
    #pair number : turning right
    if((t%2)==0):
        scooter_turning_right_flag = True
        scooter_turning_left_flag  = False
    #unpair number : turning left
    else :
        scooter_turning_right_flag = False
        scooter_turning_left_flag  = True
    plt.clf()
    plt.title("Radar chart PADT output clustering & multi tracking & KF filter", fontsize=20) # setting title
    plt.xlabel("X-axis  in m")
    plt.ylabel("Y-axis  in m")
    plt.xlim(-10,10)
    plt.ylim(-5,30)
    plot_aera ()
    if (scooter_turning_right_flag):
        plt.text(-1.5,-2, '<==', fontsize = 10)
    else :
        plt.text(0.6,-2, '==>', fontsize = 10)
        
    plotting_points = []
    target_inside_left_warning_zone = False
    target_inside_right_warning_zone = False
    target_inside_left_danger_zone = False
    target_inside_right_danger_zone = False
    target_inside_rear_danger_zone = False 
    for target_index_plot in range(len(target_list)):
        #and ( len(target_list[target_index_plot].path_history) > 1 )
        if(   (target_list[target_index_plot].target_status) and ( len(target_list[target_index_plot].path_history)>2) ):
            target_list[target_index_plot].check_collision()
            #check if this target on collision path
            #check collision probability for this target
            if(target_list[target_index_plot].position_map == "4"):
                target_inside_right_warning_zone = True
                print(" The " + target_list[target_index_plot].target_color + "  target inside right warning zone  ")
            if(target_list[target_index_plot].position_map == "3"):
                target_inside_left_warning_zone = True
                print(" The " + target_list[target_index_plot].target_color + "  target inside left_warning_zone  ")
            if(target_list[target_index_plot].position_map == "1"):
                target_inside_left_danger_zone = True
                print(" The " + target_list[target_index_plot].target_color + "  target inside left_danger_zone  ")
            if(target_list[target_index_plot].position_map == "2"):
                target_inside_right_danger_zone = True
                print(" The " + target_list[target_index_plot].target_color + "  target inside right_danger_zone  ")
            if(target_list[target_index_plot].position_map == "5"):
                target_inside_right_danger_zone = True
                print(" The " + target_list[target_index_plot].target_color + "  target inside rear_danger_zone  ")
            #plot this target
            for target_index_plot_one_target in range(len(target_list[target_index_plot].kf_update)-1):         #for target_index_plot_one_target in range(len(target_list[target_index_plot].path_history)):
                #x_to_plot = np.array(target_list[target_index_plot].path_history[target_index_plot_one_target][0])
                #y_to_plot = np.array(target_list[target_index_plot].path_history[target_index_plot_one_target][1])
                x_to_plot =np.array(target_list[target_index_plot].kf_update[target_index_plot_one_target][0])
                y_to_plot = np.array(target_list[target_index_plot].kf_update[target_index_plot_one_target][1])
                plotting_points = plt.scatter(x_to_plot,y_to_plot,color=target_list[target_index_plot].target_color,s=30)
                x1 = target_list[target_index_plot].kf_update[target_index_plot_one_target][0]
                x2= target_list[target_index_plot].kf_update[target_index_plot_one_target+1][0]
                y1= target_list[target_index_plot].kf_update[target_index_plot_one_target][1]
                y2= target_list[target_index_plot].kf_update[target_index_plot_one_target+1][1]
                plt.plot ([x1,x2],[y1,y2],color=target_list[target_index_plot].target_color,linewidth=0.5)
                figure.canvas.draw()
                figure.canvas.flush_events()
            #ploting the last point
            x_to_plot =np.array(target_list[target_index_plot].kf_update[-1][0])
            y_to_plot = np.array(target_list[target_index_plot].kf_update[-1][1])
            x2_to_plot = np.array(target_list[target_index_plot].x_projection)
            y2_to_plot = np.array(target_list[target_index_plot].y_projection)
            x111 = target_list[target_index_plot].kf_update[-1][0][0]
            x222 = target_list[target_index_plot].x_projection
            y111 = target_list[target_index_plot].kf_update[-1][1][0]
            y222 = target_list[target_index_plot].y_projection
            plt.scatter(x_to_plot,y_to_plot,color=target_list[target_index_plot].target_color,s=30)
            plt.plot ([x111,x222],[y111,y222],color='grey',linewidth=0.5,linestyle='dashdot')  # plot the line projection for each target

            if(scooter_turning_right_flag and (target_inside_right_warning_zone or target_inside_right_danger_zone)):
                print("   WARNING  TARGETS ARE COMING FROM THIS SIDE")
                plt.text(-5,-3, 'WARNING', fontsize = 10)
            if(scooter_turning_left_flag and (target_inside_left_warning_zone or target_inside_left_danger_zone)):
                print("   WARNING  TARGETS ARE COMING FROM THIS SIDE")
                plt.text(5,-3, 'WARNING', fontsize = 10)
                
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
            #if(    (target_list[target_index_check].target_status )  and (  (x_to_class<=(target_list[target_index_check].kf_update[-1][0]+1.5))  and (x_to_class>=(target_list[target_index_check].kf_update[-1][0]-1.5))  )        and       (    (y_to_class<=(target_list[target_index_check].kf_update[-1][1]+0.5)) and (y_to_class>=(target_list[target_index_check].kf_update[-1][1]-1.5))     )   ):#if x and y fit this condition so they are related to this target
            if(    (target_list[target_index_check].target_status )  and (  (x_to_class<=(target_list[target_index_check].current_x+1.5))  and (x_to_class>=(target_list[target_index_check].current_x-1.5))  )        and       (    (y_to_class<=(target_list[target_index_check].current_y)) and (y_to_class>=(target_list[target_index_check].current_y-2))     )   ):#if x and y fit this condition so they are related to this target
                match_flag = match_flag + 1
                matching_target = target_index_check
                list_for_neighbor_target.append(target_list[target_index_check])
        if (match_flag > 1):
            for first_search in range (len(list_for_neighbor_target)-1):
                nearest_target_index = 0 
                if (len(list_for_neighbor_target[first_search].path_history)<len(list_for_neighbor_target[first_search+1].path_history)):
                    nearest_target_index = first_search+1
                
                #print("runing priority algorithm")
                #distance_min = m.sqrt((x_to_class-list_for_neighbor_target[0].current_x)*(x_to_class-list_for_neighbor_target[0].current_x) + (y_to_class-list_for_neighbor_target[0].current_y)*(y_to_class-list_for_neighbor_target[0].current_y))                    # min distance
                #nearest_target_index = 0 
                #for index_priority_target in range (len(list_for_neighbor_target)):
                    #distance = m.sqrt((x_to_class-list_for_neighbor_target[index_priority_target].current_x)*(x_to_class-list_for_neighbor_target[index_priority_target].current_x) + (y_to_class-list_for_neighbor_target[index_priority_target].current_y)*(y_to_class-list_for_neighbor_target[index_priority_target].current_y))
                    #if (distance<distance_min):
                        #distance_min = distance
                        #nearest_target_index = index_priority_target
                # add new measurement for the nearest target
            list_for_neighbor_target[nearest_target_index].update(midpoint_cluster)
                
                
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

#************************************************************************************************************
#create the Kalman filter object with all Parameter needed and IC
def Kalman_filter_init(initial_x,initial_y):
    R_std = 0.35
    Q_std = 0.04
    #R_std = 1
    #Q_std = 0.1
    tracker = KalmanFilter(dim_x=4, dim_z=2)
    dt = 0.2   # time step

    tracker.F = np.array([[1, dt, 0,  0],
                          [0,  1, 0,  0],
                          [0,  0, 1, dt],
                          [0,  0, 0,  1]])
    tracker.u = 0.
    tracker.H = np.array([[1, 0, 0, 0],
                          [0, 0, 1, 0]])

    tracker.R = np.eye(2) * R_std**2
    q = Q_discrete_white_noise(dim=2, dt=dt, var=2**2)
    tracker.Q = block_diag(q, q)
    tracker.x = np.array([[initial_x, 0, initial_y, 0]]).T
    tracker.P = np.eye(4) * 500.
    return tracker

#************************************************************************************************************
def plot_aera ():
         # plot danger zone on right side 
         plt.plot ([-2,-0.5],[0,0],color='orange',linewidth=1)
         plt.plot ([-2,-2],[0,4],color='orange',linewidth=1)
         plt.plot ([-0.5,-0.5],[0,4],color='orange',linewidth=1)
         plt.plot ([-2,-0.5],[4,4],color='orange',linewidth=1)
         #plt.text(-3,2, 'BlindSpot-danger-zone', fontsize = 6)
         # plot danger zone on left side
         plt.plot ([0.5,2],[0,0],color='orange',linewidth=1)
         plt.plot ([0.5,0.5],[0,4],color='orange',linewidth=1)
         plt.plot ([2,2],[0,4],color='orange',linewidth=1)
         plt.plot ([0.5,2],[4,4],color='orange',linewidth=1)
         #plt.text(0.4,2, 'BlindSpot-danger-zone', fontsize = 6)
         #plot warning zone on right side
         plt.plot ([-2,-2],[4,10],color='yellow',linewidth=1)
         plt.plot ([-2,-0.5],[10,10],color='yellow',linewidth=1)
         plt.plot ([-0.5,-0.5],[10,4],color='yellow',linewidth=1)
         #plt.text(-3,7, 'BlindSpot-warning-zone', fontsize = 6)
         #plot warning zone on left side
         plt.plot ([0.5,0.5],[4,10],color='yellow',linewidth=1)
         plt.plot ([0.5,2],[10,10],color='yellow',linewidth=1)
         plt.plot ([2,2],[10,4],color='yellow',linewidth=1)
         #plt.text(0.4,7, 'BlindSpot-warning-zone', fontsize = 6)
         #plt.legend(["blind-spot-danger zone", "blind-spod-warning-zone"], loc ="lower right")
         # plot scooter aera   (0.5m by 1.5m @ the origin )
         plt.plot ([-0.25,0.25],[0,0],color='red',linewidth=1)
         plt.plot ([-0.25,-0.25],[0,-2],color='red',linewidth=1)
         plt.plot ([0.25,0.25],[0,-2],color='red',linewidth=1)
         plt.plot ([-0.25,0.25],[-2,-2],color='red',linewidth=1)
         plt.text(-0.25,-3, 'Scooter', fontsize = 10)
         figure.canvas.draw()
         figure.canvas.flush_events()
         


    
    

    
    
#***********************************
#********** main code **************
#***********************************

# create many target in list  (each one have unique color)
target1,target2,target3,target4,target5,target6,target7,target8,target9,target10,target11,target12,target13,target14,target15= target(color_table[0]),target(color_table[1]),target(color_table[2]),target(color_table[3]),target(color_table[4]),target(color_table[5]),target(color_table[6]),target(color_table[7]),target(color_table[8]),target(color_table[9]),target(color_table[10]),target(color_table[11]),target(color_table[12]),target(color_table[13]),target(color_table[14])
target_list = [target1,target2,target3,target4,target5,target6,target7,target8,target9,target10,target11,target12,target13,target14,target15]



# read and extract data from file
#save the text file in the same folder where this code is
#filename = '2_person_move1.txt'
#filename='2person_1.txt'
#filename='radar_output.txt'


filename='dataa2.txt'
#filename='dataa2.txt'

all_frames = read_extract_file(filename)
plt.ion()                                    # to run GUI event loop
figure, ax = plt.subplots(figsize=(10, 8))   # here we are creating sub plots
plt.title("Radar chart PADT output clustering & multi tracking & KF filter", fontsize=20) # setting title
plt.xlabel("X-axis  in m")
plt.ylabel("Y-axis  in m")
plt.xlim(-10,10)
plt.ylim(-5,30)
#plt.xlim(-3,3)
#plt.ylim(0,10)
plot_aera ()
     
#while(1):
clustering()
#plot_target()




    
        



    
    
