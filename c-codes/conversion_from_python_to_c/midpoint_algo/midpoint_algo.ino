#include <Arduino.h>
#include <stdio.h> 
#include <stdlib.h>


typedef struct
{ float distance_FDAT ;
  uint16_t magnitude_FDAT;
  float x_FDAT;
  float y_FDAT; 
  float speed_FDAT;
  float angle_FDAT;
  uint8_t group_id;
  uint32_t frame_num ;
} target_FDAT;



target_FDAT midpoint_table[10] ; 
target_FDAT cluster_all[7] ; 
//target_FDAT midpoints []:



target_FDAT midpoint_function( target_FDAT cluster_u[]   ,uint8_t index)
{
  target_FDAT point_1 ;
  target_FDAT point_2 ; 
  target_FDAT final_point ;
  double dist_min ;
  double x , y ; 
  double dist ;
  if (index> 1)   /// apply midpoint algorithm for more than one points 
  {
    // midpoint core
    // reference points 
    dist_min =  sqrt(((cluster_u[0].x_FDAT-cluster_u[1].x_FDAT)*(cluster_u[0].x_FDAT-cluster_u[1].x_FDAT))+((cluster_u[0].y_FDAT-cluster_u[1].y_FDAT)*(cluster_u[0].y_FDAT-cluster_u[1].y_FDAT)));
    point_1 =cluster_u[0];
    point_2 =cluster_u[1];
    for ( int i = 0 ; i < index-1; i++ )
        {
          for (int t = i+1 ; t < index; t++)
              {
                dist =  sqrt(((cluster_u[i].x_FDAT-cluster_u[t].x_FDAT)*(cluster_u[i].x_FDAT-cluster_u[t].x_FDAT))+((cluster_u[i].y_FDAT-cluster_u[t].y_FDAT)*(cluster_u[i].y_FDAT-cluster_u[t].y_FDAT))) ; 
                if (dist< dist_min)
                   {
                    dist_min = dist ; 
                    point_1 = cluster_u[i] ; 
                    point_2 = cluster_u[t] ; 
                   }
              }
        }
   // to get one point; make the average of those two points 
   Serial.println("results after midpoint Algorithm");
   x = (point_1.x_FDAT + point_2.x_FDAT) /2 ;
   y = (point_1.y_FDAT + point_2.y_FDAT) /2 ; 
   final_point.x_FDAT  = (point_1.x_FDAT + point_2.x_FDAT) /2 ;
   final_point.y_FDAT = (point_1.y_FDAT + point_2.y_FDAT) /2 ; 
   final_point.magnitude_FDAT =( point_1.magnitude_FDAT + point_2.magnitude_FDAT ) / 2 ;
   final_point.speed_FDAT =( point_1.speed_FDAT + point_2.speed_FDAT ) / 2 ;
   final_point.distance_FDAT = ( point_1.distance_FDAT + point_2.distance_FDAT ) / 2 ;
   final_point.group_id = point_1.group_id ;
   final_point.frame_num = point_1.frame_num ; 
   Serial.print (" x  = " );
   Serial.print(x);
   Serial.println("");
   Serial.print(" y  = " ) ;
   Serial.print(y);
   Serial.println("");
     
  
  }

  else  /// just one point 
  {
    Serial.println ("this is just one measurement no treatment needed ") ; 
    final_point =  cluster_u[0];
    
  }
  return final_point ; 

}

void setup() {
  
Serial.begin(9600) ; 
cluster_all[0].x_FDAT = 1 ; 
cluster_all[0].y_FDAT =  1 ; 

cluster_all[1].x_FDAT = 1 ;
cluster_all[1].y_FDAT = 5;

cluster_all[2].x_FDAT = 2 ;
cluster_all[2].y_FDAT = -7 ;

cluster_all[3].x_FDAT = 5 ;
cluster_all[3].y_FDAT = 6 ; 

cluster_all[4].x_FDAT =  5.25 ;
cluster_all[4].y_FDAT =  6.25 ; 

cluster_all[5].x_FDAT = 10 ;
cluster_all[5].y_FDAT =  13 ; 

cluster_all[6].x_FDAT =  -9 ; 
cluster_all[6].y_FDAT = 10 ; 

}

void loop() {
 midpoint_table[0] =  midpoint_function (cluster_all, sizeof(cluster_all)/sizeof(cluster_all[0]) );
  delay(500);
  Serial.println("------------");
  for (int z = 0 ; z<7 ; z++)
  {    Serial.println(cluster_all[z].x_FDAT);
       Serial.println(cluster_all[z].y_FDAT);
  }
  
  Serial.println("------------");

}
