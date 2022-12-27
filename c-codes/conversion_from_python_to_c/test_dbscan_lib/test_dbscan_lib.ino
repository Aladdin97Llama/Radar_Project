/* Copyright 2015 Gagarine Yaikhom (MIT License) */
#include <DBSCAN_LIB_LLAMA.h>
#include <limits.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>


/*typedef struct
{ float distance_FDAT ;
  uint16_t magnitude_FDAT;
  float x_FDAT;
  float y_FDAT; 
  float speed_FDAT;
  float angle_FDAT;
  uint8_t group_id;
} target_FDAT;*/

target_FDAT Targets_list[100];


void setup()
{
    Serial.begin(115200);  //print information using Arduino IDE Serial monitor 
    pinMode(23, OUTPUT);
}
void loop() {
    point_t *points;
    double epsilon;
    unsigned int minpts;
    unsigned int num_points;
    epsilon = 2.0 ;
    minpts  = 1 ;
    num_points = 10 ;
    parse_input(&points,Targets_list,num_points);
    if (num_points) {
        dbscan(points, num_points, epsilon,minpts, euclidean_dist);
        printf("Epsilon: %lf\n", epsilon);
        printf("Minimum points: %u\n", minpts);
        print_points(points, num_points);

    }
    free(points);
    
    
}
