#include <DBSCAN_LIB_LLAMA.h>

target_FDAT Targets_list[100] ; 


void setup() {
  // put your setup code here, to run once:
Serial.begin(115200);
for(int h = 0; h<11; h = h+1){
  Targets_list[h].x_FDAT = h +10  ; 
  Targets_list[h].y_FDAT = h + 20 ;
}
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
