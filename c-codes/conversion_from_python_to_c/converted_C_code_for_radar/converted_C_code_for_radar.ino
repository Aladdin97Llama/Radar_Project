#include <Arduino.h>
#include <stdio.h> 
#include <stdlib.h>
#include <DBSCAN_LIB_LLAMA.h>
#include <limits.h>
#include <math.h>


//PIN DEFINITION
#define RXD2 22 // 0 for esp32 C3 dev board   //IO16 FOR THE RADAR BOARD change 0 to 17
#define TXD2 23 //  1 for esp32 C3 dev board   //IO17 FOR THE RADAR BOARD change 1 to 1

// Structures 

//// this structure is defined in DBSCAN_LIB_LLAMA.h file
/*typedef struct
{ float distance_FDAT ;
  uint16_t magnitude_FDAT;
  float x_FDAT;
  float y_FDAT; 
  float speed_FDAT;
  float angle_FDAT;
  uint8_t group_id;
  uint32_t frame_num ;
} target_FDAT;*/

// Variables
const int buzzer = 3;           //2   /// esp32 wroom llama radar board   pin 26
String c = "0x78"; 
uint8_t start_frame[] = {0x49,0x4E,0x49,0x54,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00};    // for 115200 baud
uint8_t stop_frame[] =  {0x47,0x42,0x59,0x45,0x00,0x00,0x00,0x00};
uint8_t get_data[] = {0x47,0x4E,0x46,0x44,0x04,0x00,0x00,0x00,0x08,0x00,0x00,0x00};
uint8_t get_targets[] = {0x47,0x4E,0x46,0x44,0x04,0x00,0x00,0x00,0x04,0x00,0x00,0x00} ;
int table_index ;
byte TADT[50] ;    //26       //////     check the correct seize !!!!!!!
byte FDAT[2000];      ///103                    ///////////////////////////////////////////
target_FDAT Targets_list[100] ;        ///////////////////////////////////////////
int i,length_f,j,k ;
int FDAT_counter ;
int frame_poss ;
byte test ;
uint16_t a ;
uint16_t distance_TADT ;
uint16_t magnitude_TADT;
int16_t  speed_TADT;
int16_t  angle_TADT;
bool buzzer_flag  ; 
int condition ;
point_t *points;
double epsilon;
unsigned int minpts;
unsigned int num_points;

// CLASS



// FUNCTIONS 
void R_start (){
  //send start command to radar sensor 
  for(i=0; i<12; i++)
  {
    Serial1.write (start_frame[i]); 
  }
  vTaskDelay(1 / portTICK_PERIOD_MS);
  read_function();
}

void clear_data(){
  for(i=0;i<501;i++){
    FDAT[i] = 0 ;
  }
}


void clear_data1(){
  for(i=0;i<101;i++){
  Targets_list[FDAT_counter].distance_FDAT = 0 ;
  Targets_list[FDAT_counter].speed_FDAT = 0;
  Targets_list[FDAT_counter].angle_FDAT =  0 ;
  Targets_list[FDAT_counter].magnitude_FDAT =0;
  }
}




void R_targets(void *params){  /// ask for FDAT
    //send start command to radar sensor 
    
 Serial.println("Started radar task");
  while(1){
                                                   // you need to measure this period starting from here  
  clear_data();
  for(i=0; i<12; i++)
  {
    Serial1.write (get_targets[i]); 
  }
 read_FDAT();
                                                 // to here                 => reduce the vtaskdelay (180ms)
 vTaskDelay(180/ portTICK_PERIOD_MS);
}
}





void  read_function(){ //read target list bytes and print it to the serial monitor
  
  while(Serial1.available()>0){
     Serial.println( Serial1.read(),HEX);
       }
       Serial.println( ".");
            delay(5);
            }



            
void read_FDAT(){
 
  while(Serial1.available()>0){
  //Serial.println( Serial1.read(),HEX);
  FDAT[frame_poss] = Serial1.read();
 // Serial.print(FDAT[frame_poss],HEX);
 // Serial.print(" ");
  frame_poss++ ;
  frame_poss %= sizeof(FDAT);
     }
   // Serial.println("");
       /*for(j=0; j<25; j++){
       Serial.println(TADT[j],HEX) ;}*/
      if (FDAT[13]!=0x00){
     // Serial.println(" N° of target detected  ");
     // Serial.println(FDAT[13]/8);
     /* for(j=17; j<103; j++){           /// to print frame that contains all targets data in HEX
       //Serial.println(FDAT[j],HEX) ;
       }*/
     FDAT_decoder() ;       /// decode
     }
     else {
      Serial.print(0);
      Serial.print(";");
      Serial.print(0);
      Serial.print(";");
      Serial.print(0);
      Serial.print(";");
      Serial.print(0);
      Serial.print("%");
      Serial.print("/");
      buzzer_flag = false ;
     }
     delay(1);
        frame_poss = 0 ;    
  }

  




void FDAT_decoder(){
  clear_data1();
  j = 17 ;
 // condition = ((8*FDAT[13])+1) ;
 condition = FDAT[13]/8 ;
  FDAT_counter =0 ;
for(int h=0; h<condition; h=h+1){
//for(j=17; j<condition; j=j+8){
  Targets_list[FDAT_counter].distance_FDAT =    (uint16_t)(FDAT[j+1]<<8)| (uint16_t)FDAT[j] ;
  Targets_list[FDAT_counter].speed_FDAT =    (int16_t)(FDAT[j+3]<<8)|(int16_t) FDAT[j+2] ;
  Targets_list[FDAT_counter].speed_FDAT = (Targets_list[FDAT_counter].speed_FDAT/100)*1.0;
  Targets_list[FDAT_counter].angle_FDAT =    (int16_t)(FDAT[j+5]<<8)|(int16_t) FDAT[j+4] ;
  Targets_list[FDAT_counter].angle_FDAT = Targets_list[FDAT_counter].angle_FDAT/100 ;
  Targets_list[FDAT_counter].magnitude_FDAT =    (uint16_t)(FDAT[j+7]<<8)| (uint16_t)FDAT[j+6] ;
  Targets_list[FDAT_counter].magnitude_FDAT = Targets_list[FDAT_counter].magnitude_FDAT /100 ;
  Targets_list[FDAT_counter].distance_FDAT = Targets_list[FDAT_counter].distance_FDAT/100    ;       // convert from cm to m 
  Targets_list[FDAT_counter].angle_FDAT = (90-Targets_list[FDAT_counter].angle_FDAT)*0.0174533  ;    // convert from degree to radian
  Targets_list[FDAT_counter].x_FDAT = Targets_list[FDAT_counter].distance_FDAT*cos(Targets_list[FDAT_counter].angle_FDAT);
  Targets_list[FDAT_counter].y_FDAT = Targets_list[FDAT_counter].distance_FDAT*sin(Targets_list[FDAT_counter].angle_FDAT);
  FDAT_counter = FDAT_counter + 1 ;
  j = j+8 ;
}


 for(j=0; j<condition; j++){
  if ((Targets_list[j].speed_FDAT<0)){
    buzzer_flag= true ;
   
  //k = k+1 ; 
 /* Serial.println("****************************************");
  Serial.print("Target N° ");
  Serial.println(k);
  Serial.print("distance   ");*/
 // Serial.println(distance_TADT,HEX);
  Serial.print(Targets_list[j].distance_FDAT);
  Serial.print(";");
 // Serial.print("speed      ");
//   Serial.println(speed_TADT,HEX);
  Serial.print(Targets_list[j].speed_FDAT);
  Serial.print(";") ;
 // Serial.print("angle      ");
 // Serial.println(angle_TADT,HEX);
  Serial.print(Targets_list[j].angle_FDAT);
  Serial.print(";");
 // 
/// Serial.print("magnitude    ");
 //  Serial.println(magnitude_TADT,HEX);
  Serial.print(Targets_list[j].magnitude_FDAT);
  Serial.print("%");

}
else{
  buzzer_flag = false ;
}

}
// clustering 
num_points =  condition ; 
parse_input(&points,Targets_list,num_points);
dbscan(points, num_points, epsilon,minpts, euclidean_dist);
int max_cluster  = 0 ; 
for (int count_1 = 0 ; count_1 <condition ; count_1 ++ )
{
  Targets_list[count_1].group_id = points[count_1].cluster_id ;
  if(points[count_1].cluster_id>= max_cluster)
  {
    max_cluster = points[count_1].cluster_id ; // get the maximum number of clusters 
  }
  
}
free(points);  // should be called directly after allocation 
uint8_t index_cluster[11];
target_FDAT cluster_all[11][11] ; 
for (int count_2 = 0 ; count_2 < condition ; count_2 ++)
{
  switch(Targets_list[count_2].group_id)
  { // radar can give max of 12 targets in one time => max cluster = 12 
    case 0 :  // cluster 0 
        cluster_all[0][index_cluster[0]] = Targets_list[count_2] ; // execute each case for cluster instead of all frames 
        index_cluster[0] = index_cluster[0] +1 ;
        break ;
    case 1 :  // cluster 1 
        cluster_all[1][index_cluster[1]] = Targets_list[count_2] ;
        index_cluster[1] = index_cluster[1] +1 ;
        break ;
    case 2 :  // cluster 2 
        cluster_all[2][index_cluster[2]] = Targets_list[count_2] ;
        index_cluster[2] = index_cluster[2] +1 ;
        break ;
    case 3 :  // cluster 3
        cluster_all[3][index_cluster[3]] = Targets_list[count_2] ;
        index_cluster[3] = index_cluster[3] +1 ;
        break ;
    case 4 :  // cluster 4 
        cluster_all[4][index_cluster[4]] = Targets_list[count_2] ;
        index_cluster[4] = index_cluster[4] +1 ;
        break ;
    case 5 :  // cluster 5 
        cluster_all[5][index_cluster[5]] = Targets_list[count_2] ;
        index_cluster[5] = index_cluster[5] +1 ;
        break ;
    case 6 :  // cluster 6 
        cluster_all[6][index_cluster[6]] = Targets_list[count_2] ;
        index_cluster[6] = index_cluster[6] +1 ;
        break ;
    case 7 :  // cluster 7 
        cluster_all[7][index_cluster[7]] = Targets_list[count_2] ;
        index_cluster[7] = index_cluster[7] +1 ;
        break ;
    case 8 :  // cluster 8 
        cluster_all[8][index_cluster[8]] = Targets_list[count_2] ;
        index_cluster[8] = index_cluster[8] +1 ;
        break ;
    case 9:   // cluster 9 
        cluster_all[9][index_cluster[9]] = Targets_list[count_2] ;
        index_cluster[9] = index_cluster[9] +1 ;
        break ;
    case 10 : // cluster 10 
        cluster_all[10][index_cluster[10]] = Targets_list[count_2] ;
        index_cluster[10] = index_cluster[10] +1 ;
        break ;
    case 11 : // cluster 11
        cluster_all[11][index_cluster[11]] = Targets_list[count_2] ;
        index_cluster[11] = index_cluster[11] +1 ;
        break ;
    default : 
        Serial.print("  ERROR  : more than 12 targets founded during clustering  ! ");
  }
}
// midpoint Algorithm 
for (int count_3 = 0 ;  count <max_cluster+1 ; count_3 ++ )
{
  midpoints_table[i] = midpoint_function(cluster_all[i],index_cluster[i]); 
}





Serial.print("/");
k = 0 ;
  delay(10); 
}



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
//  point_t *points;
//  double epsilon;
//  unsigned int minpts;
//  unsigned int num_points;
  epsilon = 2 ;
  minpts  = 0 ;
  num_points = 0 ;
 // Define pin modes for TX and RX
  Serial.begin(115200);  //print information using Arduino IDE Serial monitor 
  Serial1.begin(115200, SERIAL_8E1, RXD2, TXD2);     // USE same baud rate for K-LD7
  pinMode(buzzer, OUTPUT);
  digitalWrite(buzzer, HIGH);
  //delay(1000);
  Serial.println("start_command");  
  R_start();
  delay(400);
  Serial.println("start_command");  
  R_start();
  delay(400);
  Serial.println("start_command");  
  R_start();
  delay(400);
  /*for (int j = 0 ; j<11 ; j++){
      Targets_list[j].x_FDAT = j +0.01 ;
      Targets_list[j].y_FDAT = j + 0.01 ; 
    }*/
  buzzer_flag = false ;
  xTaskCreate(&R_targets, "R_targets", 2048, NULL, 2, NULL);
}


void loop()
{ 
delay(10);  
}

















// code for clustering 
/*parse_input(&points,Targets_list,num_points);
    if (num_points) {
        dbscan(points, num_points, epsilon,minpts, euclidean_dist);
        printf("Epsilon: %lf\n", epsilon);
        printf("Minimum points: %u\n", minpts);
        print_points(points, num_points);
        }
    free(points);*/
