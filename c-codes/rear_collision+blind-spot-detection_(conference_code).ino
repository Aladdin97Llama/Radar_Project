#include <Arduino.h>
#include <stdio.h> 
#include <MPU9250_asukiaaa.h>
#define RXD2   9         // 0 for esp32 C3 dev board   //IO16 FOR THE RADAR BOARD change 0 to 17
#define TXD2  10       //  1 for esp32 C3 dev board   //IO17 FOR THE RADAR BOARD change 1 to 1
#define SDA_PIN 14
#define SCL_PIN 13
#define speed_controller 3
const int buzzer = 26;          //2;           //2   /// esp32 wroom llama radar board   pin 26
//////////// IMU 
MPU9250_asukiaaa mySensor;
int loop_timer, a, aX, aY, aZ,dvx,dvy,dvz,vx,vy,vz,vt,dvt,time_acceleration_last,v_total, aSqrt, gX, gY,time_acceleration, gZ, mDirection, mX, mY, mZ,time_acceleration1,old_time;
bool timer_set = true ;  
int cal_int ,j,k; 
float gyro_X_cal, gyro_Y_cal, gyro_Z_cal ;
float pitch , roll,yaw;
float angle_pitch_acc, angle_roll_acc ; 
float acc_x,acc_y,acc_z,acc_total_vector;
uint8_t sensorId;

////SPEED 
int  revolutions=0;
int rpm=0; // max value 32,767 16 bit
int  startTime=0;
int  elapsedTime;
int i ;
float w_length = 2.070;
float lastturn ; 
float SPEED ;


/////DETECTION
float total_angle ;
float last_total_angle ;  
float turning[10];
bool turn_right_flag ;
bool turn_left_flag ; 
bool release_right_flag ;
bool release_left_flag ;
bool buzzer_turn_flag ;
bool buzzer_speed_flag ;
bool buzzer_stop_flag ; 


float speed_var[10];
float speed_var_total ; 
bool stop_flag ; 
int p ;
bool decreasing_flag  ;

///// speed controller IOT variable
byte controller[100];  
int frame_poss_c;


uint8_t start_frame[] = {0x49,0x4E,0x49,0x54,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00};    // for 115200 baud
uint8_t stop_frame[] =  {0x47,0x42,0x59,0x45,0x00,0x00,0x00,0x00};
uint8_t get_data[] = {0x47,0x4E,0x46,0x44,0x04,0x00,0x00,0x00,0x08,0x00,0x00,0x00};
uint8_t get_targets[] = {0x47,0x4E,0x46,0x44,0x04,0x00,0x00,0x00,0x04,0x00,0x00,0x00} ;

typedef struct
{
  uint16_t distance_FDAT ;
  uint16_t magnitude_FDAT;
  //int16_t speed_FDAT;
  float speed_FDAT;
  int16_t angle_FDAT;

} target_FDAT;

byte TADT[50] ;    //26       //////     check the correct seize !!!!!!!
byte FDAT[103];                       ///////////////////////////////////////////
target_FDAT Targets_list[96] ;        ///////////////////////////////////////////
int length_f ;
int FDAT_counter ;
int frame_poss ;
byte test ;
//uint16_t a ;
uint16_t distance_TADT ;
uint16_t magnitude_TADT;
int16_t  speed_TADT;
int16_t  angle_TADT;
bool buzzer_flag  ; 
int condition ;






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
  for(i=0;i<50;i++){
    TADT[i] = 0 ;
  }
}

void R_data(void *params){   /// ask for TDAT 
    //send start command to radar sensor 
   // Serial.println("Started radar task");
  while(1){
  clear_data();
  for(i=0; i<12; i++)
  {
    Serial1.write (get_data[i]); 
  }
  vTaskDelay(100/ portTICK_PERIOD_MS);
//  read_function();
read_TADT();
vTaskDelay(300/ portTICK_PERIOD_MS);

      /*for(j=0; j<25; j++){
       Serial.println(TADT[j],HEX) ;}*/     
}
}








void  read_function(){ //read target list bytes and print it to the serial monitor
  
  while(Serial1.available()>0){
     Serial.println( Serial1.read(),HEX);
       }
       Serial.println( ".");
            delay(10);
            }

            
void read_TADT(){
 
  while(Serial1.available()>0){
  TADT[frame_poss] = Serial1.read();
  //Serial.println( Serial1.read(),HEX);
     frame_poss++ ;
     frame_poss %=  sizeof(TADT);
       
     }
      /*for(j=0; j<25; j++){
       Serial.println(TADT[j],HEX) ;}*/
      if (TADT[13]==0x08){
      Serial.println(" target detected");
     /* for(j=17; j<25; j++){
       //Serial.println(TADT[j],HEX) ;
       }*/
        //frame_poss = 0 ;  
       TADT_decoder();
     }
     else {
      buzzer_flag = false ;
     }
     delay(10);
        frame_poss = 0 ;    
  }

  




void TADT_decoder(){
  distance_TADT =   (uint16_t)(TADT[18]<<8)| (uint16_t)TADT[17] ;
  speed_TADT =   (int16_t)(TADT[20]<<8)|(int16_t) TADT[19] ;
  angle_TADT =   (int16_t)(TADT[22]<<8)|(int16_t) TADT[21] ;
  magnitude_TADT =   (uint16_t)(TADT[24]<<8)| (uint16_t)TADT[23] ;
  speed_TADT = ((speed_TADT/100)*1.0) ;
  speed_TADT = speed_TADT; //- SPEED ;
  angle_TADT = (angle_TADT/100); 
  magnitude_TADT = (magnitude_TADT/100);
//  if ((angle_TADT>10 || angle_TADT<-10)&&((distance_TADT<500)&&(distance_TADT>80))&& (speed_TADT<-2)){
/*  if ((speed_TADT<0)){
 // k = k+1 ;

      buzzer_flag = true ;
      Serial.println("Alert target approach ");
     Serial.println("****************************************");
      Serial.println("Alert target approach ");
      Serial.println("target data : ");
        Serial.print("distance   ");
   // Serial.println(distance_TADT,HEX);
      Serial.print(distance_TADT);
      Serial.println("  Cm");
      Serial.print("speed      ");
    //   Serial.println(speed_TADT,HEX);
      Serial.print(speed_TADT);
      Serial.println("  Km/h") ;
      Serial.print("angle      ");
     // Serial.println(angle_TADT,HEX);
      Serial.print(angle_TADT);
      Serial.println("  deg");
     // 
      Serial.print("magnitude    ");
     //  Serial.println(magnitude_TADT,HEX);
      Serial.print(magnitude_TADT);
      Serial.println("  dB");*/
//delay(10); 
 //}


 // if((angle_TADT<-10)&&((distance_TADT<1000)&&(distance_TADT>80))&& (speed_TADT<-5)&&(turn_left_flag==true)){
 if((angle_TADT<-8)&& (speed_TADT<0)&&(turn_left_flag==true)&&(buzzer_stop_flag==false)&&((distance_TADT<1000)&&(distance_TADT>80))){
 //  if((angle_TADT<0)&& (speed_TADT<0)){
    buzzer_flag = true ;
  }
//  else if (( angle_TADT>10)&&((distance_TADT<1000)&&(distance_TADT>80))&& (speed_TADT<-5)&&(turn_right_flag==true)){
else if (( angle_TADT>8)&& (speed_TADT<0)&&((distance_TADT<1000)&&(distance_TADT>80)&&(turn_right_flag==true)&&(buzzer_stop_flag==false)){
//else if (( angle_TADT>0)&& (speed_TADT<0)){
    buzzer_flag = true ;
  }
 else if(((angle_TADT<20) && (angle_TADT>-20))&&((distance_TADT<1000)&&(distance_TADT>80))&& (speed_TADT<-3)&&(buzzer_speed_flag==true)){
// else if(((distance_TADT<1000)&&(distance_TADT>80))&& (speed_TADT<0)&&(buzzer_speed_flag==true)){
    buzzer_flag = true ;
  }
 else if(((angle_TADT<20) && (angle_TADT>-20))&&((distance_TADT<7000)&&(distance_TADT>40))&& (speed_TADT<-10)&&(buzzer_stop_flag==true)){
//  else if( (speed_TADT<0)&&(buzzer_stop_flag==true)){
   // Serial.println("vehiculee stoppedd");
    buzzer_flag = true ;
  }
  else {
    buzzer_flag = false ;
  }
  delay(10); 
}






void buzzerTask(void *params)
{
 // Serial.println("Started buzzer task");
  while (1)
  {
    if (buzzer_flag)
    {
     digitalWrite(buzzer, HIGH);
      vTaskDelay(100 / portTICK_PERIOD_MS);
     digitalWrite(buzzer, LOW);
      vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    else
    {
      digitalWrite(buzzer, LOW);
      vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    delay(10);
  }
}








void setup() {
 // Define pin modes for TX and RX
  Serial.begin(9600);  //print information using Arduino IDE Serial monitor 
  Serial1.begin(115200, SERIAL_8E1, RXD2, TXD2);     // USE same baud rate for K-LD7
   pinMode(buzzer, OUTPUT);
   digitalWrite(buzzer, LOW);
  // pinMode(speed_controller, INPUT_PULLUP);           // set pin to input
  // attachInterrupt(speed_controller,interruptFunction,RISING);
  delay(1000);
  Serial.println("start_command");  
  R_start();
  delay(2000);
  Serial.println("start_command");  
  R_start();
  delay(2000);
  Serial.println("start_command");  
  R_start();
  delay(2000);
  buzzer_flag = false ;
  Wire.begin(SDA_PIN, SCL_PIN);
  mySensor.setWire(&Wire);
  mySensor.beginGyro(0x08);
  mySensor.beginAccel(0x10);
  j = 0 ;
  delay(100);
  for(cal_int = 0 ; cal_int<2000; cal_int++){
     if (mySensor.gyroUpdate() == 0) {
          gyro_Z_cal += mySensor.gyroZ(); 
       //   gyro_X_cal += mySensor.gyroX();
        //  gyro_Y_cal += mySensor.gyroY();
          j=j+1 ; 
          delay(2);
          } 
  }
   //gyro_Z_cal /= 4000;
 // gyro_X_cal /= 2000 ; 
 // gyro_Y_cal /=2000;
  gyro_Z_cal /= 2000;
  Serial.print("calibration x  ");
  Serial.println( gyro_X_cal);
  Serial.print("calibration y  ");
  Serial.println(gyro_Y_cal);
    Serial.print("calibration z  ");
  Serial.print(gyro_Z_cal);
  Serial.println("calibration done");
  Serial.print("N° of iteration  ");
  Serial.println(j);  
  release_right_flag = true ; 
  release_left_flag = true ;
  buzzer_flag = false ;
   buzzer_turn_flag = false ;
 buzzer_speed_flag = false ;
  buzzer_stop_flag = true ;
  turn_right_flag = false ;
  turn_left_flag = false ; 
delay(10);
 // pinMode(3, INPUT_PULLUP);           // set pin to input
  delay(10);
 // attachInterrupt(3,interruptFunction,RISING);
  delay(10);
  xTaskCreate(&IMU_data, "IMU_data", 2048, NULL, 3, NULL);
  xTaskCreate(&speed_data, "speed_data Task", 2048, NULL, 2, NULL);
  xTaskCreate(&turning_detection, "turning_detection Task", 2048, NULL, 3, NULL);
  xTaskCreate(&speed_analyzer, "speed_analyzer Task", 2048, NULL, 3, NULL);
  xTaskCreate(&R_data, "R_data", 2048, NULL, 2, NULL);
  xTaskCreate(&buzzerTask, "Buzzer Task", 2048, NULL, 2, NULL);
  xTaskCreate(&read_data, "read_data", 2048, NULL, 2, NULL);
delay(10);

 digitalWrite(buzzer, HIGH);
 delay(300);
 digitalWrite(buzzer, LOW);
 delay(300);
 digitalWrite(buzzer, HIGH);
 delay(300);
 digitalWrite(buzzer, LOW);
 delay(300);
}


void loop()
{ 
//Serial.println("get_command");  
//R_data();
delay(10);
  
}

void IMU_data (void *params){
  Serial.println("Started IMU task");
  loop_timer = micros();
  while(1){
      mySensor.gyroUpdate();
      //aX = mySensor.gyroX()-gyro_X_cal;
     // aY = mySensor.gyroY()-gyro_Y_cal;
      aZ = mySensor.gyroZ()-gyro_Z_cal ;       
   // pitch += aX*0.00611 ; 
    // roll += aY*0.00611 ;
     pitch += aZ*0.00611 ;
//    Serial.print("   angle z       : " );
//    Serial.print(yaw);
//    Serial.print("   angle y       : " );
//    Serial.print(roll);
//    Serial.print("   angle z       : " );
//    Serial.println(pitch);
    while(micros() - loop_timer < 4000){
      }
    delay(1);
       loop_timer = micros();
       delay(5);
  }
}


void speed_data(void *params){
  Serial.println("Started SPEED task");
while(1){
      //  if((millis()-lastturn)>5000){
      if(SPEED==0){
        //  SPEED = 0 ;   
          stop_flag = true ; 
          buzzer_stop_flag = true ;   
          }
          else {
          stop_flag = false ; 
          buzzer_stop_flag = false ;    
          }

     
     //   Serial.print(SPEED);
      // Serial.println("Km/h   ");
       delay(10) ;
   }
}



void turning_detection(void *params){
   Serial.print("Started turning detection task");
  while(1){
    total_angle = 0 ;
    for(k=0;k<4;k++){
      delay(200);
      turning[k]=pitch ;
    }
    for(k=1;k<4;k++){
      total_angle = total_angle + (turning[k]-turning[k-1]);
    }
//    if (total_angle>2){
//      turn_left_flag = false ;
//      turn_right_flag = true ; 
//    /*  release_left_flag = true ; 
//      release_right_flag = false ;*/
//      Serial.print("   angle X       : " );
//      Serial.print(pitch);
//      Serial.println("    TURNING RIGHT");
//      buzzer_turn_flag = true ; 
//      
//    }
    if ((total_angle<-1)||(total_angle>1)){
      Serial.println(" TURNING "); 
      if(total_angle<-1){
      turn_left_flag = true ;
      turn_right_flag = false ; 
      Serial.println(" TURNING LEFT");
      }
      if(total_angle>1){
        turn_left_flag = false ;
      turn_right_flag = true ; 
      Serial.println("    TURNING RIGHT");
        
      }
//      turn_left_flag = true ;
//      turn_right_flag = false ; 
//      /*release_right_flag = true ;
//      release_left_flag = false ; */
//      Serial.print("   angle X       : " );
//      Serial.print(pitch);
//      Serial.println(" TURNING LEFT");
//      buzzer_turn_flag = true ; 
      
    }
    else{
      turn_left_flag = false ;
      turn_right_flag = false ; 
      buzzer_turn_flag = false ; 
     // Serial.println(" Straight");
      
      
    }
    delay(100);
  }
}


void speed_analyzer(void *params){
   Serial.println("Started speed analyzer task");
  while(1){
    speed_var_total = 0 ;
    for (p =0;p<2;p++){
      speed_var[p] = SPEED*3.6 ; 
    delay(400);
    }
   /* for(p=1;p<3;p++){
       speed_var_total = speed_var_total + (speed_var[p]-speed_var[p-1]);
    }*/
   speed_var_total = speed_var[1] - speed_var[0] ;
    if (speed_var_total<0){
      decreasing_flag = true ; 
      //Serial.print(SPEED*3.6);
      //  Serial.println("Km/h   ");
        delay(10) ;
       Serial.println ("************************* velocity decreasing*************************");
       buzzer_speed_flag = true ; 
      // Serial.print("   angle X       : " );
       //Serial.println(pitch);
    }
    else {
       decreasing_flag = false ; 
       buzzer_speed_flag = false ;
    }
  }
}




void read_data(void *params){
while(1){
  while(Serial.available()>0){
     controller[frame_poss_c] = Serial.read();
     frame_poss_c++ ;
     }

  if (controller[1]==0x10){
//    for(i=0;i<14;i++){
//      Serial.print(controller[i],HEX);
//      Serial.print(" ");
//    }
//    Serial.println("");
//    Serial.print("vehicule speed : ");
//    Serial.println(controller[4]);
    SPEED = controller[4];
   }
   erase_data() ; 
   frame_poss_c = 0 ;
   delay(200);
}
}




/////////////////////////////////////////////////////////////////// interruption to meseaure speed //////////////////////////////////////////////
void interruptFunction() //interrupt service routine
{
  buzzer_stop_flag = false ;  
  revolutions=revolutions+1 ;
   SPEED = (w_length / ((float)(millis() - lastturn) / 1000))*3.6 ;
   lastturn = millis();  
     
}




void erase_data(){
      for(i=0;i<100;i++){
      controller[i] = 0x00 ; 
    }
}

            
  
  
