#include <ros.h>
#include <geometry_msgs/Twist.h>

#include <std_msgs/Int64.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <ros/time.h> 

// Front Right Wheel 
#define fr_right_EN         5   // PWM
#define fr_right_in1       24 //25
#define fr_right_in2        26 //24
#define fr_right_encod_A       0     // Pin 2
#define fr_right_encod_B        9    // Pin 9

// Front Left Wheel 
#define fr_left_EN         6   // PWM
#define fr_left_in1       25 //26
#define fr_left_in2        27 //27
#define fr_left_encod_A       1     // Pin 3
#define fr_left_encod_B       10     // Pin 10

// Rear Right Wheel 
#define re_right_EN        7   // PWM
#define re_right_in1       29
#define re_right_in2        30
#define re_right_encod_A      2     // Pin 21
#define re_right_encod_B       11     // Pin 11

// Front Left Wheel 
#define re_left_EN         8   // PWM
#define re_left_in1       36
#define re_left_in2        31
#define re_left_encod_A       3     // Pin 20
#define re_left_encod_B       12    // Pin 12

ros::NodeHandle  nh;

signed long fr_right_pose_encod = 0  ;
signed long fr_left_pose_encod= 0 ;

signed long re_right_pose_encod = 0;
signed long re_left_pose_encod = 0;

signed long data_pose_encod[4] = {0,0,0,0};

int list_in_01[4] ={fr_right_in1,fr_left_in1,re_right_in1,re_left_in1};
int list_in_02[4] ={fr_right_in2,fr_left_in2,re_right_in2,re_left_in2};

int list_EN[4] = {fr_right_EN,fr_left_EN,re_right_EN,re_left_EN};

int list_direct[4] = {0,0,0,0};
int list_duty[4] = {0,0,0,0};


float tsamp = 0.02;

std_msgs::Int32MultiArray array_pose_encod;     // Publisher array pose encoder
ros::Publisher pub_array_pose_encod("pose_encod",&array_pose_encod);

std_msgs::Float32MultiArray array_vels_robot;     // Publisher array pose encoder
ros::Publisher pub_array_vels_robot("vels_robot",&array_vels_robot);


std_msgs::Float32MultiArray array_vels_wheels;     // Publisher array pose encoder
ros::Publisher pub_array_vels_wheels("vels_wheels",&array_vels_wheels);

std_msgs::Float32MultiArray array_vels_enc;     // Publisher array pose encoder
ros::Publisher pub_array_vels_enc("vels_enc",&array_vels_enc);



//void cmd_vel_Cb( const geometry_msgs::Twist& data_Twist){
//  linear_x = data_Twist.linear.x;
//  angular_z = data_Twist.angular.z;
//
//  vel_right_wheel = ((2 * linear_x) + (angular_z * 235 / 1000)) / 2 ;
//  vel_left_wheel = ((2 * linear_x) - (angular_z * 235 / 1000)) / 2 ;
//
//}

void duty_Cb( const std_msgs::Int32MultiArray&  data){

  list_duty[0] = data.data[0];
  list_duty[1] = data.data[1];
  list_duty[2] = data.data[2];
  list_duty[3] = data.data[3];
}

void direct_Cb( const std_msgs::Int32MultiArray&  msg){

  list_direct[0] = msg.data[0];
  list_direct[1] = msg.data[1];
  list_direct[2] = msg.data[2];
  list_direct[3] = msg.data[3];

  //  set_pwm_right_motor(vel_right_wheel,duty_right);
//  set_pwm_left_motor(vel_left_wheel,duty_left);
  set_pwm_motor(0,list_direct[0],list_duty[0]);
  set_pwm_motor(1,list_direct[1],list_duty[1]);
  set_pwm_motor(2,list_direct[2],list_duty[2]);
  set_pwm_motor(3,list_direct[3],list_duty[3]);
}


//ros::Subscriber<geometry_msgs::Twist> sub_cmd_vel("/cmd_vel",cmd_vel_Cb);

ros::Subscriber<std_msgs::Int32MultiArray> sub_duty_motor("/duty_motor",duty_Cb);
ros::Subscriber<std_msgs::Int32MultiArray> sub_direct_motor("/direct_motor",direct_Cb);

void setup() {
  // put your setup code here, to run once:
  nh.initNode();
    nh.advertise(pub_array_pose_encod);
    nh.advertise(pub_array_vels_robot);
    
  nh.subscribe(sub_duty_motor);
  nh.subscribe(sub_direct_motor);
// Front Right Wheel 
  pinMode(fr_right_EN, OUTPUT);
  pinMode(fr_right_in1, OUTPUT);
  pinMode(fr_right_in2, OUTPUT);

  pinMode(fr_right_encod_A, INPUT_PULLUP);
  pinMode(fr_right_encod_B, INPUT_PULLUP);


// Front Left Wheel 
  pinMode(fr_left_EN, OUTPUT);
  pinMode(fr_left_in1, OUTPUT);
  pinMode(fr_left_in2, OUTPUT);

  pinMode(fr_left_encod_A, INPUT_PULLUP);
  pinMode(fr_left_encod_B, INPUT_PULLUP);

//
//// Front Right Wheel 
//  pinMode(re_right_EN, OUTPUT);
//  pinMode(re_right_in1, OUTPUT);
//  pinMode(re_right_in2, OUTPUT);
//
//  pinMode(re_right_encod_A, INPUT_PULLUP);
//  pinMode(re_right_encod_B, INPUT_PULLUP);


// Front Left Wheel 
//  pinMode(re_left_EN, OUTPUT);
//  pinMode(re_left_in1, OUTPUT);
//  pinMode(re_left_in2, OUTPUT);
//
//  pinMode(re_left_encod_A, INPUT_PULLUP);
//  pinMode(re_left_encod_B, INPUT_PULLUP);


  attachInterrupt(fr_right_encod_A, read_fr_right_enc, RISING);
  attachInterrupt(fr_left_encod_A, read_fr_left_enc, RISING);

//  attachInterrupt(re_right_encod_A, read_re_right_enc, RISING);
//  attachInterrupt(re_left_encod_A, read_re_left_enc, RISING);


}

void loop() {
  // put your main code here, to run repeatedly:

  pub_data_pose_encod();
//
//  set_pwm_motor(0,50,150);
//  set_pwm_motor(1,50,150);
//  set_pwm_motor(2,50,150);
//  set_pwm_motor(3,50,150);

//
//  digitalWrite(re_right_in1, LOW);
//    digitalWrite(re_right_in2 , HIGH);
//
//    analogWrite(re_right_EN,150);

  
  nh.spinOnce();
  delay(100);
}


void read_fr_right_enc(){
  if(digitalRead(fr_right_encod_B)==HIGH) fr_right_pose_encod --;
  if(digitalRead(fr_right_encod_B)==LOW) fr_right_pose_encod ++;
}


void read_fr_left_enc(){
  if(digitalRead(fr_left_encod_B)==HIGH) fr_left_pose_encod ++;
  if(digitalRead(fr_left_encod_B)==LOW) fr_left_pose_encod --;
}

//
//void read_re_right_enc(){
//  if(digitalRead(re_right_encod_B)==HIGH)  re_right_pose_encod--;
//  if(digitalRead(re_right_encod_B)==LOW) re_right_pose_encod ++;
//}
//
//
//void read_re_left_enc(){
//  if(digitalRead(re_left_encod_B)==HIGH) re_left_pose_encod ++;
//  if(digitalRead(re_left_encod_B)==LOW) re_left_pose_encod --;
//}


void pub_data_pose_encod(){


    
    data_pose_encod[0] = fr_right_pose_encod;
    data_pose_encod[1] = fr_left_pose_encod;
    data_pose_encod[2] = re_right_pose_encod;
    data_pose_encod[3] = re_left_pose_encod;
    
    array_pose_encod.data = data_pose_encod;
array_pose_encod.data_length = 4;
    pub_array_pose_encod.publish(&array_pose_encod);
}



void set_pwm_motor ( int stt, int direct , int duty){
  
  if (stt == 0){

  }
  
  if (direct > 0 ){
    digitalWrite(list_in_01[stt], LOW);
    digitalWrite(list_in_02[stt] , HIGH);

    analogWrite(list_EN[stt],duty);
  }

  if (direct < 0 ){
    digitalWrite(list_in_01[stt], HIGH);
    digitalWrite(list_in_02[stt] , LOW);

    analogWrite(list_EN[stt],duty);
  }

  if (duty == 0.0 ){
    digitalWrite(list_in_01[stt], LOW);
    digitalWrite(list_in_02[stt] , LOW);

    analogWrite(list_EN[stt],duty);
  }
  
}

    
