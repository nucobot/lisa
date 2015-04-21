// APM CONSTRAINTS:
// TYPE        |  ARDUINO-PIN  | APM-PIN 
//-------------+---------------+--------
// OC3A        |       5       |  pwm-8
// INT0        |       2       |  pwm-7
// INT1        |       3       |  pwm-6
// OC4A        |       6       |  pwm-5
// OC4B        |       7       |  pwm-4
// OC4C        |       8       |  pwm-3
// OC1A/PCINT5 |       11      |  pwm-2
// OC1B/PCINT6 |       12      |  pwm-1
//-------------+---------------+--------
// LEDB        |       25      |    -
// LEDY        |       26      |    -
// LEDR        |       27      |    -


#include <Arduino.h>
#include <SPI.h>
#include <math.h>
#include <Wire.h>
#include <Servo.h> 

#include <HMC5883L.h>
#include <IMU.h>
#include <DualVNH5019MotorShield.h>

#include <ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Twist.h>


//DEFINES         
#define WHEEL_SEPARATION 0.17                                
#define TRANSLATION_FACTOR 100                                                                  
#define BARO_ENABLE_PIN 40
#define IMU_ENABLE_PIN 53
#define ToD(x) (x/131)
#define ToG(x) (x*9.80665/16384)
#define BAT1_PIN 4
#define BAT2_PIN 5
#define BTN_PIN 60
#define SERVO1_PIN 7
#define SERVO2_PIN 8

#define ENCODER0_INT 0       // Interrupt number
#define ENCODER0_INT_WIRE 2  // Interrupt input pin (according to datasheet)
#define ENCODER0_REF_WIRE 5  // Reference wire to check rotation direction
#define ENCODER1_INT 1       // Interrupt number
#define ENCODER1_INT_WIRE 3  // Interrupt input pin (according to datasheet)
#define ENCODER1_REF_WIRE 6  // Reference wire to check rotation direction

// MESSAGE-RELATED DEFINES
#define IMU_ANG_VX 0  // IMU: imu_msg.angular_velocity.x
#define IMU_ANG_VY 1  // IMU: imu_msg.angular_velocity.y
#define IMU_ANG_VZ 2  // IMU: imu_msg.angular_velocity.z
#define IMU_LIN_AX 3  // IMU: imu_msg.linear_acceleration.x
#define IMU_LIN_AY 4  // IMU: imu_msg.linear_acceleration.y
#define IMU_LIN_AZ 5  // IMU: imu_msg.linear_acceleration.z
#define MAG_SCAL_X 6  // Magnetometer scaled.XAxis
#define MAG_SCAL_Y 7  // Magnetometer scaled.YAxis
#define MAG_SCAL_Z 8  // Magnetometer scaled.ZAxis
#define BAT_VOLT_1 9  // Battery voltage on BAT1_PIN
#define BAT_VOLT_2 10 // Battery voltage on BAT2_PIN
#define ENC_DATA_0 11 // Speed on encoder 0
#define ENC_DATA_1 12 // Speed on encoder 1
#define BTN_STATE  13 // Power button state
#define DATA_SIZE  14 // Overall array size
           
//FUNCTIONS
void setup();
void loop();
void motor_cb(const geometry_msgs::Twist& cmd_msg);
void srv_carpet_cb(const std_msgs::Int32& msg);
void srv_clapper_cb(const std_msgs::Int32& msg);
void encoder0_cb();
void encoder1_cb();

//VARS     
long int publish_timer;
long int encoder0_cnt, encoder1_cnt;

//DEVICES         
DualVNH5019MotorShield md(54, 55, 69, 68, 56, 57, 67, 66);
IMU imu;
HMC5883L compass;
Servo srv_clapper;
Servo srv_carpet;

//ROS          
ros::NodeHandle nh;

//ROS-MSGS
std_msgs::Float32MultiArray data_raw;

//ROS-TOPICS
ros::Publisher pub_array("apm/data_raw",&data_raw);
ros::Subscriber<std_msgs::Int32> sub_srv_carpet("carpet_servo", srv_carpet_cb);
ros::Subscriber<std_msgs::Int32> sub_srv_clapper("clapper_servo", srv_clapper_cb);
ros::Subscriber<geometry_msgs::Twist> sub_motors("cmd_vel", motor_cb);

void setup() {
 //I2C        
    Wire.begin();
 //SPI        
    SPI.begin();  
    SPI.setClockDivider(SPI_CLOCK_DIV16); 
    SPI.setBitOrder(MSBFIRST); 
    SPI.setDataMode(SPI_MODE0);
    delay(100);
 //BARO                  
    pinMode(BARO_ENABLE_PIN, OUTPUT);
    digitalWrite(BARO_ENABLE_PIN, HIGH);                                              
 //IMU             
    pinMode(IMU_ENABLE_PIN, OUTPUT);
    imu.Init(IMU_ENABLE_PIN);                   
 //COMPASS            
    compass = HMC5883L();
    compass.SetScale(1.3);
    compass.SetMeasurementMode(Measurement_Continuous);
 //SERVOS
    srv_carpet.attach(SERVO1_PIN);
    srv_clapper.attach(SERVO2_PIN);
    srv_carpet.write(90);
    srv_clapper.write(90);
 //MISCELLANEOUS
    pinMode(25, OUTPUT); // BLUE LED
    pinMode(26, OUTPUT); // YELLOW LED
    pinMode(27, OUTPUT); // RED LED
    pinMode(BTN_PIN, INPUT);
 // INTERRUPTS
    attachInterrupt(ENCODER0_INT, encoder0_cb, RISING);
    pinMode(ENCODER0_INT_WIRE, INPUT);
    pinMode(ENCODER0_REF_WIRE, INPUT);
    digitalWrite(ENCODER0_INT_WIRE, HIGH); // Turn on internal pull-up
    digitalWrite(ENCODER0_REF_WIRE, HIGH); // Turn on internal pull-up
    attachInterrupt(ENCODER1_INT, encoder1_cb, RISING);
    pinMode(ENCODER1_INT_WIRE, INPUT);
    pinMode(ENCODER1_REF_WIRE, INPUT);
    digitalWrite(ENCODER1_INT_WIRE, HIGH); // Turn on internal pull-up
    digitalWrite(ENCODER1_REF_WIRE, HIGH); // Turn on internal pull-up
 // VAR INITIALIZATION
    encoder0_cnt = 0;
    encoder1_cnt = 0;
    data_raw.data_length = DATA_SIZE; // The number of output message entries
    data_raw.data = (float *)malloc(sizeof(float)*data_raw.data_length);
 //MOTORS           
    md.init();
 //ROS        
    nh.initNode();
    nh.subscribe(sub_motors);
    nh.subscribe(sub_srv_clapper);
    nh.subscribe(sub_srv_carpet);
    nh.advertise(pub_array);
}

void loop()
{  
    if ( (millis()-publish_timer) > 20) {
        MagnetometerScaled scaled = compass.ReadScaledAxis();      
 //IMU
        data_raw.data[IMU_ANG_VX] = ToD((float)(imu.GyroX()));
        data_raw.data[IMU_ANG_VY] = ToD((float)(imu.GyroY()));
        data_raw.data[IMU_ANG_VZ] = ToD((float)(imu.GyroZ()));
        data_raw.data[IMU_LIN_AX] = ToG((float)(imu.AcceX()));
        data_raw.data[IMU_LIN_AY] = ToG((float)(imu.AcceY()));        
        data_raw.data[IMU_LIN_AZ] = ToG((float)(imu.AcceZ()));
 //COMPASS                 
        data_raw.data[MAG_SCAL_X] = scaled.XAxis;
        data_raw.data[MAG_SCAL_Y] = scaled.YAxis;
        data_raw.data[MAG_SCAL_Z] = scaled.ZAxis;
 //BATTERIES
        data_raw.data[BAT_VOLT_1] = round(((analogRead(BAT1_PIN))*15000.0*0.968096/1023.0));
        data_raw.data[BAT_VOLT_2] = round(((analogRead(BAT2_PIN))*15000.0*0.962191/1023.0));
 //ENCODERS
        data_raw.data[ENC_DATA_0] = encoder0_cnt;
        data_raw.data[ENC_DATA_1] = encoder1_cnt;
        encoder0_cnt = 0;
        encoder1_cnt = 0;
 //BUTTON
        data_raw.data[BTN_STATE] = digitalRead(BTN_PIN);
 //ROS                     
        pub_array.publish(&data_raw);
        publish_timer = millis();
    }
    nh.spinOnce();
}


void motor_cb(const geometry_msgs::Twist& cmd_msg)
{
    double right = 0;
    double left  = 0;
    
    // linear is in meters per second                                 
    right += cmd_msg.linear.x;
    left  += cmd_msg.linear.x;
    
    // angular is in radians per second                                   
    right -= cmd_msg.angular.z * WHEEL_SEPARATION / 2.0;
    left  += cmd_msg.angular.z * WHEEL_SEPARATION / 2.0;

    md.setSpeeds(int(right*TRANSLATION_FACTOR), -int(left*TRANSLATION_FACTOR));
}

void srv_carpet_cb(const std_msgs::Int32& msg)
{
    srv_carpet.write(msg.data);
}

void srv_clapper_cb(const std_msgs::Int32& msg)
{
    srv_clapper.write(msg.data);
}

void encoder0_cb(){
  if (digitalRead(ENCODER0_REF_WIRE) == HIGH) encoder0_cnt --;
  else encoder0_cnt ++;
}

void encoder1_cb(){
  if (digitalRead(ENCODER1_REF_WIRE) == HIGH) encoder1_cnt --;
  else encoder1_cnt ++;
}
