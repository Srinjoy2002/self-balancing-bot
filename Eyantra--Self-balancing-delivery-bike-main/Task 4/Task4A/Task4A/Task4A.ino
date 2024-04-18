#include <Wire.h>

#define MPU6050       0x68         
#define ACCEL_CONFIG  0x1C         
#define GYRO_CONFIG   0x1B          
#define PWR_MGMT_1    0x6B           
#define PWR_MGMT_2    0x6C            

#define BRAKE         6   
#define PWM           11  
#define DIRECTION     7   

#define enA           5
#define in1           22
#define in2           23

const uint16_t frequency = 20000;                 
const uint16_t PWMVALUE = F_CPU / frequency / 2;  
float k1 = 75.0; 
float k2 = 5.25;   
float k3 = 0.04;  
float timeloop = 10;  

int pwm = 0;
byte dir;
int32_t motor_speed; 
uint32_t timer;
long thisT, lastT = 0; 
int16_t AcX, AcY, AcZ, GyZ, gyroZ;


#define accSens 0             
#define gyroSens 1            
#define Gyro_amount 0.996     


int16_t  AcX_offset = -750;
int16_t  AcY_offset = 360;
int16_t  AcZ_offset = 0;
int16_t  GyZ_offset = 0;
int32_t  GyZ_offset_sum = 0;

float alpha = 0.40; 
float zdot;

float zangle;
float Acc_angle;

bool vertical = false;  

uint8_t i2cData[14]; 

void setup() {
  Serial.begin(115200);

  
  TCCR1B = (1 << WGM13) | (1 << CS10);  
  ICR1 = PWMVALUE;                      
 
  TCCR1A = (1 << COM1A1) | (1 << COM1B1); 
  
  setspeed(400); 
  pinMode(PWM, OUTPUT);
  pinMode(BRAKE, OUTPUT);
  pinMode(DIRECTION, OUTPUT);
  digitalWrite(BRAKE, HIGH);
  delay(1000);
  angle_setup();
}

void loop() {
  
   
    angle_get();
    if (vertical) {
      digitalWrite(BRAKE, HIGH);
      gyroZ = GyZ / 131.0;                      
      
      zdot = alpha * gyroZ + (1 - alpha) * zdot;    
      pwm = -constrain(k1 * zangle + k2 * zdot + k3 * -motor_speed, -255, 255); 
      

     Nidec_motor_control(pwm);
      motor_speed += pwm;
    } else {
      Nidec_motor_control(0);
      digitalWrite(BRAKE, LOW);
      motor_speed = 0;
    }
   delay(10);
 
  
}
