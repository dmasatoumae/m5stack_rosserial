#define M5STACK_MPU6886 
#include <M5Stack.h>
#undef ESP32
#include <ros.h>
#define ESP32
#include <sensor_msgs/Imu.h>
geometry_msgs::Vector3 acc;
geometry_msgs::Vector3 gyro;
geometry_msgs::Vector3 accOffset;
geometry_msgs::Vector3 gyroOffset;

void calibrate6886(){
  float gyroSum[3];
  float accSum[3];
  int counter = 5000;
  for(int i = 0; i < counter; i++){
    M5.IMU.getGyroData(&gyro.x,&gyro.y,&gyro.z);
    M5.IMU.getAccelData(&acc.x,&acc.y,&acc.z);
    gyroSum[0] += gyro.x;
    gyroSum[1] += gyro.y;
    gyroSum[2] += gyro.z;
    accSum[0] += acc.x;
    accSum[1] += acc.y;
    accSum[2] += acc.z;
    delay(2);
  }
    
  gyroOffset.x = gyroSum[0]/counter;
  gyroOffset.y = gyroSum[1]/counter;
  gyroOffset.z = gyroSum[2]/counter;
  accOffset.x = accSum[0]/counter;
  accOffset.y = accSum[1]/counter;
  accOffset.z = (accSum[2]/counter) - 1.0;
}

void setup(){
  M5.begin();
  M5.IMU.Init();
  calibrate6886();
  
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextColor(GREEN , BLACK);
  M5.Lcd.setTextSize(2);
  M5.Lcd.setBrightness(10);
  //nh.initNode();
  //nh.advertise(pub_imu);
}

void loop(){
  Serial.print("gyroOffset.x >>");
  Serial.println(gyroOffset.x);
  Serial.print("gyroOffset.y >>");
  Serial.println(gyroOffset.y);
  Serial.print("gyroOffset.z >>");
  Serial.println(gyroOffset.z);

  Serial.print("accOffset.x >>");
  Serial.println(accOffset.x);
  Serial.print("accOffset.y >>");
  Serial.println(accOffset.y);
  Serial.print("accOffset.z >>");
  Serial.println(accOffset.z);

  delay(10000);
}
