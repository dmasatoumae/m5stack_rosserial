#define M5STACK_MPU6886 
#include <M5Stack.h>
// "#include <ros.h>"の前に#undef EP32が必要（USB接続の場合）
#undef ESP32
#include <ros.h>
#define ESP32
#include <ros/time.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <MadgwickAHRS.h>
#include <utility/MahonyAHRS.h>
#include <Kalman.h>
//#define SENSOR_MSGS_IMU
//#define SENSOR_MSGS_FLOAT32
#define VECTOR3STAMPED

ros::NodeHandle nh;

#ifdef SENSOR_MSGS_IMU
  sensor_msgs::Imu imu_msg;
  ros::Publisher pub_imu("imu1/data_raw", &imu_msg);
  float cosRoll;
  float sinRoll;
  float cosPitch;
  float sinPitch;
  float cosYaw;
  float sinYaw;
#endif

#ifdef SENSOR_MSGS_FLOAT32
  std_msgs::Float32 angle;
  ros::Publisher pub_imu("imu1/angle", &angle);

#endif

#ifdef VECTOR3STAMPED
  geometry_msgs::Vector3Stamped rpy;
  ros::Publisher pub_imu("imu/rpy", &rpy);
#endif

float roll,pitch,yaw;


geometry_msgs::Vector3 acc;
geometry_msgs::Vector3 gyro;
geometry_msgs::Vector3 accOffset;
geometry_msgs::Vector3 gyroOffset;


Madgwick MadgwickFilter;

uint32_t timer;

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

void applycalibration(){
    gyroOffset.x = -2.29;
    gyroOffset.y = -6.62;
    gyroOffset.z = 0.62;
    accOffset.x = -0.01;
    accOffset.y = 0.01;
    accOffset.z = 0.08;
    M5.IMU.getGyroData(&gyro.x,&gyro.y,&gyro.z);
    M5.IMU.getAccelData(&acc.x,&acc.y,&acc.z);
    
    gyro.x -= gyroOffset.x;
    gyro.y -= gyroOffset.y;
    gyro.z -= gyroOffset.z;
    acc.x -= accOffset.x;
    acc.y -= accOffset.y;
    acc.z -= accOffset.z;
}

void setup(){
    M5.begin();
    M5.IMU.Init();
    //calibrate6886();
  
    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.setTextColor(GREEN , BLACK);
    M5.Lcd.setTextSize(2);
    M5.Lcd.setBrightness(10);
    //nh.getHardware()->setBaud(115200);
    nh.initNode();
    nh.advertise(pub_imu);

    
    
}

void loop() {
  applycalibration();
  //MahonyAHRSupdateIMU(gyro.x * DEG_TO_RAD, gyro.y * DEG_TO_RAD, gyro.z * DEG_TO_RAD, acc.x, acc.y, acc.z, &pitch, &roll, &yaw);
  MahonyAHRSupdateIMU(gyro.x * 0.01, gyro.y * 0.01, gyro.z * 0.01, acc.x, acc.y, acc.z, &pitch, &roll, &yaw);

  M5.Lcd.setCursor(0, 20);M5.Lcd.printf("[ IMU_BOOM1 ]");
  
  #ifdef SENSOR_MSGS_IMU
    cosRoll = cos(roll / 2.0);
    sinRoll = sin(roll / 2.0);
    cosPitch = cos(pitch / 2.0);
    sinPitch = sin(pitch / 2.0);
    cosYaw = cos(yaw / 2.0);
    sinYaw = sin(yaw / 2.0);

    imu_msg.orientation.w = cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw;
    imu_msg.orientation.x = sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw;
    imu_msg.orientation.y = cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw;
    imu_msg.orientation.z = cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw;
  
    imu_msg.linear_acceleration = acc;
    imu_msg.angular_velocity = gyro;
  
    imu_msg.header.stamp = nh.now();
    imu_msg.header.frame_id = "m5stack";
    pub_imu.publish(&imu_msg);
    M5.Lcd.setCursor(0, 30);M5.Lcd.printf("Accel[G] : x,y,z");
    M5.Lcd.setCursor(0, 50);M5.Lcd.printf("%5.2f   %5.2f   %5.2f   ",acc.x, acc.y, acc.z );
    M5.Lcd.setCursor(0, 75);M5.Lcd.printf("Gyro[o/s] : x,y,z");
    M5.Lcd.setCursor(0, 95);M5.Lcd.printf("%6.2f  %6.2f  %6.2f      ",gyro.x, gyro.y, gyro.z );
    M5.Lcd.setCursor(0, 120);M5.Lcd.printf("Rpy[deg] : x,y,z");
    M5.Lcd.setCursor(0, 140);M5.Lcd.printf("%6.2f  %6.2f  %6.2f      ",pitch, roll, yaw );
  #endif
  
  #ifdef SENSOR_MSGS_FLOAT32
    angle.data = pitch;
    pub_imu.publish(&angle);
    M5.Lcd.setCursor(0, 120);M5.Lcd.printf("Rpy[deg] : x,y,z");
    M5.Lcd.setCursor(0, 140);M5.Lcd.printf("%6.2f  %6.2f  %6.2f      ",pitch, roll, yaw );
  #endif

  #ifdef VECTOR3STAMPED
    rpy.header.stamp = nh.now();
    rpy.vector.x = roll;
    rpy.vector.y = pitch;
    rpy.vector.z = yaw;
    pub_imu.publish(&rpy);
    M5.Lcd.setCursor(0, 120);M5.Lcd.printf("Rpy[deg] : x,y,z");
    M5.Lcd.setCursor(0, 140);M5.Lcd.printf("%6.2f  %6.2f  %6.2f      ",pitch, roll, yaw );
  #endif
    
  nh.spinOnce();
  M5.update();

  //delay(1);
}
