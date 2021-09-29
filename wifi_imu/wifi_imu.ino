#define M5STACK_MPU6886
#include <M5Stack.h>
#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Imu.h>
#include <MadgwickAHRS.h>
#include <utility/MahonyAHRS.h>
#include <WiFi.h>

const char* ssid = "Adrobo-G";
const char* password = "adrobo2018";

WiFiClient client;
IPAddress server(192,168,3,20);
IPAddress ip_address;

TFT_eSprite img = TFT_eSprite(&M5.Lcd);

sensor_msgs::Imu imu_msg;
ros::Publisher pub_imu("imu/data_raw", &imu_msg);

float roll,pitch,yaw;
float cosRoll;
float sinRoll;
float cosPitch;
float sinPitch;
float cosYaw;
float sinYaw;

geometry_msgs::Vector3 acc;
geometry_msgs::Vector3 gyro;
geometry_msgs::Vector3 accOffset;
geometry_msgs::Vector3 gyroOffset;

Madgwick MadgwickFilter;

uint32_t timer;

class WiFiHardware {
  public:
    WiFiHardware() {};
    void init() {
      client.connect(server, 11411);   
    }
    int read() {
      return client.read();      
    }
    void write(uint8_t* data, int length) {
      client.write(data, length);
      }
    unsigned long time() {
      return millis(); // easy; did this one for you
    }
};

ros::NodeHandle_<WiFiHardware> nh;

void setupWiFi() {
  WiFi.begin(ssid, password);
  uint8_t i = 0;
  while (WiFi.status() != WL_CONNECTED && i++ < 20) delay(500);
  if (i == 21) {
    while (1) delay(500);
  }
}

void calibrate6886(){
  float gyroSum[3];
  float accSum[3];
  int counter = 500;
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
    M5.IMU.getGyroData(&gyro.x,&gyro.y,&gyro.z);
    M5.IMU.getAccelData(&acc.x,&acc.y,&acc.z);
    
    gyro.x -= gyroOffset.x;
    gyro.y -= gyroOffset.y;
    gyro.z -= gyroOffset.z;
    acc.x -= accOffset.x;
    acc.y -= accOffset.y;
    acc.z -= accOffset.z;
}


void setup() {
  //Serial.begin(115200);
  M5.begin();
  setupWiFi();

  M5.IMU.Init();
  calibrate6886();

  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextColor(GREEN , BLACK);
  M5.Lcd.setTextSize(2);
  M5.Lcd.setBrightness(10);
  nh.initNode();
  nh.advertise(pub_imu);
}

void loop() {
  //M5.IMU.getAccelData(&acc.x,&acc.y,&acc.z);
  //M5.IMU.getGyroData(&gyro.x,&gyro.y,&gyro.z);

  applycalibration();
  
  MahonyAHRSupdateIMU(gyro.x * DEG_TO_RAD, gyro.y * DEG_TO_RAD, gyro.z * DEG_TO_RAD, acc.x, acc.y, acc.z, &pitch, &roll, &yaw);

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

  
  M5.Lcd.setCursor(0, 0);M5.Lcd.printf("[ IMU1 ]");

  M5.Lcd.setCursor(0, 20);M5.Lcd.printf("Accel[G] : x,y,z");
  M5.Lcd.setCursor(0, 40);M5.Lcd.printf("%5.2f   %5.2f   %5.2f   ",acc.x, acc.y, acc.z );

  M5.Lcd.setCursor(0, 65);M5.Lcd.printf("Gyro[o/s] : x,y,z");
  M5.Lcd.setCursor(0, 85);M5.Lcd.printf("%6.2f  %6.2f  %6.2f      ",gyro.x, gyro.y, gyro.z );

  M5.Lcd.setCursor(0, 110);M5.Lcd.printf("Rpy[deg] : x,y,z");
  M5.Lcd.setCursor(0, 130);M5.Lcd.printf("%6.2f  %6.2f  %6.2f      ",pitch, roll, yaw );
  
  
  nh.spinOnce();
  M5.update();
}
