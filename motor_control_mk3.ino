#include "sbus.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
// Khai báo các chân điều khiển động cơ và chân nối với buzzer
const int motorPin1 = 14;
const int motorPin2 = 25;
const int motorPin3 = 26;
const int motorPin4 = 27;  
const int buzzer = 18;

// Các thuộc tính PWM
const int freq = 50;
const int motorChannel0 = 0;
const int motorChannel1 = 1;
const int motorChannel2 = 2;
const int motorChannel3 = 3;
const int resolution = 8;
const int buzzerChannel = 4;

// Biến lưu trữ thông số động cơ
int thrust1 = 0;
int thrust2 = 0;
int thrust3 = 0;
int thrust4 = 0;

// Các giá trị mặc định cho yaw, pitch, roll
int default_yaw = 1000;
int default_pitch = 1000;
int default_roll = 1000;

// PID parameter 
float pitch_setpoint = 0;
const float pitch_p_gain = 25;
const float pitch_i_gain = 0;
const float pitch_d_gain = 3;
float pitch_error = 0;
float pitch_last_error = 0;
unsigned long t1,dt,output;
float p,i,d;
bool en = true;
// Tốc độ quay cho yaw, pitch, roll
int yaw_rate;
int pitch_rate;
int roll_rate;
// IMU parameter
float ax,ay,az,gx,gy,gz;
// Đối tượng IMU 
Adafruit_MPU6050 mpu;
// Đối tượng SBUS cho việc đọc và gửi SBUS
bfs::SbusRx sbus_rx(&Serial2, 16, 17, true);
bfs::SbusTx sbus_tx(&Serial2, 16, 17, true);

// Dữ liệu SBUS
bfs::SbusData data;

void setup() {
  // Serial để hiển thị dữ liệu
  Serial.begin(115200);
  while (!Serial) {}

  // Bắt đầu giao tiếp SBUS
  sbus_rx.Begin();
  sbus_tx.Begin();

   if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.println("");
  // Cấu hình chức năng PWM cho động cơ
  ledcSetup(motorChannel0, freq, resolution);
  ledcSetup(motorChannel1, freq, resolution);
  ledcSetup(motorChannel2, freq, resolution);
  ledcSetup(motorChannel3, freq, resolution);

  // Liên kết các kênh với các chân GPIO
  ledcAttachPin(motorPin1, motorChannel0);
  ledcAttachPin(motorPin2, motorChannel1);
  ledcAttachPin(motorPin3, motorChannel2);
  ledcAttachPin(motorPin4, motorChannel3);
  // Ghi giá trị vào động cơ
    ledcWrite(motorChannel0, 11);
    ledcWrite(motorChannel1, 11);
    ledcWrite(motorChannel2, 11);
    ledcWrite(motorChannel3, 11);  
    delay(2000);
}

void loop () {
  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  ay = a.acceleration.y;
  PID_calculator();
  // Đọc dữ liệu từ SBUS
  if (sbus_rx.Read()) {
    // Lấy dữ liệu đã nhận được
    data = sbus_rx.data();
    
    // Hiển thị dữ liệu đã nhận được
//    for (int8_t i = 0; i < 6; i++) {
//      Serial.print(data.ch[i]);
//      Serial.print("\t");
//    }
    
    // Tính toán tốc độ quay cho yaw, pitch, roll
    // và cập nhật giá trị động cơ tương ứng
    thrust1  = data.ch[2];
    thrust2  = data.ch[2];
    thrust3  = data.ch[2];
    thrust4  = data.ch[2];
    yaw_rate = map(data.ch[3], 1304, 504, -1 * thrust1 / 4, thrust1 / 4);
    pitch_rate = map(data.ch[1], 504, 1304, -1 * thrust1 / 4, thrust1 / 4);
    roll_rate = map(data.ch[0], 504, 1304, -1 * thrust1 / 4, thrust1 / 4);
    
    // Kiểm tra và điều chỉnh tốc độ quay khi gần giá trị 0
    if (yaw_rate >= -10 && yaw_rate <= 10) {
      yaw_rate = 0;
    }
    if (pitch_rate >= -10 && pitch_rate <= 10) {
      pitch_rate = 0;
    }
    if (roll_rate >= -10 && roll_rate <= 10) {
      roll_rate = 0;
    }
    pitch_rate = pitch_rate + output;
    // Hiển thị tốc độ quay và cập nhật giá trị động cơ
//    Serial.print(yaw_rate);
//    Serial.print("\t");
//    Serial.print(pitch_rate);
//    Serial.print("\t");
//    Serial.print(roll_rate);
//    Serial.print("\t");
    
    // Cập nhật giá trị động cơ dựa trên tốc độ quay
    thrust1 = thrust1 + yaw_rate + pitch_rate + roll_rate;
    thrust3 = thrust3 + yaw_rate - pitch_rate - roll_rate;
    thrust2 = thrust2 - yaw_rate + pitch_rate - roll_rate;
    thrust4 = thrust4 - yaw_rate - pitch_rate + roll_rate;
    
    // Chuyển đổi giá trị động cơ và ghi vào động cơ
    int converted_thrust1 = map(thrust1, 60, 1624, 11, 25);
    int converted_thrust2 = map(thrust2, 60, 1624, 11, 25);
    int converted_thrust3 = map(thrust3, 60, 1624, 11, 25);
    int converted_thrust4 = map(thrust4, 60, 1624, 11, 25);
    
    // Ghi giá trị vào động cơ
    ledcWrite(motorChannel0, converted_thrust1);
    ledcWrite(motorChannel1, converted_thrust2);
    ledcWrite(motorChannel2, converted_thrust3);
    ledcWrite(motorChannel3, converted_thrust4);     
    
    // Hiển thị giá trị động cơ và thông tin failsafe
    Serial.print(thrust1);
    Serial.print("\t");
    Serial.print(thrust2);
    Serial.print("\t");
    Serial.print(thrust3);
    Serial.print("\t");
    Serial.print(thrust4);
//    Serial.print("\t");
//    Serial.print(data.lost_frame);
    Serial.print("\t");
    Serial.println(data.failsafe);
    
    // Gửi dữ liệu SBUS đã nhận đến các servo
    sbus_tx.data(data);
    sbus_tx.Write();
     /* Print out the values */
//  Serial.print("A-X:");
//  Serial.print(a.acceleration.x);
//  Serial.print(",");
//  Serial.print("A-Y:");
//  Serial.print(ay);
//  Serial.print(",");
//  Serial.print("A-Z:");
//  Serial.print(a.acceleration.z);
//  Serial.print(", ");
//  Serial.print("G-X:");
//  Serial.print(g.gyro.x);
//  Serial.print(",");
//  Serial.print("G-Y:");
//  Serial.print(g.gyro.y);
//  Serial.print(",");
//  Serial.print("G-Z:");
//  Serial.print(g.gyro.z);
  Serial.println("");

  delay(10);
  }
}

void PID_calculator(){
  bool roltation = true; //true for left, false for right
  t1 = millis();
  pitch_error = pitch_setpoint - ay;
  delay(10);
  dt = millis() - t1;
  if (pitch_error > 0){
  roltation = true;
  p = pitch_p_gain*pitch_error;
  i = pitch_i_gain*pitch_error/dt;
  d = pitch_d_gain*pitch_last_error;
    }
  else{
  roltation = false;
  pitch_error = abs(pitch_error);
  p = pitch_p_gain*pitch_error;
  i = pitch_i_gain*pitch_error/dt;
  d = pitch_d_gain*pitch_last_error;
      }
  pitch_last_error = pitch_error;
  if (roltation == true){
    output = -1*(p+i+d);
    }
    else{
    output = p+i+d;
    }
  Serial.print(pitch_error);
  Serial.print("\t");
  Serial.print(p);
  Serial.print("\t");
  Serial.print(i,4);
  Serial.print("\t");
  Serial.print(d,4);
  Serial.print("\t");
  Serial.print(output);
  Serial.print("\t");

  }
