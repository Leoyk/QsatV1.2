
/*
硬件构成：
    主控：ATMEGA 2560
    主频：16Mhz
    传感器：
        IIC：
          SHT31   0x44
          BMP180    0x1E
          mpu6050   0x68
          MHC5883L  0x77
          
        Serial：   
          system    Serial      （无输出） none
          ublox 6M  GPSser  Serial1 （读写）  9600
          XbepsX    UHFser  Serial2 （读写）  115200
          Bluetooth BLEser  Serial3 （可选）  9600
        
        AnalogRead：
          batte voltage A11

    执行器：
        Servo：
          ser     2
        beep：
          bee     46    
          
          
信息：

  刘要坤
  九天未来
  2018.07.11
  
*/




/**************************系统参数控制区***************************************/

#define SETGPS 1500   //GPS释放高度   单位m 1500（海拉尔气象站基准海拔700）
#define SETPRE 1500   //气压计释放高度 单位m
#define SETDIS 30000  //GPS距离     单位m

#define NSETGPSBASE   //屏蔽之后将跳过初始化中的定位操作，直接使用写入的参数作为基准经纬度。
float baseLat = 4915.00,baseLon = 11942.10;//海拉尔气象站经纬度    格式：dddmm.mmmmm
long baseTime = 0,baseAltitude = 0,gpsHight = 0;

#define blueDebug   //屏蔽以关闭蓝牙调试串口


#define GPSser Serial1  //GPS串口映射
#define UHFser Serial2  //UHF串口映射
#define BLEser Serial3  //BLE串口映射





/**************************头文件包含区***************************************/

//pt库
#define PT_USE_TIMER

#include "pt.h"

//Qsat库 包含程序内的变量定义
#include"commQINC.h"

//看门狗库
#include "avr/wdt.h"


/**************************系统变量区***************************************/

//pt库变量定义
static struct pt 
  thread1,thread2,thread3,thread4,thread5, 
  thread6,thread7,thread8,thread9,thread10; 



/**************************初始化区*****************************************/  
void setup() {

//----------------------------初始化UHF-------------------------------
  UHFser.begin(115200);
//UHF输出信息
  UHFser.println("|*------------*-------------*|");
  UHFser.println("|******    COMMSAT    *******|");
  UHFser.println("|******      Qsat     *******|");
  UHFser.println("**gh:2.3km ph:1.5km dis:30km**");
  UHFser.println("|*----------LeoYK-----------*|");
  UHFser.println("|****   2018.7.11~7.27   ****|"); 
  UHFser.println("|*------------*-------------*|");
   
   
//----------------------------初始化BLE-------------------------------
  #ifdef blueDebug
   BLEser.begin(9600);
  #endif

  
//----------------------------初始化舵机-------------------------------
  ser.attach(2);
  ser.write(50);//close
  UHFser.println("Servo is locked!");

  
//---------------------------打开迎接音乐------------------------------
//回转调
  tone(bee,500,50);  delay(70);
  tone(bee,1000,50); delay(70);
  tone(bee,5000,50); delay(70);
  tone(bee,500,50);  delay(70);
  tone(bee,1000,50); delay(70);
  delay(1000);

  
//---------------------------开启IIC通讯--------------------------------
  Wire.begin();
//-----------------检查IIC器件读取状态 应读取到1E 44 68 77--------------
  { 
    byte error, address;
    int nDevices;
    nDevices = 0;
    
    for(address = 1; address < 127; address++ ){
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0){
      UHFser.println(address,HEX);
      tone(bee,1000,50); delay(70);
      nDevices++;
    }
    else if (error==4){
      UHFser.print("Unknow error at address 0x");
      if (address<16) 
      UHFser.print("0");
      UHFser.println(address,HEX);
    }    
    }
    if (nDevices == 0)
    UHFser.println("No I2C devices found");
    else{
    
    }
    delay(1000);     
   }
//------------------------初始化传感器---------------------------------
  shtInit(0x44); 
  pressureInit();
  GPS_init();
  imu9dInit();
//--------------------初始化传感器完成音乐-----------------------------
//升调
{
tone(bee,500,50); delay(70);
tone(bee,700,50); delay(70);
tone(bee,900,50); delay(70);
tone(bee,1200,50);  delay(70);
delay(1000);
}
//-------------------------TF卡初始化----------------------------------
tfInit();
preFile(&fileName);
UHFser.print(fileName);
UHFser.println("begin");

//            get baseline
{
orderTV();
delay(10);
tempVal();

orderPV();
delay(30);
basePressure = pressureVal();
}
//获取后蜂鸣
tone(bee,500,100); 
delay(120);


//            get location GPS初始时刷新 或 由程序直接写入
#ifdef NSETGPSBASE
location(&gpsData);
baseLat = gpsData.latitude;
baseLon = gpsData.longitude;
baseTime = gpsData.minu; 
baseAltitude = gpsData.altitude; 
#endif
//获取后蜂鸣三声
tone(bee,1000,100); 
delay(120);
tone(bee,1000,100); 
delay(120);
tone(bee,1000,100); 
delay(120);


//----------------------pt线程初始化------------------------------
  PT_INIT(&thread1);
  PT_INIT(&thread2);
  PT_INIT(&thread3);
  PT_INIT(&thread4);
  PT_INIT(&thread5);
  PT_INIT(&thread6);
  PT_INIT(&thread7);
  PT_INIT(&thread8);
  PT_INIT(&thread9);  
  PT_INIT(&thread10);
//开始定时蜂鸣  
  tone(bee,1000,100); 
  delay(120);
  
//----------------------开启看门狗-------------------------------
  wdt_enable(WDTO_2S);

  
//            开启时间记录
  timeMark = millis();
}



/**************************主函数区***************************************/
void loop() {
  wdt_reset();
//pt线程运行
  thread1_entry(&thread1);
  thread2_entry(&thread2);
  thread3_entry(&thread3);
  thread4_entry(&thread4);
  thread5_entry(&thread5);
  thread6_entry(&thread6);
  thread7_entry(&thread7);
  thread8_entry(&thread8);
  thread9_entry(&thread9);
  thread10_entry(&thread10);
}



/**************************pt任务区***************************************/
//任务1：TH 100ms
static int thread1_entry(struct pt *pt){

  PT_BEGIN(pt);
  while (1){
  //请求温湿度
    orderTH(); 
    
  //延时100ms
    PT_TIMER_DELAY(pt,100); 
    
  //获取温湿度参数内
    getTH(&airTemp,&airHumility);  
    
  //存储温湿度数据
    data[0] = airTemp;
    data[1] = airHumility;
  }
  PT_END(pt);
}

//任务2：PR 100ms
static int thread2_entry(struct pt *pt){
  
  PT_BEGIN(pt);
  while (1){
  //刷新温度    
    orderTV();
    PT_TIMER_DELAY(pt,10);
    BmpTem = tempVal();
  //刷新气压  获取气压需要先得到温度
    orderPV();
    PT_TIMER_DELAY(pt,100);
    pressure = pressureVal();
  //计算气压高度
    BmpAltitude = altitudeVal(pressure,basePressure);
  //存储数据
    data[2] = basePressure;
    data[3] = pressure;
    data[4] = BmpAltitude;
  }
  PT_END(pt);
}

//任务3：GPS 50ms
static int thread3_entry(struct pt *pt){
  
  PT_BEGIN(pt);
  unsigned long timeoutGps;//GPS解算超时时间
  double degLat,degLon,degBaseLat,degBaseLon;//度化纬度、度化经度、度化初始纬度、度化初始经度
  
  while (1){
      
  //在切换任务的时候存在误解码的概率
    timeoutGps = millis();
    while(GPSser.available() && (millis() - timeoutGps < 1000)){
      GPS_Parsing(&gpsData);
    }

  //获取当前的净高度差
    gpsHight =gpsData.altitude - baseAltitude; 

  //度化度分格式的数据
    degBaseLat = DM2DD(baseLat);
    degBaseLon = DM2DD(baseLon);
    degLat = DM2DD(gpsData.latitude);
    degLon = DM2DD(gpsData.longitude);
    
  //获取地面距离
    distance = CalGPSDistance(degBaseLat,degBaseLon,degLat,degLon);



  //简单处理异常数据
    if(distance - disbuf > 100)
      distance = disbuf;
    else
      disbuf = distance;
    if(gpsData.altitude -altbuf > 1000)     
      gpsData.altitude = altbuf;
    else
      altbuf = gpsData.altitude;
    
  //存储数据
    data[5] = distance;
    data[6] = gpsData.latitude;
    data[7] = gpsData.longitude;
    data[8] = gpsData.altitude;
    data[9] = gpsData.sat;
    data[10] = gpsData.HDOP;
    
    data[11] = gpsData.fix;
    data[12] = gpsData.fixq;
    data[13] = gpsData.speed;
    data[14] = baseLat;
    data[15] = baseLon;
    data[16] = gpsData.hour;
    data[17] = gpsData.minu;
    data[18] = gpsData.sece;
   
    PT_TIMER_DELAY(pt,50);
  }
  
  PT_END(pt);
}

//任务4：IMU 50ms
static int thread4_entry(struct pt *pt){
  
  PT_BEGIN(pt);
  while (1){
  //刷新数据
    imu9d.getData();
    
  //计算数据    
    dt = (double)(micros() - timeDt) / 1000000; // Calculate delta time
    timeDt = micros();
  //横滚、俯仰    
    ROLL  = atan2(imu9d.AY, imu9d.AZ) * RAD_TO_DEG;
    PITCH = atan(-imu9d.AX / sqrt(imu9d.AY * imu9d.AY + imu9d.AZ * imu9d.AZ)) * RAD_TO_DEG;

    dgyroXrate = imu9d.GX/131; // Convert to deg/s
    gyroYrate = imu9d.GY/131; // Convert to deg/s
    
    // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
    if ((ROLL < -90 && kalAngleX > 90) || (ROLL > 90 && kalAngleX < -90)) {
    kalmanX.setAngle(ROLL);
    compAngleX = ROLL;
    kalAngleX = ROLL;
    gyroXangle = ROLL;
    } else
    kalAngleX = kalmanX.getAngle(ROLL, gyroXrate, dt); // Calculate the angle using a Kalman filter

    if (abs(kalAngleX) > 90)
    gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
    
    kalAngleY = kalmanY.getAngle(PITCH, gyroYrate, dt);
    
    gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
    gyroYangle += gyroYrate * dt;
    
    compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * ROLL; // Calculate the angle using a Complimentary filter
    compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * PITCH;  

    // Reset the gyro angle when it has drifted too much
    if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = kalAngleX;
    if (gyroYangle < -180 || gyroYangle > 180)
    gyroYangle = kalAngleY;
   
    
    YAW = atan2( imu9d.MY, imu9d.MX );
    
    declinationAngle = (4.0 + (26.0 / 60.0)) / (180 / M_PI);
    YAW += declinationAngle;
    YAW = correctAngle(YAW);
    
  //偏航
    YAW = YAW * 180/M_PI;
    HEAD = 180 - YAW;
    //  ROLL = smoothFliterGX(ROLL);
    //  PITCH = smoothFliterGY(PITCH);
    //  HEAD = YAW = smoothFliterGZ(HEAD);

  //向量和
    allAcc = sqrt( imu9d.AX * imu9d.AX + imu9d.AY * imu9d.AY + imu9d.AZ * imu9d.AZ)/16384 * CONSTANTS_ONE_G;


  //存储数据
    data[19] = imu9d.AX*0.0006;
    data[20] = imu9d.AY*0.0006;
    data[21] = imu9d.AZ*0.0006;

    data[22] = imu9d.GX/131;
    data[23] = imu9d.GY/131;
    data[24] = imu9d.GZ/131;

    data[25] = imu9d.MX;
    data[26] = imu9d.MX;
    data[27] = imu9d.MX;

    data[28] = PITCH;
    data[29] = ROLL;
    data[30] = YAW;
    data[31] = allAcc;
    
    PT_TIMER_DELAY(pt,50);
  }
  PT_END(pt);
}

//任务5：ADC 100ms
static int thread5_entry(struct pt *pt){
  PT_BEGIN(pt);
  while(1){ 
    //获取数据 + 滤波 分压电阻10K + 2K
      batteryVoltage = smoothFliterGZ(analogRead(A11)*0.0295); 
    //存储数据
      data[32] = batteryVoltage;

      PT_TIMER_DELAY(pt,100);
    }
  PT_END(pt);
}

//任务6：释放 100ms
static int thread6_entry(struct pt *pt){
  PT_BEGIN(pt);

while(1){ 

//未释放
if(takeFlag == 0){

//1.读取串口事件
 Serial2Event();
//判断是否有释放信号
 if(UHF_ReadOk){
      for(sf = 0;((inputString[sf] == ck[sf]) && (sf < 7));sf ++);
      UHF_ReadOk = 0;
      inputString = "";
    }
//如果有 info = 1  
      if(sf >= 6){  
       takeOffInfo = 1;
      }

//2.判断GPS高度是否到限制    
    if((gpsHight > SETGPS) || (BmpAltitude > SETPRE)){
      if(distance > SETDIS){
        takeOffInfo = 2;
        }
      }

}
    if(takeOffInfo > 0 && takeFlag == 0){   
      takeFlag = takeOffInfo;
      for(int sf = 0;sf < 3; sf ++){
        ser.write(110);//open
        
        if(takeOffInfo == 1)
          UHFser.println("take off by CMD: ShiFang.");
        else if(takeOffInfo == 2){
          UHFser.print("take off by gpsHight:");
          UHFser.print(gpsHight);
          UHFser.print("\tbmpHight:");
          UHFser.print(BmpAltitude);
          UHFser.print("\tdistance:");
          UHFser.println(distance);
        }

        delay(300);
      }
      }
    data[33] = takeOffInfo;
    PT_TIMER_DELAY(pt,500);
  }
  PT_END(pt);
}

//任务7：tasmit 50ms
static int thread7_entry(struct pt *pt){             

  PT_BEGIN(pt);
while(1){

     UHFout = "{\"N\":14,\"AX\":"+(String)(imu9d.AX*0.0006)+ ",\"AY\":" +(String)(imu9d.AY*0.0006)+",\"AZ\":"+(String)(imu9d.AZ*0.0006)+",\"CK\":\"FF\"}";
     UHFser.println(UHFout);
  
     UHFout = ""; 

  //GX,GY,GZ  
     UHFout = "{\"N\":15,\"GX\":"+(String)(imu9d.GX/131)+ ",\"GY\":" +(String)(imu9d.GY/131)+",\"GZ\":"+(String)(imu9d.GZ/131)+",\"CK\":\"FF\"}";
     UHFser.println(UHFout);
     
  //MX,MY,MZ  
     UHFout = "{\"N\":16,\"MX\":"+(String)(imu9d.MX)+ ",\"MY\":" +(String)(imu9d.MY)+",\"MZ\":"+(String)(imu9d.MZ)+",\"CK\":\"FF\"}";
     UHFser.println(UHFout);
  //YPR
  //PITCH  
     UHFout = "{\"N\":3,\"PIT\":"+(String)(PITCH*10)+",\"CK\":\"FF\"}";
     UHFser.println(UHFout);
  
     UHFout = "";   
  //ROLL                                                                                                                                                                          
     UHFout = "{\"N\":4,\"ROL\":"+(String)(ROLL*10)+",\"CK\":\"FF\"}";
     UHFser.println(UHFout);
  
     UHFout = "";   
  //yaw
     UHFout = "{\"N\":5,\"YAW\":"+(String)(YAW*10)+",\"CK\":\"FF\"}";
     UHFser.println(UHFout);
  
     UHFout = "";  
     
  //head
     UHFout = "{\"N\":6,\"HEAD\":"+(String)(HEAD*10)+",\"CK\":\"FF\"}";
     UHFser.println(UHFout);
  
     UHFout = ""; 

    PT_TIMER_DELAY(pt,50);
  }
  PT_END(pt);
}//任务10：tictok

//任务8：trans 100ms
static int thread8_entry(struct pt *pt){             

  PT_BEGIN(pt);
while(1){
  
  //蓝牙输出
   #ifdef blueDebug
    BLEser.print("distance:");
    BLEser.print(distance);
    BLEser.print("m\t");
    BLEser.print("\tBmpAltitude:");
    BLEser.print(BmpAltitude);
    BLEser.print("m\t");
    BLEser.print("\tgpsHight:");
    BLEser.print(gpsHight);
    BLEser.println("m");
   #endif   
   
   UHFout = "{\"N\":1,\"INTP\":"+(String)(airTemp*10)+",\"CK\":\"FF\"}";
   UHFser.println(UHFout);
   UHFout = "";
   UHFout = "{\"N\":2,\"INHM\":"+(String)(airHumility*10)+",\"CK\":\"FF\"}";
   UHFser.println(UHFout);
   UHFout = "";
     
//baseline presure
   UHFout = "{\"N\":17,\"BSP\":"+(String)(basePressure)+",\"CK\":\"FF\"}";
   UHFser.println(UHFout);

   UHFout = "";  
//nowtime presure   
   UHFout = "{\"N\":9,\"PRS\":"+(String)(pressure)+",\"CK\":\"FF\"}";
   UHFser.println(UHFout);

   UHFout = ""; 

   
//altitude under baseline   
   UHFout = "{\"N\":18,\"BALT\":"+(String)(BmpAltitude)+",\"CK\":\"FF\"}";
   UHFser.println(UHFout);
   UHFout = "";   

  
//latitude   
   UHFout = "{\"N\":7,\"LAT\":";
   UHFser.print(UHFout);
   UHFser.print(gpsData.latitude);
   UHFout = ",\"CK\":\"FF\"}";
   UHFser.println(UHFout);

   UHFout = "";   
//longitude
   UHFout = "{\"N\":8,\"LON\":";
   UHFser.print(UHFout);
   UHFser.print(gpsData.longitude);
   UHFout = ",\"CK\":\"FF\"}";
   UHFser.println(UHFout);

   UHFout = "";   
//GPS hight 
   UHFout = "{\"N\":21,\"GALT\":"+(String)(gpsData.altitude)+",\"CK\":\"FF\"}";
   UHFser.println(UHFout);

   UHFout = "";   

   
//groud speed
   UHFout = "{\"N\":12,\"UVE\":"+(String)(gpsData.speed)+",\"CK\":\"FF\"}";
   UHFser.println(UHFout);

   UHFout = "";  

    
//sat
   UHFout = "{\"N\":19,\"NSAT\":"+(String)(gpsData.sat)+",\"CK\":\"FF\"}";
   UHFser.println(UHFout);

   UHFout = "";   
//HDOP
   UHFout = "{\"N\":20,\"HDOP\":"+(String)(gpsData.HDOP)+",\"CK\":\"FF\"}";
   UHFser.println(UHFout);

   UHFout = "";   

  //acc
     UHFout = "{\"N\":13,\"LUXE\":"+(String)(distance)+",\"CK\":\"FF\"}";
     UHFser.println(UHFout);
  
     UHFout = "";  
   

  UHFout = "{\"N\":27,\"VSLR\":"+(String)(batteryVoltage*10)+",\"CK\":\"FF\"}";
  UHFser.println(UHFout);

  UHFout = "";  

    PT_TIMER_DELAY(pt,100);
  }
  PT_END(pt);
}

//任务9：logger 100ms
static int thread9_entry(struct pt *pt){             

  PT_BEGIN(pt);
while(1){
     
openFile();
SwriteData("\r\ncode run:");
iwriteData(gpsData.minu - baseTime);
SwriteDataLn("min");
SwriteData("\r\n\r\ncode speed:");
iwriteData(millis() - timeMark - 100);
SwriteDataLn("ms");
timeMark = millis();

SwriteData("\r\n\r\nGPS Time");
iwriteData(gpsData.hour);
SwriteData(":");
iwriteData(gpsData.minu);
SwriteData(":");
iwriteDataLn(gpsData.sece);

for(cntNum = 0;cntNum < 34;cntNum ++){ 

  iwriteData(cntNum);
  SwriteData(": ");
  writeData(data[cntNum]);
  SwriteData("\t");

  }  
  
SwriteDataLn(""); 
SwriteDataLn(""); 
closeFile();

    PT_TIMER_DELAY(pt,100);
  }
  PT_END(pt);
}

//任务10：定时响
static int thread10_entry(struct pt *pt){             

  PT_BEGIN(pt);
while(1){
    tone(bee,1000,100);
    PT_TIMER_DELAY(pt,5000);
  }
  PT_END(pt);
}





/**************************串口监视区***************************************/
void  Serial2Event() {
  while (UHFser.available()) {
    char inChar = (char)UHFser.read();
    
      inputString += inChar;
    if (inChar == '\n') 
      UHF_ReadOk = true;
    
  }
} 
  
