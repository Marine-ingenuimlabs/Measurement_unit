//A8 is for pressure sensor; A9 and A10 are for current and voltage
 /*       Example of APM_Compass library (HMC5843 sensor).
 *       Code by Jordi MuÒoz and Jose Julio. DIYDrones.com
 */
 #include "Timer.h"
#include <avr/io.h> 
#include <util/crc16.h>
#include <FastSerial.h>
#include <AP_Common.h>
#include <AP_Compass.h> // Compass Library
#include <AP_Math.h>            // ArduPilot Mega Vector/Matrix math Library
#include <I2C.h>

#include <Arduino_Mega_ISR_Registry.h>
#include <AP_PeriodicProcess.h>
#include <AP_AnalogSource.h>
#include <AP_Airspeed.h>
#include <AP_InertialSensor.h>
#include <AP_ADC.h>
#include <AP_IMU.h>
#include <AP_GPS.h>
#include <AP_AHRS.h>
#include <AP_Baro.h>
#include <Filter.h>
#include <AP_Progmem.h>

#define PRINT_SERIAL 0
#define PRINT_BINARY 1
#define SENDING_PROTOCOL PRINT_SERIAL

FastSerialPort0(Serial); //for Debug
FastSerialPort3(Serial3); //for binary mode



#define COMPASS_INIT_FAILED 0x0A 
#define COMPASS_INIT_SUCCESS 0x0B
#define COMPASS_NOT_HEALTHY 0x0C
#define COMPASS_MAX_H_ERR 200   //maximum not healthy compass error 
#define ToRad(x) (x*0.01745329252)  // *pi/180
#define ToDeg(x) (x*57.2957795131)  // *180/pi

// uncomment this for a APM2 board
#define APM_HARDWARE_PIRATES
#define WITH_GPS 0
#define AP_BARO_MS5611_I2C    4
#define PIRATES_CRIUS_AIO_PRO_V1 6
#define CONFIG_IMU_MPU6000_I2C
#define GYRO_ADDR 0x68
#define PIRATES_SENSOR_BOARD PIRATES_CRIUS_AIO_PRO_V1
# define A_LED_PIN        13
# define B_LED_PIN        31
# define C_LED_PIN        30
# define LED_ON           HIGH
# define LED_OFF          LOW
# define MAG_ORIENTATION  ROTATION_YAW_180  //4


AP_Compass_HMC5843 compass;
AP_InertialSensor_MPU6000_I2C ins(GYRO_ADDR, PIRATES_SENSOR_BOARD);
AP_Baro_MS5611_I2C     barometer;

Arduino_Mega_ISR_Registry isr_registry;
AP_TimerProcess  scheduler;


static GPS         *g_gps;
AP_IMU_INS imu(&ins);
AP_AHRS_DCM  ahrs(&imu, g_gps);

Vector3f acc;
Vector3f gyro;
float errorz;
uint32_t timer;
uint32_t timer_10hz,timer_50hz,timer_100hz;
float heading;

uint8_t compass_init_cnt=0,compass_health_cnt=0;

//variables for US300 INIT////////////////////////////////
 float RC = 0.04;
 float RC_Init = 0.0015; 
float voltage,last_voltage,a,b;
Timer t_init;
float sensorValue,voltage_init,last_init_voltage;
int i,j,k;
float pressure;
float depth;
float p;


//SERIAL DATA///////////////////////////////////////////
#define NB_FLOAT_50HZ 9
#define NB_FLOAT_10HZ 6
#define PACKET_SIZE_50HZ (4*NB_FLOAT_50HZ)  //(4*NB_FLOAT)+6
#define PACKET_SIZE_10HZ (4*NB_FLOAT_10HZ)  


typedef union _data {
  float f[NB_FLOAT_50HZ];
  uint8_t  s[PACKET_SIZE_50HZ]; 
} myData;
myData q_50hz;

typedef union _data1 {
  float f[NB_FLOAT_10HZ];
  uint8_t  s[PACKET_SIZE_10HZ]; 
} myData1;
myData1 q_10hz;

/////////////////////////////////////////////////////

static void flash_leds(bool on)
{
    digitalWrite(A_LED_PIN, on?LED_OFF:LED_ON);
    digitalWrite(C_LED_PIN, on?LED_ON:LED_OFF);
}


void setup()
{ //US300 Init
   b = 0.01/(0.01+RC_Init);
   a = 0.01/(0.01 + RC);
   last_voltage = 0;
   int tickEvent2 = t_init.every(10, init_press_sensor);
   last_init_voltage = (((float) analogRead(A8) * 5.0) / 1023.0);
    last_voltage = (((float) analogRead(A8) * 5.0) / 1023.0);;
     k=0;
     j=0;
     while(j!=16){
       t_init.update();
     };
     
     Serial.println("US300 INIT COMPLETE");
     Serial.println(depth);
     
  
    Serial.begin(38400, 128, 256); //baudrate rx_size tx_size
    Serial3.begin(38400, 128, 256); 
 //   while(Serial.read() >= 0) ; // flush the receive buffer

    
    I2c.begin();
    I2c.timeOut(20);
    I2c.setSpeed(true);
    
      isr_registry.init();
      scheduler.init(&isr_registry);
      imu.init(IMU::COLD_START, delay, flash_leds, &scheduler); //IMU::COLD_START=0  IMU::WARM_START=1
      imu.init_accel(delay, flash_leds);

   
	
  
   // Serial.println("Compass library test (HMC5843 and HMC5883L)");
    while (!compass.init()&& compass_init_cnt<5) {
       compass_init_cnt++;
       delay(10);
      //try to init for 5 time and if still failed send fault error 
      //  Serial.println("compass initialisation failed!");
      //  while (1) ;
    }
    if(compass_init_cnt>=5) 
    {
      //compass failed to init  send report to pc 
    //  Serial.println("COMPASS_INIT_FAILED");
    //  Serial3.write(COMPASS_INIT_FAILED);
      return;
    }
    else
    {
      Serial.println("COMPASS_INIT_SUCCESS");
   //   Serial3.write(COMPASS_INIT_SUCCESS);
    }

    compass.set_orientation(ROTATION_YAW_180); // set compass's orientation on aircraft.
    compass.set_offsets(0,0,0); // set offsets to account for surrounding interference
    compass.set_declination(ToRad(0.0)); // set local difference between magnetic north and true north

   // Serial.print("Compass auto-detected as: ");
   /* switch( compass.product_id ) {
    case AP_COMPASS_TYPE_HIL:
        Serial.println("HIL");
        break;
    case AP_COMPASS_TYPE_HMC5843:
        Serial.println("HMC5843");
        break;
    case AP_COMPASS_TYPE_HMC5883L:
        Serial.println("HMC5883L");
        break;
    default:
        Serial.println("unknown");
        break;
    }*/

  
 //    if (!compass._use_for_yaw.load()) 
     //  compass._use_for_yaw.set_and_save(1);
      
  //  delay(10);
    timer=micros();
}


void loop()
{


   
     
     timer  = micros();
     
     compass.accumulate();
    if((timer- timer_10hz) >= 100000L && compass.read()) //@10HZ read Mag
    {
        timer_10hz = timer;
      

        if (!compass.healthy) {
           // Serial.println("not healthy");
            compass_health_cnt++;
        }
        else if (compass_health_cnt< COMPASS_MAX_H_ERR)
        {
        heading = compass.calculate_heading(ahrs.get_dcm_matrix()); 
	compass.null_offsets();
        }
        else
        { //report to pc that we have a health problem
          // Serial.println("COMPASS_NOT_HEALTHY"); 
          // Serial3.write(COMPASS_NOT_HEALTHY);
           return;
        }
        
        send_data10hz();
    }
    
    
     if((timer- timer_50hz) >= 20000L) //@50HZ Print
    {
        timer_50hz =timer;
       //  Vector3f drift  = ahrs.get_gyro_drift();
        
          // display all to user
     /*   Serial.printf("Heading: %.2f (%3u,%3u,%3u) ",
                      ToDeg(heading),
                      compass.mag_x,
                      compass.mag_y,
                      compass.mag_z);
           */           
                     /*
	Serial.printf_P(PSTR("r:%4.1f  p:%4.1f y:%4.1f hdg=%.2f \n"),
						ToDeg(ahrs.roll),
						ToDeg(ahrs.pitch),
						ToDeg(ahrs.yaw),
                                                ToDeg(heading));*/
       send_data50hz();                                       
						

//Serial.printf_P(PSTR("acc_x=%.2f,acc_y=%.2f,acc_z=%.2f \t gyro_x=%.2f,gyro_y=%.2f,gyro_z=%.2f  \n"), acc.x,acc.y,acc.z,gyro.x,gyro.y,gyro.z);
     //   Serial.printf_P(PSTR("hdg=%.2f, err=%.2f \n"),ToDeg(heading),ToDeg(errorz));
    
    }
    
    
      if((timer- timer_100hz) >= 10000L   && imu.new_data_available()) //@100HZ update DCM
    {
        timer_100hz = timer;
        ahrs.update(); 
        acc= imu.get_accel();
        gyro=ahrs.get_gyro();  
        
         
  
  
  //depth calculation
    k=0;
   for ( i=0 ; i<64 ; i++){
         
         k +=(float) analogRead(A8);
         
      }
      sensorValue = k/64.0;
  voltage = ((sensorValue * 5.0) / 1023.0);
  voltage = a * voltage + (1-a) * last_voltage;
  pressure = ((voltage-voltage_init) / (4.0));
  p = pressure*100.0;
  depth = 70.0 * p;
    last_voltage = voltage;
    //Serial.println(depth);
         
    }
    
}











/*Optimized CRC-16 calculation.

Polynomial: x^16 + x^15 + x^2 + 1 (0xa001)
Initial value: 0xffff
*/


/*
 *  GCS Protocol
 *
 *  4	Ardupilot Header
 *  D
 *  5	Payload length
 *  1	Message ID
 *  1	Message Version
 *  9	Payload byte 1
 *  8	Payload byte 2
 *  7	Payload byte 3
 *  A	Checksum byte 1
 *  B	Checksum byte 2

*/

 //  Header MessageID Data(nByte) CRC16_Highbyte CRC16_LowByte
void send_data10hz()
{

  uint8_t k=0;

  
   //HEADERS
  Serial3.write(0xFE);
  Serial3.write(0xCB);

 //STORING DATA
 q_10hz.f[k]=(((float) analogRead(A9))*5/1024);q_10hz.f[k+1]=(((float) analogRead(A10))*5/1024);q_10hz.f[k+2]=2000.3; //A9 - A10 for current/Voltage
 k+=3;
 q_10hz.f[k]=(((float) analogRead(A11))*5/1024);q_10hz.f[k+1]=(((float) analogRead(A12))*5/1024);q_10hz.f[k+2]=2000.6; //A11 - A12 for temperature in-out
    
  //CRC16 Calculation
  uint16_t crc=0xFFFF;
  crc= crc16(crc, 0xFE);
  crc= crc16(crc, 0xCB);
  for ( uint8_t i = 0 ; i < PACKET_SIZE_10HZ ; i++ ) //CRC calculation starting from Header byte 
       crc= crc16(crc, q_10hz.s[i]);
   
 
  //send through serial port
  for( uint8_t  i=0;i<PACKET_SIZE_10HZ;i++)
  {
  Serial3.write(q_10hz.s[i]);
   
  }
  //Sending CRC
  Serial3.write((uint8_t)(crc>>8));
  Serial3.write((uint8_t)(crc & 0x00FF));
}




void send_data50hz()
{

  uint8_t k=0;
   //HEADERS
  Serial3.write(0xFE);
  Serial3.write(0xCA);

 //STORING DATA
  /*q_50hz.f[k]=acc.x;q_50hz.f[k+1]=acc.y;q_50hz.f[k+2]=acc.z;
  k+=3;
  q_50hz.f[k]=gyro.x;q_50hz.f[k+1]=gyro.y;q_50hz.f[k+2]=gyro.y;
  k+=3;
  q_50hz.f[k]=ToDeg(ahrs.roll);q_50hz.f[k+1]=ToDeg(ahrs.pitch);q_50hz.f[k+2]=ToDeg(ahrs.yaw);
  k+=3;
  q_50hz.f[k]=-1001.0;q_50hz.f[k+1]=-1001.2;q_50hz.f[k+2]=-1001.3;
    */
    q_50hz.f[k]=1001.1;q_50hz.f[k+1]=1001.3;q_50hz.f[k+2]=1001.7;
  k+=3;
  q_50hz.f[k]=2001.3;q_50hz.f[k+1]=1999.3;q_50hz.f[k+2]=1500.2;
  k+=3;
  q_50hz.f[k]=1020.6;q_50hz.f[k+1]=1023.7;q_50hz.f[k+2]=1014.1;
  k+=3;
  q_50hz.f[k]=-1001.0;q_50hz.f[k+1]=-1001.2;q_50hz.f[k+2]=-1001.3;
  //CRC16 Calculation
  uint16_t crc=0xFFFF;
  crc= crc16(crc, 0xFE);
  crc= crc16(crc, 0xCA);
  for ( uint8_t i = 0 ; i < PACKET_SIZE_50HZ ; i++ ) //CRC calculation starting from Header byte 
       crc= crc16(crc, q_50hz.s[i]);
   
 
  //send Data through serial port
  for( uint8_t  i=0;i<PACKET_SIZE_50HZ;i++)
  {
  Serial3.write(q_50hz.s[i]);
}
  
  //Sending CRC
  Serial3.write((uint8_t)(crc>>8));

  Serial3.write((uint8_t)(crc & 0x00FF));
  //Serial.println(crc);
}



 static const unsigned short crc_table[256] = {
   0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
  0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
  0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
  0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
  0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
  0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
  0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
  0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
  0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
  0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
  0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
  0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
  0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
  0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
  0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
  0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
  0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
  0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
  0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
  0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
  0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
  0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
  0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
  0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
  0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
  0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
  0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
  0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
  0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
  0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
  0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
  0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
};


 unsigned short crc16(unsigned short crcval, unsigned char newchar)
{
crcval = (crcval >> 8) ^ crc_table[(crcval ^ newchar) & 0x00ff];
return crcval;
}


void init_press_sensor()
{
   k=0;
  // read the input on analog pin 0:
  
  
   for ( i=0 ; i<64 ; i++){
         
         k +=(float) analogRead(A8);
         
      }
      sensorValue = k/64.0;
  voltage_init = ((sensorValue * 5.0) / 1023.0);
  voltage_init = b * voltage_init + (1-b) * last_init_voltage;
  last_init_voltage = voltage_init;
  j++;
 
 
}

