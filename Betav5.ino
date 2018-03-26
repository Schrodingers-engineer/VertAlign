#include <Wire.h>
#include <EEPROM.h>

#define DEVICE_A 	(0x1D)    //first ADXL345 device address
#define DEVICE_B 	(0x53)    //second ADXL345 device address
#define TO_READ		(6)        //num of bytes we are going to read each time (two bytes for each axis)

#define SERIAL_BAUDRATE (9600)

//state definitions
#define GOOD      (10)
#define BAD       (11)
#define FAILING   (12)
#define RECOVERING (13)  

#define ADXL345_DATA_FORMAT		(0x31)		// Data Format Control

byte buff[TO_READ];  
byte buff2[TO_READ];      //6 bytes buffer for saving data read from the device
char str[512];            //string buffer to transform data before sending it to the serial port

#define vibrator  (5)
#define led_1     (6)
#define led_2     (7)
#define led_3     (8)
#define led_bluetooth   (12)
#define bt_calib (2)

#define regAddress  (0x32)      //first axis-acceleration-data register on the ADXL345

int xa = 0, ya = 0, za = 0;  
int xb = 0, yb = 0, zb = 0;

int cal_xa = 0, cal_ya = 0, cal_za = 0; 
int cal_xb = 0, cal_yb = 0, cal_zb = 0;

int eep_xa = 0, eep_ya = 0, eep_za = 0; 
int eep_xb = 0, eep_yb = 0, eep_zb = 0;

int state = GOOD;
long timer = 0;
int max_timer = 2;

int bt_calib_ctr = 0;

//set threshold here for the posture notification and states
int xthreshold = 45;
int ythreshold = 30;
int zthreshold = 30;

int button_state = 0;
int volt_vibe = 0;

int vibe_ctr = 0;

byte incoming[2];
int int_read = 0;

void setup() {
  Wire.begin();        // join i2c bus (address optional for master)
  Serial.begin(SERIAL_BAUDRATE);  // start serial for output
  
  //Turning on both ADXL345s
  writeTo(DEVICE_A, 0x2D, 0);   
  writeTo(DEVICE_A, 0x2D, 16); 
  writeTo(DEVICE_A, 0x2D, 8); 
  
  writeTo(DEVICE_B, 0x2D, 24);
  writeTo(DEVICE_B, 0x2D, 16);
  writeTo(DEVICE_B, 0x2D, 8);
  
  //Set range for highest sensitivity (2g)
  setRange(2);
  
  //Set pin modes for other components
  //LED_BUILTIN - power indicator
  pinMode(vibrator, OUTPUT);
  pinMode(led_1, OUTPUT);
  pinMode(led_2, OUTPUT);
  pinMode(led_3, OUTPUT);
  pinMode(led_bluetooth, OUTPUT);
  pinMode(bt_calib, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);

//  Serial.print("------------5 SECOND DELAY-----------");
//  Serial.write(10);
//  delay(5000);
  EEP_display();
  //delay(500);
  //calibrate();
}
  
void loop() {  
//  digitalWrite(led_1, LOW);
//  digitalWrite(led_2, LOW);
//  digitalWrite(led_3, LOW);
//  digitalWrite(led_bluetooth, LOW);
//  digitalWrite(LED_BUILTIN, HIGH);

  button_state = digitalRead(bt_calib);
  digitalWrite(LED_BUILTIN, button_state);

  if (button_state == HIGH)
  {
    if (++bt_calib_ctr > 30)
    {
      calibrate();
      bt_calib_ctr = 0;
    }
  }
  else
  {
    if (bt_calib_ctr <= 30 && bt_calib_ctr > 0)
    {
      if (max_timer == 2)
      {
        max_timer = 5;
        Serial.print("5");
        Serial.write(10);
        digitalWrite(led_1, LOW);
        digitalWrite(led_2, HIGH);
        digitalWrite(led_3, LOW);
      }
      else if (max_timer == 5)
      {
        max_timer = 10;
        Serial.print("10");
        Serial.write(10);
        digitalWrite(led_1, LOW);
        digitalWrite(led_2, LOW);
        digitalWrite(led_3, HIGH);
      }
      else
      {
        max_timer = 2;
        Serial.print("2");
        Serial.write(10);
        digitalWrite(led_1, HIGH);
        digitalWrite(led_2, LOW);
        digitalWrite(led_3, LOW);
      }
     }
  
    bt_calib_ctr = 0;
  }
  
//  digitalWrite(led_3, button_state);

  readFrom(DEVICE_A, regAddress, TO_READ, buff); //read the acceleration data from the ADXL345  
   //each axis reading comes in 10 bit resolution, ie 2 bytes.  Least Significat Byte first!!
   //thus we are converting both bytes in to one int
  xa = (((int)buff[1]) << 8) | buff[0];   
  ya = (((int)buff[3])<< 8) | buff[2];
  za = (((int)buff[5]) << 8) | buff[4];
  
  readFrom(DEVICE_B, regAddress, TO_READ, buff2); //read the acceleration data from the second ADXL345
  xb = (((int)buff2[1]) << 8) | buff2[0];   
  yb = (((int)buff2[3])<< 8) | buff2[2];
  zb = (((int)buff2[5]) << 8) | buff2[4];

  switch(state) {
    case GOOD:
      // the moment it exits good posture, go FAILING
      digitalWrite(led_1, HIGH);
      digitalWrite(led_2, LOW);
      digitalWrite(led_3, LOW);
      if (!checkThreshold(xa, ya, za, xb, yb, zb))
      {
        timer = 0;
        state = FAILING;
      }
      analogWrite(vibrator, 0);
      break;
    case BAD:
      if (checkThreshold(xa, ya, za, xb, yb, zb))
        {
          timer = 0;
          state = RECOVERING;
        }
      if (++vibe_ctr % 5 == 0)
      {
        analogWrite(vibrator, 140);
      }
      else
      {
        analogWrite(vibrator, 0);
      }
      break;
    case FAILING:
    // if bad posture sustained past max_timer, go to BAD
      if (!checkThreshold(xa, ya, za, xb, yb, zb))
      {
        if (++timer >= max_timer*10)
        {
          state = BAD;
          Serial.print("BAD");    // this is to be sent to the app
          Serial.write(10);
        }
      }
      // if bad posture lasts less than max_timer, go back to GOOD
      else
      {
        if (++timer < max_timer*10)
        {
          state = GOOD;
        }
      }
      analogWrite(vibrator, 0);
      break;
    case RECOVERING:
      if (checkThreshold(xa, ya, za, xb, yb, zb))
      {
        if (++timer >= max_timer*10)
        {
          state = GOOD;
          Serial.print("GOOD");
          Serial.write(10);
        }
      }
      else
      {
        if (++timer < max_timer*10)
        {
          state = BAD;
        }
      }
      if (++vibe_ctr % 5 == 0)
      {
        analogWrite(vibrator, 140);
      }
      else
      {
        analogWrite(vibrator, 0);
      }

      break;
  
  }
  //the 100ms of dead space is eliminated through this loop
  // checks serial every 20ms for a command
  for (int ctr = 0; ctr < 5; ctr++)
  {
    if (Serial.available() > 0) 
    {
        parseBluetooth();
    }
  delay(20);
  }
  //It appears that delay is needed in order not to clog the port
  
}

//---------------- Functions---------------------------

void parseBluetooth()
{
   // read the incoming integer:

//    int_read = Serial.parseInt(); 
    byte byte_read;
    byte bytes_read[2];
    byte_read = Serial.readBytes(bytes_read,2);
    sprintf(str, "SERIAL INPUT: %d %d", bytes_read[0], bytes_read[1]);  
    Serial.print(str);
    Serial.write(10);
    
    if(bytes_read[0] == 255)
    {
      calibrate();
    }
    else if (bytes_read[0] == 127)
    {
      sprintf(str, "%d", max_timer);
      Serial.print(str);
      Serial.write(10);
      for (int i = 1; i < 6; i++)
      {
        digitalWrite(LED_BUILTIN, i % 2);
        delay(50);
      }
      digitalWrite(LED_BUILTIN, LOW);
    }
    else if (bytes_read[0] == 0)
    {
//      max_timer = int_read - 20000;
      max_timer = bytes_read[1];
      Serial.print("Timer set.");
      Serial.write(10);
      digitalWrite(led_1, LOW);
      digitalWrite(led_2, LOW);
      digitalWrite(led_3, LOW);
    }

delay(1000);
}

//Writes val to address register on device
void writeTo(int device, byte address, byte val) {
   Wire.beginTransmission(device); //start transmission to device 
   Wire.write(address);        // send register address
   Wire.write(val);        // send value to write
   Wire.endTransmission(); //end transmission
}

bool checkThreshold (int t_xa, int t_ya, int t_za, int t_xb, int t_yb, int t_zb)
{
  
  if (abs(t_xa - eep_xa) > xthreshold || abs(t_ya - eep_ya) > ythreshold || abs(t_za - eep_za) > zthreshold || abs(t_xb - eep_xb) > xthreshold || abs(t_yb - eep_yb) > ythreshold || abs(t_zb - eep_zb) > zthreshold)
  {
    return false;
  }
  else
  {
    return true;
  }
}

void calibrate()
{
  sprintf(str, "Calibrating...");
  Serial.print(str);
  Serial.write(10);
  analogWrite(vibrator, 140);
  delay(50);
  analogWrite(vibrator, 0);
  for (int i = 0; i < 100; i++)
  {
    readFrom(DEVICE_A, regAddress, TO_READ, buff); //read the acceleration data from the ADXL345  
    //each axis reading comes in 10 bit resolution, ie 2 bytes.  Least Significat Byte first!!
    //thus we are converting both bytes in to one int
    cal_xa += (((int)buff[1]) << 8) | buff[0];   
    cal_ya += (((int)buff[3])<< 8) | buff[2];
    cal_za += (((int)buff[5]) << 8) | buff[4];
    
    readFrom(DEVICE_B, regAddress, TO_READ, buff2); //read the acceleration data from the second ADXL345
    cal_xb += (((int)buff2[1]) << 8) | buff2[0];   
    cal_yb += (((int)buff2[3])<< 8) | buff2[2];
    cal_zb += (((int)buff2[5]) << 8) | buff2[4];
//    sprintf(str, "Sensor A: %d %d %d ----- Sensor B: %d %d %d ", cal_xa, cal_ya, cal_za, cal_xb, cal_yb, cal_zb); 
//    Serial.print(str);
//    Serial.write(10);

    digitalWrite(LED_BUILTIN, i % 2);
    delay(100);
  }

  //averaging out values obtained
//  sprintf(str, "Saving to memory.");
//  Serial.print(str);
//  Serial.write(10);
  cal_xa /= 100;
  cal_ya /= 100;
  cal_za /= 100;
  cal_xb /= 100;
  cal_yb /= 100;
  cal_zb /= 100;
//  sprintf(str, "AVERAGE: Sensor A: %d %d %d ----- Sensor B: %d %d %d ", cal_xa, cal_ya, cal_za, cal_xb, cal_yb, cal_zb); 
//  Serial.print(str);
//  Serial.write(10);
  
  EEPROMWrite16(0, cal_xa + 1000);
  EEPROMWrite16(2, cal_ya + 1000);
  EEPROMWrite16(4, cal_za + 1000);
  EEPROMWrite16(6, cal_xb + 1000);
  EEPROMWrite16(8, cal_yb + 1000);
  EEPROMWrite16(10, cal_zb + 1000);

  EEP_display();
  Serial.print("Calibration done.");
  Serial.write(10);
}

// This function populates the vars with EEP values
void EEP_display()
{
  eep_xa = EEPROMRead16(0) - 1000;
  eep_ya = EEPROMRead16(2) - 1000;
  eep_za = EEPROMRead16(4) - 1000;
  eep_xb = EEPROMRead16(6) - 1000;
  eep_yb = EEPROMRead16(8) - 1000;
  eep_zb = EEPROMRead16(10) - 1000;
  
//  sprintf(str, "EEPROM values: Sensor A: %d %d %d ----- Sensor B: %d %d %d ", eep_xa, eep_ya, eep_za, eep_xb, eep_yb, eep_zb);
//  Serial.print(str);
//  Serial.write(10);

}

//This function will write a 2 byte, 16-bit integer into EEPROM
//Adapted from something I found online - JL
void EEPROMWrite16(int address, int value)
{
  //Decomposition from a long to 4 bytes by using bitshift.
  //One = Most significant -> Two = Least significant byte
  byte two = (value & 0xFF);
  byte one = ((value >> 8) & 0xFF);
  
  //Write the 2 bytes into the eeprom memory.
  EEPROM.write(address, two);
  EEPROM.write(address + 1, one);
}

// reads a 16bit integer from eeprom
// same source as the write
int EEPROMRead16(int address)
{
  //Read the 2 bytes from the eeprom memory.
  int two = EEPROM.read(address);
  int one = EEPROM.read(address + 1);
  
  //Return the recomposed int by using bitshift.
  return ((two << 0) & 0xFF) + ((one << 8) & 0xFFFF);
}


//Reads num bytes starting from address register on device in to buff array
void readFrom(int device, byte address, int num, byte buff[]) {
  Wire.beginTransmission(device); //start transmission to device 
  Wire.write(address);        //sends address to read from
  Wire.endTransmission(); //end transmission
  
  Wire.beginTransmission(device); //start transmission to device
  Wire.requestFrom(device, num);    // request 6 bytes from device
  
  int i = 0;
  while(Wire.available())    //device may send less than requested (abnormal)
  { 
    buff[i] = Wire.read(); // receive a byte
    i++;
  }
  Wire.endTransmission(); //end transmission
}

//Sets the range for both accelerometers
void setRange(int val) {
	byte _s;
	byte _b;
	
	switch (val) {
		case 2:  
			_s = B00000000; 
			break;
		case 4:  
			_s = B00000001; 
			break;
		case 8:  
			_s = B00000010; 
			break;
		case 16: 
			_s = B00000011; 
			break;
		default: 
			_s = B00000000;
	}
	readFrom(DEVICE_A, ADXL345_DATA_FORMAT, 1, &_b);
	_s |= (_b & B11101100);
	writeTo(DEVICE_A, ADXL345_DATA_FORMAT, _s);
	writeTo(DEVICE_B, ADXL345_DATA_FORMAT, _s);
}
