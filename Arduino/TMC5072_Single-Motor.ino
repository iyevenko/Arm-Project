#include <SPI.h>

/* Datasheet: https://www.trinamic.com/fileadmin/assets/Products/ICs_Documents/TMC5072_datasheet.pdf
 *  The trinamic TMC5072 motor controller and driver operates through an 
 *  SPI interface.  Each datagram is sent to the device as an address byte
 *  followed by 4 data bytes.  This is 40 bits (8 bit address and 32 bit word).
 *  Each register is specified by a one byte (MSB) address: 0 for read, 1 for 
 *  write.  The MSB is transmitted first on the rising edge of SCK.
 *  
 * Arduino Pins   Eval Board Pins    Motor Wires
 * 51 MOSI        32 SPI1_SDI
 * 50 MISO        33 SPI1_SDO
 * 52 SCK         31 SPI1_SCK
 * 48 CS          30 SPI1_CSN
 * 17 DIO         8 DIO0 (DRV_ENN)
 * 11 DIO         23 CLK16
 * GND            2 GND
 * +5V            5 +5V
 *                01A1, 02A1         A1 (Black)
*                 01A2, 02A2         A2 (Green)
*                 01B1, 02B1         B1 (Red)
*                 01B2, 02B2         B2 (Blue)
 */

int chipCS = 48;
const byte CLOCKOUT = 11;
int enable = 17;

// See Pg. 64
// microsteps/second (256 microsteps per step, 200 steps per revolution -> 51200 microsteps/second = 60 RPM)
// Apparrently I have to multiply by 2, maybe because of CHOPCONF (see bottom of Pg. 64) but I have that set to 0b0000...
const unsigned long ONE_REV = 200L*256L;//*2L;

void setup() {
  // put your setup code here, to run once:
  pinMode(chipCS,OUTPUT);
  pinMode(CLOCKOUT,OUTPUT);
  pinMode(enable, OUTPUT);
  
  digitalWrite(chipCS,HIGH);

  //set up CLK signal
  TCCR1A = bit (COM1A0);                //toggle OC1A on Compare Match
  TCCR1B = bit (WGM12) | bit (CS10);    //CTC, no prescaling
  OCR1A = 0;                            //output every cycle

  // SPI setup
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV16);
  SPI.setDataMode(SPI_MODE3);
  SPI.begin();

  Serial.begin(9600);

  delay(10);
  
  generalConfig();
  digitalWrite(enable, LOW); // IMPORTANT: Enable driver after configuring single driver mode

  // Read Pg. 88-92, they explain how to tune these settings. For example the motor is currently emmitting a high frequency noise, so check the flow chart on Pg. 89 to try to fix that.
  // Cringe, but watch this video: https://www.youtube.com/watch?v=Tlng9MqrOlg
  // Try using 12V, it didn't rly work for me. The arm is gonna have a 24V supply so it needs to work
  // Try checking current with a multimeter, I think my shitty power supply doesn't draw enough amps to do 12V
  chopperConfig();
  pwmConfig();
  setCurrentControl(5, 10, 1); // IHOLD_IRUN: IHOLD=5, IRUN=10 (max current), IHOLDDELAY=1
  setRampBufferTime(10000); // TZEROWAIT: 10000/65536 (65536 ~= 2sec)
  setStealthChopThresholdVelocity(400000); // VHIGH=400 000: Set VHIGH to a high value to allow stealthChop
  setCoolStepThresholdVelocity(30000); // VCOOLTHRS=30000: Set upper limit for stealthChop to about 30RPM

  // spinConstantVelocity(100000, ONE_REV);
  goToPosition(10*ONE_REV, 0, 500000, 2*ONE_REV, 300000, 3*ONE_REV, 300000, 500000, 10); // This should be going way faster maybe u can fix this.
  delay(30000); // First thing for you to do: implement reading from VACTUAL and/or XACTUAL registers to go back when done
  goToPosition(0, 0, 5000000, 20*ONE_REV, 3000000, 40*ONE_REV, 3000000, 5000000, 10); //go back
}


void spinConstantVelocity(unsigned long accel, signed long vel) {
  int rampmode;
  
  if (vel >= 0) {
    rampmode = 1; // RAMPMODE=1 (positive velocity) 
  } else {
    rampmode = 2; // RAMPMODE=2 (negative velocity) 
  }
  
  setMaxAcceleration(accel); // Datasheet says to set same as vMax to make things simple but u can play with this
  setMaxVelocity(abs(vel)); 
  setRampMode(rampmode);
}

// Lots of params for six point ramp generator (See Pg. 65)
// ****Read page 66-69 vv useful****
// params explained in more depth on Pg. 31-32
void goToPosition(signed long pos, unsigned long startVel, unsigned long a1, unsigned long v1, unsigned long aMax, unsigned long vMax, unsigned long dMax, unsigned long d1, unsigned long stopVel) {
  setStartVelocity(startVel); // Usually just set to 0. Also make sure VSTART <= VSTOP
  setFirstAcceleration(a1); // Set greater than amax for that sexy sixpoint ramp
  setFirstVelocity(v1); // idk you can play with this I'm just gonna put it to halfway
  setMaxAcceleration(aMax); // Datasheet says set to same as vMax to make things simple but u can play with this
  setMaxVelocity(vMax); // go nuts
  setMaxDeceleration(dMax); // Same as aMax for symmetry
  setFirstDeceleration(d1); // IMPORTANT: DON'T SET TO 0 - even if V1 = 0 for single ramp mode. Same as a1 for symmetry
  setStopVelocity(stopVel); // IMPORTANT: DON'T SET TO 0 - use something like 10 (small but not 0). Also make sure VSTOP >= VSTART
  setRampMode(0);
  setPosition(pos); // negative for backwards
}

void loop() {
  // put your main code here, to run repeatedly:

}

void sendData(unsigned long address, unsigned long datagram, bool WRITE_notREAD) {
  //TMC5072 drivers take 40 bit data: 8 address and 32 data

  //For writing into the chip an additional 0x80 has to be added to the address
  if (WRITE_notREAD) {
    address += 0x80;
    Serial.println("Send:\tAddress\tDatagram");
    Serial.print("\t");
    Serial.print(address,HEX);
    Serial.print("\t");
    Serial.println(datagram,HEX);
  }
    
  unsigned long i_datagram;

  digitalWrite(chipCS,LOW);
  delayMicroseconds(10);

  SPI.transfer(address);

  i_datagram |= SPI.transfer((datagram >> 24) & 0xff);
  i_datagram <<= 8;
  i_datagram |= SPI.transfer((datagram >> 16) & 0xff);
  i_datagram <<= 8;
  i_datagram |= SPI.transfer((datagram >> 8) & 0xff);
  i_datagram <<= 8;
  i_datagram |= SPI.transfer((datagram) & 0xff);
  delayMicroseconds(10);
  digitalWrite(chipCS,HIGH);

  if (WRITE_notREAD) {
    address -= 0x80;
  }
  Serial.print("Received: ");
  Serial.println(i_datagram,HEX);
  Serial.print(" from register: ");
  Serial.println(address,HEX);
  Serial.println("");
}

void readData(unsigned long address) {
  sendData(address, 0x0, false);
}

// Pg. 28 (GCONF)
void generalConfig() {
  //                         10
  //                         09876543210
  unsigned long datagram = 0b00000001001; // Single driver enabled, PP and INT available
  sendData(0x0, datagram, true);
}

// Pg. 42 (CHOPCONF)
void chopperConfig() {
  // Disable 29-24 when not in STEP/DIR
  //                         3 2         1         0
  //                         10987654321098765432109876543210
  unsigned long datagram = 0b00000000000000010000000011000101; //TOFF=5, HSTRT=4, HEND=1, TBL=2, CHM=0 (spreadCycle), Enable driver
  sendData(0x6C, datagram, true);
}

// Pg. 46 (PWMCONF)
void pwmConfig() {
  //                         2 1         0
  //                         1098765432109876543210
  unsigned long datagram = 0b0001000000000111001000; // PWM_CONF: AUTO=1, 2/1024 Fclk, Switch amplitude limit=200, Grad=1

  sendData(0x10, datagram, true);
}

// Pg. 33 (IHOLD_IRUN)
void setCurrentControl(unsigned long IHOLD, unsigned long IRUN, unsigned long IHOLDDELAY) {
  unsigned long datagram = 0b0;
  datagram |= (IHOLD);
  datagram |= (IRUN << 8);
  datagram |= (IHOLDDELAY << 16);

  sendData(0x30, datagram, true);
}

// Pg. 32 (TZEROWAIT)
void setRampBufferTime(unsigned long TZEROWAIT) {
  sendData(0x2C, TZEROWAIT, true);
}

// Pg. 33 (VHIGH)
void setStealthChopThresholdVelocity(unsigned long VHIGH) {
  sendData(0x32, VHIGH, true);
}

//Pg. 33 (VCOOLTHRS)
void setCoolStepThresholdVelocity(unsigned long VCOOLTHRS) {
  sendData(0x31, VCOOLTHRS, true);
}

//Pg. 31 (RAMPMODE)
void setRampMode(unsigned long RAMPMODE) {
  sendData(0x20, RAMPMODE, true);
}

//Pg. 31 (VSTART)
void setStartVelocity(unsigned long VSTART) {
  sendData(0x23, VSTART, true);
}

//Pg. 31 (A1)
void setFirstAcceleration(unsigned long a1) {
  sendData(0x24, A1, true);
}

//Pg. 31 (V1)
void setFirstVelocity(unsigned long V1) {
  sendData(0x25, V1, true);
}

// Pg. 31 (AMAX)
void setMaxAcceleration(unsigned long AMAX) {
  sendData(0x26, AMAX, true);
}

// Pg. 31 (VMAX)
void setMaxVelocity(unsigned long VMAX) {
  sendData(0x27, VMAX, true);
}

//Pg. 31 (DMAX)
void setMaxDeceleration(unsigned long DMAX) {
  sendData(0x28, DMAX, true);
}

//Pg. 31 (D1)
void setFirstDeceleration(unsigned long D1) {
  sendData(0x2A, D1, true);
}

//Pg. 31 (VSTOP)
void setStopVelocity(unsigned long VSTOP){
  sendData(0x2B, VSTOP, true);
}

// Pg. 31 (XTARGET)
void setPosition(unsigned long XTARGET) {
  sendData(0x2D, XTARGET, true);
}
