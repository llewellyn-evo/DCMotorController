#include <SPI.h>
#include <timer.h>

#define LED       13

/* Signal Defines for DRV8704 */
#define SLEEP_n   9
#define RESET     8
#define CS        7

/* Motor Control Pins */
#define AIN1      46
#define AIN2      45
#define BIN1      5
#define BIN2      2

/* Defines for Register Address of DRV8704 */
#define CTRL      0x00
#define TORQUE    0x01
#define OFF       0x02
#define BLANK     0x03
#define DECAY     0x04
#define DRIVE     0x06
#define STATUS    0x07

/* Motor Control States */
#define BRAKE     3
#define FORWARD   2
#define BACK      1
#define STOP      0

/* Create a timer with default settings */
auto timer = timer_create_default(); 

struct DRV8704{
  bool faults[6];
  uint8_t speedA,speedB;
  uint8_t stateA, stateB;
};


DRV8704 controller = {
   .faults = {0,0,0,0,0,0},
   /* Set Default Speed of motors to 10% */
   .speedA = 25,
   .speedB = 25,
   /* Default State of Motors */
   .stateA = STOP,
   .stateB = STOP
};


void setup() {
  /* Set LED Pin as output */
  pinMode(LED, OUTPUT);

  /* Initlialize Digital Pins for DRV8704 */
  pinMode(SLEEP_n, OUTPUT);
  pinMode(RESET, OUTPUT);
  pinMode(CS, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  /* Set Initial Values for Digital Pins */
  digitalWrite(SLEEP_n, HIGH);
  digitalWrite(RESET, LOW);
  digitalWrite(CS, LOW);

  /* PWM for Motor 1 */
  TCCR5B = TCCR5B & B11111000 | B00000001;  // for PWM frequency of 31372.55 Hz
  TCCR3B = TCCR3B & B11111000 | B00000001;  // for PWM frequency of 31372.55 Hz

  /* Reset The Board */
  digitalWrite(RESET, HIGH);
  /* Set initial value for PWM */
  analogWrite(AIN1, 0);
  analogWrite(AIN2, 0);
  analogWrite(BIN1, 0);
  analogWrite(BIN2, 0);

  /* Setup Serial Communication */
  Serial.begin(115200);

  /* Setup SPI Communications for DRV8704 */
  SPI.begin();
  
  /* Turn LED on after Initialization completed */
  digitalWrite(LED, HIGH);
  
  Serial.println("**************************************");
  Serial.println("**********DRV8704 Test Setup**********");
  Serial.println("**************************************");

  /* Remove from reset */
  digitalWrite(RESET, LOW);
  
  /* Timers */
  timer.every(1000, printFault , &controller);
  timer.every(50, setMotors , &controller);


  /* Inititalize DRV8704 Parameters */
}


void open() {
  digitalWrite(CS, HIGH);
  SPI.beginTransaction(SPISettings(100000, MSBFIRST, SPI_MODE0));  
}

void close(){
  SPI.endTransaction();
  digitalWrite(CS, LOW);
}

int read(unsigned int address){
  /*
    Read from a register over SPI using Arduino SPI library.
    Args: address -> int 0xX where X <= 7
    Return: integer representing register value. 
    Example:  data = spiReadReg(0x6);
  */ 
  unsigned int value;
  address = address << 12; // allocate zeros for data
  address |= 0x8000; // set MSB to read (1)
  open();
  value = SPI.transfer16(address); // transfer read request, recieve data
  close();
  
  return value;
}

void write(unsigned int address, unsigned int value){
   /*
  Write to register over SPI using Arduino SPI library.
  address : int 0xX where X <= 0x7, 
  value : int to be written (as binary) to register. (12 bits)
  returns : true if write successful, false otherwise
  Example:  spiWriteReg(0x6, 0x0FF0);
  */
  unsigned int packet=0;

  address = address << 12; // build packet skelleton
  address &= ~0x8000; // set MSB to write (0)
  packet = address | value;
  open();  // open comms
  SPI.transfer16(packet);
  close(); // close
}

void getFault(DRV8704 *temp) {
  memset(temp->faults , 0 , sizeof(temp->faults));
  unsigned int current = read(STATUS) & 0x03F;
  if (current & 0x1) {
    temp->faults[0] = true;
  }
  if (current & 0x2) {
    temp->faults[1] = true;
  }
  if (current & 0x4) {
    temp->faults[2] = true;
  }
  if (current & 0x8) {
    temp->faults[3] = true;
  }
  if (current & 0x10) {
    temp->faults[4] = true;
  }
  if (current & 0x20) {
    temp->faults[5] = true;
  }
}

bool printFault(DRV8704 *temp){
  char str[100];
  getFault(temp);
  memset(str, 0, sizeof(str));
  sprintf(str, "FAULTS -- OTS %d  AOCP %d  BOCP %d  APDF %d  BPDF %d  UVLO %d\r\n",temp->faults[0]
                                                                                  ,temp->faults[1]
                                                                                  ,temp->faults[2]
                                                                                  ,temp->faults[3]
                                                                                  ,temp->faults[4]
                                                                                  ,temp->faults[5]);
  Serial.print(str);

  if (temp->faults[0]){
    Serial.println("Over Temperature Fault");
    clearFault(0);
  }

  if (temp->faults[1]){
    Serial.println("Channel A Over Current");
    clearFault(1);
  }

  if (temp->faults[2]){
    Serial.println("Channel B Over Current");
    clearFault(2);
  }

  if (temp->faults[3]){
    Serial.println("Channel A Predriver Fault");
    clearFault(3);
  }

  if (temp->faults[4]){
    Serial.println("Channel B Predriver Fault");
    clearFault(4);
  }

  if (temp->faults[5]){
    Serial.println("Under Voltage fault");
    clearFault(5);
  }
  
  return true; // repeat? true
}

/*
clears a Fault if there is one.
value: OTS - over temp                  (0) (auto clear)    
    AOCP - Channel A over current       (1)
    BOCP - Channel B "      "           (2)
    APDF - Channel A predriver fault    (3)
    BPDF - Channel B "         "        (4)
    UVLO - Undervoltage                 (5) (should be auto clear, But apparently not)
*/
void clearFault(int value) {
  write(STATUS, 0<<value); 
}


void setMotors(DRV8704 *temp){
  switch(temp->stateA){
    case STOP:
      analogWrite(AIN1 , 0);
      analogWrite(AIN2 , 0);
    break;

    case FORWARD:
      analogWrite(AIN1 , 255 - temp->speedA);
      analogWrite(AIN2 , 255);
    break;

    case BACK:
      analogWrite(AIN1 , 255);
      analogWrite(AIN2 , 255 - temp->speedA);
    break;

    default:
    break;
  }
  switch(temp->stateB){
    case STOP:
      analogWrite(BIN1 , 0);
      analogWrite(BIN2 , 0);
    break;

    case FORWARD:
      analogWrite(BIN1 , 255 - temp->speedB);
      analogWrite(BIN2 , 255);
    break;

    case BACK:
      analogWrite(BIN1 , 255);
      analogWrite(BIN2 , 255 - temp->speedB);
    break;

    default:
    break;
  }
  return true; // repeat? true
}

unsigned int getTDecay() {
  return read(DECAY) & 0x0FF;
}

bool setTDecay(unsigned int value) {
  unsigned int current = read(DECAY)  & ~0xF000;
  unsigned int outgoing;

  if(value <= 255 && value >= 0) {
    outgoing = current & 0xF00; // clear bits 7-0
    outgoing |= value; // set bits 7-0
  } else {
    outgoing = current; // do nothing
    return false;
  }
  write(DECAY, outgoing);
  
  return (getTDecay == value) ? true : false;
}

char * getDecMode() {
  unsigned int current = read(DECAY) & 0x700;
  char* get = "none";

  if (current == 0x000) {
    get = "slow";
  } else if (current == 0x200) {
    get = "fast";
  } else if (current == 0x300) {
    get = "mixed";
  } else if (current == 0x500) {
    get = "auto";
  }
  return get;
}

bool setDecMode(char* value) {
  unsigned int current = read(DECAY)  & ~0xF000;
  unsigned int outgoing;

  if(value == "slow") {
    outgoing = current & ~0x700;// clear bits 10-8
  } else if (value == "fast") {
    outgoing = current & ~0x700; // clear bits 10-8
    outgoing |= 0x200; // set bit 9
  } else if (value == "mixed") {
    outgoing = current & ~0x700; // clear bits 10-8
    outgoing |= 0x300; // set bits 9-8
  } else if (value == "auto") {
    outgoing = current & ~0x700; // clear bits 10-8
    outgoing |= 0x500; // set bits 10 and 8
  } else {
    outgoing = current; // do nothing
    return false;
  }
  write(DECAY, outgoing);
  return (getDecMode() == value) ? true: false;
}



int getIDriveN() {
  unsigned int current = read(DRIVE) & 0x300;
  int get = 0;

  if (current == 0x000) {
    get = 100;
  } else if (current == 0x100) {
    get = 200;
  } else if (current == 0x200) {
    get = 300;
  } else if (current == 0x300) {
    get = 400;
  }

  return get;
}

int getIDriveP() {
  unsigned int current = read(DRIVE) & 0xC00;
  int get = 0;

  if (current == 0x000) {
    get = 50;
  } else if (current == 0x400) {
    get = 100;
  } else if (current == 0x800) {
    get = 150;
  } else if (current == 0xC00) {
    get = 200;
  }

  return get;
}

char* getHbridge() {
  unsigned int current = read(CTRL) & 0x001;
  char* get = "none";

  if (current == 0) {
    get = "off";
  } else if (current == 1) {
    get = "on";
  }

  return get;
}

int getISGain() {
  unsigned int current = read(CTRL) & 0x300;
  int get = 0;

  if (current == 0x000) {
    get = 5;
  } else if (current == 0x100) {
    get = 10;
  } else if (current == 0x200) {
    get = 20;
  } else if (current == 0x300) {
    get = 40;
  }

  return get;
}

int getDTime() {
  unsigned int current = read(CTRL) & 0xC00;
  int get = 0;

  if (current == 0x000) {
    get = 410;
  } else if (current == 0x400) {
    get = 460;
  } else if (current == 0x800) {
    get = 670;
  } else if (current == 0xC00) {
    get = 880;
  }

  return get;
}

unsigned int getTorque() {
  return read(TORQUE) & 0x0FF;
}

unsigned int getTOff() {
  return read(OFF) & 0x0FF;
}

unsigned int getTBlank() {
  return read(BLANK) & 0x0FF;
}

int getOCPThresh() {
  unsigned int current = read(DRIVE) & 0x003;
  int get = 0;

  if (current == 0x000) {
    get = 250;
  } else if (current == 0x001) {
    get = 500;
  } else if (current == 0x002) {
    get = 750;
  } else if (current == 0x003) {
    get = 1000;
  }

  return get;
}

float getOCPDeglitchTime() {
  unsigned int current = read(DRIVE) & 0x00C;
  float get = 0;

  if (current == 0x000) {
    get = 1.05;
  } else if (current == 0x002) {
    get = 2.1;
  } else if (current == 0x004) {
    get = 4.2;
  } else if (current == 0x00C) {
    get = 8.4;
  }

  return get;
}

int getTDriveN() {
  unsigned int current = read(DRIVE) & 0x030;
  int get = 0;

  if (current == 0x000) {
    get = 263;
  } else if (current == 0x010) {
    get = 525;
  } else if (current == 0x020) {
    get = 1050;
  } else if (current == 0x030) {
    get = 2100;
  }

  return get;
}

int getTDriveP() {
  unsigned int current = read(DRIVE) & 0x0C0;
  int get = 0;

  if (current == 0x000) {
    get = 263;
  } else if (current == 0x040) {
    get = 525;
  } else if (current == 0x080) {
    get = 1050;
  } else if (current == 0x0C0) {
    get = 2100;
  }

  return get;
}



bool setIDriveN(int value) {
  unsigned int current = read(DRIVE) & ~0xF000;
  unsigned int outgoing;

  if (value == 100) {
    outgoing = current & ~0x300; // clear bits 9-8
  } else if (value == 200) {
    outgoing = current & ~0x300; // clear bits 9-8
    outgoing |= 0x100; // set bit 8
  } else if (value == 300) {
    outgoing = current & ~0x300; // clear bits 9-8
    outgoing |= 0x200; // set bit 9
  } else if (value == 400) {
    outgoing = current | 0x300; // set bits 9-8
  } else {
    outgoing = current;
    return false;
  }

  write(DRIVE, outgoing);
  return (getIDriveN == value) ? true : false ;
}

bool setIDriveP(int value) {
  unsigned int current = read(DRIVE) & ~0xF000;
  unsigned int outgoing;

  if (value == 50) {
    outgoing = current & ~0xC00; // clear bits 11-10
  } else if (value == 100) {
    outgoing = current & ~0xC00; // clear bits 11-10
    outgoing |= 0x400; // set bit 8
  } else if (value == 150) {
    outgoing = current & ~0xC00; // clear bits 11-10
    outgoing |= 0x800; // set bit 9
  } else if (value == 200) {
    outgoing = current | 0xC00; // set bits 11-10
  } else {
    outgoing = current;
    return false;
  }

  write(DRIVE, outgoing);
  return (getIDriveP() == value) ? true : false;
}

bool setHbridge(char* value) {
  // clear bits 16-13 from the read data (not used)
  unsigned int current = read(CTRL) & ~0xF000;
  unsigned int outgoing;

  if (value == "off") {
    outgoing = current & ~0x001; // clear bit 0
  } else if (value == "on") {
    outgoing = current | 0x001; // set bit 0
  } else {
    outgoing = current; // do nothing
    return false;
  }

  write(CTRL, outgoing);

  return (getHbridge() == value ? true : false);
}

bool setISGain(int value) {
  unsigned int current = read(CTRL) & ~0xF000;
  unsigned int outgoing;

  if (value == 5) {
    outgoing = current & ~0x300; // clear bits 9-8
  } else if (value == 10) {
    outgoing = current | 0x100; // set bit 8
    outgoing &= ~0x200; // clear bit 9
  } else if (value == 20) {
    outgoing = current | 0x200; // set bit 9
    outgoing &= ~0x100; // clear bit 8
  } else if (value == 40) {
    outgoing = current | 0x300; // set bits 9-8
  } else {
    outgoing = current; // do nothing
    return false;
  }
  
  write(CTRL, outgoing);

  return (getISGain() == value) ? true : false;
}

bool setDTime(int value) {
  unsigned int current = read(CTRL) & ~0xF000;
  unsigned int outgoing;
  
  if (value == 410) {
    outgoing = current & ~0xC00; // clear bits 11-10
  } else if (value == 460) {
    outgoing = current | 0x400; // set bit 10
    outgoing &= ~0x800; // clear bit 11
  } else if (value == 670) {
    outgoing = current | 0x800; // set bit 11
    outgoing &= ~0x400; // clear bit 10
  } else if (value == 880) {
    outgoing = current | 0xC00; // set bits 11-10
  } else {
    outgoing = current; // do nothing
    return false;
  }

  write(CTRL, outgoing);

  return (getDTime() == value) ? true : false;
}

bool setTorque(unsigned int value) {
  unsigned int current = read(TORQUE) & ~0xF000;
  unsigned int outgoing;

  if(value <= 255 && value >= 0) {
    outgoing = current & 0xF00; // clear bits 7-0
    outgoing |= value; // set bits 7-0
  } else {
    outgoing = current; // do nothing
    return false;
  }

  write(TORQUE, outgoing);
  return (getTorque() == value) ? true : false;
}

bool setTOff(unsigned int value) {
  unsigned int current = read(OFF) & ~0xF000;
  unsigned int outgoing;

  if(value <= 255 && value >= 0) {
    outgoing = current & 0xF00; // clear bits 7-0
    outgoing |= value; // set bits 7-0
  } else {
    outgoing = current; // do nothing
    return false;
  }
  
  write(OFF, outgoing);
  return (getTOff() == value) ? true : false;
}

bool setTBlank(unsigned int value) {
  unsigned int current = read(BLANK) & ~0xF000;
  unsigned int outgoing;

  if(value <= 255 && value >= 0) {
    outgoing = current & 0xF00; // clear bits 7-0
    outgoing |= value; // set bits 7-0
  } else {
    outgoing = current; // do nothing
    return false;
  }
  
  write(BLANK, outgoing);
  return (getTBlank() == value) ? true: false ;
}

bool setTDriveN(int value) {
  unsigned int current = read(DRIVE) & ~0xF000;
  unsigned int outgoing;

  if (value == 263) {
    outgoing = current & ~0x030; // clear bits 5-4
  } else if (value == 525) {
    outgoing = current & ~0x030; // clear bits 5-4
    outgoing |= 0x010; // set bit 4
  } else if (value == 1050) {
    outgoing = current & ~0x030; // clear bits 5-4
    outgoing |= 0x020; // set bit 5
  } else if (value == 2100) {
    outgoing = current | 0x030; // set bits 5-4
  } else {
    outgoing = current;
    return false;
  }

  write(DRIVE, outgoing);
  return (getTDriveN() == value) ? true : false;
}

bool setTDriveP(int value) {
  unsigned int current = read(DRIVE) & ~0xF000;
  unsigned int outgoing;

  if (value == 263) {
    outgoing = current & ~0x0C0; // clear bits 7-6
  } else if (value == 525) {
    outgoing = current & ~0x0C0; // clear bits 7-6
    outgoing |= 0x040; // set bit 6
  } else if (value == 1050) {
    outgoing = current & ~0x0C0; // clear bits 7-6
    outgoing |= 0x080; // set bit 7
  } else if (value == 2100) {
    outgoing = current | 0x0C0; // set bits 7-6
  } else {
    outgoing = current;
    return false;
  }
  
  write(DRIVE, outgoing);
  return (getTDriveP() == value) ? true : false;
}

bool setOCPThresh(int value) {
  unsigned int current = read(DRIVE)  & ~0xF000;
  unsigned int outgoing;

  if (value == 250) {
    outgoing = current & ~0x003; // clear bits 1-0
  } else if (value == 500) {
    outgoing = current & ~0x003; // clear bits 1-0
    outgoing |= 0x001; // set bit 0
  } else if (value == 750) {
    outgoing = current & ~0x003; // clear bits 1-0
    outgoing |= 0x002; // set bit 1
  } else if (value == 1000) {
    outgoing = current | 0x003; // set bits 1-0
  } else {
    outgoing = current;
    return false;
  }
  
  write(DRIVE, outgoing);
  return (getOCPThresh() == value) ? true : false;
}

bool setOCPDeglitchTime(float value) {
  unsigned int current = read(DRIVE) & ~0xF000;
  unsigned int outgoing;

  if (value == 1.05) {
    outgoing = current & ~0x00C; // clear bits 3-2
  } else if (value == 2.1) {
    outgoing = current & ~0x00C; // clear bits 3-2
    outgoing |= 0x004; // set bit 2
  } else if (value == 4.2) {
    outgoing = current & ~0x00C; // clear bits 3-2
    outgoing |= 0x008; // set bit 3
  } else if (value == 8.4) {
    outgoing = current | 0x00C; // set bits 3-2
  } else {
    outgoing = current;
    return false;
  }

  write(DRIVE, outgoing);
  return (getOCPDeglitchTime() == value) ? true : false;
}

void loop() {
  char ch;
  timer.tick();
  if (Serial.available()){
    ch = Serial.read();
    switch(ch){
      case 'i':
      case 'I':
        /* Increase PWM value for Motor A */
        (controller.speedA <= 250)?  controller.speedA += 5 : controller.speedA = 255;
        Serial.print("SpeedA = ");
        Serial.println(controller.speedA);
      break;

      case 'k':
      case 'K':
        /* Decrement PWM Value for Motor A */
        (controller.speedA >= 5)?  controller.speedA -= 5 : controller.speedA = 0;
        Serial.print("SpeedA = ");
        Serial.println(controller.speedA);
      break;

      case 'o':
      case 'O':
        /* Increase PWM value for Motor B */
        (controller.speedB <= 250)?  controller.speedB += 5 : controller.speedB = 255;
        Serial.print("SpeedB = ");
        Serial.println(controller.speedB);
      break;

      case 'l':
      case 'L':
        /* Decrement PWM Value for Motor B */
        (controller.speedB >= 5)?  controller.speedB -= 5 : controller.speedB = 0;
        Serial.print("SpeedB = ");
        Serial.println(controller.speedB);
      break;

      case 'q':
      case 'Q':
        controller.stateA = FORWARD;
        Serial.println("Motor A - FORWARD");
      break;

      case 'a':
      case 'A':
        controller.stateA = STOP;
        Serial.println("Motor A - STOP");
      break;

      case 'z':
      case 'Z':
        controller.stateA = BACK;
        Serial.println("Motor A - BACKWARD");
      break;

      case 'w':
      case 'W':
        controller.stateB = FORWARD;
        Serial.println("Motor B - FORWARD");
      break;

      case 's':
      case 'S':
        controller.stateB = STOP;
        Serial.println("Motor B - STOP");
      break;

      case 'x':
      case 'X':
        controller.stateB = BACK;
        Serial.println("Motor B - BACKWARD");
      break;

      case 'j':
      case 'J':
        Serial.println("Disabling H-Bridge");
        setHbridge("off");
      break;

      case 'u':
      case 'U':
        Serial.println("Enabling H-Bridge");
        setHbridge("on");
      break;

      default:
      break;
    }
  }
}
