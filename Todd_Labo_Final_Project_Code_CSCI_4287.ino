#include "SD.h"
#define SD_ChipSelectPin 53
#include "TMRpcm.h"
#include "SPI.h"


// Radio
TMRpcm tmrpcm;

const int chipSelect = 53;
boolean carIsOn = false;

const byte CAR_START_INT_PIN = 3;
const byte RADIO_INT_PIN = 18;
boolean radioOn = true;
volatile byte car_state = LOW;



//#include <ServoTimer2.h>
//ServoTimer2 myservo;
// Motors
  #include <Wire.h>
  #include <Adafruit_MotorShield.h>
  #include "utility/Adafruit_MS_PWMServoDriver.h"

  
  Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
  Adafruit_DCMotor *myMotor1 = AFMS.getMotor(1);
  Adafruit_DCMotor *myMotor2 = AFMS.getMotor(2);
  Adafruit_DCMotor *myMotor3 = AFMS.getMotor(3);
  Adafruit_DCMotor *myMotor4 = AFMS.getMotor(4);

  const int SLOW_SPEED = 30;
  const int MED_SPEED = 75;
  const int FAST_SPEED = 200;

// HC-SR04 
#include <SR04.h>
#include "SR04.h"
#include <NewPing.h> 

#define TRIG_PIN 22
#define ECHO_PIN 4

//long a;
long leftDist;
long rightDist;

SR04 sr04 = SR04(ECHO_PIN,TRIG_PIN);
#define maximum_distance 200
const int PING_DISTANCE = 25;
NewPing sonar(TRIG_PIN, ECHO_PIN, maximum_distance); //sensor function


// Servo 
#include <ServoTimer2.h>
ServoTimer2 myservo;

int pos;    // variable to store the servo position
int const INITIAL_POS = 1150;

//LCD Screen
#include <LiquidCrystal.h> // includes the LiquidCrystal Library 
LiquidCrystal lcd(48, 38, 42, 8, 7, 44); // Creates an LC object. Parameters: (rs, enable, d4, d5, d6, d7) 

// Temperature (Thermister)
const int INTERRUPT_PIN = 2;

volatile float steinhart;
boolean displayFaren = true;

 // For thermister 
#define THERMISTORPIN A1         
// resistance at 25 degrees C
#define THERMISTORNOMINAL 10000       
#define TEMPERATURENOMINAL 25   
// how many samples to take and average, more takes longer
// but is more 'smooth'
#define NUMSAMPLES 5
// The beta coefficient of the thermistor (usually 3000-4000)
#define BCOEFFICIENT 3950
// the value of the 'other' resistor
#define SERIESRESISTOR 10000  
int samples[NUMSAMPLES];  


void setup() {
  Serial.begin(9600);

  //Radio
  tmrpcm.speakerPin = 46;
  if (!SD.begin(SD_ChipSelectPin)) {
    Serial.println("SD fail");
    return;
  }
  
  //pinMode(18, OUTPUT);
  pinMode(CAR_START_INT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(CAR_START_INT_PIN), carStart, RISING);
  pinMode(RADIO_INT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RADIO_INT_PIN), playRadio, FALLING);
  //Serial.begin(9600);
   
  //Servo
  myservo.attach(6);  // attaches the servo on pin 6 to the servo object
  myservo.write(INITIAL_POS);
  
  delay(1000); 

  // For lcd screen
    lcd.begin(16,2); // Initializes the interface to the LCD screen, and specifies the dimensions (width and height) of the display } 

  AFMS.begin();

  // PhotoResistor
    pinMode(A5, INPUT);
    pinMode(40, OUTPUT);
  
    pinMode(INTERRUPT_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), setTempDisplayType, FALLING );

}

ISR(TIMER4_COMPA_vect) {
  setHeadlights();
  computeTemp();
}


void playRadio(){
 
  if (!radioOn){
    
    // Get random numer
    long randNumber;
    randomSeed(millis());
    randNumber = random(6);
    tmrpcm.setVolume(5);
    lcd.setCursor(0, 1);
    Serial.println(randNumber);

    switch (randNumber){
      case 0: 
        tmrpcm.play("Spirit.wav");
        lcd.print("SPIRIT OF RADIO");
        break;
      
      case 1: 
        tmrpcm.play("Hand.wav");
        lcd.print("HAND CAN'T ERASE");
        break;
      
      case 2: 
        tmrpcm.play("NewThing.wav");
        lcd.print("NEW THING");
        break;
      
      case 3: 
        tmrpcm.play("Rival.wav");
        lcd.print("ELECTRIC MAN");
        break;
      
      case 4: 
        tmrpcm.play("Floyd.wav");
        lcd.print("HAVE A CIGAR");
        break;
      
      case 5: 
        tmrpcm.play("Opeth.wav");
        lcd.print("HARLEQUIN FOREST");
        break;
      
      default: 
        tmrpcm.stopPlayback();
        lcd.print("playback error");
        break;
      }
      
  }else{
    tmrpcm.stopPlayback();
    lcd.setCursor(0, 1);
    lcd.print("                 ");
    
  }
  radioOn = !radioOn;
} 



void carStart(){
  if (!carIsOn){
    carIsOn = !carIsOn;
    tmrpcm.setVolume(5);
    tmrpcm.play("Car.wav");
    delay(9000);
  
  Serial.println("car start done");

  // Set TIMER4_COMPA_vect Timer
    cli();
      TCCR4A = 0;
      TCCR4B = 0;
      TCNT4  = 0;
      OCR4A = 31250;
      TCCR4B |= (1 << WGM12); 
      TCCR4B |= (1 << CS12);   // set bit 2 for the prescaler to be 1024 (combined with OCR1A set to 32250 gives an approx 2 second delay)
      TCCR4B |= (1 << CS10);   // set bit 0 for the prescaler to be 1024
      TIMSK4 |= (1 << OCIE4A);
    sei();

  //Start Car moving forward
    myMotor1->setSpeed(MED_SPEED);
    myMotor2->setSpeed(MED_SPEED);
    myMotor3->setSpeed(MED_SPEED);
    myMotor4->setSpeed(MED_SPEED);
  
    myMotor1->run(FORWARD);
    myMotor2->run(FORWARD);
    myMotor3->run(FORWARD);
    myMotor4->run(FORWARD);
  }
  else{
      tmrpcm.stopPlayback();
      carIsOn = !carIsOn;
      Serial.println("car is off");
      stopCar();
   } 
}

void stopCar(){
      myMotor1->run(RELEASE);
      myMotor2->run(RELEASE);
      myMotor3->run(RELEASE);
      myMotor4->run(RELEASE);}
  


void computeTemp(){

  // take N samples in a row, with a slight delay
  for (uint8_t i = 0; i < NUMSAMPLES; i++) {
    samples[i] = analogRead(THERMISTORPIN);
  }
  
  // average all the samples out
  float average = 0;
  for (uint8_t i=0; i< NUMSAMPLES; i++) {
     average += samples[i];
  }
  average /= NUMSAMPLES;
  
  // convert the value to resistance
  average = 1023 / average - 1;
  average = SERIESRESISTOR / average;
  
  steinhart = average / THERMISTORNOMINAL;          // (R/Ro)
  steinhart = log(steinhart);                       // ln(R/Ro)
  steinhart /= BCOEFFICIENT;                        // 1/B * ln(R/Ro)
  steinhart += 1.0 / (TEMPERATURENOMINAL + 273.15); // + (1/To)
  steinhart = 1.0 / steinhart;                      // Invert
  steinhart -= 273.15;  

  displayTemp(displayFaren, steinhart);
}

 
void setTempDisplayType(){
  displayFaren = !displayFaren;
  computeTemp();
}


void displayTemp(boolean displayType, float temp){
    if(displayType){
       float far = temp * 1.8 + 32.0;
       lcd.setCursor(0, 0);
       lcd.print(far, 0);  // second argument means print 0 decimal places
       lcd.print(char(223)); // Degree Symbol
       lcd.print("F");
    }
    else{
       lcd.setCursor(0, 0);
       lcd.print(temp, 0);  // second argument means print 0 decimal places
       lcd.print(char(223)); // Degree Symbol
       lcd.print("C");
    }   
}  

void setHeadlights(){
  int value = analogRead(A0);
//  Serial.print("Analog value : ");
//  Serial.println(value);

  if(value < 300){
    digitalWrite(40, HIGH);
  }
  else{
    digitalWrite(40, LOW);
  }
}


int getDistance(){
  delay(70);
  int cm = sonar.ping_cm();
  if (cm == 0){
    cm = 250;
  }
  return cm;
}

  void avoidObject(){
    // Stop
      myMotor1->run(RELEASE);
      myMotor2->run(RELEASE);
      myMotor3->run(RELEASE);
      myMotor4->run(RELEASE);
      
    // First backup
      myMotor1->setSpeed(MED_SPEED);
      myMotor2->setSpeed(MED_SPEED);
      myMotor3->setSpeed(MED_SPEED);
      myMotor4->setSpeed(MED_SPEED);

      myMotor1->run(BACKWARD);
      myMotor2->run(BACKWARD);
      myMotor3->run(BACKWARD);
      myMotor4->run(BACKWARD);
      
      delay(1000);  // Shoukd use millis idea instead

      myMotor1->run(RELEASE);
      myMotor2->run(RELEASE);
      myMotor3->run(RELEASE);
      myMotor4->run(RELEASE);

    //look right and get distance
      myservo.write(750);
      delay(500);
      rightDist = getDistance();
      Serial.print("Right Distance: ");
      Serial.println(rightDist); 
      delay(500);

    // Look Left and get distance
      myservo.write(1500);
      delay(500);
      leftDist = getDistance();;
      Serial.print("Left Distance: ");
      Serial.println(leftDist);
      delay(500);
   
      myservo.write(INITIAL_POS);
      
      if(leftDist > rightDist){
        Serial.println("Go Left");
        // Turn Left
          myMotor1->setSpeed(SLOW_SPEED);
          myMotor2->setSpeed(FAST_SPEED);
          myMotor3->setSpeed(FAST_SPEED);
          myMotor4->setSpeed(SLOW_SPEED);
   
          myMotor1->run(FORWARD);
          myMotor2->run(FORWARD);
          myMotor3->run(FORWARD);
          myMotor4->run(FORWARD);
      }
  
      else{
        Serial.println("Go Right");
        // Turn Right
          myMotor1->setSpeed(FAST_SPEED);
          myMotor2->setSpeed(SLOW_SPEED);
          myMotor3->setSpeed(SLOW_SPEED);
          myMotor4->setSpeed(FAST_SPEED);
        
          myMotor1->run(FORWARD);
          myMotor2->run(FORWARD);
          myMotor3->run(FORWARD);
          myMotor4->run(FORWARD);
      }
     delay(1500); 

    //Set Car Forward
      myMotor1->run(RELEASE);
      myMotor2->run(RELEASE);
      myMotor3->run(RELEASE);
      myMotor4->run(RELEASE);
      
      myMotor1->setSpeed(MED_SPEED);
      myMotor2->setSpeed(MED_SPEED);
      myMotor3->setSpeed(MED_SPEED);
      myMotor4->setSpeed(MED_SPEED);
    
      myMotor1->run(FORWARD);
      myMotor2->run(FORWARD);
      myMotor3->run(FORWARD);
      myMotor4->run(FORWARD);
 }


void loop() {
  
  if(getDistance() < PING_DISTANCE){
    Serial.print("Range ");
    Serial.println(getDistance());
    avoidObject();
  }

}
