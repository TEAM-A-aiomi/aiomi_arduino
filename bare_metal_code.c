//Basic setup from https://create.arduino.cc/projecthub/milanistef/introduction-to-bare-metal-programming-in-arduino-uno-f3e2b4

#include <avr/io.h>
#include <util/delay.h>

#include <Servo.h>

//TODO change pins to port numbers

//interrupts: arduino.cc/reference/en/language/functions/external-interrupts/attachinterrupt/
// comparator interrupt https://forum.arduino.cc/index.php?topic=149840.0, https://www.youtube.com/watch?v=QdJrJXQaiy8, http://www.gammon.com.au/forum/?id=11916,

//TODO Oliver, Harris, and Krupa update to match pinouts

const int tempInPin =  PORTC2; //A2;
const int tempOutPin = PORTC0; //A0;
// highest temp corresponds to 358, lowest temp corresponds to 20

const int greenLedPin = PORTD7; //7
const int redLedPin = PORTB0; //8

//https://vetco.net/products/rain-sensor-for-arduino?gclid=Cj0KCQjww_f2BRC-ARIsAP3zarGmMg3D0nxuw0fdA1LHU2nxvRJa2JZ81LpUyxbbfHLFhP5XPMcqfJ0aAt3hEALw_wcB
// rain sensor can either return an analog value or digital signal

//the only interrupt pins on the uno are UNO pins 2 and 3 - https://arduino.stackexchange.com/questions/1784/how-many-interrupt-pins-can-an-uno-handle#:~:text=There%20are%20only%20two%20external,edges%2C%20or%20on%20low%20level.
// followed this tutorial to set up slide switch https://www.instructables.com/id/Slide-Switch-With-Arduino-Uno-R3/
const int rainSwitchPin = PORTD2; //2;
const int nightSwitchPin = PORTD4; //4, nothing connected to 4

const int rainPin = PORTC1; //A1;

const int windowPosPinBare = PORTB2; //10;
const int windowBlindsPinBare = PORTB1; //9;

const int windowPosPin = 10;
const int windowBlindsPin = 9;


const int lightPin = PORTC3; //A3;

const int lockPin = PORTB4; //12;

const int tempDiffLedPin = PORTD5; //5;


// END PIN DECLARATIONS

const int tolerance = 10; //TODO Oliver figure out the int equilavent of 1deg Fahrenheit

const int WINDOW_CLOSED = 0;
const int WINDOW_OPEN = 180;
const int WINDOW_NIGHT = 0;

const int NIGHT_LIGHT_LEVEL = 300;
const int BRIGHT_LIGHT_LEVEL = 600;

const int BLINDS_CLOSED = 0;
const int BLINDS_OPEN = 180;

const int ANALOG_MIN = 0;
const int ANALOG_MAX = 255;

Servo windowPosServo;
Servo windowBlindsServo;

int lastSetWindowAngle;
int lastSetBlindsAngle;

//suggested to declare as volatile for use with interrupt
volatile bool isRaining = false;

#define MS_DELAY 3000

#define B 0
#define C 1
#define D 2

int main(void)
{
  //DIGITAL OUTPUT SETUP
  /*Set to one the fifth bit of DDRB to one
    **Set digital pin 13 to output mode */
  //DDRB |= _BV(DDB5);

  // //digital INPUT  SETUPbuttonPin setup - see https://arduino.stackexchange.com/questions/75927/i-am-trying-to-read-input-from-5th-pin-of-port-b
  // //Set digital pin 7 to input mode
  // DDRD &= ~_BV(DDD7);
  // // disable internal pull up
  // PORTD &= ~_BV(DDD7);

  analogReadSetup();
  setUpAnalogWrite();

  // TOOD finish setup, see setup function below
  bool nightMode = false;
  int insideReading = 0;
  int outsideReading = 0;
  int lightReading = 0;


  //configure rainSwitchPin (on port D, PD2) as digital input pin
  DDRD &= ~_BV(DDD2);
  PORTD &= ~_BV(DDD2);

  //configure nightSwitchPin (on port D, PD4) as digital input pin
  DDRD &= ~_BV(DDD4);
  PORTD &= ~_BV(DDD4);

  windowPosServo.attach(windowPosPin);
  windowBlindsServo.attach(windowBlindsPin);

  attachInterrupt(digitalPinToInterrupt(rainSwitchPin), changeRainingStatus, CHANGE);

  while (1)
  {
    //Sample code for digitalWrite
    // /*Set to one the fifth bit of PORTB to one
    // **Set to HIGH the pin 13 */
    // PORTB |= _BV(PORTB5);

    // /*Wait 3000 ms */
    // _delay_ms(MS_DELAY);

    // /*Set to zero the fifth bit of PORTB
    // **Set to LOW the pin 13 */
    // PORTB &= ~_BV(PORTB5);

    // /*Wait 3000 ms */
    // _delay_ms(MS_DELAY);

    nightMode = readDigital(D, PORTD4);
    //insideReading = readAnalog(tempInPin); //commenting out to force temp diff because analogRead breaks after the first run 
    outsideReading = readAnalog(tempOutPin);
    lightReading = readAnalog(lightPin);

    setWindowPosition(insideReading, outsideReading, lightReading, nightMode);
  }
}

/**
void setup()
{
  pinMode(tempInPin, INPUT);
  pinMode(tempOutPin, INPUT);
  
  pinMode(greenLedPin, OUTPUT);
  pinMode(redLedPin, OUTPUT);
  
  pinMode(rainPin, INPUT);
  
  //pinMode(windowPosPin, OUTPUT);
  //pinMode(windowBlindsPin, OUTPUT);
  
  windowPosServo.attach(windowPosPin);
  windowBlindsServo.attach(windowBlindsPin);
  
  pinMode(lightPin, INPUT);
  
  pinMode(lockPin, OUTPUT);
  
  pinMode(rainSwitchPin, INPUT);
  
  pinMode(nightSwitchPin, INPUT);
  
  attachInterrupt(digitalPinToInterrupt(rainSwitchPin), changeRainingStatus, CHANGE);
  
  
  Serial.begin(9600);
  
  //windowPosServo.write(0);
  //windowBlindsServo.write(180);
  
  setWindowAngle(WINDOW_CLOSED);  
}

void loop()
{
 //testAllComponents();
 bool nightMode = readDigital(nightSwitchPin);
  int insideReading = readAnalog("inside: ", tempInPin);
  int outsideReading = readAnalog("outside: ", tempOutPin);
  int lightReading = readAnalog("light: ", lightPin);
  //TODO get other readings, change setWindowPosition to use them
  setWindowPosition(insideReading, outsideReading, lightReading, nightMode);
  
}
*/

//interrupt that toggles the isRaining variable
void changeRainingStatus()
{
  isRaining = !isRaining;
}

/**Don't need to reimplement this, it was just for testing
void testAllComponents(){
  turnOnOffLed(redLedPin);
  turnOnOffLed(greenLedPin);
  turnOnOffLed(lockPin);

  readDigital(nightSwitchPin);

  readAnalog("temp in: ", tempInPin);
  readAnalog("temp out: ", tempOutPin);
  readAnalog("light: ", lightPin);
  readAnalog("rain: ", rainPin);
  
  //windowPosServo.write(0);
  //delay(100);
  //windowPosServo.write(90);
  //delay(100);
  //windowPosServo.write(180);
  //delay(100);
  
  //windowBlindsServo.write(0);
  //delay(100);
  //windowBlindsServo.write(90);
  //delay(100);
  //windowBlindsServo.write(180);
  //delay(100);
  
  //testServo(windowPosServo);
  //testServo(windowBlindsServo);
  
  int angle = 0;
  for(angle = 0; angle <= 180; angle+=1){
    windowPosServo.write(angle);
    delay(15);
    //Serial.println("set servo to " + String(angle));
  } 
    for(angle = 180; angle >= 0; angle-=1){
    windowPosServo.write(angle);
    delay(15);
    //Serial.println("set servo to " + String(angle));
  } 
  for(angle = 0; angle <= 180; angle+=1){
    windowBlindsServo.write(angle);
    delay(15);
    //Serial.println("set servo to " + String(angle));
  }
  for(angle = 180; angle >= 0; angle-=1){
    windowBlindsServo.write(angle);
    delay(15);
    //Serial.println("set servo to " + String(angle));
  }
  
}

void turnOnOffLed(int pin){
  digitalWrite(pin, HIGH);
  delay(100);
  digitalWrite(pin, LOW);
  delay(100);
}*/

// int readAnalog(String desc, int pin){
//   int reading = analogRead(pin);
//   delay(100);
//   Serial.println(desc + reading);
//   delay(100);
//   return reading;
// }

void analogReadSetup()
{
  //ADC setup
  //datasheet 24.9.1
  //REFS1, REFS0 = 0, 1 means AVcc with external capacitor at AREF bit
  //REFS1 = 0
  ADMUX &= ~_BV(REFS1);
  //REFS0 = 1
  ADMUX |= _BV(REFS0);

  //left adjust ADC reading ADMUX(ADLAR) = 1
  //right adjust ADMUX(ADLAR) = 0
  ADMUX &= ~_BV(ADLAR);

  //enable ADC by making ADEN bit = 1
  ADCSRA |= _BV(ADEN);

  //enable ADIE bit in ADCSRA and I-bit in SREG so we can get ADC interrupts
  ADCSRA |= _BV(ADIE);
  //SREG |= _BV(I);
}

uint16_t readAnalog(int pin)
{
  uint8_t channel_num = 0;
  switch (pin)
  {
  case PORTC0:
    channel_num = 0;
    break;
  case PORTC1:
    channel_num = 1;
    break;
  case PORTC2:
    channel_num = 2;
    break;
  case PORTC3:
    channel_num = 3;
    break;
  case PORTC4:
    channel_num = 4;
    break;
  case PORTC5:
    channel_num = 5;
    break;
  case PORTC6:
    channel_num = 6;
    break;
  default:
    channel_num = 0;
    break;
  }

  //Select channel to read
  ADMUX |= channel_num;
  //write logical zero to Power Reduction ADC bit, PRADC
  PRR |= ~_BV(PRADC);
  //write logical one to the ADC Start Conversion bit ADSC
  ADCSRA |= _BV(ADSC);

  //wait for conversion to complete
  // ADIF = 1 when complete
  uint8_t complete_mask = 0 | _BV(ADIF);

  // ADCSRA & complete_mask is 0 while we are waiting, 1 in ADIF place when complete
  while (ADCSRA & complete_mask == 0)
  {
    //do nothing, wait
  }

  //read upper and lower bytes, must read ADCL first for 10 bit result
  uint8_t adc_low = ADCL;
  uint8_t adc_high = ADCH;
  
  _delay_ms(100); //delay to space out readings
  //write logical zero to the ADC Start Conversion bit ADSC
  ADCSRA &= ~_BV(ADSC);
  uint16_t reading = (adc_high << 8) | adc_low;
  _delay_ms(100); //delay to space out readings
  return reading;
}

bool readDigital(int port, int pin)
{
  //  int reading = digitalRead(pin);
  //   delay(100);
  //   if(reading == HIGH){
  //     return true;
  //   }
  //   else{
  //     return false;
  //   }
  int buttonState = 0;
  switch (port)
  {
  case 0: //PORTB
    buttonState = PINB & _BV(pin);
    break;
  case 1: //PORTB
    buttonState = PIND & _BV(pin);
    break;
  default:
    //Error in readDigital port, turn on digital pin 13 (built in LED)
    setDigital(B, PORTB5, true);
  }

  // masked buttonState = 0, means value of pin bit is 0
  if (buttonState == 0)
  {
    return false;
  }

  return true;
}

void setDigital(int port, int pin, bool value)
{
  //TODO implement in bare metal c
  switch (port)
  {
  case B:
    if (value == true)
    {
      // digitalWrite(pin, HIGH);
      PORTB |= _BV(pin);
    }
    else
    {
      //digitalWrite(pin, LOW);
      PORTB &= ~_BV(pin);
    }
    break;
  case D:
    if (value == true)
    {
      // digitalWrite(pin, HIGH);
      PORTD |= _BV(pin);
    }
    else
    {
      //digitalWrite(pin, LOW);
      PORTD &= ~_BV(pin);
    }
    break;
    //TODO fill in
  }

  _delay_ms(500);
}

void setUpAnalogWrite(){
  //DDRB |= _BV(DDB2);
  
  // Fast PWM, clear OC1B on compare match, set at BOTTOM (non-inverting)
  //TCCR1B |= _BV(COM1B1); //COM1B1 = 1
  //TCCR1B &= ~_BV(COM1B0); //COM1B0 = 0
  
  //TCCR1B |= 1 << COM0B0;
  
  //TCCR1B |= (1 << WGM00);
 // TCCR1B |= (1 << WGM01);
  
 // TCCR1A |= 1 << CS00;
  
 
  DDRD |= 1 << PORTD5;
  
  TCCR0A |= 1 << COM0B0;
  TCCR0A |= 1 << WGM00;
  TCCR0A |= 1 << WGM01;
  
  TCCR0B |= 1 << CS00;
  
  
  // Parts inspired by/copied from  Servo.cpp Arduino library
  
//    TCCR1A = 0;             // normal counting mode
//    TCCR1B = _BV(CS11);     // set prescaler of 8
//    TCNT1 = 0;              // clear the timer count
  
  //TIFR1 |= _BV(OCF1A);     // clear any pending interrupts;
  //  TIMSK1 |=  _BV(OCIE1A) ; // enable the output compare interrupt
  
  
  
}

void setAnalog(int value)
{
  //Hardcode for digital pin 5 because of timer set up
  OCR0B = value;
    _delay_ms(30);
}

void setWindowPosition(int insideReading, int outsideReading, int lightReading, bool nightMode)
{
  //isRaining is determined by interrupt, close the window all the way if it is raining
  if (isRaining)
  {
    setWindowAngle(WINDOW_CLOSED);
    setAnalog(0);
  }
  else if (nightMode && lightReading < NIGHT_LIGHT_LEVEL)
  {
    // light level indicated night time, so set window to night time position and lock
    setWindowAngle(WINDOW_NIGHT);
    if (WINDOW_NIGHT == WINDOW_CLOSED)
    {
      lockWindow();
      setAnalog(255);
    }
  }
  else if (lightReading > BRIGHT_LIGHT_LEVEL)
  {
    int angle = 165; //TODO figure out math
    setBlindsAngle(angle);
    setAnalog(0);
  }
  else if (insideReading > (outsideReading - tolerance))
  {
    //open window
    //(358-120)/180 = 1.82
    // 358/2 = 179
    // 20/2 = 10

    //TODO mess with this scaling
    int angle = (insideReading - outsideReading) / 2;

    // open the window
    setWindowAngle(angle);

    setAnalog(angle);

    //close the blinds
    //TODO adjust to make sure that it works even in blinds open and angle are different scales
    //setBlindsAngle(BLINDS_OPEN - angle);
  }
  else if (outsideReading > (insideReading - tolerance))
  {
    //close window because it is more than tolerance hotter outside than inside

    int angle = WINDOW_CLOSED;

    setWindowAngle(angle);

    setAnalog(angle);

    //close the blinds all the way
    //setBlindsAngle(BLINDS_CLOSED);
  }
}

void setWindowAngle(int angle)
{

  /**if (angle == lastSetWindowAngle)
  {
    indicateSuccess;
  }
  else*/
  if (angle >= WINDOW_CLOSED && angle <= WINDOW_OPEN)
  {
    unlockWindow();
    indicateError();
    windowPosServo.write(angle);
    //setAnalog(windowPosPin, angle); //Discovered we can't analogWrite to a Servo, so decided to use Servo library
    _delay_ms(50);
    lastSetWindowAngle = angle;

    while (lastSetWindowAngle != angle)
    {
      _delay_ms(10);
    }

    indicateSuccess();
  }
  else
  {
    Serial.println("Invalid Window Position");
    indicateError();
  }
}

void setBlindsAngle(int angle)
{
  if (angle >= BLINDS_CLOSED && angle <= BLINDS_OPEN && angle != lastSetBlindsAngle)
  {
    windowBlindsServo.write(angle);

    _delay_ms(50);
    lastSetBlindsAngle = angle;

    while (lastSetBlindsAngle != angle)
    {
      _delay_ms(10);
    }
  }
  else
  {
    Serial.println("Invalid Blinds Position");
  }
}

//TODO Oliver functions after this comment - change to use bare metal wrapper and bare metal delay
void indicateError()
{
  //TODO Oliver Update when pins change
  setDigital(D, greenLedPin, false);
  _delay_ms(50);
  setDigital(B, redLedPin, true);
  _delay_ms(50);
}

void indicateSuccess()
{
  //TODO Oliver Update when pins change
  setDigital(B, redLedPin, false);
  _delay_ms(50);
  setDigital(D, greenLedPin, true);
  _delay_ms(50);
}

void lockWindow()
{
  setDigital(B, lockPin, true);
  _delay_ms(50);
}

void unlockWindow()
{
  setDigital(B, lockPin, false);
  _delay_ms(50);
}
