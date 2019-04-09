#include <ros.h>
#include <std_msgs/String.h>

ros::NodeHandle nh;

std_msgs::String str_msg;
std_msgs::Float64 flame_reading;
ros::Publisher sound("sound", &str_msg);
ros::Publisher flame("flame", &flame_reading);
ros::Subscriber<std_msgs::Empty> sub("toggle_extinguisher", &messageCb );
char hello[5] = "sound";

boolean clipping = 0;
boolean doneWithSound = false;


//data storage variables
byte newData = 0;
byte prevData = 0;

//freq variables
unsigned int timer = 0;//counts period of wave
unsigned int period;
int frequency;

byte correct = 0;
byte wrong = 0;

const int bluePin = 5;
const int redPin = 6;

const int FLAME = A3;

const int FLAME_INROOM = 800;
const int FLAME_INFRONT = 40;

const int AIN1 = 5;
const int AIN2 = 6;

void messageCb(const std_msgs::Empty& toggle_msg){
  digitalWrite(6, HIGH-digitalRead(6));   // blink the led
}

void checkFlames() {
  int flame_detector = analogRead(FLAME);
  flame_detector = analogRead(FLAME);
  //Serial.println(flame_detector);
  if(flame_detector <= FLAME_INFRONT){
     analogWrite(13, 255); //LED On
     flame_reading = flame_detector;
     flame.publish(&flame_reading)

  }
}

void setup(){
  Serial.begin(19200);
  pinMode(6, OUTPUT);
  nh.initNode();
  nh.advertise(sound);
  nh.advertise(flame);
  nh.subscribe(sub);
  cli();//diable interrupts

  //set up continuous sampling of analog pin 0

  //clear ADCSRA and ADCSRB registers
  ADCSRA = 0;

  ADMUX |= (1 << REFS0); //set reference voltage
  ADMUX |= (1 << ADLAR); //left align the ADC value- so we can read highest 8 bits from ADCH register only

  ADCSRA |= (1 << ADPS2) | (1 << ADPS0); //set ADC clock with 32 prescaler- 16mHz/32=500kHz
  ADCSRA |= (1 << ADATE); //enabble auto trigger
  ADCSRA |= (1 << ADIE); //enable interrupts when measurement complete
  ADCSRA |= (1 << ADEN); //enable ADC
  ADCSRA |= (1 << ADSC); //start ADC measurements

  sei();//enable interrupts
  Serial.println("here");
  do {
    if (clipping){//if currently clipping
      PORTB &= B11011111;//turn off clippng indicator led
      clipping = 0;
    }
    frequency = 38462/period;//timer rate/period
    //print results
    Serial.print(frequency);
    Serial.println(" hz");

    if (3306 < frequency && 4294 > frequency) {
      correct++;
    } else {
      wrong++;
    }
    if (wrong + correct > 20) {
      //Serial.print(wrong);
      //Serial.print("");
      //Serial.println(correct);
      if (correct>wrong*3) {
        doneWithSound = true;
      } else {
        //digitalWrite(CORRLED, 0);
      }
      correct = 0;
      wrong = 0;
    }

    delay(50);
  } while(!doneWithSound);

  cli();//diable interrupts

  //set up continuous sampling of analog pin 0

  //clear ADCSRA and ADCSRB registers
  ADCSRA = 0b10000111;
  ADMUX = 0;

  sei();//enable interrupts

  analogReference(DEFAULT);

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);

  str_msg.data = hello;
  Serial.print("printing");
  sound.publish( &str_msg );
  nh.spinOnce();
  /*
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  delay(2000);
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  */
/*  while(true) {
    Serial.println(analogRead(A2));
    delay(50);
  }*/
}

ISR(ADC_vect) {//when new ADC value ready
  if (!doneWithSound) {
    prevData = newData;//store previous value
    newData = ADCH;//get value from A0
    if (prevData < 129 && newData >=129){//if increasing and crossing midpoint
      period = timer;//get period
      timer = 0;//reset timer
    }


    if (newData == 0 || newData == 1023){//if clipping
      PORTB |= B00100000;//set pin 13 high- turn on clipping indicator led
      clipping = 1;//currently clipping
    }

    timer++;//increment timer at rate of 38.5kHz
  }
}

void loop() {
  checkFlames();
  //flamePrintThread.check();
  nh.spinOnce();;
}
