long dF, dL, dR;
enum MotorPinID {
    L_F = 0,
    L_B,
    R_F,
    R_B,
    NUM_OF_MOTOR_PIN
};
enum UltrasonicPinID {
    U_F = 0,
    U_L,
    U_R,
    NUM_OF_ULTRASONIC_PIN
};

static const uint8_t motorPins[NUM_OF_MOTOR_PIN] = {23, 22, 32, 33};  // L_F, L_B, R_F, R_B
static const uint8_t usTrigPins[NUM_OF_ULTRASONIC_PIN] = {19, 17, 21};  // F, L, R
static const uint8_t usEchoPins[NUM_OF_ULTRASONIC_PIN] = {18, 16 ,4};  // F, L, R

void initialPins()
{
  // Attach pins to the PWM controller.
    ledcAttachPin(motorPins[L_F], L_F);
    ledcAttachPin(motorPins[L_B], L_B);
    ledcAttachPin(motorPins[R_F], R_F);
    ledcAttachPin(motorPins[R_B], R_B);
    ledcSetup(L_F, 10000, 8);   // 10kHz, 8 bit resolution
    ledcSetup(L_B, 10000, 8);   // 10kHz, 8 bit resolution
    ledcSetup(R_F, 10000, 8); // 10kHz, 8 bit resolution
    ledcSetup(R_B, 10000, 8); // 10kHz, 8 bit resolution
    ledcWrite(L_F, 0);
    ledcWrite(L_B, 0);
    ledcWrite(R_F, 0);
    ledcWrite(R_B, 0);
}

long ultrasonicGetDistance(uint8_t trig, uint8_t echo)
{
    long duration;

    pinMode(trig, OUTPUT);
    digitalWrite(trig, LOW);
    delayMicroseconds(2);
    digitalWrite(trig, HIGH);
    delayMicroseconds(5);
    digitalWrite(trig, LOW);

    pinMode(echo, INPUT);
    duration = pulseIn(echo, HIGH, 5000000L);
    return duration / 29 / 2;
}

void reportUltrasonic()
{
    //long dF, dL, dR;

    dF = ultrasonicGetDistance(usTrigPins[U_F], usEchoPins[U_F]);
    dL = ultrasonicGetDistance(usTrigPins[U_L], usEchoPins[U_L]);
    dR = ultrasonicGetDistance(usTrigPins[U_R], usEchoPins[U_R]);

    Serial.printf("F:%d cm, L:%d cm, R:%d cm ", dF, dL, dR);
    Serial.println();
}

void forward(){
  ledcWrite(L_F, 200);
  ledcWrite(L_B, 0);
  ledcWrite(R_F, 200);
  ledcWrite(R_B, 0);
  }
void back(){
  ledcWrite(L_F, 0);
  ledcWrite(L_B, 200);
  ledcWrite(R_F, 0);
  ledcWrite(R_B, 200);
  }
void pause(){
  ledcWrite(L_F, 0);
  ledcWrite(L_B, 0);
  ledcWrite(R_F, 0);
  ledcWrite(R_B, 0);
  }
void right(){
  ledcWrite(L_F, 200);
  ledcWrite(L_B, 0);
  ledcWrite(R_F, 0);
  ledcWrite(R_B, 200);
  }
  
void left(){
  ledcWrite(L_F, 0);
  ledcWrite(L_B, 200);
  ledcWrite(R_F, 200);
  ledcWrite(R_B, 0);
  }


void setup() {
  // put your setup code here, to run once:
  initialPins();
  Serial.begin(115200);
  
 

}

void loop() {
  reportUltrasonic();
  forward();
  if(dR <= 10 && dR > 6 && dF > 8 ) {
    right();
    delay(50);
    Serial.printf("right a little");
  } else if ((dR <= 5 || dR >300) && dF > 8) {
    left();
    delay(50);
  }
  
  if(dR > 15) {
    forward();
    delay(800);
    right();
    Serial.print("right turn");
    delay(450);
    forward();
    delay(350);
  } else {
    if(dF <=7 && dR <= 15){
      forward();
      delay(150);
      left();
      Serial.print("left turn");
      delay(300);
    
    } else {
      forward();
      delay(150);
    }
  }
  delay(150);


  
  
}
