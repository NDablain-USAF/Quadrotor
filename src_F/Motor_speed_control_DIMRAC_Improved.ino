volatile byte counter[2] = {0,0}; // Make it volatile so that it can be accessed anywhere and stored on RAM
const byte ENC_PINS[2] = {2,3}; // Pins with interrupts, assume 3 is on channel A and 2 is on channel B
const byte PWM_PIN = 5; // Pin for PWM output
const byte BUTTON_PINS[2] = {11,12}; // Pins connected to pushbuttons
const byte BUTTON_POWER_PIN = 13; // Pin for powering buttons

void setup() {
  Serial.begin(9600); // Serial communication
  pinMode(BUTTON_PINS[0],INPUT_PULLUP);
  pinMode(BUTTON_PINS[1],INPUT_PULLUP);
  pinMode(BUTTON_POWER_PIN,OUTPUT);
  pinMode(PWM_PIN,OUTPUT); // Setup PWM signal pin
  pinMode(ENC_PINS[0],INPUT_PULLUP); // Setup encoder pins
  pinMode(ENC_PINS[1],INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_PINS[0]),revs1,RISING); // Set interrupt on pins 2&3, detect falling and rising edges which when turning CW should occur closest together 
  attachInterrupt(digitalPinToInterrupt(ENC_PINS[1]),revs2,FALLING);
}

void loop() {
  static unsigned long t[4];

  static int check;
  static int reset;
  checkstop(&check);
  begin(t, &check, &reset);

  static double wm;
  static int wd;
  int interval = 1e3;
  if ((micros()-t[1])>=interval){
    measure(t, &wm, &reset);
  }

  performance(t, &wm, &wd, &reset);
  
  DIMRAC(t, &wm, &wd, &reset);
}

void begin(unsigned long t[], int* check, int* reset){
  while(*check==0){
    analogWrite(PWM_PIN,0);
    delay(500);
    if (digitalRead(BUTTON_PINS[1])==0){
      t[0] = micros();
      t[1] = micros();
      t[2] = millis();
      ++*reset;
      ++*check;
    }
  }
}

void checkstop(int* check){
  if (digitalRead(BUTTON_PINS[0])==0){
    *check = 0;
  }
}

void measure(unsigned long t[], double* wm, int* reset){
  if (*reset>0){
    counter[0] = 0;
    counter[1] = 0;
    *wm = 0;
  }
  else{
    noInterrupts();
    double count = counter[0]+counter[1];
    counter[0] = 0;
    counter[1] = 0;
    interrupts();
    double dt = micros()-t[1];
    *wm = (count/(17*2))*(1e6/dt)*2*PI; // Find the rpm, 17 pulses per revolution, 60000 ms per minute, dt ms per interval. 
    t[1] = micros(); // Reset for future interval 
  }
}

void performance(unsigned long t[], double *wm, int *wd, int* reset){
  static byte index[4];
  static int window[25]; // Establish an array "window" of size n
  static int largest;
  static int ts;
  static int tr;
  static double MP;
  int r[2] = {1200,950}; // Initial setpoint from rest, target step speed (rad/s)

  if (reset>0){
    largest = 0;
    ts = 0;
    tr = 0;
    MP = 0;
    for (byte i=0;i<4;i++){
      index[i] = 0;
    }
    for (byte i=0;i<25;i++){
      window[i] = 0;
    }
  }

  *wd = r[index[0]]; // Desired motor speed (rad/s)
  byte n = sizeof(window)/sizeof(window[0]);
  window[index[2]] = *wm;
  index[2]++;
  if (index[2]==n){
    index[2] = 0;
  }
  long sum;
  for (byte i = 0;i<n;i++){
    sum += window[i];
  }
  double m_avg = sum/n; // Moving average calculation
  double ess = abs(m_avg-*wd); // Steady state error calculation
  if (*wm>largest){
    largest = *wm;
  }
  if ((abs(*wm-*wd)<10)&&(index[1]==0)){
    tr = millis()-t[3];
    index[1]++;
  }

  if ((abs(m_avg-*wd)<1.5)&&(index[3]<2)){
    ts = millis()-t[2]; // Settling time calculation
    MP = ((largest-*wd)/(*wd))*100; // Percent overshoot calculation
    t[2] = millis();
    index[3]++;
    if (index[0]<1){
      index[0]++;
      t[3] = millis();
      index[1] = 0;
    }
  }
  /*
  Serial.print("Steady State Error (rad/s): ");
  Serial.print(ess);
  Serial.print("  ");
  Serial.print("Rise Time (ms): ");
  Serial.print(tr);
  Serial.print("  ");
  Serial.print("Settling Time (ms): ");
  Serial.print(ts);
  Serial.print("  ");
  Serial.print("Percent Overshoot (%): ");
  Serial.print(MP);
  Serial.print("  ");
  Serial.print("Motor Speed (rad/s): ");
  Serial.println(*wm);
  */
}

void DIMRAC(unsigned long t[], double *wm, int *wd, int* reset){
  static double wref;
  static double ahat;
  static double bhat; 
  static double u;
  
  if (*reset>0){
    wref = 0;
    ahat = 0;
    bhat = 0;
    u = 0;
    *reset = 0;
  }
  
  double gama = 0.00003;
  double gamb = 0.00003;
  double bmin = 1;
  double aref = -7.0;
  double bref = 7.0;
  double bhatdot;

  if (bhat==0){
    bhat = 5;
  }

  double dt = (micros()-t[0])/1e6;
  t[0] = micros();
  double wrefdot = (aref*wref)+(bref*(*wd));
  wref += (wrefdot*dt);
  double e = *wm-wref;
  double ahatdot = gama*(*wm)*e;
  if ((abs(bhat)>bmin)||((bhat==bmin)&&(u*e>0))){
    bhatdot = gamb*u*e;
  }
  else{
    bhatdot = 0;
  }
  ahat += (ahatdot*dt);
  bhat += (bhatdot*dt);
  u = ((1/bhat)*((aref-ahat)*(*wm) + bref*(*wd)));
  int input = constrain(u,0,255); // u constrained to range of 0-255
  analogWrite(PWM_PIN,input);
  Serial.print(*wm);
  Serial.print(" ,");
  Serial.println(*wd);
}

void revs1(){
  ++counter[0];
}

void revs2(){
  ++counter[1];
}
