/*
  Robotic Leg - 3 Joint Servo Control + Sensor Reading (Serial Monitor)
  Elegoo UNO R3 + L298N Motor Driver
*/

#include <Servo.h>

// ---------- Pin Definitions ----------
const int SERVO1_PIN = 9;
const int SERVO2_PIN = 10;
const int SERVO3_PIN = 11;

// L298N Channel A - vacuum pump motor
const int ENA = 5;
const int IN1 = 7;

// L298N Channel B - solenoid valve
const int ENB = 6;
const int IN3 = 8;

const int PUMP_SPEED = 255; 

// Sensor Pin
const int SENSOR_PIN = A0;

// ---------- Motion & Timing Parameters ----------
const int ANGLE_MIN = -90;        
const int ANGLE_MAX = 90;         
const int CENTER_OFFSET = 90;     

const float MANUAL_SPEED = 60.0;  
const float HOME_SPEED   = 90.0;  

const unsigned long UPDATE_INTERVAL_MS = 15;   
const unsigned long SENSOR_INTERVAL_MS = 200;  

// ---------- Servo objects ----------
Servo servo1, servo2, servo3;

// ---------- Per-joint runtime state ----------
struct Joint {
  Servo* servo;
  float angle;       
  bool moveLeft;      
  bool moveRight;     
  bool homing;        
};

Joint joints[3];
unsigned long lastUpdate = 0;
unsigned long lastSensorUpdate = 0;

// ---------- Vacuum State ----------
bool vacuumActive = false;
bool vacuumReleasing = false;
unsigned long vacuumReleaseTime = 0;

void setup() {
  Serial.begin(115200); 

  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);
  servo3.attach(SERVO3_PIN);

  // Initialize L298N pins
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);

  joints[0] = { &servo1, 0, false, false, false };
  joints[1] = { &servo2, 0, false, false, false };
  joints[2] = { &servo3, 0, false, false, false };

  for (int i = 0; i < 3; i++) {
    writeJointAngle(i);
  }
  
  // Ensure pump and valve are off at startup
  digitalWrite(IN1, LOW); analogWrite(ENA, 0);
  digitalWrite(IN3, LOW); analogWrite(ENB, 0);

  Serial.println(F("Ready."));
}

void loop() {
  handleSerial();

  unsigned long now = millis();
  
  // -- Servo Control Loop --
  if (now - lastUpdate >= UPDATE_INTERVAL_MS) {
    float dt = (now - lastUpdate) / 1000.0;
    lastUpdate = now;
    for (int i = 0; i < 3; i++) {
      updateJoint(i, dt);
    }
  }

  // -- Vacuum Solenoid Timeout (0.5 sec release) --
  if (vacuumReleasing && (now - vacuumReleaseTime >= 500)) {
    // Turn off solenoid valve to save power/prevent overheat
    digitalWrite(IN3, LOW);
    analogWrite(ENB, 0);
    vacuumReleasing = false;
    Serial.println(F("Valve closed after 0.5s release."));
  }

  // -- Sensor Read Loop --
  if (now - lastSensorUpdate >= SENSOR_INTERVAL_MS) {
    lastSensorUpdate = now;
    int adc = analogRead(SENSOR_PIN);
    float voltage = adc * 5.0 / 1023.0;
    Serial.print("ADC = ");
    Serial.print(adc);
    Serial.print("   Voltage = ");
    Serial.println(voltage, 3);
  }
}

void handleSerial() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n' || c == '\r' || c == ' ') continue; 
    applyKey(c);
  }
}

void applyKey(char key) {
  switch (key) {
    case 'q': toggleLeft(0);  break;
    case 'e': toggleRight(0); break;
    case 'w': goHome(0);      break;

    case 'a': toggleLeft(1);  break;
    case 'd': toggleRight(1); break;
    case 's': goHome(1);      break;

    case 'z': toggleLeft(2);  break;
    case 'c': toggleRight(2); break;
    case 'x': goHome(2);      break;

    case 'v': case 'V': setVacuum(true);  Serial.println(F("Vacuum ON (gripping).")); break;
    case 'b': case 'B': setVacuum(false); Serial.println(F("Vacuum OFF (releasing...).")); break;

    case '?': printStatus(); break;

    default: break; 
  }
}

void toggleLeft(int i) {
  Joint &j = joints[i];
  j.homing = false;
  j.moveRight = false;
  j.moveLeft = !j.moveLeft; 
}

void toggleRight(int i) {
  Joint &j = joints[i];
  j.homing = false;
  j.moveLeft = false;
  j.moveRight = !j.moveRight;
}

void goHome(int i) {
  Joint &j = joints[i];
  j.moveLeft = false;
  j.moveRight = false;
  j.homing = true;
}

void updateJoint(int i, float dt) {
  Joint &j = joints[i];
  bool changed = false;

  if (j.homing) {
    float step = HOME_SPEED * dt;
    if (fabs(j.angle) <= step) {
      j.angle = 0;
      j.homing = false;
    } else {
      j.angle += (j.angle > 0) ? -step : step;
    }
    changed = true;
  } else if (j.moveLeft) {
    j.angle -= MANUAL_SPEED * dt;
    changed = true;
  } else if (j.moveRight) {
    j.angle += MANUAL_SPEED * dt;
    changed = true;
  }

  if (changed) {
    float clamped = constrain(j.angle, ANGLE_MIN, ANGLE_MAX);
    if (clamped != j.angle) {
      j.angle = clamped;
      j.moveLeft = false;
      j.moveRight = false;
    }
    writeJointAngle(i);
  }
}

void setVacuum(bool on) {
  vacuumActive = on;
  if (on) {
    vacuumReleasing = false;
    // Pump ON, Valve CLOSED (gripping)
    digitalWrite(IN1, HIGH);
    analogWrite(ENA, PUMP_SPEED);
    digitalWrite(IN3, LOW);
    analogWrite(ENB, 0);
  } else {
    // Pump OFF, Valve OPEN (releasing)
    digitalWrite(IN1, LOW);
    analogWrite(ENA, 0);
    digitalWrite(IN3, HIGH);
    analogWrite(ENB, PUMP_SPEED);
    
    // Start the 0.5s timeout timer
    vacuumReleasing = true;
    vacuumReleaseTime = millis();
  }
}

void writeJointAngle(int i) {
  int pulse = (int)round(joints[i].angle) + CENTER_OFFSET;
  joints[i].servo->write(pulse);
}

void printStatus() {
  for (int i = 0; i < 3; i++) {
    Serial.print(F("Joint")); Serial.print(i + 1);
    Serial.print(F(": angle="));
    Serial.print(joints[i].angle);
    Serial.print(F(" moveLeft="));
    Serial.print(joints[i].moveLeft);
    Serial.print(F(" moveRight="));
    Serial.print(joints[i].moveRight);
    Serial.print(F(" homing="));
    Serial.println(joints[i].homing);
  }
  Serial.print(F("Vacuum: "));
  Serial.println(vacuumActive ? F("ON") : F("OFF"));
}
