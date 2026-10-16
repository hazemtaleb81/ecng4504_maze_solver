/*
 ECNG 4504 - LINE FOLLOWING + RECOVERY
 Arduino Mega 2560 + L298N
 Five IR sensors

 BLACK = LOW
 WHITE = HIGH

 Right priority at intersections.
 Slow full-circle lost-line search.
*/

// MOTOR PINS
#define ENA 5
#define IN1 22
#define IN2 23
#define IN3 24
#define IN4 25
#define ENB 6

const byte sensor[5] = {30,31,32,33,34};

// WORKING SPEED SETTINGS
const int SPEED = 75;
const int TURN_SPEED = 75;
const int SCAN_SPEED = 75;

// SLOWER RECOVERY
const int SEARCH_SPEED = 65;

// ORIGINAL TURN SETTINGS
const unsigned long NODE_TIME = 130;
const unsigned long MIN_TURN = 180;
const unsigned long MAX_TURN = 2200;
const unsigned long COOLDOWN = 300;

// NEW RECOVERY SETTINGS
const unsigned long LOST_CONFIRM_MS = 120;
const unsigned long FULL_SEARCH_MS = 6000;
const unsigned long ALIGN_MS = 30;
const unsigned long TURN_PROTECTION_MS = 350;

// INTERSECTION FILTER
const unsigned long NODE_CONFIRM_MS = 15;

// LINE FOLLOWING
const float KP = 18.0;

// STATE
bool nodeArmed = false;
bool searching = false;
bool recoveryFailed = false;
bool hasCompletedTurn = false;

unsigned long lastNode = 0;
unsigned long lastTurnFinished = 0;
unsigned long lostSince = 0;
unsigned long searchStart = 0;
unsigned long alignedSince = 0;

// -1 LEFT, +1 RIGHT
int lastDirection = 1;

// MOTOR CONTROL
void drive(int left, int right) {

  digitalWrite(IN1, left >= 0 ? HIGH : LOW);
  digitalWrite(IN2, left >= 0 ? LOW : HIGH);

  digitalWrite(IN3, right >= 0 ? HIGH : LOW);
  digitalWrite(IN4, right < 0 ? HIGH : LOW);

  analogWrite(ENA, constrain(abs(left),0,255));
  analogWrite(ENB, constrain(abs(right),0,255));
}

void stopCar() {
  analogWrite(ENA,0);
  analogWrite(ENB,0);
}

// SENSOR READING
// Internal bit 1 = BLACK
byte readLine() {

  byte s = 0;

  for(int i=0; i<5; i++) {
    if(digitalRead(sensor[i]) == LOW)
      s |= (1 << i);
  }

  return s;
}

bool leftBranch(byte s) {
  return (s & 0b00011) == 0b00011;
}

bool rightBranch(byte s) {
  return (s & 0b11000) == 0b11000;
}

// ORIGINAL 90 DEGREE TURN
bool turn90(int dir) {

  stopCar();
  delay(60);

  drive(dir * TURN_SPEED,
       -dir * TURN_SPEED);

  unsigned long start = millis();
  bool leftOldLine = false;
  unsigned long stableStart = 0;

  while(millis() - start < MAX_TURN) {

    byte s = readLine();
    bool centerBlack = s & 0b00100;

    if(!centerBlack) {
      leftOldLine = true;
      stableStart = 0;
    }

    if(leftOldLine &&
       millis() - start > MIN_TURN &&
       centerBlack) {

      if(stableStart == 0)
        stableStart = millis();

      if(millis() - stableStart > 30) {

        stopCar();
        delay(70);

        lastTurnFinished = millis();
        hasCompletedTurn = true;

        return true;
      }
    }
    else if(!centerBlack) {
      stableStart = 0;
    }
  }

  stopCar();
  Serial.println("TURN FAILED");
  return false;
}

// SLOW FULL-CIRCLE SEARCH
void recoverLine() {

  if(!searching) {

    searching = true;
    searchStart = millis();
    alignedSince = 0;
    nodeArmed = false;

    stopCar();

    Serial.println("LINE LOST - SEARCHING");
  }

  byte s = readLine();

  bool centerBlack = (s & 0b00100) != 0;

  // Require stable center detection
  if(centerBlack) {

    if(alignedSince == 0)
      alignedSince = millis();

    // Pause to prevent overshooting
    stopCar();

    if(millis() - alignedSince >= ALIGN_MS) {

      searching = false;
      alignedSince = 0;
      lostSince = 0;
      lastNode = millis();

      Serial.println("LINE RECOVERED");
      return;
    }

  } else {

    alignedSince = 0;

    // Rotate in one direction only
    drive(
      lastDirection * SEARCH_SPEED,
      -lastDirection * SEARCH_SPEED
    );
  }

  // Stop if full search time expires
  if(millis() - searchStart >= FULL_SEARCH_MS) {

    stopCar();
    searching = false;
    recoveryFailed = true;

    Serial.println("FULL SEARCH FAILED");
  }
}

void setup() {

  Serial.begin(115200);

  pinMode(ENA,OUTPUT);
  pinMode(ENB,OUTPUT);

  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  pinMode(IN3,OUTPUT);
  pinMode(IN4,OUTPUT);

  for(int i=0; i<5; i++)
    pinMode(sensor[i],INPUT);

  stopCar();

  Serial.println("ROBOT READY");
  delay(3000);

  lastNode = millis();
}

void loop() {

  if(recoveryFailed) {
    stopCar();
    return;
  }

  byte s = readLine();

  // CONTINUE SEARCH WITHOUT INTERRUPTION
  if(searching) {
    recoverLine();
    return;
  }

  // CONFIRM LOST LINE
  if(s == 0) {

    stopCar();

    // Protect against false recovery
    // immediately following a turn.
    if(hasCompletedTurn &&
       millis() - lastTurnFinished <
       TURN_PROTECTION_MS) {

      lostSince = 0;
      return;
    }

    if(lostSince == 0)
      lostSince = millis();

    if(millis() - lostSince >= LOST_CONFIRM_MS) {
      recoverLine();
    }

    return;
  }

  // Black detected: cancel lost timer
  lostSince = 0;

  // ARM INTERSECTION DETECTION
  if(s == 0b00100 ||
     s == 0b00110 ||
     s == 0b01100 ||
     s == 0b01110) {

    nodeArmed = true;
  }

  // INTERSECTION DETECTION
  if(nodeArmed &&
     millis() - lastNode > COOLDOWN &&
     (leftBranch(s) || rightBranch(s))) {

    // Confirm branch instead of reacting
    // immediately to one noisy reading.
    bool firstL = leftBranch(s);
    bool firstR = rightBranch(s);

    delay(NODE_CONFIRM_MS);

    byte check = readLine();

    bool L = firstL && leftBranch(check);
    bool R = firstR && rightBranch(check);

    if(L || R) {

      drive(SCAN_SPEED,SCAN_SPEED);

      unsigned long start = millis();

      while(millis() - start < NODE_TIME) {

        byte x = readLine();

        if(leftBranch(x)) L = true;
        if(rightBranch(x)) R = true;
      }

      stopCar();

      int direction = 0;

      // RIGHT PRIORITY
      if(R)
        direction = 1;
      else if(L)
        direction = -1;

      if(direction != 0) {

        lastDirection = direction;

        if(!turn90(direction)) {
          recoveryFailed = true;
          stopCar();
          return;
        }
      }

      nodeArmed = false;
      lastNode = millis();
      return;
    }
  }

  // NORMAL LINE FOLLOWING
  int sum = 0;
  int count = 0;

  for(int i=0; i<5; i++) {

    if(s & (1 << i)) {
      sum += i - 2;
      count++;
    }
  }

  if(count > 0) {

    float error = (float)sum / count;

    if(error < -0.3)
      lastDirection = -1;

    else if(error > 0.3)
      lastDirection = 1;

    int correction = (int)(KP * error);

    int leftSpeed = SPEED + correction;
    int rightSpeed = SPEED - correction;

    // PRESERVE WORKING FOLLOWING SPEED
    drive(
      constrain(leftSpeed,0,70),
      constrain(rightSpeed,0,70)
    );
  }
}
