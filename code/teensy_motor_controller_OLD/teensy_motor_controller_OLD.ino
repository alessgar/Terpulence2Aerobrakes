#define HWSERIAL Serial1

// Motor Constants
#define MOTOR_DIRECTION_PIN 5
#define MOTOR_STEP_PIN 6
#define MOTOR_ENABLE_PIN 4
#define MOTOR_FAULT_PIN 3
#define MOTOR_STEPS_PER_REVOLUTION 250
#define MOTOR_SPEED 400                 // microseconds between each step; 200 microseconds is fastest with accurate movement, going below will make it too fast
#define MOTOR_MAX_DEGREES 540.0        // Maximum actuation
#define MOTOR_MIN_DEGREES 0.0           // Minimum actuation
#define MOTOR_ANGLE_MOVEMENT 540.0     // How many degrees to move at a time max

float curActuation = 0.0f;              // Used to track active actuation state
float desiredActuation = 0.0f;          // Used to track desired actuation state

char receivedChars[64];
boolean newData = false;

void setup() {
  //Serial.begin(115200);
  //while (!Serial);
  
  HWSERIAL.begin(115200);
  while (!HWSERIAL);
  Serial1.setTX(0);
  Serial1.setRX(1);
  //Serial.println(F("Starting Actuation Test Program!"));

  // Setup Actuator Flaps
  // Declare pins as Outputs
  pinMode(MOTOR_STEP_PIN, OUTPUT);
  pinMode(MOTOR_DIRECTION_PIN, OUTPUT);
  pinMode(MOTOR_ENABLE_PIN, OUTPUT);
  digitalWrite(MOTOR_ENABLE_PIN, HIGH); //disable motor by default

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(500);
  digitalWrite(LED_BUILTIN, LOW);
  //Serial.println(F("Test Program Ready!"));
}

void loop() {
  recvWithEndMarker();
  if(newData){
    digitalWrite(LED_BUILTIN, HIGH);
    newData = false;
    desiredActuation = String(receivedChars).toFloat();
    Serial.println(desiredActuation);
    digitalWrite(LED_BUILTIN, LOW);
    //while (HWSERIAL.available() > 0) {
    //    HWSERIAL.read();
    //}
  }
  
  rotateFlaps();
}

void recvWithEndMarker() {
    static byte ndx = 0;
    char endMarker = '\n';
    char rc;
    
    if (HWSERIAL.available() > 0) {
        rc = HWSERIAL.read();

        if (rc != endMarker) {
            receivedChars[ndx] = rc;
            ndx++;
            if (ndx >= 64) {
                ndx = 63;
            }
        }
        else {
            receivedChars[ndx] = '\0'; // terminate the string
            ndx = 0;
            newData = true;
        }
    }
}

// Wrapper for motor functions, to prevent misuse and allow for gradual actuation and adjustment between sensor readings
void rotateFlaps() {
  // Figure out how much we WANT to actuate and in what direction
  float diff = curActuation - desiredActuation;
  bool isOpening = false;
  if (diff < 0.0f) {
    isOpening = true;
    diff *= -1;
  }

  // Cap how much we want to actuate to prevent delaying other code for too long
  float angle = diff;
  if (angle > MOTOR_ANGLE_MOVEMENT) {
    angle = MOTOR_ANGLE_MOVEMENT;
  }

  // Do nothing if actuation would violate bounds
  if (isOpening && curActuation + angle > MOTOR_MAX_DEGREES) {
    return;
  } else if (!isOpening && curActuation - angle < MOTOR_MIN_DEGREES) {
    return;
  }

  // If no issues found, and angle is substantial enough, actuate in the needed direction
  if (angle > 0.1f) {
    if (isOpening) {
      rotateClockwise(angle);
    } else {
      rotateCounterClockwise(angle);
    }
  }
}

// close the flaps
void rotateClockwise(float angle)
{
  float steps = (angle / 360) * MOTOR_STEPS_PER_REVOLUTION;
  // Set motor direction clockwise
  digitalWrite(MOTOR_DIRECTION_PIN, HIGH);
  // Enable the motor
  digitalWrite(MOTOR_ENABLE_PIN, LOW);

  for (int x = 0; x < steps; x++)
  {
    digitalWrite(MOTOR_STEP_PIN, HIGH);
    delayMicroseconds(MOTOR_SPEED); //don't go below 200!!!
    digitalWrite(MOTOR_STEP_PIN, LOW);
    delayMicroseconds(MOTOR_SPEED);
  }
  // Disable the motor
  digitalWrite(MOTOR_ENABLE_PIN, HIGH);
  curActuation += angle;
}

// Open the flaps
void rotateCounterClockwise(float angle)
{
  float steps = (angle / 360) * MOTOR_STEPS_PER_REVOLUTION;
  // Set motor direction counter-clockwise
  digitalWrite(MOTOR_DIRECTION_PIN, LOW);
  // Enable the motor
  digitalWrite(MOTOR_ENABLE_PIN, LOW);
  for (int x = 0; x < steps; x++)
  {
    digitalWrite(MOTOR_STEP_PIN, HIGH);
    delayMicroseconds(MOTOR_SPEED); //don't go below 200!!!
    digitalWrite(MOTOR_STEP_PIN, LOW);
    delayMicroseconds(MOTOR_SPEED);
  }
  // Disable the motor
  digitalWrite(MOTOR_ENABLE_PIN, HIGH);
  curActuation -= angle;
}
