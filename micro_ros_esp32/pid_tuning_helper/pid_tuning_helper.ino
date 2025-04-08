#include <ESP32Encoder.h>
#include <algorithm> // For std::sort
#include <stdio.h>

// Configuration - change these as needed
#define CONTROL_LEFT_MOTOR true  // Set to false to control right motor instead
constexpr int POTENTIOMETER_PIN = 34;  // Pin for speed control potentiometer

// Robot parameters
constexpr double MAX_MOTOR_SPEED = 500.0;  // Max speed in RPS
constexpr int MAX_MOTOR_DRIVER_DUTYCYCLE = 190; //255*(3/4) because we are running 4S Lipo
constexpr int ENCODER_TICKS_PER_REVOLUTION = 14 * 2;
constexpr double ENCODER_PT1_TIMECONSTANT_S = 0.1;

constexpr unsigned long UPDATE_INTERVAL_ENCODER = 10;  // In ms
constexpr unsigned long UPDATE_INTERVAL_PID_CONTROL = 10;  // In ms

// PID controller parameters - adjust these during tuning
double KP = 8.0;
double KI = 10.0;
double KD = 0.0;
double KI_MAX = 30.0;

// Pin definitions
constexpr int LEFT_ENCODER_C2_PIN = 18;
constexpr int LEFT_ENCODER_C1_PIN = 19;
constexpr int RIGHT_ENCODER_C1_PIN = 5;
constexpr int RIGHT_ENCODER_C2_PIN = 4;

constexpr int MOTORDRIVER_LEFT_CW_PIN = 32;
constexpr int MOTORDRIVER_LEFT_CCW_PIN = 16;
constexpr int MOTORDRIVER_LEFT_PWM_PIN = 17;

constexpr int MOTORDRIVER_RIGHT_CW_PIN = 25;
constexpr int MOTORDRIVER_RIGHT_CCW_PIN = 26;
constexpr int MOTORDRIVER_RIGHT_PWM_PIN = 27;

constexpr int MOTORDRIVER_STBY_PIN = 33;

// Encoder class
class Encoder {
public:
    Encoder(int pin1, int pin2, int ticksPerRevolution, float timeConstant, unsigned long updateInterval)
        : pin1(pin1), pin2(pin2), ticksPerRevolution(ticksPerRevolution), timeConstant(timeConstant), updateInterval(updateInterval*1000) {}
    
    void init() {
        pinMode(pin1, INPUT_PULLUP);
        pinMode(pin2, INPUT_PULLUP);

        // Enable the weak pull up resistors
        ESP32Encoder::useInternalWeakPullResistors = puType::up;

        encoder.attachHalfQuad(pin1, pin2);
        encoder.setCount(0);
        
        lastUpdateTime = micros();
    }

    void update() {
        unsigned long currentTime = micros();

        unsigned long deltaTime = currentTime - lastUpdateTime;
        
        if (updateInterval == 0 || deltaTime >= updateInterval) {
            
            long currentCount = encoder.getCount();
            
            // Calculate raw speed
            
            if (deltaTime > 0) {  // Prevent division by zero
                double rawRps = (currentCount - lastCount) / (ticksPerRevolution * (deltaTime / 1000000.0));
                
                // Apply PT1 filter if time constant is positive
                if (timeConstant > 0.0) {
                    double alpha = deltaTime / (timeConstant * 1000000.0 + deltaTime);
                    filteredRps = alpha * rawRps + (1.0 - alpha) * filteredRps;
                } else {
                    // No filtering if time constant is zero or negative
                    filteredRps = rawRps;
                }
            }
            
            lastCount = currentCount;
            lastUpdateTime = currentTime;
        }
    }

    void resetCount() {
        encoder.setCount(0);
        lastCount = 0;
    }

    double getSpeed() const {
        return filteredRps;
    }

    double getAngularPosition() const {
        return (lastCount / (double)ticksPerRevolution);
    }

    void setTimeConstant(float tc) {
        timeConstant = tc;
    }

    void setUpdateInterval(unsigned long interval) {
        updateInterval = interval * 1000;
    }

private:
    ESP32Encoder encoder;
    int pin1, pin2;
    int ticksPerRevolution;
    long lastCount = 0;
    unsigned long lastUpdateTime = 0;
    unsigned long updateInterval;
    double filteredRps = 0.0;
    double timeConstant;
};

// MotorDriver class
class MotorDriver {
public:
    MotorDriver(int pin1, int pin2, int pin3, const int maxDutyCycle, int channel, int freq = 5000, int resolution = 8) : 
        pin1(pin1), pin2(pin2), pin3(pin3), 
        maxDutyCycle(maxDutyCycle),
        channel(channel),
        freq(freq),
        resolution(resolution) {}

    void init() {
        pinMode(pin1, OUTPUT);
        pinMode(pin2, OUTPUT);
        
        // Configure LEDC channel
        ledcSetup(channel, freq, resolution);
        ledcAttachPin(pin3, channel);
    }

    void update() {
        if (dutyCycle > 0) {
            digitalWrite(pin2, LOW);
            digitalWrite(pin1, HIGH);
            ledcWrite(channel, abs(dutyCycle));
        } else if (dutyCycle < 0) {
            digitalWrite(pin1, LOW);
            digitalWrite(pin2, HIGH);
            ledcWrite(channel, abs(dutyCycle));
        } else {
            ledcWrite(channel, 0);
        }
    }

    void stop() {
        digitalWrite(pin1, LOW);
        digitalWrite(pin2, LOW);
        ledcWrite(channel, 0);
    }

    void setMotorDutyCycle(int dutyCycle) {
        this->dutyCycle = (int)constrain(dutyCycle, -maxDutyCycle, maxDutyCycle);
    }

    int getMotorDutyCycle() const { return dutyCycle; }

private:
    int pin1, pin2, pin3;
    int dutyCycle = 0;
    int maxDutyCycle;
    int channel;
    int freq;
    int resolution;
};

// PIDController class
class PIDController {
public:
    PIDController(double Kp, double Ki, double Kd, double KiMax) 
        : Kp(Kp), Ki(Ki), Kd(Kd), KiMax(KiMax), error(0), integral(0), derivative(0), previousError(0) {
          lastTime = micros();
          }

    double compute(double desiredValue, double measuredValue) {
        double deltaTime = (micros() - lastTime) / 1000000.0;
        error = desiredValue - measuredValue;
        integral = integral + error*deltaTime;
        //integral = constrain(integral, -KiMax, KiMax);  // Anti-windup
        derivative = (error - previousError)/deltaTime;
        previousError = error;
        lastTime = micros();

        return (Kp * error + Ki * integral + Kd * derivative);
    }

    void pause(){
      lastTime = micros();
    }

    void pidReset() {
        error = 0.0;
        integral = 0.0;
        derivative = 0.0;
        previousError = 0.0;
        lastTime = 0;
    }

    void setPidValues(double Kp, double Ki, double Kd, double KiMax) {
        this->Kp = Kp;
        this->Ki = Ki;
        this->Kd = Kd;
        this->KiMax = KiMax;
        pidReset();
    }

    double getKp() const { return Kp; }

    double getKi() const {return Ki; }

    double getKd() const { return Kd; }

    double getKiMax() const { return KiMax; }

    double getError() const { return error; }

    double getIntegral() const { return integral; }

    double getDerivative() const { return derivative; }

    double getPreviousError() const { return previousError; }

private:
    double Kp, Ki, Kd, KiMax;
    double error, integral, derivative, previousError;
    long lastTime = 0;
};

class PidMotor {
public:

    PidMotor(Encoder& encoder, MotorDriver& motorDriver, PIDController& pid, unsigned int controllCycleTime)
        : encoder(encoder), motorDriver(motorDriver), pid(pid), controllCycleTime(controllCycleTime) {}

    void init(){
      encoder.init();
      motorDriver.init();
    }
    
    void update() {
        
        encoder.update(); //TODO das besser organisieren außerhalb der Klasse
        
        unsigned long currentTime = millis();
        if (currentTime - lastUpdateTime >= controllCycleTime) {

            lastUpdateTime = currentTime;

            if (desiredMotorSpeed == 0.0){
              motorTimeoutThresh ++;
            } else {
              motorTimeoutThresh = 0;
            }
            
            if (motorTimeoutThresh > 100) {   // equals 100 * controllCycleTime until motors go to sleep
              motorDriver.stop();
              pid.pidReset();
            } else {
              int pidValue = int(round(pid.compute(desiredMotorSpeed, encoder.getSpeed()))); // Apply PID control   //TODO ist round wirklich auf vor dezimal?
              motorDriver.setMotorDutyCycle(pidValue); // Set motor speed based on PWM
              motorDriver.update();
            }
        }
    }

    void setDesiredMotorSpeed(double desiredMotorSpeed) {
        this->desiredMotorSpeed = desiredMotorSpeed;
    }
    
    double getDesiredMotorSpeed() const {
        return desiredMotorSpeed;
    }

    double getMeasuredMotorSpeed() const {
        return encoder.getSpeed();
    }

    double getAngularMotorPosition() {
        return encoder.getAngularPosition();
    }

private:
    Encoder& encoder;
    MotorDriver& motorDriver;
    PIDController& pid;
    double desiredMotorSpeed = 0; // Desired motor speed (RPS)
    unsigned long controllCycleTime;
    unsigned long lastUpdateTime = 0;
    unsigned long motorTimeoutThresh = 0;
};

// Initialize encoders
Encoder leftEncoder(LEFT_ENCODER_C1_PIN, LEFT_ENCODER_C2_PIN, ENCODER_TICKS_PER_REVOLUTION, ENCODER_PT1_TIMECONSTANT_S, UPDATE_INTERVAL_ENCODER);
Encoder rightEncoder(RIGHT_ENCODER_C1_PIN, RIGHT_ENCODER_C2_PIN, ENCODER_TICKS_PER_REVOLUTION,ENCODER_PT1_TIMECONSTANT_S, UPDATE_INTERVAL_ENCODER);

// Initialize motor controllers
MotorDriver leftMotorDriver(MOTORDRIVER_LEFT_CW_PIN, MOTORDRIVER_LEFT_CCW_PIN, MOTORDRIVER_LEFT_PWM_PIN,  MAX_MOTOR_DRIVER_DUTYCYCLE, 0);
MotorDriver rightMotorDriver(MOTORDRIVER_RIGHT_CW_PIN, MOTORDRIVER_RIGHT_CCW_PIN, MOTORDRIVER_RIGHT_PWM_PIN, MAX_MOTOR_DRIVER_DUTYCYCLE, 1);

// Initialize PID controllers
PIDController leftPid(KP, KI, KD, KI_MAX);
PIDController rightPid(KP, KI, KD, KI_MAX);

// Initialize motors
PidMotor leftMotor(leftEncoder, leftMotorDriver, leftPid, UPDATE_INTERVAL_PID_CONTROL);
PidMotor rightMotor(rightEncoder, rightMotorDriver, rightPid, UPDATE_INTERVAL_PID_CONTROL);

// Select which motor to control based on configuration
PidMotor& selectedMotor = CONTROL_LEFT_MOTOR ? leftMotor : rightMotor;
MotorDriver& selectedMotorDriver = CONTROL_LEFT_MOTOR ? leftMotorDriver : rightMotorDriver;

void setup() {
  Serial.begin(115200);
  pinMode(MOTORDRIVER_STBY_PIN, OUTPUT);
  digitalWrite(MOTORDRIVER_STBY_PIN, HIGH);
  pinMode(POTENTIOMETER_PIN, INPUT);

  selectedMotor.init();

  Serial.println("Setup complete");
  Serial.println("DesiredSpeed,MeasuredSpeed");
}

static unsigned long lastPrintTime = 0;
const unsigned long PRINT_INTERVAL_MS = 10; // 10ms print interval

const int NUM_SAMPLES = 100;  // Number of samples for averaging
int potValues[NUM_SAMPLES];
int potIndex = 0;

int readFilteredPot() {
  potValues[potIndex] = analogRead(POTENTIOMETER_PIN);
  potIndex = (potIndex + 1) % NUM_SAMPLES;
  
  long sum = 0;
  for (int i = 0; i < NUM_SAMPLES; i++) {
    sum += potValues[i];
  }
  return sum / NUM_SAMPLES;
}

#include <math.h> // For sin() function

// Sine wave parameters
const float SINE_AMPLITUDE = 100.0;  // ±1 RPS (adjust as needed)
const float SINE_FREQUENCY = 0.4;   // 0.5 Hz (complete cycle every 2 seconds)
unsigned long loopStartTime = millis();

void loop() {
  //int potValue = readFilteredPot();
  //double targetSpeed = map(potValue, 0, 4095, -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED);

  float elapsedSeconds = (millis() - loopStartTime) / 1000.0;
  double targetSpeed = SINE_AMPLITUDE * sin(2 * PI * SINE_FREQUENCY * elapsedSeconds);

  // Set motor speed
  selectedMotor.setDesiredMotorSpeed(targetSpeed);
  selectedMotor.update();


  if (millis() - lastPrintTime >= PRINT_INTERVAL_MS) {
    lastPrintTime = millis();

    Serial.print("desired:");
    Serial.print(selectedMotor.getDesiredMotorSpeed());
    Serial.print(",");
    Serial.print("measured:");
    Serial.println(selectedMotor.getMeasuredMotorSpeed());
  }

}
