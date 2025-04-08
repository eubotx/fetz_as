#include <ESP32Encoder.h>
#include <algorithm> // For std::sort

// Configuration - change these as needed
#define CONTROL_LEFT_MOTOR true  // Set to false to control right motor instead
constexpr int POTENTIOMETER_PIN = 34;  // Pin for speed control potentiometer

// Robot parameters
constexpr double WHEEL_DIAMETER = 0.04;
constexpr double WHEEL_DISTANCE = 0.12;
constexpr double MAX_MOTOR_SPEED = 65.0;  // Max speed in RPS
constexpr int MAX_MOTOR_DRIVER_DUTYCYCLE = 250;
constexpr int ENCODER_TICKS_PER_REVOLUTION = 12 * 2;
constexpr double ENCODER_PT1_TIMECONSTANT_S = 0.0;
constexpr double GEARBOX_RATIO = 1.0/200.0;

constexpr unsigned long UPDATE_INTERVAL_ENCODER = 2;  // In ms
constexpr unsigned long UPDATE_INTERVAL_PID_CONTROL = 2;  // In ms

// PID controller parameters - adjust these during tuning
double KP = 2.0;
double KI = 0.5;
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
            
            if (deltaTime > 0) {  // Prevent division by zero
                double rawRps = (currentCount - lastCount) / (ticksPerRevolution * (deltaTime / 1000000.0));
                
                if (timeConstant > 0.0) {
                    double alpha = deltaTime / (timeConstant * 1000000.0 + deltaTime);
                    filteredRps = alpha * rawRps + (1.0 - alpha) * filteredRps;
                } else {
                    filteredRps = rawRps;
                }
            }
            
            lastCount = currentCount;
            lastUpdateTime = currentTime;
        }
    }

    double getSpeed() const {
        return filteredRps;
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
        : Kp(Kp), Ki(Ki), Kd(Kd), KiMax(KiMax), error(0), integral(0), derivative(0), previousError(0) {}

    double compute(double desiredValue, double measuredValue) {
        double deltaTime = (micros() - lastTime) / 1000000.0;
        error = desiredValue - measuredValue;
        integral = integral + error;
        integral = constrain(integral, -KiMax, KiMax);
        derivative = error - previousError;
        previousError = error;
        lastTime = micros();

        return ((Kp * error) + (Ki * integral) + (Kd * derivative)) * deltaTime;
    }

    void setPidValues(double Kp, double Ki, double Kd, double KiMax) {
        this->Kp = Kp;
        this->Ki = Ki;
        this->Kd = Kd;
        this->KiMax = KiMax;
    }

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
        encoder.update();
        
        unsigned long currentTime = millis();
        if (currentTime - lastUpdateTime >= controllCycleTime) {
            lastUpdateTime = currentTime;
            
            int pidValue = int(round(pid.compute(desiredMotorSpeed, encoder.getSpeed())));
            motorDriver.setMotorDutyCycle(motorDriver.getMotorDutyCycle() + pidValue);
            motorDriver.update();
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

private:
    Encoder& encoder;
    MotorDriver& motorDriver;
    PIDController& pid;
    double desiredMotorSpeed = 0;
    unsigned long controllCycleTime;
    unsigned long lastUpdateTime = 0;
};

// Initialize encoders
Encoder leftEncoder(LEFT_ENCODER_C1_PIN, LEFT_ENCODER_C2_PIN, ENCODER_TICKS_PER_REVOLUTION, ENCODER_PT1_TIMECONSTANT_S, UPDATE_INTERVAL_ENCODER);
Encoder rightEncoder(RIGHT_ENCODER_C1_PIN, RIGHT_ENCODER_C2_PIN, ENCODER_TICKS_PER_REVOLUTION, ENCODER_PT1_TIMECONSTANT_S, UPDATE_INTERVAL_ENCODER);

// Initialize motor controllers
MotorDriver leftMotorDriver(MOTORDRIVER_LEFT_CW_PIN, MOTORDRIVER_LEFT_CCW_PIN, MOTORDRIVER_LEFT_PWM_PIN, MAX_MOTOR_DRIVER_DUTYCYCLE, 0);
MotorDriver rightMotorDriver(MOTORDRIVER_RIGHT_CW_PIN, MOTORDRIVER_RIGHT_CCW_PIN, MOTORDRIVER_RIGHT_PWM_PIN, MAX_MOTOR_DRIVER_DUTYCYCLE, 1);

// Initialize PID controllers
PIDController leftPid(KP, KI, KD, KI_MAX);
PIDController rightPid(KP, KI, KD, KI_MAX);

// Initialize motors
PidMotor leftMotor(leftEncoder, leftMotorDriver, leftPid, UPDATE_INTERVAL_PID_CONTROL);
PidMotor rightMotor(rightEncoder, rightMotorDriver, rightPid, UPDATE_INTERVAL_PID_CONTROL);

// Select which motor to control based on configuration
PidMotor& selectedMotor = CONTROL_LEFT_MOTOR ? leftMotor : rightMotor;

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

void loop() {
    // Read potentiometer (0-4095) and map to motor speed range (-MAX_MOTOR_SPEED to +MAX_MOTOR_SPEED)
    int potValue = analogRead(POTENTIOMETER_PIN);
    double targetSpeed = map(potValue, 0, 4095, -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED);
    
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
      Serial.print(",");
      Serial.print("measured:");
      Serial.println(selectedMotor.getMeasuredMotorSpeed());
  }
    
}
