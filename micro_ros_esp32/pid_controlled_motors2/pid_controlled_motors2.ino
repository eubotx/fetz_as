#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float64.h>
#include <std_msgs/msg/float64_multi_array.h>
#include <geometry_msgs/msg/twist.h>
#include <ESP32Encoder.h>
#include <algorithm> // For std::sort

// Debug macros (uncomment to enable)
#define DEBUG_MOTORCONTROL_LEFT    // Enable left debug topic
#define DEBUG_MOTORCONTROL_RIGHT   // Enable right debug topic
//#define DEBUG_LOOPTIME             // Enable loop time debug topic


// Robot parameters
constexpr double WHEEL_DIAMETER = 0.04;
constexpr double WHEEL_DISTANCE = 0.12;
constexpr double MAX_MOTOR_SPEED = 65;  // Max speed in RPS
constexpr int MAX_MOTOR_DRIVER_DUTYCYCLE = 255;
constexpr int ENCODER_TICKS_PER_REVOLUTION = 12 * 2;
constexpr double GEARBOX_RATIO = 200.0;

constexpr unsigned long UPDATE_INTERVAL_PID_CONTROL = 2;

// PID controller parameters
constexpr double KP = 3.0;
constexpr double KI = 0.5;
constexpr double KD = 0.0;
constexpr double KI_MAX = 30.0;

// WiFi configuration
//constexpr const char* MY_IP = "192.168.8.171"; // Bjoern Gelb DO NOT DELETE
constexpr const char* MY_IP = "192.168.8.204"; // Flo Gelb
constexpr const char* MY_SSID = "GL-MT300N-V2-7d8";
constexpr const char* MY_PASSWORD = "goodlife";
constexpr int MY_PORT = 8888;

// Pin definitions
constexpr int LEFT_ENCODER_C1_PIN = 23;
constexpr int LEFT_ENCODER_C2_PIN = 22;
constexpr int RIGHT_ENCODER_C1_PIN = 19;
constexpr int RIGHT_ENCODER_C2_PIN = 18;
constexpr int MOTORCONTROLLER_IN1_PIN = 14;
constexpr int MOTORCONTROLLER_IN2_PIN = 27;
constexpr int MOTORCONTROLLER_IN3_PIN = 25;
constexpr int MOTORCONTROLLER_IN4_PIN = 26;

// ROS2 entities

#ifdef DEBUG_MOTORCONTROL_LEFT
rcl_publisher_t debugMotorcontrolLeftPublisher;
std_msgs__msg__Float64MultiArray debugMotorcontrolLeftMsg;
#endif

#ifdef DEBUG_MOTORCONTROL_RIGHT
rcl_publisher_t debugMotorcontrolRightPublisher; 
std_msgs__msg__Float64MultiArray debugMotorcontrolRightMsg;
#endif

#ifdef DEBUG_LOOPTIME
rcl_publisher_t debugLooptimePublisher;
std_msgs__msg__Float64MultiArray debugLooptimeMsg;
#endif

rcl_subscription_t twistSubscriber;
rcl_subscription_t pidTuningSubscriber;
std_msgs__msg__Float64MultiArray pidTuningMsg;
geometry_msgs__msg__Twist twistMsg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

// Error handling macros
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if (temp_rc != RCL_RET_OK) {} }
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if (temp_rc != RCL_RET_OK) {} }

// Encoder class
class Encoder {
public:
    Encoder(int pin1, int pin2, double ticksPerRevolution) : pin1(pin1), pin2(pin2), ticksPerRevolution(ticksPerRevolution) {}
    
    void init() {
        pinMode(pin1, INPUT_PULLUP);
        pinMode(pin2, INPUT_PULLUP);

        // Enable the weak pull up resistors
        ESP32Encoder::useInternalWeakPullResistors = puType::up;

        encoder.attachHalfQuad(pin1, pin2);
        encoder.setCount(0);
    }

    void update() {
        long currentCount = encoder.getCount();
        long delta = currentCount - lastCount;
        lastCount = currentCount;
        speed = ((double) delta) / ticksPerRevolution / (micros() / 1000000.0);
    }

    void resetCount() {
        encoder.setCount(0);
    }

    double getSpeed() const {
        return speed;
    }

    double getAngularPosition(){
        return ((double)lastCount) / ticksPerRevolution;
    }

private:
    ESP32Encoder encoder;
    int pin1, pin2;
    double ticksPerRevolution;
    long lastCount = 0;
    double speed = 0;
};

// MotorDriver class
class MotorDriver {
public:
    MotorDriver(int pin1, int pin2, const int maxDutyCycle) : pin1(pin1), pin2(pin2), maxDutyCycle(maxDutyCycle), dutyCycle(0) {}

    void init(){
        pinMode(pin1, OUTPUT);
        pinMode(pin2, OUTPUT);
    }

    void update() {
        if (dutyCycle > 0) {
            digitalWrite(pin2, LOW);
            analogWrite(pin1, dutyCycle);
        } else if (dutyCycle < 0) {
            digitalWrite(pin1, LOW);
            analogWrite(pin2, -dutyCycle);
        } else {
            digitalWrite(pin1, LOW);
            digitalWrite(pin2, LOW);
        }
    }

    void setMotorDutyCycle(int dutyCycle){
        this->dutyCycle = constrain(dutyCycle, -maxDutyCycle, maxDutyCycle);
    }

    int getMotorDutyCycle() const { return dutyCycle; }

private:
    int pin1, pin2;
    int dutyCycle;
    int maxDutyCycle;
};

// PIDController class
class PIDController {
public:
    PIDController(double Kp, double Ki, double Kd, double KiMax) 
        : Kp(Kp), Ki(Ki), Kd(Kd), KiMax(KiMax), error(0), integral(0), derivative(0), previousError(0) {}

    double compute(double desiredValue, double measuredValue) {
        error = desiredValue - measuredValue;
        double deltaTime = (double)(micros() - lastTime) / 1000000.0;
        integral += error * deltaTime;
        integral = constrain(integral, -KiMax, KiMax);  // Anti-windup
        derivative = error - previousError * deltaTime;
        previousError = error;
        lastTime = micros();

        return (Kp * error) + (Ki * integral) + (Kd * derivative);
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

    void update() {
        unsigned long currentTime = millis();
        if (currentTime - lastUpdateTime >= controllCycleTime) {

            lastUpdateTime = currentTime;

            encoder.update(); // Update encoder readings        
            int pidValue = int(round(pid.compute(desiredMotorSpeed, encoder.getSpeed()))); // Apply PID control
            motorDriver.setMotorDutyCycle(motorDriver.getMotorDutyCycle() + pidValue); // Set motor speed based on PWM
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
};

class Wheel {
public:
    Wheel(PidMotor& pidMotor, double gearboxRatio)
        : pidMotor(pidMotor), gearboxRatio(gearboxRatio) {}

    void update() {
        pidMotor.update();
    }

    void setDesiredWheelSpeed(double desiredWheelSpeed) {
        this->desiredWheelSpeed = desiredWheelSpeed; // Set wheel speed
        // Convert wheel speed to motor speed using the gearbox ratio
        pidMotor.setDesiredMotorSpeed(desiredWheelSpeed * gearboxRatio);
    }

    double getDesiredWheelSpeed() const {
        return desiredWheelSpeed;
    }

    double getMeasuredWheelSpeed() {
        return pidMotor.getMeasuredMotorSpeed() / gearboxRatio;
    }

    double getAngularPosition() {
        return pidMotor.getAngularMotorPosition() / gearboxRatio;
    }

private:
    PidMotor& pidMotor;
    double desiredWheelSpeed = 0.0; // Desired wheel speed (RPS)
    double angularWheelPosition = 0.0;
    double gearboxRatio; // Gearbox ratio to convert wheel speed to motor speed
};

// RobotControl Class
class RobotControl {
public:
    RobotControl(Wheel& leftWheel, Wheel& rightWheel, double wheelDiameter, double wheelDistance)
        : leftWheel(leftWheel), rightWheel(rightWheel), wheelDiameter(wheelDiameter), wheelDistance(wheelDistance) {}

    void calculateAndSetWheelSpeeds(double linear, double angular) {
        double linearWheelSpeed = (2.0 * linear) / wheelDiameter;
        double angularWheelSpeed = (angular * wheelDistance) / (wheelDiameter / 2.0);

        double lefttWheelSpeed = linearWheelSpeed - angularWheelSpeed;
        double rightWheelSpeed = linearWheelSpeed + angularWheelSpeed;
        
        leftWheel.setDesiredWheelSpeed(lefttWheelSpeed);
        rightWheel.setDesiredWheelSpeed(rightWheelSpeed);
    }

    void updateDrives() {
        leftWheel.update();
        rightWheel.update();
    }

    private:
        Wheel& leftWheel;
        Wheel& rightWheel;
        const double wheelDiameter;
        const double wheelDistance;

};

#ifdef DEBUG_LOOPTIME
// Loop time variables
constexpr int LOOPTIME_WINDOW_SIZE = 10; // Number of cycles to track
unsigned long loopTimes[LOOPTIME_WINDOW_SIZE] = {0}; // Array to store loop times
int loopTimeIndex = 0; // Index for the current loop time
unsigned long lastLoopTime = 0; // Time of the last loop iteration


// Function to calculate loop time statistics
void collectLooptime() {
    unsigned long currentTime = millis();
    unsigned long loopTime = currentTime - lastLoopTime;
    lastLoopTime = currentTime;

    // Store the current loop time in the array
    loopTimes[loopTimeIndex] = loopTime;
    loopTimeIndex = (loopTimeIndex + 1) % LOOPTIME_WINDOW_SIZE;
}
#endif

// Initialize encoders
Encoder leftEncoder(LEFT_ENCODER_C1_PIN, LEFT_ENCODER_C2_PIN, ENCODER_TICKS_PER_REVOLUTION);
Encoder rightEncoder(RIGHT_ENCODER_C1_PIN, RIGHT_ENCODER_C2_PIN, ENCODER_TICKS_PER_REVOLUTION);

// Initialize motor controllers
MotorDriver leftMotorDriver(MOTORCONTROLLER_IN1_PIN, MOTORCONTROLLER_IN2_PIN, MAX_MOTOR_DRIVER_DUTYCYCLE);
MotorDriver rightMotorDriver(MOTORCONTROLLER_IN3_PIN, MOTORCONTROLLER_IN4_PIN, MAX_MOTOR_DRIVER_DUTYCYCLE);

// Initialize PID controllers
PIDController leftPid(KP, KI, KD, KI_MAX);
PIDController rightPid(KP, KI, KD, KI_MAX);

// Initialize motors
PidMotor leftMotor(leftEncoder, leftMotorDriver, leftPid, UPDATE_INTERVAL_PID_CONTROL);
PidMotor rightMotor(rightEncoder, rightMotorDriver, rightPid, UPDATE_INTERVAL_PID_CONTROL);

// Initialize wheels
Wheel leftWheel(leftMotor, GEARBOX_RATIO);
Wheel rightWheel(rightMotor, GEARBOX_RATIO);

// Initialize robot control
RobotControl robot(leftWheel, rightWheel, WHEEL_DIAMETER, WHEEL_DISTANCE);


// Timer callback
void timer_callback(rcl_timer_t* timer, int64_t last_call_time) {
    RCLC_UNUSED(last_call_time);
    if (timer != NULL) {

#ifdef DEBUG_MOTORCONTROL_LEFT
        // Publish left debug information (PID + PWM)
        debugMotorcontrolLeftMsg.data.data[0] = leftMotor.getDesiredMotorSpeed();  // Desired value
        debugMotorcontrolLeftMsg.data.data[1] = leftMotor.getMeasuredMotorSpeed(); // Measured value
        debugMotorcontrolLeftMsg.data.data[2] = leftMotor.getDesiredMotorSpeed() - leftMotor.getMeasuredMotorSpeed(); // Error
        debugMotorcontrolLeftMsg.data.data[3] = leftMotorDriver.getMotorDutyCycle(); // PWM duty cycle
        RCSOFTCHECK(rcl_publish(&debugMotorcontrolLeftPublisher, &debugMotorcontrolLeftMsg, NULL));
#endif

#ifdef DEBUG_MOTORCONTROL_RIGHT
        // Publish right debug information (PID + PWM)
        debugMotorcontrolRightMsg.data.data[0] = rightMotor.getDesiredMotorSpeed();  // Desired value
        debugMotorcontrolRightMsg.data.data[1] = rightMotor.getMeasuredMotorSpeed(); // Measured value
        debugMotorcontrolRightMsg.data.data[2] = rightMotor.getDesiredMotorSpeed() - rightMotor.getMeasuredMotorSpeed(); // Error
        debugMotorcontrolRightMsg.data.data[3] = rightMotorDriver.getMotorDutyCycle(); // PWM duty cycle
        RCSOFTCHECK(rcl_publish(&debugMotorcontrolRightPublisher, &debugMotorcontrolRightMsg, NULL));
#endif

#ifdef DEBUG_LOOPTIME
        // Calculate statistics
        unsigned long sortedLoopTimes[LOOPTIME_WINDOW_SIZE];
        memcpy(sortedLoopTimes, loopTimes, sizeof(loopTimes));
        std::sort(sortedLoopTimes, sortedLoopTimes + LOOPTIME_WINDOW_SIZE);

        unsigned long minLoopTime = sortedLoopTimes[0];
        unsigned long maxLoopTime = sortedLoopTimes[LOOPTIME_WINDOW_SIZE - 1];
        unsigned long medianLoopTime = sortedLoopTimes[LOOPTIME_WINDOW_SIZE / 2];

        unsigned long sumLoopTimes = 0;
        for (int i = 0; i < LOOPTIME_WINDOW_SIZE; i++) {
            sumLoopTimes += sortedLoopTimes[i];
        }
        unsigned long averageLoopTime = sumLoopTimes / LOOPTIME_WINDOW_SIZE;

        
        // Publish loop time statistics
        
        debugLooptimeMsg.data.data[0] = medianLoopTime;  // Median
        debugLooptimeMsg.data.data[1] = minLoopTime;     // Min
        debugLooptimeMsg.data.data[2] = maxLoopTime;     // Max
        debugLooptimeMsg.data.data[3] = averageLoopTime; // Average
        RCSOFTCHECK(rcl_publish(&debugLooptimePublisher, &debugLooptimeMsg, NULL));
#endif
    }
}

// Twist message callback
void cmd_vel_callback(const void* msgin) {
    const geometry_msgs__msg__Twist* msg = (const geometry_msgs__msg__Twist*)msgin;
    robot.calculateAndSetWheelSpeeds(msg->linear.x, msg->angular.z);
}

// PID tuning callback
void pid_tuning_callback(const void* msgin) {
    const std_msgs__msg__Float64MultiArray* msg = (const std_msgs__msg__Float64MultiArray*)msgin;

    // Ensure the message contains exactly 4 values (Kp, Ki, Kd, KiMax)
    if (msg->data.size == 4) {
        double Kp = msg->data.data[0];
        double Ki = msg->data.data[1];
        double Kd = msg->data.data[2];
        double KiMax = msg->data.data[3];

        // Update PID parameters for both left and right controllers
        leftPid.setPidValues(Kp, Ki, Kd, KiMax);
        leftPid.pidReset();
        rightPid.setPidValues(Kp, Ki, Kd, KiMax);
        rightPid.pidReset();
    }
}


void setup() {
    set_microros_wifi_transports((char*)MY_SSID, (char*)MY_PASSWORD, (char*)MY_IP, MY_PORT);



    allocator = rcl_get_default_allocator();
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCCHECK(rclc_node_init_default(&node, "diff_drive_bot_esp32", "", &support));
    RCCHECK(rclc_subscription_init_default(&twistSubscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel"));

#ifdef DEBUG_MOTORCONTROL_LEFT
    // Initialize left debug publisher
    RCCHECK(rclc_publisher_init_default(
        &debugMotorcontrolLeftPublisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
        "left_debug"
    ));
    debugMotorcontrolLeftMsg.data.capacity = 4;
    debugMotorcontrolLeftMsg.data.size = 4;
    debugMotorcontrolLeftMsg.data.data = (double*)malloc(4 * sizeof(double));
#endif

#ifdef DEBUG_MOTORCONTROL_RIGHT
    // Initialize right debug publisher
    RCCHECK(rclc_publisher_init_default(
        &debugMotorcontrolRightPublisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
        "right_debug"
    ));
    debugMotorcontrolRightMsg.data.capacity = 4;
    debugMotorcontrolRightMsg.data.size = 4;
    debugMotorcontrolRightMsg.data.data = (double*)malloc(4 * sizeof(double));
#endif

#ifdef DEBUG_LOOPTIME
    // Initialize loop time debug publisher
    RCCHECK(rclc_publisher_init_default(
        &debugLooptimePublisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
        "looptime_debug"
    ));
    debugLooptimeMsg.data.capacity = 4;
    debugLooptimeMsg.data.size = 4;
    debugLooptimeMsg.data.data = (double*)malloc(4 * sizeof(double));
#endif

    // Initialize the PID tuning subscriber
    RCCHECK(rclc_subscription_init_default(
        &pidTuningSubscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
        "pid_tuning"
    ));


    const unsigned int timerTimeout = 100;
    RCCHECK(rclc_timer_init_default2(&timer, &support, RCL_MS_TO_NS(timerTimeout), timer_callback, true));
    RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));
    RCCHECK(rclc_executor_add_subscription(&executor, &twistSubscriber, &twistMsg, &cmd_vel_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &pidTuningSubscriber, &pidTuningMsg, &pid_tuning_callback, ON_NEW_DATA));
}

void loop() {
    RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));

    robot.updateDrives();

#ifdef DEBUG_LOOPTIME
    // Get looptime data
    collectLooptime();
#endif
}
