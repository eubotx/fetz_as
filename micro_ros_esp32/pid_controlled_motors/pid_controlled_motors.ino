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
constexpr int ENCODER_TICKS_PER_MOTOR_REVOLUTION = 12 * 2;
constexpr double GEARBOX_RATIO = 200.0;

// Calculation intervals
constexpr unsigned long UPDATE_INTERVAL_WHEEL_SPEED_MEAS = 2;
bool newWheelspeedMeasurementAvailable = false;

// PID controller parameters
double KP = 1.0;
double KI = 0.5;
double KD = 0.0;
double KI_MAX = 30.0;  // for PWM influence calculate Ki * KI_MAX

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
constexpr int MOTORDRIVER_IN1_PIN = 14;
constexpr int MOTORDRIVER_IN2_PIN = 27;
constexpr int MOTORDRIVER_IN3_PIN = 25;
constexpr int MOTORDRIVER_IN4_PIN = 26;
constexpr int MOTORDRIVER_ENA_PIN = 33;
constexpr int MOTORDRIVER_ENB_PIN = 32;


// PID Controller Structure
struct PIDController {
    double Kp;
    double Ki;
    double Kd;
    double KiMax;
    double error;
    double integral;
    double derivative;
    double previousError;
    long lastControlTime;
};

// Function prototypes
void PID_Init(PIDController* pid, double Kp, double Ki, double Kd, double KiMax);
double PID_Compute(PIDController* pid, double desiredValue, double measuredValue);
void calculateDesiredMotorSpeeds(float linear, float angular);
void setMotorPwm(int pinForward, int pinBackward, int dutyCycle);
void timer_callback(rcl_timer_t* timer, int64_t last_call_time);
void cmd_vel_callback(const void* msgin);
void pid_tuning_callback(const void* msgin);

// Encoder variables
ESP32Encoder encoderLeft;
ESP32Encoder encoderRight;
volatile long encoderCountLeft = 0;
volatile long encoderCountRight = 0;
long lastEncoderCountLeft = 0;
long lastEncoderCountRight = 0;

unsigned long lastMotorspeedMeasurementTime = 0;

// Motor speed variables in RPS
double measuredMotorspeedLeft = 0;
double measuredMotorspeedRight = 0;
double desiredMotorspeedLeft = 0;
double desiredMotorspeedRight = 0;

// PID Controllers for left and right motors
PIDController pidLeft;
PIDController pidRight;

// Motorcontroller PWM duty cycle variables
int motorPwmDutyCycleLeft = 0;
int motorPwmDutyCycleRight = 0;

// Loop time variables
constexpr int LOOPTIME_WINDOW_SIZE = 10; // Number of cycles to track
unsigned long loopTimes[LOOPTIME_WINDOW_SIZE] = {0}; // Array to store loop times
int loopTimeIndex = 0; // Index for the current loop time
unsigned long lastLoopTime = 0; // Time of the last loop iteration

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

// Function to calculate motor speed in RPS
void calculateMeasuredMotorSpeed(long encoderCountLeft, long encoderCountRight) {
    unsigned long currentTime = millis();
    unsigned long elapsedTime = currentTime - lastMotorspeedMeasurementTime;

    if (elapsedTime >= UPDATE_INTERVAL_WHEEL_SPEED_MEAS) {
        long deltaLeftTicks = encoderCountLeft - lastEncoderCountLeft;
        long deltaRightTicks = encoderCountRight - lastEncoderCountRight;

        // Calculate speed in rotations per second (RPS)
        measuredMotorspeedLeft = (deltaLeftTicks / (double)ENCODER_TICKS_PER_MOTOR_REVOLUTION) / (elapsedTime / 1000.0);
        measuredMotorspeedRight = (deltaRightTicks / (double)ENCODER_TICKS_PER_MOTOR_REVOLUTION) / (elapsedTime / 1000.0);

        lastEncoderCountLeft = encoderCountLeft;
        lastEncoderCountRight = encoderCountRight;
        
        lastMotorspeedMeasurementTime = currentTime;
        newWheelspeedMeasurementAvailable = true;
    }
}

void calculateDesiredMotorSpeeds(float linear, float angular) {
    
    // Calculate the desired wheel speeds in radians per second (RPS)
    double linearWheelSpeed = (2.0 * linear) / WHEEL_DIAMETER;
    double angularWheelSpeed = (angular * WHEEL_DISTANCE) / (WHEEL_DIAMETER / 2.0);

    // Adjust for the gearbox reduction ratio
    // The motor speed must be 200 times higher than the wheel speed
    double linearMotorSpeed = linearWheelSpeed * GEARBOX_RATIO;
    double angularMotorSpeed = angularWheelSpeed * GEARBOX_RATIO;

    // Calculate the desired motor speeds for left and right motors
    desiredMotorspeedLeft = linearMotorSpeed - angularMotorSpeed;
    desiredMotorspeedRight = linearMotorSpeed + angularMotorSpeed;

    // Constrain the motor speeds to the maximum allowed speed
    desiredMotorspeedLeft = constrain(desiredMotorspeedLeft, -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED);
    desiredMotorspeedRight = constrain(desiredMotorspeedRight, -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED);
}


// PID Initialization
void PID_Init(PIDController* pid, double Kp, double Ki, double Kd, double KiMax) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->KiMax = KiMax;
    pid->error = 0;
    pid->integral = 0;
    pid->derivative = 0;
    pid->previousError = 0;
}

// PID Compute
double PID_Compute(PIDController* pid, double desiredValue, double measuredValue) {
    pid->error = desiredValue - measuredValue;
    long timedelta = micros() - pid->lastControlTime;
    pid->integral = pid->integral + pid->error * (timedelta / 1000000.0); //normalized for looptime Ki/s
    pid->integral = constrain(pid->integral, -pid->KiMax, pid->KiMax);  // Anti-windup
    pid->derivative = pid->error - pid->previousError;
    pid->previousError = pid->error;
    pid->lastControlTime = micros();
    
    return (pid->Kp * pid->error) + (pid->Ki * pid->integral) + (pid->Kd * pid->derivative);
}

// Set motor PWM
void setMotorPwmOld(int pinForward, int pinBackward, int dutyCycle) {
    digitalWrite(MOTORDRIVER_ENA_PIN, HIGH);
    digitalWrite(MOTORDRIVER_ENB_PIN, HIGH);


    if (dutyCycle > 0) {
        digitalWrite(pinBackward, LOW);
        analogWrite(pinForward, dutyCycle);
    } else if (dutyCycle < 0) {
        digitalWrite(pinForward, LOW);
        analogWrite(pinBackward, -dutyCycle);
    } else {
        digitalWrite(pinForward, LOW);
        digitalWrite(pinBackward, LOW);
    }
}

// Set motor PWM
void setMotorPwm(int pinForward, int pinBackward, int pinEna, int dutyCycle) {
    if (dutyCycle >= 0) {
            digitalWrite(pinBackward, LOW);
            digitalWrite(pinForward, HIGH);
            analogWrite(pinEna, abs(dutyCycle));
        } else if (dutyCycle < 0) {
            digitalWrite(pinForward, LOW);
            digitalWrite(pinBackward, HIGH);
            analogWrite(pinEna, abs(dutyCycle));
        } else {
            digitalWrite(pinEna, LOW);    //TODO was ist hier richtig damit der motor nicht hard stoppt, mal so probieren
            //alogWrite(pin3, abs(dutyCycle));  //oder so
        }
}

// Timer callback
void timer_callback(rcl_timer_t* timer, int64_t last_call_time) {
    RCLC_UNUSED(last_call_time);
    if (timer != NULL) {

#ifdef DEBUG_MOTORCONTROL_LEFT
        // Publish left debug information (PID + PWM)
        debugMotorcontrolLeftMsg.data.data[0] = desiredMotorspeedLeft;  // Desired value
        debugMotorcontrolLeftMsg.data.data[1] = measuredMotorspeedLeft; // Measured value
        debugMotorcontrolLeftMsg.data.data[2] = desiredMotorspeedLeft - measuredMotorspeedLeft; // Error
        debugMotorcontrolLeftMsg.data.data[3] = motorPwmDutyCycleLeft; // PWM duty cycle
        RCSOFTCHECK(rcl_publish(&debugMotorcontrolLeftPublisher, &debugMotorcontrolLeftMsg, NULL));
#endif

#ifdef DEBUG_MOTORCONTROL_RIGHT
        // Publish right debug information (PID + PWM)
        debugMotorcontrolRightMsg.data.data[0] = desiredMotorspeedRight; // Desired value
        debugMotorcontrolRightMsg.data.data[1] = measuredMotorspeedRight; // Measured value
        debugMotorcontrolRightMsg.data.data[2] = desiredMotorspeedRight - measuredMotorspeedRight; // Error
        debugMotorcontrolRightMsg.data.data[3] = motorPwmDutyCycleRight; // PWM duty cycle
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
    calculateDesiredMotorSpeeds(msg->linear.x, msg->angular.z);
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
        PID_Init(&pidLeft, Kp, Ki, Kd, KiMax);
        PID_Init(&pidRight, Kp, Ki, Kd, KiMax);
    }
}

// Function to calculate loop time statistics
void collectLooptime() {
    unsigned long currentTime = millis();
    unsigned long loopTime = currentTime - lastLoopTime;
    lastLoopTime = currentTime;

    // Store the current loop time in the array
    loopTimes[loopTimeIndex] = loopTime;
    loopTimeIndex = (loopTimeIndex + 1) % LOOPTIME_WINDOW_SIZE;
}

// Setup function
void setup() {
    set_microros_wifi_transports((char*)MY_SSID, (char*)MY_PASSWORD, (char*)MY_IP, MY_PORT);

    pinMode(LEFT_ENCODER_C1_PIN, INPUT_PULLUP);
    pinMode(LEFT_ENCODER_C2_PIN, INPUT_PULLUP);
    pinMode(RIGHT_ENCODER_C1_PIN, INPUT_PULLUP);
    pinMode(RIGHT_ENCODER_C2_PIN, INPUT_PULLUP);
    pinMode(MOTORDRIVER_IN1_PIN, OUTPUT);
    pinMode(MOTORDRIVER_IN2_PIN, OUTPUT);
    pinMode(MOTORDRIVER_IN3_PIN, OUTPUT);
    pinMode(MOTORDRIVER_IN4_PIN, OUTPUT);

    pinMode(MOTORDRIVER_ENA_PIN, OUTPUT);
    pinMode(MOTORDRIVER_ENB_PIN, OUTPUT);
    digitalWrite(MOTORDRIVER_ENA_PIN, HIGH);
    digitalWrite(MOTORDRIVER_ENB_PIN, HIGH);





    // Enable the weak pull up resistors
    ESP32Encoder::useInternalWeakPullResistors = puType::up;

    encoderLeft.attachHalfQuad(LEFT_ENCODER_C1_PIN, LEFT_ENCODER_C2_PIN);
    encoderRight.attachHalfQuad(RIGHT_ENCODER_C1_PIN, RIGHT_ENCODER_C2_PIN);
    encoderLeft.setCount(0);
    encoderRight.setCount(0);

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

    PID_Init(&pidLeft, KP, KI, KD, KI_MAX);
    PID_Init(&pidRight, KP, KI, KD, KI_MAX);
}

// Loop function
void loop() {
    RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
    
    encoderCountLeft = encoderLeft.getCount();
    encoderCountRight = encoderRight.getCount();

    calculateMeasuredMotorSpeed(encoderCountLeft, encoderCountRight);

    // Uncomment to enable PID control
    if (newWheelspeedMeasurementAvailable) {
        motorPwmDutyCycleLeft = motorPwmDutyCycleLeft + int( round( PID_Compute( &pidLeft, desiredMotorspeedLeft, measuredMotorspeedLeft   ) ) );
        motorPwmDutyCycleLeft =  constrain(motorPwmDutyCycleLeft , -255, 255 );
        motorPwmDutyCycleRight = motorPwmDutyCycleRight + int( round( PID_Compute(&pidRight, desiredMotorspeedRight, measuredMotorspeedRight ) ) );
        motorPwmDutyCycleRight = constrain(motorPwmDutyCycleRight , -255, 255 );
        newWheelspeedMeasurementAvailable = false;
    }

    setMotorPwm(MOTORDRIVER_IN1_PIN, MOTORDRIVER_IN2_PIN, MOTORDRIVER_ENB_PIN, motorPwmDutyCycleLeft);
    setMotorPwm(MOTORDRIVER_IN3_PIN, MOTORDRIVER_IN4_PIN, MOTORDRIVER_ENA_PIN, motorPwmDutyCycleRight);

#ifdef DEBUG_LOOPTIME
    // Get looptime data
    collectLooptime();
#endif
}
