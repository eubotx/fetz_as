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

// Robot parameters
constexpr double WHEEL_DIAMETER = 0.04;
constexpr double WHEEL_DISTANCE = 0.12;
constexpr double MAX_MOTOR_SPEED = 65;  // Max speed in RPS
constexpr int ENCODER_TICKS_PER_MOTOR_REVOLUTION = 12 * 2;
constexpr double GEARBOX_RATIO = 200.0;

// PID controller parameters
constexpr double KP = 3.0;
constexpr double KI = 0.5;
constexpr double KD = 0.0;
constexpr double KI_MAX = 30.0;

// Pin definitions
constexpr int LEFT_ENCODER_C1_PIN = 23;
constexpr int LEFT_ENCODER_C2_PIN = 22;
constexpr int RIGHT_ENCODER_C1_PIN = 19;
constexpr int RIGHT_ENCODER_C2_PIN = 18;
constexpr int MOTORCONTROLLER_IN1_PIN = 14;
constexpr int MOTORCONTROLLER_IN2_PIN = 27;
constexpr int MOTORCONTROLLER_IN3_PIN = 25;
constexpr int MOTORCONTROLLER_IN4_PIN = 26;

// Encoder class
class Encoder {
public:
    Encoder(int pinA, int pinB) {
        encoder.attachHalfQuad(pinA, pinB);
        encoder.setCount(0);
    }

    void update() {
        long currentCount = encoder.getCount();
        long delta = currentCount - lastCount;
        lastCount = currentCount;
        speed = ((double) delta / (double) ENCODER_TICKS_PER_MOTOR_REVOLUTION) / (micros() / 1000000.0);
    }

    double getSpeed() const {
        return speed;
    }

private:
    ESP32Encoder encoder;
    long lastCount = 0;
    double speed = 0;
};

// Motor class
class Motor {
public:
    Motor(int pin1, int pin2) : pin1(pin1), pin2(pin2) {
        pinMode(pin1, OUTPUT);
        pinMode(pin2, OUTPUT);
    }

    void setPWM(int dutyCycle) {
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

private:
    int pin1, pin2;
};

// PIDController class
class PIDController {
public:
    PIDController(double Kp, double Ki, double Kd, double KiMax) 
        : Kp(Kp), Ki(Ki), Kd(Kd), KiMax(KiMax), error(0), integral(0), derivative(0), previousError(0) {}

    double compute(double desiredValue, double measuredValue) {
        error = desiredValue - measuredValue;
        long deltaTime = (micros() - lastTime) / 1000000.0;
        integral += error * deltaTime;
        integral = constrain(integral, -KiMax, KiMax);  // Anti-windup
        derivative = error - previousError * deltaTime;
        previousError = error;
        lastTime = micros();

        return (Kp * error) + (Ki * integral) + (Kd * derivative);
    }

private:
    double Kp, Ki, Kd, KiMax;
    double error, integral, derivative, previousError;
    long lastTime = 0;
};

class Wheel {
public:
    Wheel(Encoder& encoder, Motor& motor, PIDController& pid, double gearboxRatio)
        : encoder(encoder), motor(motor), pid(pid), gearboxRatio(gearboxRatio) {}

    void updateMotorSpeed() {
        encoder.update(); // Update encoder readings
        motorPwm = int(round(pid.compute(desiredMotorSpeed, encoder.getSpeed()))); // Apply PID control
        motorPwm = constrain(motorPwm, -255, 255); // Constrain PWM within valid limits
        motor.setPWM(motorPwm); // Set motor speed based on PWM
    }

    void setMotorSpeed(double desiredMotorSpeed) {
        this->desiredMotorSpeed = desiredMotorSpeed; // Set motor speed directly
    }

    void setWheelSpeed(double desiredWheelSpeed) {
        this->desiredWheelSpeed = desiredWheelSpeed; // Set wheel speed
        // Convert wheel speed to motor speed using the gearbox ratio
        desiredMotorSpeed = desiredWheelSpeed * gearboxRatio;
    }

    double getDesiredMotorSpeed() const {
        return desiredMotorSpeed;
    }

    double getDesiredWheelSpeed() const {
        return desiredWheelSpeed;
    }

private:
    Encoder& encoder;
    Motor& motor;
    PIDController& pid;
    double gearboxRatio; // Gearbox ratio to convert wheel speed to motor speed
    double desiredMotorSpeed = 0; // Desired motor speed (RPS)
    double desiredWheelSpeed = 0; // Desired wheel speed (RPS)
    int motorPwm = 0; // Motor PWM duty cycle
};

// RobotControl Class
class RobotControl {
public:
    Wheel leftWheel;
    Wheel rightWheel;
    unsigned long lastUpdateTime = 0;
    double linearSpeed = 0;
    double angularSpeed = 0;

    RobotControl(PIDController pidLeft, PIDController pidRight)
        : leftWheel(LEFT_ENCODER_C1_PIN, LEFT_ENCODER_C2_PIN, MOTORCONTROLLER_IN1_PIN, MOTORCONTROLLER_IN2_PIN, pidLeft),
          rightWheel(RIGHT_ENCODER_C1_PIN, RIGHT_ENCODER_C2_PIN, MOTORCONTROLLER_IN3_PIN, MOTORCONTROLLER_IN4_PIN, pidRight) {}

    void calculateWheelSpeeds(float linear, float angular) {
        double linearWheelSpeed = (2.0 * linear) / WHEEL_DIAMETER;
        double angularWheelSpeed = (angular * WHEEL_DISTANCE) / (WHEEL_DIAMETER / 2.0);
        double linearMotorSpeed = linearWheelSpeed * GEARBOX_RATIO;
        double angularMotorSpeed = angularWheelSpeed * GEARBOX_RATIO;

        leftWheel.updateMotorSpeed(linearMotorSpeed - angularMotorSpeed, leftWheel.getMeasuredSpeed(leftWheel.encoder.getCount(), leftWheel.encoder.lastCount, millis() - lastUpdateTime), millis() - lastUpdateTime);
        rightWheel.updateMotorSpeed(linearMotorSpeed + angularMotorSpeed, rightWheel.getMeasuredSpeed(rightWheel.encoder.getCount(), rightWheel.encoder.lastCount, millis() - lastUpdateTime), millis() - lastUpdateTime);
    }

    void update() {
        unsigned long currentTime = millis();
        if (currentTime - lastUpdateTime >= 2) {
            lastUpdateTime = currentTime;
            // Update motor speeds based on control logic
        }
    }
};


void setup() {
    // Initialization code (setup micro-ROS, etc.)
    // For simplicity, assume that setup code for micro-ROS and other components is here
}

void loop() {
    // ROS executor loop
    // For simplicity, assume that ROS loop and timer callbacks are managed here
    
    // Update wheel speeds based on current encoder readings and PID control
    leftWheel.updateSpeed();
    rightWheel.updateSpeed();
}
