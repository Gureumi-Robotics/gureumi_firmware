#include <Arduino.h>
#include <ArduinoJson.h>
#include <ESP32Encoder.h>
#include <cmath>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/string.h>
#include <rosidl_runtime_c/string_functions.h>
#include <rclc/executor.h>
// r 1322.2
// l 1318.6

struct Esp32Status {
    int esp32 = 0;
};

constexpr int M1A_25 = 25;
constexpr int M1B_26 = 26;
constexpr int M2A_14 = 14;
constexpr int M2B_27 = 27;
constexpr int PWM_OUTPUT = 150;
constexpr int PWM_MAX = 200;
int motor1_pwm = PWM_OUTPUT;
int motor2_pwm = PWM_OUTPUT;
int pwm_correction = 0;

double left_speed = 0;
double right_speed = 0;


ESP32Encoder rightEncoder;
ESP32Encoder leftEncoder;

long long previousRightEncoderCount = 0;
long long previousLeftEncoderCount = 0;
unsigned long previousSpeedTime = 0;

rcl_node_t esp32Node = rcl_get_zero_initialized_node();
rcl_publisher_t heartbeatPublisher = rcl_get_zero_initialized_publisher();
rcl_publisher_t motorStatusPublisher = rcl_get_zero_initialized_publisher();

rcl_subscription_t motorCommandSubscriber = rcl_get_zero_initialized_subscription();
std_msgs__msg__String motorCommandMessage;
String receivedMotorCommand;
rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();

std_msgs__msg__String heartbeatMessage;
std_msgs__msg__String motorStatusMessage;

rcl_allocator_t allocator = rcl_get_default_allocator();
rclc_support_t support;
unsigned long previousHeartbeatTime = 0;
unsigned long motor_commant_timer_ = 0;
Esp32Status esp_status;

void motor_command_callback(const void *message) {
    const auto *command = static_cast<const std_msgs__msg__String *>(message);
    receivedMotorCommand = command->data.data;

    if (receivedMotorCommand == "forward") {
        ledcWrite(M2B_27, 0);
        ledcWrite(M1A_25, 0);

        ledcWrite(M1B_26, constrain(motor1_pwm, 0, PWM_MAX));
        ledcWrite(M2A_14, constrain(motor2_pwm, 0, PWM_MAX));
    } else if (receivedMotorCommand == "stop") {
        pwm_correction = 0;

        motor1_pwm = PWM_OUTPUT;
        motor2_pwm = PWM_OUTPUT;
        ledcWrite(M1B_26, 0);
        ledcWrite(M2B_27, 0);
        ledcWrite(M1A_25, 0);
        ledcWrite(M2A_14, 0);
    }
    motor_commant_timer_ = millis();
}

void setup() {
    Serial.begin(115200);


    set_microros_serial_transports(Serial);
    rcl_ret_t supportResult = rclc_support_init(&support, 0, nullptr, &allocator);

    if (supportResult != RCL_RET_OK) {
        while (true) {
            delay(1000);
        }
    }

    rcl_ret_t nodeResult = rclc_node_init_default(&esp32Node, "esp32_status_node", "", &support);

    if (nodeResult != RCL_RET_OK) {
        while (true) {
            delay(1000);
        }
    }

    rcl_ret_t heartbeatPub_ = rclc_publisher_init_default(
        &heartbeatPublisher,
        &esp32Node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "/esp32/status"
    );

    if (heartbeatPub_ != RCL_RET_OK) {
        while (true) {
            delay(1000);
        }
    }

    rcl_ret_t motorStatusPub_ = rclc_publisher_init_default(
        &motorStatusPublisher,
        &esp32Node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "/motor/status"
    );

    if (motorStatusPub_ != RCL_RET_OK) {
        while (true) {
            delay(1000);
        }
    }

    rcl_ret_t subscriberResult = rclc_subscription_init_default(
        &motorCommandSubscriber,
        &esp32Node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "/robot/motor_command"
    );
    if (subscriberResult != RCL_RET_OK) {
        while (true) {
            delay(1000);
        }
    }

    static char motorCommandBuffer[64];
    motorCommandMessage.data.data = motorCommandBuffer;
    motorCommandMessage.data.size = 0;
    motorCommandMessage.data.capacity = sizeof(motorCommandBuffer);
    rcl_ret_t executorResult = rclc_executor_init(
        &executor, &support.context, 1, &allocator
    );
    if (executorResult != RCL_RET_OK) {
        while (true) {
            delay(1000);
        }
    }
    rcl_ret_t callbackResult = rclc_executor_add_subscription(
        &executor,
        &motorCommandSubscriber,
        &motorCommandMessage,
        &motor_command_callback,
        ON_NEW_DATA
    );

    if (callbackResult != RCL_RET_OK) {
        while (true) {
            delay(1000);
        }
    }

    ESP32Encoder::useInternalWeakPullResistors = puType::none;

    rightEncoder.attachFullQuad(32, 33);
    leftEncoder.attachFullQuad(18, 19);

    rightEncoder.clearCount();
    leftEncoder.clearCount();


    pinMode(M1A_25, OUTPUT);
    pinMode(M1B_26, OUTPUT);
    pinMode(M2A_14, OUTPUT);
    pinMode(M2B_27, OUTPUT);

    // 시작할 때 구동 신호가 나가지 않도록 두 핀을 LOW로 설정한다.
    digitalWrite(M1A_25, LOW);
    digitalWrite(M1B_26, LOW);
    digitalWrite(M2A_14, LOW);
    digitalWrite(M2B_27, LOW);

    // 두 핀의 출력을 PWM으로 조절할 수 있게 설정한다
    bool readyA = ledcAttach(M1A_25, 1000, 8);
    bool readyB = ledcAttach(M1B_26, 1000, 8);
    bool readyA2 = ledcAttach(M2A_14, 1000, 8);
    bool readyB2 = ledcAttach(M2B_27, 1000, 8);

    // 설정에 실패하면 구동 단계로 넘어가지 않는다.
    if (!readyA || !readyB || !readyA2 || !readyB2) {
        while (true) {
            delay(1000);
        }
    }
    // 처음에는 두 출력을 모두 0으로 둔다.
    ledcWrite(M1A_25, 0);
    ledcWrite(M1B_26, 0);
    ledcWrite(M2A_14, 0);
    ledcWrite(M2B_27, 0);

    // 초기값 encoder, time
    previousRightEncoderCount = rightEncoder.getCount();
    previousLeftEncoderCount = leftEncoder.getCount();
    previousSpeedTime = millis();
}

void loop() {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));


    unsigned long now = millis();
    if (now - previousHeartbeatTime >= 1000) {
        esp_status.esp32 = 1;

        JsonDocument status;
        status["esp32"] = esp_status.esp32;
        String statusJson;
        serializeJson(status, statusJson);
        if (!rosidl_runtime_c__String__assign(&heartbeatMessage.data, statusJson.c_str())) {
            return;
        }

        previousHeartbeatTime = now;
        rcl_ret_t publishResult = rcl_publish(&heartbeatPublisher, &heartbeatMessage, nullptr);

        JsonDocument motorStatus;
        motorStatus["left"]["speed_kmh"] = left_speed;
        motorStatus["left"]["encoder_count"] = leftEncoder.getCount();
        motorStatus["left"]["pwm"] = ledcRead(M1B_26);

        motorStatus["right"]["speed_kmh"] = right_speed;
        motorStatus["right"]["encoder_count"] = rightEncoder.getCount();
        motorStatus["right"]["pwm"] = ledcRead(M2A_14);

        String motorStatusJson;
        serializeJson(motorStatus, motorStatusJson);
        if (!rosidl_runtime_c__String__assign(&motorStatusMessage.data, motorStatusJson.c_str())) {
            return;
        }
        rcl_ret_t motorPublishResult = rcl_publish(&motorStatusPublisher, &motorStatusMessage, nullptr);
    }

    unsigned long elapsedMs = now - previousSpeedTime;
    // 0.1초 이상 지나면 코드 실행
    if (elapsedMs >= 100) {
        // 현재 카운트
        long long rightCount = rightEncoder.getCount();
        long long leftCount = leftEncoder.getCount();
        // n초 동안 몇 encoder 변화가 있었는지
        long long rightCountChange = rightCount - previousRightEncoderCount;
        long long leftCountChange = leftCount - previousLeftEncoderCount;

        double right_distanceM = (rightCountChange / 1347.0) * 0.2041;
        double left_distanceM = (leftCountChange / 1274.0) * 0.2041;


        double elapsedSeconds = elapsedMs / 1000.0;
        right_speed = std::fabs((right_distanceM / elapsedSeconds) * 3.6);
        left_speed = std::fabs((left_distanceM / elapsedSeconds) * 3.6);

        if (receivedMotorCommand == "forward") {
            const double speed_difference = left_speed - right_speed;
            if (std::fabs(speed_difference) > 0.03) {
                const int correction_step = 1;

                if (speed_difference > 0) {
                    pwm_correction += correction_step;
                } else {
                    pwm_correction -= correction_step;
                }
            }
            const int correction_limit = min(PWM_OUTPUT, PWM_MAX - PWM_OUTPUT);
            pwm_correction = constrain(
                pwm_correction, -correction_limit, correction_limit
            );
            motor1_pwm = PWM_OUTPUT - pwm_correction;
            motor2_pwm = PWM_OUTPUT + pwm_correction;
            ledcWrite(M1B_26, motor1_pwm);
            ledcWrite(M2A_14, motor2_pwm);
        }

        previousRightEncoderCount = rightCount;
        previousLeftEncoderCount = leftCount;
        previousSpeedTime = now;

        if (now - motor_commant_timer_ >= 300) {
            receivedMotorCommand = "stop";
            ledcWrite(M1A_25, 0);
            ledcWrite(M1B_26, 0);
            ledcWrite(M2A_14, 0);
            ledcWrite(M2B_27, 0);
        }


        // // 양쪽 바퀴 회전

        // delay(5000);

        // 두 출력을 모두 꺼서 5초 정지한다.
        // ledcWrite(M1A_25, 0);
        // ledcWrite(M1B_26, 0);
        // ledcWrite(M2A_14, 0);
        // ledcWrite(M2B_27, 0);
        // delay(5000);


        // // 뒤로 회전 양쪽
        // ledcWrite(M1A_25, 0);
        // ledcWrite(M2A_14, 0);
        // ledcWrite(M1B_26, PWM_OUTPUT);
        // ledcWrite(M2B_27, PWM_OUTPUT);
        // delay(5000);

        // 두 출력을 모두 꺼서 5초 정지한다.
        // ledcWrite(M1A_25, 0);
        // ledcWrite(M1B_26, 0);
        // ledcWrite(M2A_14, 0);
        // ledcWrite(M2B_27, 0);esp32_status_sub_
        // delay(5000);
    }
}
