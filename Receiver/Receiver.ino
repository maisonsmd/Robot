constexpr float ACCEL = 10000;          // steps/s^2, gia toc
constexpr float MICRO_STEP = 1000;      // vi buoc
constexpr float MAX_RPM = 700;          // max RPM
constexpr float ROTATE_ONLY_RPM = 100;  // toc do quay tai cho
constexpr float MIN_ROTATE_RATIO = 0.5; // 0.0 -> 1.0, ti le banh cham / banh nhanh khi vua tien vua queo

constexpr float MAX_V = MAX_RPM * MICRO_STEP / 60;// steps/s, van toc toi da
constexpr float ROTATE_ONLY_V = ROTATE_ONLY_RPM * MICRO_STEP / 60;

// dao chieu dong co?
constexpr bool INVERT_LEFT_DIR = true;
constexpr bool INVERT_RIGHT_DIR = false;

#include "LedFlasher.h"
#include "Button.h"

#define LOGGER
#define USE_DUMPER
#define UART_PACKET_SIZE 16
#define UART_PACKET_MIN_INTERVAL 16000
#include "DataPacker2.h"
#include "AsyncUart.h"

#include "Logger.h"
#include "Schedule.h"
#include "watchdog_reset.h"

#include "SingleStepper.h"

#define SCHEDULER_SOURCE millis()

constexpr uint8_t controller_id{ 0x01 };
constexpr uint8_t robot_id{ 0x02 };

constexpr uint8_t PIN_RELAY_MOTOR_POWER{ PB0 };
constexpr uint8_t PIN_RELAY_RED_LIGHT{ PA7 };
constexpr uint8_t PIN_RELAY_1{ PA6 };
constexpr uint8_t PIN_RELAY_2{ PA5 };
constexpr uint8_t PIN_MOTOR_ENABLE{ PA0 };
constexpr uint8_t PIN_LMOTOR_PUL{ PA4 };
constexpr uint8_t PIN_LMOTOR_DIR{ PA3 };
constexpr uint8_t PIN_RMOTOR_PUL{ PA2 };
constexpr uint8_t PIN_RMOTOR_DIR{ PA1 };

/* Led patterns */
uint32_t pled_disconnected[]{ 100, 400 };
#define PLED_DISCONNECTED pled_disconnected, (sizeof(pled_disconnected) / sizeof(uint32_t)), false
uint32_t pled_system[]{ 10, 140 };
#define PLED_SYSTEM pled_system, (sizeof(pled_system) / sizeof(uint32_t)), false

enum class ControlStyle {
    NONE,
    MOVE_AND_ROTATE,
    ROTATE_ONLY
};

ControlStyle control_style{ ControlStyle::NONE };

UartData control_packet;
uint8_t data_map[] = { 1, 1, 1, 1, 1, 1, 2, 2, 2 };
AsyncUart lora(&Serial1, robot_id);

bool sw_emergency{ true };
bool sw_enable{ false };
bool sw_relay_1{ false };
bool sw_relay_2{ false };
int16_t joystick_x{ 0 };
int16_t joystick_y{ 0 };
int16_t max_v_percent{ 0 };
LedFlasher led_system(PC13, LOW);

bool has_connection{ false };
uint32_t last_response_ms{ 0 };

uint32_t m_last_response{ 0 };
float max_velocity{ 0.0f };
float left_velocity{ 0.0f };
float right_velocity{ 0.0f };

//SingleStepper stepper_left( PIN_LMOTOR_PUL, PIN_LMOTOR_DIR, &Timer3 );
//SingleStepper stepper_right( PIN_RMOTOR_PUL, PIN_RMOTOR_DIR, &Timer4 );

single_stepper stepper_left(PIN_LMOTOR_PUL, PIN_LMOTOR_DIR, &Timer3);
single_stepper stepper_right(PIN_RMOTOR_PUL, PIN_RMOTOR_DIR, &Timer4);

float map_float(float x, float  in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void setup() {
    Serial.begin(115200);
    Serial.setTimeout(3);
    lora.begin(57600);

    // LORA MD0-1
    pinMode(PB15, OUTPUT);
    pinMode(PA8, OUTPUT);
    digitalWrite(PB15, LOW);
    digitalWrite(PA8, LOW);

    pinMode(PIN_RELAY_MOTOR_POWER, OUTPUT);
    pinMode(PIN_RELAY_RED_LIGHT, OUTPUT);
    pinMode(PIN_RELAY_1, OUTPUT);
    pinMode(PIN_RELAY_2, OUTPUT);
    pinMode(PIN_MOTOR_ENABLE, OUTPUT);

    led_system.setLoop(true);
    led_system.setRunning(true);
    led_system.setPattern(PLED_SYSTEM);

    // 600ms watchdog
    //iwdg_init(iwdg_prescaler::IWDG_PRE_256, 100);

#define motor_init_isr(motor) \
		motor.timer_on->pause();\
		motor.timer_on->attachInterrupt(0, []() {\
			motor.isr_on();\
		});\

    motor_init_isr(stepper_left);
    motor_init_isr(stepper_right);
    stepper_left.init(INVERT_LEFT_DIR);
    stepper_right.init(INVERT_RIGHT_DIR);

    Timer2.pause();
    Timer2.setPeriod(20);
    Timer2.attachInterrupt(0, []() {
        const uint32_t current_us = micros();
        stepper_left.isr_off(current_us);
        stepper_right.isr_off(current_us);
    });
    Timer2.resume();
    Timer2.refresh();

    stepper_left.set_accel(ACCEL);
    stepper_right.set_accel(ACCEL);
}

// #define TEST_COMMAND

void loop() {
    const uint32_t current_us = micros();
    //iwdg_feed();
    lora.update();
    led_system.update();

    stepper_left.update(current_us);
    stepper_right.update(current_us);

    stepper_left.isr_off(current_us);
    stepper_right.isr_off(current_us);

#ifndef TEST_COMMAND
    stepper_left.set_peak_velocity(left_velocity);
    stepper_right.set_peak_velocity(right_velocity);
#else
    if (Serial.available()) {
        char c = Serial.read();
        auto v = Serial.parseFloat();

        while (Serial.available()) {
            Serial.read();
        }

        INFOF("command: %c value %f", c, v);

        if (c == 'a') {
            stepper_left.set_accel(v);
            stepper_right.set_accel(v);
        }
        if (c == 'v') {
            stepper_left.set_peak_velocity(v);
            stepper_right.set_peak_velocity(v);
        }
    }
    DO_EVERY(1000) {
        INFOF("v %5.0f  %ld/%ld", stepper_left.current_velocity, stepper_left.current_step, stepper_left.temp_target_step);
    }

    return;
#endif
    // check connection
    if (millis() > 2000UL
        && millis() < last_response_ms + 2000UL) {
        if (!has_connection) {
            has_connection = true;
            INFO("CONTROLLER CONNECTED");
        }
    }
    else if (has_connection) {
        has_connection = false;
        INFO("CONTROLLER DISCONNECTED");

        //sw_emergency = true; // disable only, don't cut the power
        sw_enable = false;
        left_velocity = 0;
        right_velocity = 0;
        // TODO: fast stop motors
    }
    // read from buffer
    if (lora.available()) {
        last_response_ms = millis();
        control_packet = lora.read();
        control_packet.setSizeMap(data_map, sizeof(data_map));
        //INFOF("message from %d", control_packet.get<uint8_t>(0));

        uint8_t index = 2;
        sw_emergency = control_packet.get<bool>(index++);
        sw_enable = control_packet.get<bool>(index++);
        sw_relay_1 = control_packet.get<bool>(index++);
        sw_relay_2 = control_packet.get<bool>(index++);
        joystick_x = control_packet.get<int16_t>(index++);
        joystick_y = control_packet.get<int16_t>(index++);
        max_v_percent = control_packet.get<int16_t>(index++);

        max_velocity = float(max_v_percent) / 100.0f * MAX_V;
        const auto px = float(joystick_x) / 100.0f;
        const auto py = float(joystick_y) / 100.0f;

        if (sw_emergency) {
          max_velocity = 0;
          stepper_left.fast_stop();
          stepper_right.fast_stop();
        }
        
        // inplace rotate
        if (joystick_y == 0) {
            if (control_style == ControlStyle::NONE
                || control_style == ControlStyle::ROTATE_ONLY) {
                control_style = ControlStyle::ROTATE_ONLY;
                
                left_velocity = px * ROTATE_ONLY_V;
                right_velocity = -px * ROTATE_ONLY_V;
            }
            else {
                left_velocity = 0;
                right_velocity = 0;
            }
        }
        // move + rotate
        else {
            if (control_style == ControlStyle::NONE
                || control_style == ControlStyle::MOVE_AND_ROTATE) {
                control_style = ControlStyle::MOVE_AND_ROTATE;
                // rotate left?
                if (joystick_x < 0) {
                    right_velocity = py * max_velocity;
                    left_velocity = map_float(px, -1, 0, right_velocity * MIN_ROTATE_RATIO, right_velocity);
                }
                // rotate right
                else {
                    left_velocity = py * max_velocity;
                    right_velocity = map_float(px, 1, 0, left_velocity * MIN_ROTATE_RATIO, left_velocity);
                }
            }
        }
        if (joystick_x == 0 && joystick_y == 0) {
            left_velocity = 0;
            right_velocity = 0;
            control_style = ControlStyle::NONE;
        }
        //DEBUGF("left %ld right %ld", (long)left_velocity, (long)right_velocity);

        DO_EVERY(500) {
            UartData resp;
            resp.push(robot_id);
            resp.push(controller_id);

            lora.write(resp);

            DEBUGF("Em:%d En:%d R1:%d R2:%d x%d y%d max %d",
               sw_emergency,
               sw_enable,
               sw_relay_1,
               sw_relay_2,
               joystick_x, joystick_y, max_v_percent);
        }
    }

    digitalWrite(PIN_RELAY_RED_LIGHT, (has_connection && sw_enable) ? HIGH : LOW);

    // emergency stop?
    digitalWrite(PIN_RELAY_MOTOR_POWER, sw_emergency ? LOW : HIGH);
    // enable?
    digitalWrite(PIN_MOTOR_ENABLE, sw_enable ? LOW : HIGH);
    // 2 spare relays
    digitalWrite(PIN_RELAY_1, sw_relay_1 ? HIGH : LOW);
    digitalWrite(PIN_RELAY_2, sw_relay_2 ? HIGH : LOW);
}
