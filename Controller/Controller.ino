#define LOGGER
#define USE_DUMPER
#define UART_PACKET_SIZE 16
#define UART_PACKET_MIN_INTERVAL 16000

#define SCHEDULER_SOURCE millis()

#include "LedFlasher.h"
#include "Button.h"
#include "Joystick.h"

#include "DataPacker2.h"
#include "AsyncUart.h"

#include "Logger.h"
#include "Schedule.h"
#include "watchdog_reset.h"

constexpr uint8_t controller_id = 0x01;
constexpr uint8_t robot_id = 0x02;

/* Led patterns */
uint32_t pled_disconnected[]{ 100, 400 };
#define PLED_DISCONNECTED pled_disconnected, (sizeof(pled_disconnected) / sizeof(uint32_t)), false
uint32_t pled_system[]{ 10, 140 };
#define PLED_SYSTEM pled_system, (sizeof(pled_system) / sizeof(uint32_t)), false

uint32_t pled_error[]{ 50, 250, 50, 250, 50, 700 };
#define PLED_ERROR pled_error, (sizeof(pled_error) / sizeof(uint32_t)), false

uint8_t data_map[] = { 1, 1, 1, 1, 1, 1, 2, 2, 2 };

/* Inputs and outputs */
LedFlasher led_power(PB12, HIGH);
LedFlasher led_ready(PB13, HIGH);
LedFlasher led_connection(PB14, HIGH);
LedFlasher led_system(PC13, LOW);

Button sw_emergency(LOW);
Button sw_enable(LOW);
Button sw_relay_1(LOW);
Button sw_relay_2(LOW);

Joystick joystick(4095, 0.1f, PA5, PA1);
Joystick max_v(4095, 0.0f, PA3, 255);

UartData control_packet;
AsyncUart lora(&Serial1, controller_id);

uint32_t last_response_ms{ 0 };
bool has_connection{ false };
bool is_low_battery{ false };
float battery_voltage{ 8.4f };
bool is_enable_allowed{ false };

enum class RelayButtonAllowStatus {
  UNKNOWN,
  WAIT_RELEASE,
  ALLOWED
};

RelayButtonAllowStatus relay_btn_allow_status { RelayButtonAllowStatus::UNKNOWN };

int16_t _x{ 0 }, _y{ 0 }, _max_v{ 0 };

void setup() {
    Serial.begin(115200);
    Serial.setTimeout(3);
    lora.begin(57600);

    pinMode(PB15, OUTPUT);
    pinMode(PA8, OUTPUT);
    digitalWrite(PB15, LOW);
    digitalWrite(PA8, LOW);

    sw_emergency.attach(PB1, true);
    sw_enable.attach(PB0, true);
    sw_relay_1.attach(PA7, true);
    sw_relay_2.attach(PA6, true);

    led_power.setLoop(true);
    led_power.setRunning(true);
    led_power.on();

    led_ready.setLoop(true);
    led_ready.setRunning(true);
    led_ready.off();

    led_connection.setLoop(true);
    led_connection.setRunning(true);
    led_connection.setPattern(PLED_DISCONNECTED);

    led_system.setLoop(true);
    led_system.setRunning(true);
    led_system.setPattern(PLED_SYSTEM);

    const auto onRelayBtnDown = []() {
        if (!sw_relay_1.isTriggered() || !sw_relay_2.isTriggered()) {
            return;
        }
        
        if (is_enable_allowed){
            INFO("Already ALLOWED");
            return;
        }

        if (!sw_enable.isTriggered()) {
            INFO("ENABLE SW is not turned ON");
            return;
        }
            
        if (_x == 0 && _y == 0 && _max_v < 20) {
            INFO("ENABLE CONFIRMED!");
            is_enable_allowed = true;
            relay_btn_allow_status = RelayButtonAllowStatus::WAIT_RELEASE;
        }
        else {
            INFO("x, y, max_v INVALID!");
        }
    };

    const auto onRelayBtnUp = []() {
       if (!sw_relay_1.isTriggered() 
           && !sw_relay_2.isTriggered() 
           && relay_btn_allow_status == RelayButtonAllowStatus::WAIT_RELEASE) {
            DEBUG("RELAY ALLOWED!");
            relay_btn_allow_status = RelayButtonAllowStatus::ALLOWED;
       }
    };

    sw_relay_1.onSensorTriggered = onRelayBtnDown;
    sw_relay_2.onSensorTriggered = onRelayBtnDown;
    sw_relay_1.onSensorReleased = onRelayBtnUp;
    sw_relay_2.onSensorReleased = onRelayBtnUp;

    sw_enable.onSensorTriggered = []() {
        if (!is_low_battery)
            led_power.off();
        if (!is_enable_allowed) {
            INFO("ENABLED NOT ALLOWED");
        }
    };
    // 600ms watchdog
    //iwdg_init(iwdg_prescaler::IWDG_PRE_256, 100);
}

void loop() {
    //iwdg_feed();
    lora.update();

    // update states
    led_power.update();
    led_ready.update();
    led_connection.update();
    led_system.update();

    sw_emergency.update();
    sw_enable.update();
    sw_relay_1.update();
    sw_relay_2.update();

    joystick.update();
    max_v.update();

    _x = joystick.getX() * 100.0f;
    _y = joystick.getY() * 100.0f;
    _max_v = max_v.getX() * 50.0f + 50.0f;

    DO_EVERY(150) {
        float vadc = float(analogRead(PA0)) * 3.3f / 4095.0f;
        float vbatt = vadc * (10.0f + 22.0f) / 10.0f;
        battery_voltage = 0.8 * battery_voltage + 0.2 * vbatt;
        //DEBUGF("batt=%4.2fv", battery_voltage);
    }

    if (battery_voltage < 6) {
        if (!is_low_battery) {
            is_low_battery = true;
            led_power.setPattern(PLED_ERROR);
        }
    }/*
    else if (low_battery) {
      low_battery = false;
    }*/

    if (sw_enable.isTriggered()) {
        if (!is_low_battery)
            led_power.off();
            
        if (is_enable_allowed)
            led_ready.on();
        else
            led_ready.setPattern(PLED_ERROR);
    }
    else {
        if (!is_low_battery)
            led_power.on();
            
        led_ready.off();
    }

    if (millis() > 2000UL
        && millis() < last_response_ms + 2000UL) {
        if (!has_connection) {
            INFO("RECEIVER CONNECTED");
            led_connection.on();
            has_connection = true;
        }
    }
    else if (has_connection) {
        INFO("RECEIVER DISCONNECTED");
        has_connection = false;
        led_connection.setPattern(PLED_DISCONNECTED);
    }
    
     DO_EVERY(1000) {
       DEBUGF("Em:%d En:%d R1:%d R2:%d x%d y%d max %d",
         !sw_emergency.isTriggered(),
         sw_enable.isTriggered(),
         sw_relay_1.isTriggered(),
         sw_relay_2.isTriggered(),
         _x, _y, _max_v);
     } 

    auto _enabled = sw_enable.isTriggered();
    auto _relay_1 = sw_relay_1.isTriggered();
    auto _relay_2 = sw_relay_2.isTriggered();
    // _emergency: 0: released, 1: presed
    auto _emergency = !sw_emergency.isTriggered();
    
    if (!is_enable_allowed) {
      _x = 0;
      _y = 0;
      _max_v = 0;

      _relay_1 = false;
      _relay_2 = false;
      _enabled = false;
    }

    if (relay_btn_allow_status != RelayButtonAllowStatus::ALLOWED) {
      _relay_1 = false;
      _relay_2 = false;
    }

    control_packet.clear();
    control_packet.push(controller_id);
    control_packet.push(robot_id);
    control_packet.push(_emergency);
    control_packet.push(_enabled);
    control_packet.push(_relay_1);
    control_packet.push(_relay_2);
    control_packet.push(_x);
    control_packet.push(_y);
    control_packet.push(_max_v);

    DO_EVERY(100) {
        lora.write(control_packet);
        if (lora.available()) {
            control_packet = lora.read();
            control_packet.setSizeMap(data_map, sizeof(data_map));
            //INFOF("message from %d", control_packet.get<uint8_t>(0));
            last_response_ms = millis();
        }
    }
}
