#include <Button2.h>
#include <HortusRotary.h>
#include <HortusWifi.h>

#define DEBUG 1
#define EN_PIN1 26 // enable (CFG6)
#define DIR_PIN1 33 // direction
#define STEP_PIN1 25 // step
#define ACTIVE1 0

#define EN_PIN2 16 // enable (CFG6)
#define DIR_PIN2 5 // direction
#define STEP_PIN2 17 // step
#define ACTIVE2 4

#define BUTTONPIN1 27
#define BUTTONPIN2 2
#define ROTARY1_PIN1 14
#define ROTARY1_PIN2 12
#define ROTARY2_PIN1 15
#define ROTARY2_PIN2 13

#define MAX_SPEED 5000
#define MIN_SPEED 200
#define DEF_SPEED 1000
#define STEPVALUE 100

int debounce = 50;
int button_state = 0;
int button2_state = 0;

Button2 button1, button2;
HortusRotary encoder1(ROTARY1_PIN1, ROTARY1_PIN2, HortusRotary::LatchMode::FOUR3);
HortusRotary encoder2(ROTARY2_PIN1, ROTARY2_PIN2, HortusRotary::LatchMode::FOUR3);

typedef struct {
    byte number;
    bool state;
    unsigned long last_time;
    int dur;
    int level;
    int step_pin;
} Motor;

Motor motor1 = { 1, false, 0, DEF_SPEED, LOW, STEP_PIN1 };
Motor motor2 = { 2, false, 0, DEF_SPEED, LOW, STEP_PIN2 };

String MOTOR_1_STATE = "/symphonia/1/state";
String MOTOR_2_STATE = "/symphonia/2/state";
String MOTOR_1_SPEED = "/symphonia/1/speed";
String MOTOR_2_SPEED = "/symphonia/2/speed";
String AWAKE = "/symphonia/awake";

void setup()
{

    Serial.begin(115200);

    // Hardware Interface section

    button1.begin(BUTTONPIN1);
    button2.begin(BUTTONPIN2);
    button1.setReleasedHandler(released);
    button2.setReleasedHandler(released);
    encoder1.setStepValue(STEPVALUE);
    encoder2.setStepValue(STEPVALUE);
    encoder1.setPosition(DEF_SPEED);
    encoder2.setPosition(DEF_SPEED);

    // Wifi and OSC section

    HortusWifi(HortusWifi::Connection::HORTUS, 40, AWAKE);
      
    OscWiFi.subscribe(HortusWifi::RECV_PORT, MOTOR_1_STATE,
        [](const OscMessage& m) {
            float _state = m.arg<float>(0);
            set_motor_state(&motor1, _state);
            
            _print("[ OSC RECEIVED ] ");
            _println(MOTOR_1_STATE);
            
            send_osc((String&)m.address(), _state);
        });

    OscWiFi.subscribe(HortusWifi::RECV_PORT, MOTOR_1_SPEED,
        [](const OscMessage& m) {
            float _speed = m.arg<float>(0);
            set_motor_speed(encoder1, _speed);

            _print("[ OSC RECEIVED ] ");
            _println(MOTOR_1_SPEED);
            
        });

    OscWiFi.subscribe(HortusWifi::RECV_PORT, MOTOR_2_STATE,
        [](const OscMessage& m) {
            float _state = m.arg<float>(0);
            set_motor_state(&motor2, _state);

            _print("[ OSC RECEIVED ] ");
            _println(MOTOR_2_STATE);
            
            send_osc((String&)m.address(), _state);
        });

    OscWiFi.subscribe(HortusWifi::RECV_PORT, MOTOR_2_SPEED,
        [](const OscMessage& m) {
            float _speed = m.arg<float>(0);
            set_motor_speed(encoder2, _speed);

            _print("[ OSC RECEIVED ] ");
            _println(MOTOR_2_SPEED);
        });

    pinMode(ACTIVE1, OUTPUT);
    digitalWrite(ACTIVE1, LOW);
    pinMode(ACTIVE2, OUTPUT);
    digitalWrite(ACTIVE2, LOW);

    delay(2000);

    // motor 1
    pinMode(EN_PIN1, OUTPUT);
    digitalWrite(EN_PIN1, HIGH); // deactivate driver (LOW active)
    pinMode(DIR_PIN1, OUTPUT);
    digitalWrite(DIR_PIN1, LOW); // LOW to CCW
    // digitalWrite(DIR_PIN1, HIGH); //HIGH to CW
    pinMode(STEP_PIN1, OUTPUT);
    digitalWrite(STEP_PIN1, LOW);
    digitalWrite(EN_PIN1, LOW); // activate driver

    // motor 2
    pinMode(EN_PIN2, OUTPUT);
    digitalWrite(EN_PIN2, HIGH); // deactivate driver (LOW active)
    pinMode(DIR_PIN2, OUTPUT);
    digitalWrite(DIR_PIN2, LOW); // LOW to CCW
    // digitalWrite(DIR_PIN, HIGH); //HIGH to CW
    pinMode(STEP_PIN2, OUTPUT);
    digitalWrite(STEP_PIN2, LOW);
    digitalWrite(EN_PIN2, LOW); // activate driver
}

void loop()
{
    OscWiFi.update();
    button1.loop();
    button2.loop();
    check_speed(&motor1, encoder1);
    check_speed(&motor2, encoder2);
    check_motor(&motor1);
    check_motor(&motor2);
}

void released(Button2& btn)
{
    if (btn == button1) {
        motor1.state = !motor1.state;
        send_osc(MOTOR_1_STATE, (int)(motor1.state));

        _print("[ BUTTON ACTION ] SWITCH STATE ON MOTOR 1: ");
        _println(String(motor1.state));
    } else if (btn == button2) {
        motor2.state = !motor2.state;
        send_osc(MOTOR_2_STATE, (int)(motor2.state));
        _print("[ BUTTON ACTION ] SWITCH STATE ON MOTOR 2: ");
        _println(String(motor2.state));
    }
}

void set_motor_state(Motor* m, float value)
{ 
    _print("[ SET ] MOTOR ");
    _print(String(m->number));
    _print(" STATE: ");
    
    if (value) {
        m->state = true;
        _println(String(1));
    }
    else {
        m->state = false;
        _println(String(0));
    }
}

void set_motor_speed(HortusRotary& _enc, float _spd)
{
    int _speed = (int)_spd / STEPVALUE * STEPVALUE;
    _speed = constrain(_speed, MIN_SPEED, MAX_SPEED);
    _enc.setPosition(_speed);
}

void check_motor(Motor* m)
{
    unsigned long _now = micros();

    if (m->state) {
        if ((_now - m->last_time) >= m->dur) {
            m->level = !(m->level);
            digitalWrite(m->step_pin, m->level);
            m->last_time = _now;
        }
    }
}

void check_speed(Motor* m, HortusRotary& _enc)
{
    _enc.tick();
    int newPos = _enc.getPosition();

    if (m->dur != newPos) {
        
        int _newPos = constrain(newPos, MIN_SPEED, MAX_SPEED);
        _enc.setPosition(_newPos);
        m->dur = _newPos;

        _print("[ NEW MOTOR DURATION ] MOTOR ");
        _print(String(m->number));
        _print(": ");
        _println(String(_newPos));
      
        if (m->number == 1)
            send_osc(MOTOR_1_SPEED, m->dur);
        else if (m->number == 2)
            send_osc(MOTOR_2_SPEED, m->dur);
    }
}

void send_osc(String& addr, int value) 
{
    OscWiFi.send(HortusWifi::HOST, HortusWifi::SEND_PORT, addr, value);
}

int _print(String s) {
  #if DEBUG
    Serial.print(s);
  #endif
  return 0;
}

int _println(String s) {
  #if DEBUG
    Serial.println(s);
  #endif
  return 0;
}
