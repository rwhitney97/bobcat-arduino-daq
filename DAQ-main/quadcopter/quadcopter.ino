

#include <Servo.h>
#include <Streaming.h>
#include <SBUS2.h>
#include <NMEAGPS.h>

#include "D:/General/GitHub/bobcat-common/serial_comms/include/serial_comms/serial_msg_common.h"
#include "config.h"
#include "quadcopter.h"


void setup() {

  Serial.begin(115200);   // 0 : Degbug Terminal or Pi
  Rx.begin();             // 1 : Transmitter Radio
  Serial3.begin(9600);    // 3 : GPS Receiver (receive only)

  Motor1.attach(motor1_pin);
  Motor2.attach(motor2_pin);
  Motor3.attach(motor3_pin);
  Motor4.attach(motor4_pin);

  pinMode(arm_led_pin, OUTPUT);
  pinMode(buzzer_pin, OUTPUT);
  digitalWrite(buzzer_pin, HIGH);
  delay(250);
  digitalWrite(buzzer_pin, LOW);
  delay(250);

}

void loop() {

  HandleSerialMsg();
  HandleGpsMsg();

  //successful connection to TX
  if (Rx.read(&channel[0], &failsafe, &lost_frames)) {
    is_armed = map(channel[4], 172, 1811, 0, 1);
    buzzer_on = map(channel[5], 172, 1811, 0, 1);
    vehicle_state = map(channel[6], 172, 1811, 0, 2);   // switch A: 0 = manual   1 = acro    2 = angle
    last_joystick_frame_recvd = 0;

    if (is_armed) {
      digitalWrite(arm_led_pin, HIGH);
      thr_stick = map(channel[0], 172, 1811, 0, 1000);
      roll_stick = map(channel[1], 172, 1811, -500, 500);
      pitch_stick = map(channel[2], 172, 1811, -500, 500);
      yaw_stick = map(channel[3], 172, 1811, -500, 500);
    }
    else {
      digitalWrite(arm_led_pin, LOW);
      thr_stick = 0;
      roll_stick = 0;
      pitch_stick = 0;
      yaw_stick = 0;
    }

    if (buzzer_on)  digitalWrite(buzzer_pin, HIGH);
    else  digitalWrite(buzzer_pin, LOW);

  }
  //executes [joystick_timeout_ms] after losing connection to TX or if received DAQ_SERIAL_MSG::ENGAGE_FAILSAFE over Serial
  else if ((last_joystick_frame_recvd > joystick_timeout_ms) || engage_failsafe) {
    EngageFailsafe();
  }

}


void HandleSerialMsg() {

  //Receive
  static bool recv_in_progress = false;
  static uint8_t msg_idx = 0;
  char recvd_char;

  while (Serial.available() > 0 && is_new_msg == false)
  {
    recvd_char = Serial.read();
    if (recv_in_progress)
    {
      if (recvd_char != SERIAL_MSG_END_INDICATOR)
      {
        recvd_chars[msg_idx] = recvd_char;
        msg_idx++;

        if (msg_idx >= msg_max_len)
        {
          msg_idx = msg_max_len - 1;
        }
      }
      else
      {
        recvd_chars[msg_idx] = '\0'; // terminate the string
        recv_in_progress = false;
        msg_idx = 0;
        is_new_msg = true;
      }
    }

    else if (recvd_char == SERIAL_MSG_START_INDICATOR) {
      recv_in_progress = true;
    }
  }

  if (is_new_msg) {
    ParseCompleteSerialMsg();
    is_new_msg = false;
  }


  //Send
  char msg_payload[msg_max_len];
  if (last_gps_msg_publish > gps_publish_rate_ms) {
    //send gps data over Serial
    last_gps_msg_publish = 0;
  }
  if (last_joystick_msg_publish > joystick_publish_rate_ms) {
    //send stick data over Serial
    Serial << SERIAL_MSG_START_INDICATOR << uint8_t(DAQ_SERIAL_MSG::JOYSTICK_MSG) << "," << thr_stick << "," <<
           roll_stick << "," << pitch_stick << "," << yaw_stick << "," << is_armed << "," <<
           vehicle_state << SERIAL_MSG_END_INDICATOR << endl;
    last_joystick_msg_publish = 0;
  }
}


void ParseCompleteSerialMsg() {

  char * strtok_idx = 0; // this is used by strtok() as an index
  strtok_idx = strtok(recvd_chars, ",");
  daq_serial_msg_type = atoi(strtok_idx);
  uint8_t serial_msg_data_len;
  int16_t serial_msg_data[msg_max_len];
  
  switch (daq_serial_msg_type) {
    case uint8_t(DAQ_SERIAL_MSG::NOT_A_MSG) :
      break;

    case uint8_t(DAQ_SERIAL_MSG::PING) :
      Serial << SERIAL_MSG_START_INDICATOR << uint8_t(DAQ_SERIAL_MSG::PING) << SERIAL_MSG_END_INDICATOR << endl;
      break;

    case uint8_t(DAQ_SERIAL_MSG::ENGAGE_FAILSAFE) :
      engage_failsafe = 1;
      break;

    case uint8_t(DAQ_SERIAL_MSG::MOTOR_CMD) :
      serial_msg_data_len = uint8_t(DAQ_SERIAL_MSG_LEN::MOTOR_CMD);
      SplitMsg(serial_msg_data, strtok_idx, serial_msg_data_len);
      WriteMotors(serial_msg_data[0], serial_msg_data[1], serial_msg_data[2], serial_msg_data[3]);
      break;

    case uint8_t(DAQ_SERIAL_MSG::LED_CMD) :
      break;

    default:
      break;
  }




}


void SplitMsg(int16_t* data, char* strtok_idx, uint8_t serial_msg_data_len) {
  for (byte ii = 0; ii < serial_msg_data_len; ii++) {
    strtok_idx = strtok(NULL, ",");
    data[ii] = atoi(strtok_idx);
  }
}


void HandleGpsMsg() {
  while (Gps.available(Serial3)) {
    Fix = Gps.read();
    is_gps_fix = true;
  }
}


void EngageFailsafe() {
  //  while(1) {
  StopMotors();
  digitalWrite(buzzer_pin, HIGH);
  delay(250);
  digitalWrite(buzzer_pin, LOW);
  delay(750);
  Serial << "FAILSAFE" << endl;
  //  }
}

void WriteMotors(int16_t motor1_cmd, int16_t motor2_cmd, int16_t motor3_cmd, int16_t motor4_cmd) {
  Motor1.writeMicroseconds(motor1_cmd + joystick_mid);
  Motor2.writeMicroseconds(motor2_cmd + joystick_mid);
  Motor3.writeMicroseconds(motor3_cmd + joystick_mid);
  Motor4.writeMicroseconds(motor4_cmd + joystick_mid);
}

void StopMotors() {
  Motor1.writeMicroseconds(1000);
  Motor2.writeMicroseconds(1000);
  Motor3.writeMicroseconds(1000);
  Motor4.writeMicroseconds(1000);
}
