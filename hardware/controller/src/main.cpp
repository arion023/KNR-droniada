#include <Arduino.h>
#include <common/mavlink.h>

void armMotors();
void disarmMotors();
void setMotorSpeed(uint8_t target_system, uint8_t target_component, float speed);
void delayWithHeartbeat(unsigned long delayTime);
void sendHeartbeat();
void setFlightMode(int mode);
void printAttitude() ;
void przyspieszanieSilnikami();
void MavRequestData();


HardwareSerial Serial2(PA3, PA2);

uint8_t sysid = 255;
uint8_t compid = 2;

uint8_t target_system = 1;
uint8_t target_component = 209;

unsigned long int previousMillis = 0L;
unsigned long int INTERVAL = 1000L;
int currentFlightMode = 0;

void setup() {
    Serial.begin(115200);
    Serial2.begin(115200);

    pinMode(PC13, OUTPUT);
}

void loop() {

unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= INTERVAL) {
    previousMillis= currentMillis; 

    setFlightMode(2);
    // if (currentFlightMode == 0) {
    //         currentFlightMode = 2; // Change to AltHold mode
    //         Serial.println("AltHold");
    //     } else {
    //         currentFlightMode = 0; // Change to Stabilize mode
    //         Serial.println("Stabilize");

    //     }
    //     setFlightMode(currentFlightMode);

    // MavRequestData();
    // sendHeartbeat();
    // printAttitude();
    przyspieszanieSilnikami();
  }
}

void setFlightMode(int mode) {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    // MAV_CMD_DO_SET_MODE: Set mode command, with custom mode for Guided
    mavlink_msg_set_mode_pack(sysid, compid, &msg, target_system, target_component, mode);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    Serial2.write(buf, len);
    Serial.println("Flight mode set");
    }

void armMotors() {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    mavlink_msg_command_long_pack(sysid, compid, &msg, target_system, target_component,
                                  MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    Serial2.write(buf, len);
    Serial.println("Motors armed");
}

void disarmMotors() {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    mavlink_msg_command_long_pack(sysid, compid, &msg, target_system, target_component,
                                  MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    Serial2.write(buf, len);
    Serial.println("Motors disarmed");
}

void setMotorSpeed(uint8_t target_system, uint8_t target_component, float speed) {

    if (currentFlightMode != 4){
        return;
    } else {
        mavlink_message_t msg;
        uint8_t buf[MAVLINK_MAX_PACKET_LEN];

        // Send SET_ACTUATOR_CONTROL_TARGET command to change motor speed
        float controls[8] = {0, 0, 0, 0, 0, 0, 0, 0}; // Initialize all controls to zero
        controls[0] = speed; // Assuming control channel 0 is for motor speed
        controls[1] = speed; // Assuming control channel 0 is for motor speed
        controls[2] = speed; // Assuming control channel 0 is for motor speed
        controls[3] = speed; // Assuming control channel 0 is for motor speed

        mavlink_msg_set_actuator_control_target_pack(sysid, compid, &msg, millis(), target_system, target_component, 0, controls);
        uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
        Serial2.write(buf, len);

        Serial.print("Motor speed set to: ");
        Serial.println(speed);
    }
}

void delayWithHeartbeat(unsigned long delayTime) {
    unsigned long start_time = millis();
    while (millis() - start_time < delayTime) {
        sendHeartbeat();
        delay(1000); // Adjust delay as needed for sending heartbeat
    }
}

void sendHeartbeat() {
    // Preparing heartbeat package
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    mavlink_msg_heartbeat_pack(sysid, compid, &msg, MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_GENERIC, MAV_MODE_PREFLIGHT, 0, MAV_STATE_ACTIVE);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

    Serial2.write(buf, len);
    Serial.println("Heartbeat sent");
}

void przyspieszanieSilnikami(){
    setFlightMode(4);
    delayWithHeartbeat(2000); // Poczekaj 2 sekundy na potwierdzenie zmiany trybu

    // Uzbrój drona
    armMotors();
    delayWithHeartbeat(5000); // Poczekaj 5 sekund

    // Zwiększ prędkość do 20% i poczekaj 5 sekund
    setMotorSpeed(target_system, target_component, 20.0);
    delayWithHeartbeat(5000);

    // Zwiększ prędkość do 30% i poczekaj 5 sekund
    setMotorSpeed(target_system, target_component, 30.0);
    delayWithHeartbeat(5000);

    // Zmniejsz prędkość do minimum i poczekaj 5 sekund
    setMotorSpeed(target_system, target_component, 0.0);
    delayWithHeartbeat(5000);

    // Rozbrój drona
    disarmMotors();
    delayWithHeartbeat(5000); // Poczekaj 5 sekund
}

void printAttitude() {
    mavlink_message_t msg;
    mavlink_status_t status;

    while (Serial2.available() > 0) {
        uint8_t c = Serial2.read();
        Serial.println(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status));
        if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
                Serial.println("Wchodze1");

            switch (status.parse_state) {
                case MAVLINK_PARSE_STATE_IDLE:
                    // Stan spoczynku, nie robimy nic
                    break;
                case MAVLINK_PARSE_STATE_GOT_STX:
                    // Otrzymaliśmy nagłówek, możemy przetwarzać wiadomość
                    break;
                case MAVLINK_PARSE_STATE_GOT_LENGTH:
                    // Otrzymaliśmy długość, możemy przetwarzać wiadomość
                    break;
                case MAVLINK_PARSE_STATE_GOT_PAYLOAD:
                    // Otrzymaliśmy ładunek, możemy przetwarzać wiadomość
                    break;
                case MAVLINK_PARSE_STATE_GOT_CRC1:
                    // Otrzymaliśmy pierwszy bajt sumy kontrolnej, możemy przetwarzać wiadomość
                    break;
                default:
                    break;
            }

            // Przetwarzaj tylko wiadomości o orientacji (ATTITUDE) lub wysokości (GLOBAL_POSITION_INT)
            if (msg.msgid == MAVLINK_MSG_ID_ATTITUDE || msg.msgid == MAVLINK_MSG_ID_GLOBAL_POSITION_INT) {
                mavlink_attitude_t attitude;
                mavlink_msg_attitude_decode(&msg, &attitude);

                float roll = attitude.roll * 180.0 / M_PI;   // Convert radians to degrees
                float pitch = attitude.pitch * 180.0 / M_PI; // Convert radians to degrees
                float yaw = attitude.yaw * 180.0 / M_PI;     // Convert radians to degrees

                Serial.print("Roll: ");
                Serial.print(roll);
                Serial.print(", Pitch: ");
                Serial.print(pitch);
                Serial.print(", Yaw: ");
                Serial.println(yaw);
            }
        }
    }
}

void MavRequestData()
{
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // To be setup according to the needed information to be requested from the Pixhawk
  const int  maxStreams = 2;
  const uint8_t MAVStreams[maxStreams] = {MAV_DATA_STREAM_EXTENDED_STATUS, MAV_DATA_STREAM_EXTRA1};
  const uint16_t MAVRates[maxStreams] = {0x02,0x05};
    
  for (int i=0; i < maxStreams; i++) {

    mavlink_msg_request_data_stream_pack(2, 200, &msg, 1, 0, MAVStreams[i], MAVRates[i], 1);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    Serial.write(buf, len);
  }
}