#include <Arduino.h>
#include <common/mavlink.h>
// #include <cmath>

// Definicja struktury danych od Raspi
struct Command {
  char type[3];
  float param1;
  float param2;
  float param3;
  uint8_t checksum;
};

// Obsluga mavlink
void MavRequestData();
void sendHeartbeat();
void sendCommandLong(uint16_t command, float param1, float param2, float param3, float param4, float param5, float param6, float param7);
void sendMavlinkMessage(mavlink_message_t* msg); 
void handleMessage(mavlink_message_t* msg);
void receiveMavlink();

// Podstawowe sterowanie FC
void mavSetMode(int flightMode);
void armMotors(bool arm);
void setAltitude(float altitude);
void setMotorSpeed(int motor_id, int speed);
void setSpeed(float vx, float vy, float vz);
void setYaw(float yaw_angle);
void takeoff(float alt);
void takeoffWithLoiter(float alt);
void mavDelay(int seconds);
void flyToGPS(double lat, double lon, float alt);
void reposition(double lat, double lon, float alt);

// Loty automatyczne i testy
void automat();
void testAutomat();
void ascendToAltitude(float target_altitude, float climb_rate);
void test1();
void armAndTakeoff(float target_altitude);

// Obsluga komunikacji z Raspi
void readFromRaspi();
float reverseFloat(const float inFloat);
void convertCommand(Command &cmd);

// Reszta funkcji
void clearSerialBuffer();
void convertLocalToGPS(double lat, double lon, double x, double y, double& new_lat, double& new_lon); 
void eulerToQuaternion(float roll, float pitch, float yaw, float quaternion[4]);
double radToDeg(double radians);
double convertYaw(double angle);

// Deklaracja pinów portów szeregowych
HardwareSerial mavlinkSerial(PA3, PA2); // RX TX
HardwareSerial raspiSerial(PB7, PB6); // RX TX || WHITE YELLOW

unsigned long int previousMillis = 0L;
unsigned long int INTERVAL = 1000L;

// Konfiguracja MAVLINK
uint8_t system_type = MAV_TYPE_GENERIC;
uint8_t autopilot_type = MAV_AUTOPILOT_INVALID;
uint8_t system_mode = MAV_MODE_PREFLIGHT;
uint32_t custom_mode = 0;
uint8_t system_state = MAV_STATE_STANDBY;
uint8_t sysid = 255;
uint8_t compid = 2;
uint8_t type = MAV_TYPE_QUADROTOR;
uint8_t target_system = 1;
uint8_t target_component = 0;

// Reszta zmiennych konfiguracyjnych oraz stałych
String receivedData = "";
float alt = 0.0; 
float roll = 0.0;
float pitch = 0.0;
float yaw = 0.0;
const double EARTH_RADIUS = 6378137.0; 
double current_lat = 0.0; 
double current_lon = 0.0; 
bool missionState;

// const double PI = 3.14159265358979323846;
#define PI 3.1415926535897932384626433832795

// Definicje masek
#define MAVLINK_MSG_SET_ATTITUDE_TARGET_IGNORE_ROLL_RATE 0x01
#define MAVLINK_MSG_SET_ATTITUDE_TARGET_IGNORE_PITCH_RATE 0x02
#define MAVLINK_MSG_SET_ATTITUDE_TARGET_IGNORE_YAW_RATE 0x04
#define MAVLINK_MSG_SET_ATTITUDE_TARGET_IGNORE_THRUST 0x08

void setup() {
  Serial.begin(57600);
  mavlinkSerial.begin(57600);
  raspiSerial.begin(57600, SERIAL_8N1);

  pinMode(PC13, OUTPUT);
  clearSerialBuffer();
  mavSetMode(0);
  delay(10000);
}

void loop() {
    static bool test1Initiated = false;
     bool missionState = true;

    while(missionState){
      unsigned long currentMillis = millis();
      if (currentMillis - previousMillis >= INTERVAL) {
        previousMillis = currentMillis;
        
        Serial.println(raspiSerial.available());

        // Samoczynne podlecenie drona po uruchomieniu zasilania
        // if (!test1Initiated) {
        //     armAndTakeoff(5.0);
        //     test1Initiated = true;
        //     mavDelay(1000);
        // }

        // MavRequestData();
        // receiveMavlink();

        // armAndTakeoff(5);
        // readFromRaspi();

      // Serial.println(raspiSerial.available());
    
      // Po otrzymaniu komendy 'FIN' 
      // missionState = false;
      // }
      // armAndTakeoff(5.0);

      // mavSetMode(4);
      // armMotors(1);
      // reposition(current_lat,current_lon,2);  
      // delay(4000);
      // reposition(current_lat,current_lon,0);    
      // reposition(current_lat,current_lon,0);    
      test1();
  
      // test1();

    }
    // mavSetMode(6);
}
}

//######################################################################//
//                PODSTAWOWE FUNKCJE DO STEROWANIA DRONEM
//######################################################################//

// Funkcja wysyłająca polecenie do ustawiania predkosci drona
void setSpeed(float vx, float vy, float vz) {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Wypełnienie wiadomości MAVLink
  mavlink_msg_set_position_target_local_ned_pack(
      sysid,
      compid,
      &msg,
      0, // czas w milisekundach od rozpoczęcia misji
      target_system,
      target_component,
      MAV_FRAME_LOCAL_NED,
      0b0000111111000111, // Maska ignorująca wszystkie inne ustawienia oprócz prędkości
      0, 0, 0,            // x, y, z (nie używane)
      vx, vy, -vz,  // prędkości w kierunku x, y, z
      0, 0, 0,            // przyspieszenia w kierunku x, y, z (nie używane)
      0, 0                // yaw, yaw_rate (nie używane)
  );

  // Serializacja wiadomości do bufora
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

  // Wysłanie wiadomości przez port szeregowy
  mavlinkSerial.write(buf, len);
}

void reposition(double lat, double lon, float alt){
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  mavlink_msg_command_long_pack(
  sysid, compid, &msg, target_system, target_component, MAV_CMD_DO_REPOSITION, 0, -1, 0, 0, 0, lat, lon, alt);

  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  mavlinkSerial.write(buf, len);
  Serial.println("Delay");
}

void mavDelay(int seconds){
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  mavlink_msg_command_long_pack(
  sysid, compid, &msg, target_system, target_component, MAV_CMD_NAV_DELAY, 0, 0, 0, 0, seconds, 0, 0, 0);

  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  mavlinkSerial.write(buf, len);
  Serial.println("Delay");
}

void mavSetMode(int flightMode) {

  // TRYBY LOTU //
  // 0 -> Stabilize
  // 2 -> Acro
  // 4 -> Guided
  // 5 -> Loiter
  // 6 -> RTL

  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  mavlink_msg_set_mode_pack(sysid, compid, &msg, 1, 209, flightMode);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  mavlinkSerial.write(buf, len);
  Serial.println("Tryb lotu zmieniony");
}

void armMotors(bool arm) {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  mavlink_msg_command_long_pack(
    sysid, compid, &msg, target_system, target_component, MAV_CMD_COMPONENT_ARM_DISARM, 0, arm, 0, 0, 0, 0, 0, 0
  );

  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  mavlinkSerial.write(buf, len);
  Serial.println(arm ? "Motors armed" : "Motors disarmed");
}

void setAltitude(float altitude) {
  mavlink_message_t msg;
  mavlink_msg_set_position_target_global_int_pack(sysid, compid, &msg, millis(), target_system, target_component, MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                                                   0b0000111111111000, 0, 0, altitude , 0, 0, 0, 0, 0, 0, 0, 0);
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  mavlinkSerial.write(buf, len);
}

void setMotorSpeed(int motor_id, int speed) {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Assuming motor_id starts from 1 for the first motor
    mavlink_msg_rc_channels_override_pack(sysid, compid, &msg, target_system, target_component,
                                        motor_id == 1 ? speed : UINT16_MAX,
                                        motor_id == 2 ? speed : UINT16_MAX,
                                        motor_id == 3 ? speed : UINT16_MAX,
                                        motor_id == 4 ? speed : UINT16_MAX,
                                        UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX, // Remaining channels set to neutral (UINT16_MAX)
                                        UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX);

  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  mavlinkSerial.write(buf, len);
  Serial.print("Set motor ");
  Serial.print(motor_id);
  Serial.print(" speed to ");
  Serial.println(speed);
}

void flyToGPS(double lat, double lon, float alt) {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Ustawienie waypoint
  mavlink_msg_command_long_pack(sysid, compid, &msg, target_system, target_component,
                                MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, lat, lon, alt);

  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  mavlinkSerial.write(buf, len);

  Serial.print("Flying to Latitude: ");
  Serial.print(lat, 8);
  Serial.print(" Longitude: ");
  Serial.print(lon, 8);
  Serial.print(" Altitude: ");
  Serial.println(alt, 2);
}

void takeoff(float alt){
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Ustawienie waypoint
  mavlink_msg_command_long_pack(sysid, compid, &msg, target_system, target_component,
                                MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, alt);

  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  mavlinkSerial.write(buf, len);
}

void takeoffWithLoiter(float alt){
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Ustawienie waypoint
  mavlink_msg_command_long_pack(sysid, compid, &msg, target_system, target_component,
                                MAV_CMD_NAV_LOITER_TO_ALT, 0, 1, 0, 0, 0, 0, 0, alt);

  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  mavlinkSerial.write(buf, len);
}

void setYaw(float yaw_angle) {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Przetworzenie kąta na radiany
  float yaw = radians(yaw_angle);

  // Ustawienia bitowe dla maski ignorującej inne ustawienia
  uint16_t type_mask = MAVLINK_MSG_SET_ATTITUDE_TARGET_IGNORE_ROLL_RATE |
                       MAVLINK_MSG_SET_ATTITUDE_TARGET_IGNORE_PITCH_RATE |
                       MAVLINK_MSG_SET_ATTITUDE_TARGET_IGNORE_THRUST;

  // Kąty w radianach
  float q[4];
  eulerToQuaternion(0, 0, yaw, q);

  // Wysyłanie wiadomości MAVLink SET_ATTITUDE_TARGET
  mavlink_msg_set_attitude_target_pack(sysid, compid, &msg, millis() / 1000, target_system, target_component, type_mask, q, 0, 0, 0, 0);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  mavlinkSerial.write(buf, len);
  Serial.print("Yaw angle set to: ");
  Serial.println(yaw_angle);
}

void precisionLand(double lat, double lon, float alt) {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Komenda do lądowania w aktualnych współrzędnych GPS z precyzyjnym lądowaniem
  mavlink_msg_command_long_pack(sysid, compid, &msg, target_system, target_component,
                                MAV_CMD_NAV_LAND, 0, 0, 1, NAN, NAN, lat, lon, alt);

  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  mavlinkSerial.write(buf, len);

  Serial.print("Landing at Latitude: ");
  Serial.print(lat, 8);
  Serial.print(" Longitude: ");
  Serial.print(lon, 8);
  Serial.print(" Altitude: ");
  Serial.println(alt, 2);
}

//######################################################################//
//                          AUTOMATYCZNY LOT I TESTY
//######################################################################//

void armAndTakeoff(float target_altitude) {

  armMotors(1);
  mavSetMode(4);

  // for (int i = 1; i <= 4; ++i) {
  //   setMotorSpeed(i, 1200); // 1400 to przykład 20% zakresu PWM
  // }
  // // mavDelay(2);
  // delay(2000);
  // for (int i = 1; i <= 4; ++i) {
  //   setMotorSpeed(i, 1300); // 1400 to przykład 20% zakresu PWM
  // }
  // // mavDelay(2);
  // delay(2000);
  // for (int i = 1; i <= 4; ++i) {
  //   setMotorSpeed(i, 1400); // 1400 to przykład 20% zakresu PWM
  // }
  // // mavDelay(2);
  // delay(2000);

  setSpeed(0,0,0.5);
  mavDelay(2);

  flyToGPS(current_lat, current_lon, 3);
   mavSetMode(3);

  // mavDelay(5);
  delay(2000);

  flyToGPS(current_lat, current_lon, 2);
  // mavDelay(5);
  delay(4000);


  mavSetMode(9);
  delay(5000);

  mavSetMode(4);
}


void test1() {
  armMotors(1);
  mavSetMode(5);

  for (int i = 1; i <= 4; ++i) {
    setMotorSpeed(i, 1200); // 1400 to przykład 20% zakresu PWM
  }
  // mavDelay(2);
  delay(2000);
  for (int i = 1; i <= 4; ++i) {
    setMotorSpeed(i, 1400); // 1400 to przykład 20% zakresu PWM
  }
  // mavDelay(2);
  delay(2000);

  for (int i = 1; i <= 4; ++i) {
    setMotorSpeed(i, 1500); // 1400 to przykład 20% zakresu PWM
  }
  delay(2000);


  setSpeed(0,0,-1);
  delay(2000);
  mavSetMode(2);
}

void testAutomat() {
  // Ustaw tryb Loiter (np. mode 5)
  mavSetMode(4);

  setSpeed(1.0,0,0);
  delay(1000);
  setSpeed(-1,0,0);
  delay(1000);


  mavSetMode(9);
}

void automat() {

  armMotors(1);
  delay(100);

  mavSetMode(5); // Mav mode loiter 
  delay(100);

  // Set motor speeds to 20% (assuming PWM range is 1000-2000)
  for (int i = 1; i <= 3; ++i) {
    setMotorSpeed(i, 1200); // 1200 is 20% of the 1000-2000 range
  }
  delay(3000);

    // Set motor speeds to 20% (assuming PWM range is 1000-2000)
  for (int i = 1; i <= 3; ++i) {
    setMotorSpeed(i, 1400); // 1200 is 20% of the 1000-2000 range
  }
  delay(3000);

  setAltitude(4.0);
  delay(10000);

  // Land the drone
  mavSetMode(6); // Mode 9 is typically RTL (Return to Launch) or you can use 6 for LAND
  delay(10000);

  // Disarm the drone after landing
  armMotors(0);
  while (true); // Stop execution
}

void ascendToAltitude(float target_altitude, float climb_rate) {
  while (alt < target_altitude) {
    setSpeed(0.0,0.0,1.0);
    delay(1000); // Czekaj chwilę przed kolejną aktualizacją
  }
  // Po osiągnięciu docelowej wysokości, ustaw tryb utrzymywania wysokości
  setAltitude(target_altitude);
}


//######################################################################//
//                       OBSŁUGA WIADOMOŚCI MAVLINK                     
//######################################################################//

void MavRequestData() {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    // Żądanie wiadomości o wysokości (ID 74)
    // Serial.println("Requesting altitude data stream...");
    mavlink_msg_request_data_stream_pack(sysid, compid, &msg, target_system, target_component, MAV_DATA_STREAM_POSITION, 0.2, 1);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    mavlinkSerial.write(buf, len);
    delay(100); // krótka przerwa między żądaniami

    // Żądanie wiadomości o kątach położenia przestrzennego (ID 30, 33)
    mavlink_msg_request_data_stream_pack(sysid, compid, &msg, target_system, target_component, MAV_DATA_STREAM_EXTRA1, 1, 1);
    len = mavlink_msg_to_send_buffer(buf, &msg);
    mavlinkSerial.write(buf, len);
    delay(100);

    // Żądanie wiadomości o statusie rozszerzonym
    // Serial.println("Requesting extended status data stream...");
    mavlink_msg_request_data_stream_pack(sysid, compid, &msg, target_system, target_component, MAV_DATA_STREAM_EXTENDED_STATUS, 1, 1);
    len = mavlink_msg_to_send_buffer(buf, &msg);
    mavlinkSerial.write(buf, len);
    delay(100);

    // Żądanie wiadomości o surowych danych sensorów
    // Serial.println("Requesting raw sensors data stream...");
    mavlink_msg_request_data_stream_pack(sysid, compid, &msg, target_system, target_component, MAV_DATA_STREAM_RAW_SENSORS, 1, 1);
    len = mavlink_msg_to_send_buffer(buf, &msg);
    mavlinkSerial.write(buf, len);
    delay(100);
}

void receiveMavlink() {
  mavlink_message_t msg;
  mavlink_status_t status;

  while (mavlinkSerial.available() > 0) {
    uint8_t c = mavlinkSerial.read();

    if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
      // Sprawdź, czy wiadomość ma ID 74, 30, lub 33
      if (msg.msgid == 74 || msg.msgid == 30 || msg.msgid == 33 || msg.msgid == 24) {
        Serial.print("Received message with ID: ");
        Serial.println(msg.msgid);
        handleMessage(&msg);
      }
    }
  }
}

void handleMessage(mavlink_message_t* msg) {
  switch (msg->msgid) {
    case 74: { // MAVLINK_MSG_ID_ALTITUDE
        // Serial.println("Handling altitude message...");
        mavlink_altitude_t altitude;
        mavlink_msg_altitude_decode(msg, &altitude);
        float alt = altitude.altitude_local/1000; // Wartość w metrach
        Serial.print("Current Altitude: ");
        Serial.println(alt, 4);
      }
      break;

    case 30: { // MAVLINK_MSG_ID_ATTITUDE
        mavlink_attitude_t attitude;
        mavlink_msg_attitude_decode(msg, &attitude);
        float roll = radToDeg(attitude.roll);// Skalowanie o 10
        float pitch = radToDeg(attitude.pitch); // Skalowanie o 10
        float yaw = convertYaw(radToDeg(attitude.yaw)); // Skalowanie o 10
        
        // Formatowanie i drukowanie danych
        Serial.print("Roll: ");
        Serial.print(roll, 2);
        Serial.println("°");

        Serial.print("Pitch: ");
        Serial.print(pitch, 2);
        Serial.println("°");

        Serial.print("Yaw: ");
        Serial.print(yaw, 2);
        Serial.println("°");
      }
      break;

    case 33: { // MAVLINK_MSG_ID_GLOBAL_POSITION_INT
        mavlink_global_position_int_t global_position;
        mavlink_msg_global_position_int_decode(msg, &global_position);
        float current_lat = global_position.lat; // Wartość w milimetrach
        float current_lon = global_position.lon;  // Wartość w milimetrach
        // Przekonwertowane na readable        
        float convLat = current_lat/10000000;
        float convLon = current_lon/10000000;
        float altitude = global_position.relative_alt/1000;
        Serial.print("Latitude: ");
        Serial.println(convLat, 6);
        Serial.print("Longitude: ");
        Serial.println(convLon, 6);
        Serial.print("Altitude: ");
        Serial.println(altitude, 4);
      }
      break;

      case 24: { // MAVLINK_MSG_ID_GPS_RAW_INT
            // Serial.println("Handling GPS raw data message...");
            mavlink_gps_raw_int_t gps_raw;
            mavlink_msg_gps_raw_int_decode(msg, &gps_raw);
            float alt = gps_raw.alt / 1000.0; // Wartość w metrach
            Serial.print("GPS Altitude: ");
            Serial.println(alt, 4);
        }
        break;
  }
}

void sendHeartbeat() {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  mavlink_msg_heartbeat_pack(sysid, compid, &msg, type, autopilot_type, system_mode, custom_mode, system_state);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  
  mavlinkSerial.write(buf, len);
  Serial.println("Heartbeat sent");
}

void sendCommandLong(uint16_t command, float param1, float param2, float param3, float param4, float param5, float param6, float param7) {
  mavlink_message_t msg;
  mavlink_msg_command_long_pack(sysid, compid, &msg, target_system, target_component, command, 0, param1, param2, param3, param4, param5, param6, param7);
  sendMavlinkMessage(&msg);
}

void sendMavlinkMessage(mavlink_message_t* msg) {
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t len = mavlink_msg_to_send_buffer(buf, msg);
  mavlinkSerial.write(buf, len);
}

//######################################################################//
//                     OBSŁUGA KOMUNIKACJI Z RASPI                    
//######################################################################//

// Funkcja do konwersji bajtów na floaty
void convertCommand(Command &cmd) {
  cmd.param1 = reverseFloat(cmd.param1);
  cmd.param2 = reverseFloat(cmd.param2);
  cmd.param3 = reverseFloat(cmd.param3);
}

// Funkcja do obliczania sumy kontrolnej
uint8_t calculateChecksum(const uint8_t *data, size_t length) {
  uint8_t checksum = 0;
  for (size_t i = 0; i < length; ++i) {
    checksum ^= data[i];
  }
  return checksum;
}

void readFromRaspi() {
  static uint8_t buffer[sizeof(Command) + 1]; // +1 na znak '@'
  static size_t buffer_index = 0;

  while (raspiSerial.available() > 0) {
    char c = raspiSerial.read();
    buffer[buffer_index++] = c;

    // Sprawdź, czy doszliśmy do końca komendy (znak '@')
    if (c == '@') {
      if (buffer_index == sizeof(Command) + 1) {
        Command cmd;
        memcpy(&cmd, buffer, sizeof(Command));

        // Sprawdzenie sumy kontrolnej
        uint8_t receivedChecksum = cmd.checksum;
        cmd.checksum = 0; // Ustawienie na 0, aby obliczyć sumę kontrolną z danych
        uint8_t calculatedChecksum = calculateChecksum((uint8_t*)&cmd, sizeof(Command));

        if (1) { //receivedChecksum == calculatedChecksum
          convertCommand(cmd);
          Serial.print("Received command: ");
          Serial.write(cmd.type, 3);
          Serial.print(" with parameters: ");
          Serial.print(cmd.param1);
          Serial.print(", ");
          Serial.print(cmd.param2);
          Serial.print(", ");
          Serial.println(cmd.param3);

          switch (cmd.type[0]) {
            case 'D': // DST
              if (strncmp(cmd.type, "DST", 3) == 0) {
                double new_lat, new_lon;
                convertLocalToGPS(current_lat, current_lon, cmd.param1, cmd.param2, new_lat, new_lon);
                flyToGPS(new_lat, new_lon, cmd.param3); // Użyj przekonwertowanych współrzędnych GPS
                
              }
              break;

            case 'V': // VEL
              if (strncmp(cmd.type, "VEL", 3) == 0) {
                setSpeed(cmd.param1, cmd.param2, cmd.param3);
              }
              break;

            case 'R': // ROT
              if (strncmp(cmd.type, "ROT", 3) == 0) {
                setYaw(cmd.param1);
              }
              break;

            case 'L': // LND
              if (strncmp(cmd.type, "LND", 3) == 0) {
                precisionLand(current_lat, current_lon, 0.0); // Lądowanie w aktualnych współrzędnych GPS
                Serial.println("STR");

              }
              break;

            case 'S': // STR
              if (strncmp(cmd.type, "STR", 3) == 0) {
                armMotors(true);
                Serial.println("STR");
              }
              break;

            default:
              raspiSerial.print("UNK@");
              break;
          }

          // Wysyłanie ACK, aby potwierdzić odebranie komendy
          raspiSerial.print("ACK@");
        } else {
          raspiSerial.print("CHKERR@");
        }
      } else {
        raspiSerial.print("ERR@");
      }

      // Zresetuj bufor
      buffer_index = 0;
    }

    // Ochrona przed przepełnieniem bufora
    if (buffer_index >= sizeof(Command) + 1) {
      buffer_index = 0;
    }
  }
}

float reverseFloat(const float inFloat) {
  float retVal;
  char *floatToConvert = (char*)&inFloat;
  char *returnFloat = (char*)&retVal;

  // Reverse the byte order
  returnFloat[0] = floatToConvert[3];
  returnFloat[1] = floatToConvert[2];
  returnFloat[2] = floatToConvert[1];
  returnFloat[3] = floatToConvert[0];

  return retVal;
  }

//######################################################################//
//                             RESZTA FUNKCJI
//######################################################################//

void clearSerialBuffer() {
  while (mavlinkSerial.available() > 0) {
    mavlinkSerial.read();
    raspiSerial.read();
  }
}

void convertLocalToGPS(double lat, double lon, double x, double y, double& new_lat, double& new_lon) {
  // Konwersja lokalnych współrzędnych na zmiany szerokości i długości geograficznej
  double dLat = y / EARTH_RADIUS;
  double dLon = x / (EARTH_RADIUS * cos(M_PI * lat / 180.0));

  // Nowa szerokość i długość geograficzna
  new_lat = lat + (dLat * 180.0 / M_PI);
  new_lon = lon + (dLon * 180.0 / M_PI);
}

void eulerToQuaternion(float roll, float pitch, float yaw, float quaternion[4]) {
  float cosRoll = cos(roll * 0.5);
  float sinRoll = sin(roll * 0.5);
  float cosPitch = cos(pitch * 0.5);
  float sinPitch = sin(pitch * 0.5);
  float cosYaw = cos(yaw * 0.5);
  float sinYaw = sin(yaw * 0.5);

  quaternion[0] = cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw;
  quaternion[1] = sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw;
  quaternion[2] = cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw;
  quaternion[3] = cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw;
}

double radToDeg(double radians){
    return radians * (180.0 / PI);
}

double convertYaw(double angle) {
    // Normalizacja kąta do zakresu [0, 360)
    double newAngle = fmod((angle + 360.0), 360.0);
    if (newAngle < 0) {
        newAngle += 360.0;
    }
    return newAngle;
}

// TESTOWE KOMENDY DO PORUSZANIA

// 'DST', 0, 0, 5, 0, @