
//Joystick
bool is_armed = false;
bool buzzer_on = false;

uint16_t channel[16];
uint16_t lost_frames = 0;

int16_t thr_stick = 0;
int16_t yaw_stick = 0;
int16_t pitch_stick = 0;
int16_t roll_stick = 0;

uint8_t failsafe;
uint8_t vehicle_state = 0;

elapsedMillis last_joystick_frame_recvd;
elapsedMillis last_joystick_msg_publish;

Servo Motor1, Motor2, Motor3, Motor4;
SBUS Rx(Serial1);


//GPS
bool is_gps_fix = 0;

static NMEAGPS Gps;
static gps_fix Fix;

elapsedMillis last_gps_msg_publish;

NMEAGPS gps;
gps_fix fix;


//DAQ <-> Pi Comms
bool is_new_msg = false;
bool engage_failsafe = false;

uint8_t daq_serial_msg_type = 0;

char recvd_chars[msg_max_len];


//Functions
void HandleSerialMsg();
void HandleGpsMsg();
void ParseCompleteSerialMsg();
void SplitMsg(int16_t*, char*, uint8_t);
void EngageFailsafe();
void WriteMotors(int16_t,int16_t,int16_t,int16_t);
void StopMotors();
