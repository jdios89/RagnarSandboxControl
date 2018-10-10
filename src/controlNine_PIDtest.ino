#include <FlexCAN.h>
#include <SMC66Registers.h>
#include <Messenger.h> //Library for communicating with the computer
#include <PID_v1.h>
#include <CANsmc.h>
#include <Trigonometrics.h>
#include <math.h>
#include <Chrono.h>
#include <Ragnarkinematics.h>
#include <RagnarDynamics.h>
#include <Matrix.h>
#include <RMatrix.h>

#ifndef __MK66FX1M0__
#error "Teensy 3.6 with dual CAN bus is required to run this example"
#endif

FlexCAN CANbus1(1000000, 1, 1, 1);
FlexCAN CANbus0(1000000, 0, 1, 1);
Messenger myMessenger = Messenger();
Chrono chronofeedback; 
Chrono chronocontrol;

float floto; 
//static uint8_t hex[17] = "0123456789abcdef";
// Nodes id of the motors stored in variables and array
uint8_t nodesid[4] = {0x06, 0x07, 0x08, 0x09};
uint8_t nodeid1 = nodesid[0];
uint8_t nodeid2 = nodesid[1];
uint8_t nodeid3 = nodesid[2];
uint8_t nodeid4 = nodesid[3];
bool streampos = false; 
bool sinewavestart = false;
bool sinewavestartpos = false; 
bool computedtorque = false; 
float f = atan2f(2.0,1.0);

String inputString = "";         // a String to hold incoming data
boolean stringComplete = false;  // whether the string is complete
int32_t encoder_pos[4];
int32_t encoder_pos1;
int32_t encoder_pos2;
int32_t encoder_pos3;
int32_t encoder_pos4;
//uint16_t object_index_32 = 0x2012;
float r_current = 5; //run current in mA
float rb_current = 250; //base current
float sb_current = r_current * 0.2;
int32_t last_vel = 0;
///// Control variables
int32_t follow_error = 0;
double follow_errorr = 0;

double torque = 0;
double torque_f = 0;
unsigned long lt = millis();
unsigned long ltt = millis();
unsigned long last = millis();
unsigned long last2 = millis();
unsigned long lastt = micros();
int period = 20;
float position_offfset = 180000;
float position_rad = 0;
int flag = 0;
//const float J = 8.4e-5; // rotor inertia
const float J = 5.4e-2; // rotor inertia

bool impedance = false;
bool computed = false;
double destorque = 0;

double vspeed_rs = 0;
double velocity_counts = 0;
int period2 = 1000;
bool flago = true;
uint32_t velocity_mode = 0x00000001;
uint32_t position_mode = 0x00000002;
int32_t posiref = 180000;

int periodltt = 500;
int32_t passive_mode = 0;
int32_t last_position[] = {0, 0, 0, 0};

int32_t torqueper = 0;
int32_t enco_position = 0;
int32_t last_enco_position = 0;

uint32_t actualTorque = 0;
int32_t ffollow_error = 0;
uint32_t runcurrent = 0;
int32_t pist = 0;
int32_t pesol = 0;

int cc = 1;
bool bandera = true;
float  copy_actualTorque[] = {0, 0, 0, 0};
int32_t copy_ffollow_error = 0;
uint32_t copy_runcurrent = 0;
int32_t copy_enco_position = 0;
int32_t copy_pist = 0;
double copy_velocity_counts = 0;
unsigned long diftime = 0;
unsigned long lastTime = 0;
unsigned long tdiftime = 0;
unsigned long tlastTime = 0;
unsigned long controldt = 0;
double pref = position_offfset;
unsigned long ttiming = micros(); // variable for timing service reply in CAN service messages
bool stream_data = false;
bool flaggo = true;
int i = 0;
bool flagop = true;
bool zeroBool = false;
unsigned long lasto = micros();
int32_t last_pos = 180000;
double velo = 0;
double pos_err_I = 0;
unsigned long Ts = 0;
int control_dt = 10;
unsigned long lasttime_control = millis();
bool first = true;
bool skift = true;
unsigned long lastp = millis();
unsigned long change = millis();
bool elchange = false;
unsigned long controlllast = millis();
unsigned long posstream = millis(); 
bool motion_enable = false;
double Setpoint0, Input0, Output0 = 0;
double Setpoint1, Input1, Output1 = 0;
double Setpoint2, Input2, Output2 = 0;
double Setpoint3, Input3, Output3 = 0;
// kp 5 desired oscillations good
//double Kp = 8.0 * 0.00087, Ki = 0 * 0.00087, Kd = 0.03 * 0.00087;
double Kp = 8.0 * 0.00087, Ki = 1 * 0.00087, Kd = 0.06 * 0.00087;
//double Kp = 6.0 * 0.00087, Ki = 0.0 * 0.00087, Kd = 0.0 * 0.00087;

double Kp1 = 4.0 * 0.00087, Ki1 = 0.3 * 0.00087, Kd1 = 0.02 * 0.00087;
double Kp2 = 4.0 * 0.00087, Ki2 = 0.3 * 0.00087, Kd2 = 0.02 * 0.00087;
double Kp3 = 4.0 * 0.00087, Ki3 = 0.3 * 0.00087, Kd3 = 0.02 * 0.00087;


CANsmc SMCCAN(&CANbus1, &Serial, nodesid);

// PID PID0(&SMCCAN.encoder_posd0, &Output0, &Setpoint0, Kp, Ki, Kd, DIRECT);
// PID PID1(&SMCCAN.encoder_posd1, &Output1, &Setpoint1, Kp1, Ki1, Kd1, DIRECT);
// PID PID2(&SMCCAN.encoder_posd2, &Output2, &Setpoint2, Kp2, Ki2, Kd2, DIRECT);
// PID PID3(&SMCCAN.encoder_posd3, &Output3, &Setpoint3, Kp3, Ki3, Kd3, DIRECT);

float torquevar[1000]; 
bool started = false; 
int nodeintest = 0; 
float maivel = 5; 
float maivelval = 5; 
// Kinematic variables

float a = 280.0 / 1000.0;
float b = 114.0 / 1000.0;
float alpha = pi / 12;
float beta = pi / 4.0;
float l = 300.0 / 1000.0;
float L = 600.0 / 1000.0;
float r = 100.0 / 1000.0;
float gama = pi / 3.0;

float x[8] = {a, b, alpha, beta, l , L, r, gama};
float parameter[4][8]; 
float pose[6] = {0.0, 0.0, -0.4, 0.0, 0.0, 0.0};
float thetaf[4]; // stores joint positions 
float passivef[4][2]; // store passive joint positions 
float sc[4][6]; // store fixed angles cosines and sines 
float sctf[8]; // stores theta sines and cosines 
float scez[4][4]; // stores zeta eta sines cosines 
// Dynamic parameters   mp     Ip           mb     Ib    mL 
float massiparams[6] = {0.2, 0.0032, 0.0, 0.16, 0.0035, 0.126};

//** Control variables 
// error in task space 
float e[3] = {0.0, 0.0, 0.0};
// desired pos in task space
float x_d[3] = {0.0, 0.0, -0.35};
// error derivative
float de[3] = {0.0, 0.0, 0.0};

float di[3] = {0.0, 0.0, 0.0};
float diac[3] = {0.0, 0.0, 0.0};
// last error 
float e_l[3] = {0.0, 0.0, 0.0};
// kp
float kp_p = 12.0; 
float kp[3][3] = {{kp_p*4, 0.0, 0.0},
                 {0.0, kp_p*5, 0.0},
                 {0.0, 0.0, kp_p/2.0}};
// kd 
float kd_d = 1.0; 
float kd[3][3] = {{kd_d, 0.0, 0.0},
                  {0.0, kd_d, 0.0},
                  {0.0, 0.0, kd_d}};
float ki_i = 0.0; 
float ki[3][3] = {{ki_i, 0.0, 0.0},
                  {0.0, ki_i, 0.0},
                  {0.0, 0.0, ki_i}};

void setup() {
  // initialize serial:

  Serial.begin(250000);
  // reserve 200 bytes for the inputString:
  inputString.reserve(200);
  //if using enable pins on a transceiver they need to be set on
  pinMode(28, OUTPUT);
  pinMode(35, OUTPUT);

  digitalWrite(28, LOW);
  digitalWrite(35, LOW);
  delay(500);
  // compute the ragnar parameters 
  ragnarParams(x, &parameter);
  scfixed(parameter, &sc);

  Serial.println("Initialize");
  CANbus1.begin();
  delay(3000); // a small delay of 3000 ms to let the CANBUS initialize
  CAN_message_t inMsg;
  while (CANbus1.available()) { //Clear out the canbus
    CANbus1.read(inMsg);
    hexDumpAll(inMsg, &Serial);
  }

  Serial.println("CAN started"); // letting know the canbus is initialized
  for (int i = 0; i < 4; i++) SMCCAN.setPassiveMode(nodesid[i]);
  Serial.println("Set passive mode");
  delay(200);
  // Set the motor to passive mode and wait for the reply
  ttiming = micros();
  // for (int i = 0; i < 4; i++) SMCCAN.setPositionMode(nodesid[i]);
  // Serial.println("Set position mode");
  // homePosition(1500, 30); //current, velocity
  // Serial.println("Sent to home");
  delay(6000);
  Serial.print("CAN service timing microseconds: ");
  Serial.println(micros() - ttiming);
  myMessenger.attach(OnReceived);
  for (int i = 0; i < 4; i++) encoder_pos[i] = SMCCAN.getEncoderPosition(nodesid[i]);
  // Print encoder positions
  Serial.print("enc1: ");
  Serial.print(encoder_pos[0]);
  Serial.print(" enc2: ");
  Serial.print(encoder_pos[1]);
  Serial.print(" enc3: ");
  Serial.print(encoder_pos[2]);
  Serial.print(" enc4: ");
  Serial.println(encoder_pos[3]);
  while (CANbus1.available()) { //Clear out the canbus
    CANbus1.read(inMsg);
    hexDumpAll(inMsg, &Serial);
  }
  sb_current = 10;
  uint32_t curscaleinc = 17;
  curscaleinc = 25;
  curscaleinc = 80; 
  uint32_t curscaledec = 17;
  curscaledec = 25;
  //curscaledec = 5; 
  //  curscaleinc = 1;
    curscaledec = 80;
  uint32_t curscalefactor = 35;
  curscalefactor = 50;
  curscalefactor = 80; 
  // 8 8 4 is for torque analysis 4 works good
  // 8 8 2 is for torque analysis 5 works better

  // set current sacale factor

  for (int i = 0; i < 4; i++) {
    SMCCAN.writeToRegister(nodesid[i], CUR_SCALE_FACTOR, curscalefactor);
    SMCCAN.waitForReply(nodesid[i], CUR_SCALE_FACTOR, true);
  }
  Serial.print("time: ");
  Serial.println(micros() - ttiming);
  for (int i = 0; i < 4; i++) {
    // set current scale factor for increasing
    SMCCAN.writeToRegister(nodesid[i], CUR_SCALE_INC, curscaleinc);
    SMCCAN.waitForReply(nodesid[i], CUR_SCALE_INC, true);
  }
  Serial.print("time: ");
  Serial.println(micros() - ttiming);
  for (int i = 0; i < 4; i++) {
    // set current scale factor decreasing
    SMCCAN.writeToRegister(nodesid[i], CUR_SCALE_DEC, curscaledec);
    SMCCAN.waitForReply(nodesid[i], CUR_SCALE_DEC, true);
  }
  Serial.print("time: ");
  Serial.println(micros() - ttiming);
  // set a starting operational velocity
  uint32_t start_velocity = 0xA;
  start_velocity = 0x01;
  start_velocity = 0x00;
  start_velocity = 5000;
  start_velocity = 50000;
  start_velocity = 0x01;
  ttiming = micros();
  for (int i = 0; i < 4; i++) {
    SMCCAN.writeToRegister(nodesid[i], V_START, start_velocity);
    SMCCAN.waitForReply(nodesid[i], V_START, true);
  }
  start_velocity = 150 * 100; 
  for (int i = 0; i < 4; i++) {
    SMCCAN.writeToRegister(nodesid[i], V_SOLL, start_velocity);
    SMCCAN.waitForReply(nodesid[i], V_SOLL, true);
  }
  Serial.print("time: ");
  Serial.println(micros() - ttiming);

  // set a position window for correcting position
  uint32_t inposition = 200;
  ttiming = micros();
  for (int i = 0; i < 4; i++) {
    SMCCAN.writeToRegister(nodesid[i], IN_POSITION_WINDOW, inposition);
    SMCCAN.waitForReply(nodesid[i], IN_POSITION_WINDOW, true);
  }
  Serial.print("time: ");
  Serial.println(micros() - ttiming);

  // set a very high acceleration
  uint32_t asoll = 50000; //rpm/s
  asoll = 1000; 
  ttiming = micros();
  for (int i = 0; i < 4; i++) {
    SMCCAN.writeToRegister(nodesid[i], A_SOLL, asoll);
    SMCCAN.waitForReply(nodesid[i], A_SOLL, true);
  }
  Serial.print("time: ");
  Serial.println(micros() - ttiming);
  for (int i = 0; i < 4; i++) last_position[i] = SMCCAN.getEncoderPosition(nodesid[i]);
  ltt = millis();
  last = micros();

  for (int i = 0; i < 4; i++)  SMCCAN.setRunCurrent(nodesid[i], 400);
  for (int i = 0; i < 4; i++)  SMCCAN.setVelocityInt32(nodesid[i], 15000);
  //for (int i = 0; i < 4; i++)  SMCCAN.setPositionMode(nodesid[i]);
  lastt = micros();
  Serial.println("Setup done");
  for (int i = 0; i < 4; i++)  SMCCAN.setRunCurrent(nodesid[i], 700);
  for (int i = 0; i < 4; i++)  SMCCAN.setPassiveMode(nodesid[i]);
  for (int i = 0; i < 4; i++)  SMCCAN.setVelocityInt32(nodesid[i], 70000);
  for (int i = 0; i < 4; i++)  SMCCAN.setAsoll(nodesid[i], (int32_t) 50000);
  SMCCAN.setRunCurrent(nodesid[nodeintest], 500);

  //SEtup the PID test for arm 2
  Setpoint0 = 0;
  Setpoint1 = 0;
  Setpoint2 = 0;
  Setpoint3 = 0;
  SMCCAN.readVariables(); 
  // PID0.SetMode(AUTOMATIC);
  // PID1.SetMode(AUTOMATIC);
  // PID2.SetMode(AUTOMATIC);
  // PID3.SetMode(AUTOMATIC);
  // PID0.SetOutputLimits(-2000.0, 2000.0);
  // PID0.SetSampleTime(10);
  // PID1.SetOutputLimits(-2000.0, 2000.0);
  // PID1.SetSampleTime(10);
  // PID2.SetOutputLimits(-2000.0, 2000.0);
  // PID2.SetSampleTime(10);
  // PID3.SetOutputLimits(-2000.0, 2000.0);
  // PID3.SetSampleTime(10);

  for(int i=1; i<1000; i++) torquevar[i] = i; 
  //Serial.println(torquevar);
  SMCCAN.readVariables();
  SMCCAN.readVariables();
  
}
long time_torque = millis();
long time_incdec = millis();
bool flag_incdec = true;
float torquo = 0;
long time_telemetry = millis();
unsigned long timo = millis();
unsigned long tomi = millis();

float u2 = 0; 
float e2i = 0; 
float e2p = 0; 
unsigned long pidtime = millis(); 
bool fricstart = false; 
unsigned long timeskift = millis();
bool skifto = false; 
double torqueval = 300; 
int timeinterval = 500; 
int maxval = 600; 
int ter = 1; 
unsigned long intervaltime = millis(); 

bool ctvelstart = false; 

// vsoll = 200 asoll = 503 cur 1800 
// f = 0.2 
// float a0[] = {  284149  , -267771 ,  -123443 ,  -252363};
// float a1[] = {  -62765  ,      0   , 58428  ,      1};
// float b1[] = {  -17711   ,     0    ,14324   ,     0};
// float a2[] = {  677250   , 676847    ,597430  , 1047511};
// float b2[] = {  -1104237  , -1143829 ,  -1145347  ,  -875604};
// vsoll = 102 asoll = 118 cur 1800 
// f = 0.1
float a0[] = {   255228  , -286217 ,  -147082 ,  -257310};
float a1[] = {  258 ,  9012    ,  0    ,  0};
float b1[] = {   -1374  , -21584  ,     -1  ,      0};
float a2[] = {  368130 ,  455781 ,  450347  , 520503};
float b2[] = {   -566758  , -450655  , -504626 ,  -451269};
// int32_t q_maxs[] = { 674202 , 247868 , 715801 , 312403};
int32_t q_maxs[] = { 590253 , 307868 , 585800 , 302403};

//int32_t q_mins[] = {-283762 ,-676531, -298460, -695393};
int32_t q_mins[] = {-303762 ,-585802, -298460, -575893};



// vsoll = 150 asoll = 290 cur 1800 
// f = 0.15
// float a0[] = { 258547, -267783, -147441, -257311};
// float a1[] = { -2, 2, 1, 4};
// float b1[] = { 5, -5, 1, 2};
// float a2[] = { 622801, 644146, -499174, 881978};
// float b2[] = { -809196, -760801, -881920, 538429};
// f = 0.25 vsoll = 250 asoll = 783
// float a0[] = { 258544, -286881, -123606, -285066};
// float a1[] = { -21, 24926, -70394, 34992};
// float b1[] = { -7, -55259, -24737, -81221};
// float a2[] = { 984549, 1199749, 1010385, 1186241};
// float b2[] = { -1388212, -1058834, -1260446, -1121694};

// this are deprecated 
// f = 0.3 
//float a0[] = {  261692 ,  -147908 ,  -147908 ,  -147908};
//float a1[] = {     3728 ,  3728 ,  3728  , 3728};
//float b1[] = {  1211  , 1211  , 1211 ,  1211};
//float a2[] = { 605934 ,  605934 ,  605934 ,  605934};
//float b2[] = {  -833997  , -833996  , -833996 ,  -833996};
// f = 0.2 
// float a0[] = {  303596 ,  -244087 ,   -39287  , -244087};
// float a1[] = {  17788 ,  79444 ,  79443  , 79444};
// float b1[] = {  17660  , 23690  , 23690 ,  23690};
// float a2[] = {  329444  , 524682 ,  524682 ,  524682};
// float b2[] = {  -973596  , -810889 ,  -810890 ,  -810889}

// f = 0.1
//float a0[] = {  202329 ,  -290682 ,  -149173 ,  -287637};
//float a1[] = {  -18448 ,   -8153 ,     771 ,  -10531};
//float b1[] = {  70102 ,  31754  ,   214  , 41016};
//float a2[] = {  323431 ,  305999  , 422938 ,  313103};
//float b2[] = {  -516544 ,  -556605  , -531446 ,  -569531};
bool skip = false; 
float maitorval = 0; 
bool cttorquestart = false; 
bool cttorquestarto = false; 
float g = 9.805; 
float torquesf[4];
float qf[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
unsigned long _timelapsedcontrol = micros(), _timecontrol = micros(); 

void loop(){ 

  //tomi = micros();
  SMCCAN.readVariables();
  scth(SMCCAN.joints_position, &sctf);
  ragnarFKpassivef(&parameter, &SMCCAN.joints_position, &pose, &passivef, sc);
  for (int i=0; i<4; i++) qf[i] = SMCCAN.joints_position[i]; 
  for (int i=4; i<7; i++) qf[i] = pose[i-4]; 
  
  if(chronocontrol.hasPassed(7)){
  // if(false){
    _timelapsedcontrol = micros() - _timecontrol; 
    float dt = 0.01; 
    tomi = micros();
    if(computedtorque){

      float ddqf[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
      Matrix<float> mx(3,1,(float*)pose); 
      Matrix<float> mx_d(3,1,(float*)x_d);
      Matrix<float> me = mx_d - mx; 
      for(int i=0;i<3;i++){
        de[i] = (me._entity[i][0] - e_l[i]) / dt;
        e_l[i] = me._entity[i][0];
        di[i] = me._entity[i][0] * dt; 
        diac[i] = diac[i] + di[i]; 
      } 
      // float diode = expf(2.9); 
      Matrix<float> mde(3,1,(float*)de);
      Matrix<float> mdi(3,1,(float*)diac);
      
      Matrix<float> mkp(3,3,(float*)kp);
      Matrix<float> mkd(3,3,(float*)kd);
      Matrix<float> mki(3,3,(float*)ki);
      
      Matrix<float> mddx = mkp * me + mkd * mde + mki * mdi; 
      // mddx.show();
      for(int i=0; i<3; i++) ddqf[i+4] = mddx._entity[i][0]; 
      float impdnc[3]; 
      for(int i=0; i<3; i++) impdnc[i] = mddx._entity[i][0]; 
      
      float fakedtheta[4] = {0.0,0.0,0.0,0.0};
      float fakeddtheta[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
      ragnarTorquesfimp(qf, SMCCAN.joints_velocity, fakeddtheta, parameter, massiparams,
      //ragnarTorquesf(qf, SMCCAN.joints_velocity, ddqf, parameter, massiparams,
        sc, sctf, g, &torquesf, impdnc);
      for(int i=0; i<4; i++) torquesf[i] = torquesf[i] * TORQUE_TO_AMPS
                                           * 1000.0 / REDUCTION;
    }
    else
      for(int i=0; i<4; i++) torquesf[i] = 0.0; 
    
     for(int i=0; i<4; i++) SMCCAN.setTorque(nodesid[i], torquesf[i]);
    
    chronocontrol.restart(); 
    _timecontrol = micros();
  }

   if(chronofeedback.hasPassed(10)){
  //  if(false){
    
    // Matrix<float> mx_dd(1,3,(float*)x_d);
    // Matrix<float> mxx(1,3,(float*)pose); 
    // Matrix<float> mes = mx_dd - mxx;
    // Serial.print("error: ");
    // mes.show(); 
    // // Serial.print("qf: ");
    // Matrix<float> mde(1,7,(float*)qf);
    // mde.show(); 
    
      
    //Matrix<float> mpass(4, 2, (float*) passivef);
    //mpass.show();
    Serial.print("t-th-pos-vel-TmA-error: ");
    Serial.print(millis());
    // Serial.print(_timelapsedcontrol);
    Serial.print(" ");
    for(int i=0;i<4;i++) {
      Serial.print(SMCCAN.joints_position[i],4);
      Serial.print(" ");
    }
    for(int i=0;i<3;i++) {
      Serial.print(pose[i],4);
      Serial.print(" ");
    }
    for(int i=0;i<4;i++) {
      Serial.print(SMCCAN.joints_velocity[i],4);
      Serial.print(" ");
    }
    for(int i=0;i<4;i++) {
      Serial.print(torquesf[i],0);
      Serial.print(" ");
    }
    for(int i=0;i<3;i++) {
      Serial.print(x_d[i]-pose[i],4);
      Serial.print(" ");
    }
    
    

    //tomi = micros();
    // Serial.print(" ");1
    // Serial.print(SMCCAN.joints_velocity[0]);
    // Serial.print(" ");
    // Serial.print(SMCCAN.joints_velocity[1]);
    // Serial.print(" ");
    // Serial.print(SMCCAN.joints_velocity[2]);
    // Serial.print(" ");
    // Serial.print(SMCCAN.joints_velocity[3]);
    Serial.println();
    chronofeedback.restart();
  }   
  
  
  // setTorque receives the torque in mA
  /*
  PID0.Compute();
  PID1.Compute();
  PID2.Compute();
  PID3.Compute();
*/
  ////////pidCompute(); 
  //  Serial.println(micros() - tomi);

// 1 amp 600 f 1 
// 2 amp 1000 f 1 
// 3 amp 1000 f 0.5
// 4 amp 800 f 0.4
// 5 amp 600 f 0.45  

  if (cttorquestart) {
    if(!skifto) maivel = maitorval;
    else maivel = -maitorval;   
    if (SMCCAN.encoder_pos[nodeintest] > q_maxs[nodeintest]) { 
        skifto = true;
        SMCCAN.setTorque(nodesid[nodeintest], 0);
        skip = true; 
    }  
    else if (SMCCAN.encoder_pos[nodeintest] < q_mins[nodeintest]) { 
        skifto = false;      
        SMCCAN.setTorque(nodesid[nodeintest], 0);
        skip = true; 
    }
    SMCCAN.setTorque(nodesid[nodeintest], maivel);
    delay(1); 
  }
  if (cttorquestarto) {
    if(!skifto) maivel = maitorval;
    else maivel = -maitorval;   
    if (SMCCAN.encoder_pos[nodeintest] > q_maxs[nodeintest]) { 
        skifto = true;
        SMCCAN.setTorque(nodesid[nodeintest], 0);
        skip = true; 
    }  
    else if (SMCCAN.encoder_pos[nodeintest] < q_mins[nodeintest]) { 
        skifto = false;      
        SMCCAN.setTorque(nodesid[nodeintest], 0);
        skip = true; 
    }
    SMCCAN.setTorque(nodesid[nodeintest], maivel);
    delay(1); 
  }

  if (ctvelstart) {
    
    if(!skifto) maivel = maivelval;
    else maivel = -maivelval;   
    
    

    
      if (SMCCAN.encoder_pos[nodeintest] > q_maxs[nodeintest]) { 
        skifto = true;
        SMCCAN.setVelocityInt32(nodesid[nodeintest], 0); 
        skip = true; 
      }  
      else if (SMCCAN.encoder_pos[nodeintest] < q_mins[nodeintest]) { 
        skifto = false;      
        SMCCAN.setVelocityInt32(nodesid[nodeintest], 0);
        skip = true; 
      }
      SMCCAN.setVelocityInt32(nodesid[nodeintest], (int32_t) maivel * 100);
  } 
    

  if (sinewavestartpos) {
    float f = 0.1; 
    float omega = 2 * PI * f;
    //float omega = 2 * f;
    float to = micros()/1000000.0; 
    float amp = 501316; 
    int angle = (omega*to)*180/PI;
    //int angle = (omega*to)*180;  
   
    // int32_t postosend = amp*isinff(angle) + offset2;

    // This is the optimized trajectory to send 

    int32_t postosend = a0[nodeintest] + (a1[nodeintest]/omega)*isinff(angle) - (b1[nodeintest]/omega)*icosff(angle) + (a2[nodeintest]/(2*omega))*isinff(2*angle) - (b2[nodeintest]/(2*omega))*icosff(2*angle);
    Output2 = postosend; 
    //
    
    SMCCAN.setPSOLL(nodesid[nodeintest], postosend);
    delay(1); 
  }

  if (sinewavestart) {
    float f = 1.0; 
    //float omega = 2 * PI * f;
    float omega = 2 * f;
    float to = micros()/1000000.0; 
    float amp = 400; 
    //int angle = (omega*to)*180/PI;
    int angle = (omega*to)*180;  
    Output2 = amp*isinff(angle); 
  }


  if (fricstart){
    if(millis() - intervaltime > 15) {
      intervaltime = millis();
      ter++;
      if(!skifto) 
        Output2 = torquevar[ter];
      else
        Output2 = -torquevar[ter];
    }
    if (!skifto && millis()-timeskift > timeinterval){
      Output2 = torquevar[ter];
      timeskift = millis();
      skifto = true; 
      
      if(ter>maxval) ter = 1; 
    }
    if (skifto && millis() - timeskift > timeinterval) {
      Output2 = -torquevar[ter];
      timeskift = millis(); 
      skifto = false;
      
      if(ter>maxval) ter = 1;  
    }

  }
  else if(!fricstart && !sinewavestart ){  
    //Output2 = 0;
  } 
  

  if (!motion_enable){
  //  for (int i = 0; i < 4; i++) SMCCAN.setTorque(nodesid[i], 0);
  }
  if(motion_enable && !started){
    // SMCCAN.setPositionMode(nodesid[nodeintest]);
    // SMCCAN.setRunCurrent(nodesid[nodeintest], 1500); 
    started = true; 
  }
  if (millis() - time_torque > 1 && motion_enable) { //change the zero to ten afterwards
    if (!stream_data) {
      Serial.print("time: ");
      Serial.println(micros());
      timo = micros();
    }
    ///////SMCCAN.setTorque(nodesid[nodeintest], Output2);

    
    /*
    setTorque(nodesid[3], Output3);
    setTorque(nodesid[0], Output2);
    setTorque(nodesid[1], Output3);
    */

    time_torque = millis();
  }

  if (millis() - posstream > 20 && streampos) {
      Serial.print("Positions: ");
      Serial.print(SMCCAN.joints_position[0]);
      Serial.print(" ");
      Serial.print(SMCCAN.joints_position[1]);
      Serial.print(" ");
      Serial.print(SMCCAN.joints_position[2]);
      Serial.print(" ");
      Serial.print(SMCCAN.joints_position[3]);
      Serial.println();
      posstream = millis();
  }
  if (millis() - time_telemetry > 0 && stream_data) {
    // SMCCAN.readVariables();
    sendTelemetry();
    time_telemetry = millis();
  }
  lastp = micros();
  ReadSerial();

  // CAN_message_t inMsg;
  
  if (stringComplete) {
    inputString = "";
    stringComplete = false;
  }
}

// ------------------------------------------------------------
// ------------FUNCTIONS --------------------------------------
float kde = 0; 
float kpe = 0; 
float kie = 0; 
void pidCompute(){
  if (millis()-pidtime>5){
  SMCCAN.readVariables();
  float e = Setpoint2 - SMCCAN.encoder_pos[2]; 
  float dt = millis() - pidtime; 
  dt *= 0.001;
  e2i += e * dt; 
  kpe = Kp * e; 
  kie = Ki * e2i; 
  kde =  Kd * (e-e2p)/dt; 
  u2 = kpe + kie + kde; 
  Output2 = u2; 
  e2p = e; 
  pidtime = millis();  
  }
}

void sendTelemetry() {
  // Serial.print("t_Vr_Cr_Tp_Fler_EncPos_Va_Ve ");
  Serial.print("t_Tr_Cr_Tp_Fler_EncPos ");
  Serial.print(micros());
  Serial.print(" ");
  Serial.print(maivel); 
  Serial.print(" ");
  Serial.print(SMCCAN.run_currents[nodeintest]);
  Serial.print(" ");
  Serial.print(SMCCAN.actual_torque_percent[nodeintest]);
  Serial.print(" ");
  Serial.print(SMCCAN.follow_err[nodeintest]);
  Serial.print(" ");
  Serial.println(SMCCAN.encoder_pos[nodeintest]);
  // Serial.print(" ");
  // Serial.print(SMCCAN.actual_velocity[nodeintest]);
  // Serial.print(" ");
  // Serial.println(SMCCAN.encoder_velocity[nodeintest]);

  /*
  
  Serial.print("otTppdi ");
  Serial.print(Output2, 8);
  Serial.print(" ");
  uint32_t telruncurrent = SMCCAN.run_currents[2];
  float ftelruncurrent = telruncurrent * 5.87;
  float telactualtorque = SMCCAN.actual_torque_percent[2];
  float torquecur = ftelruncurrent * telactualtorque / 100.0;
  int32_t elrr = SMCCAN.follow_err[2];
  int32_t pos2 = SMCCAN.encoder_pos[2];
  if (elrr < 0) {
    torquecur *= -1;
  }
  Serial.print(torquecur);
  Serial.print(" ");
  float ppos2 = pos2;
  Serial.print(ppos2);
  Serial.print(" ");
  Serial.print(kpe);
  Serial.print(" ");
  Serial.print(kde);
  Serial.print(" ");
  Serial.print(kie);
  Serial.println();
*/
/*
  Serial.print(" ");
  Serial.print(Output1);
  Serial.print(" ");
  telruncurrent = SMCCAN.run_currents[3];
  ftelruncurrent = telruncurrent * 5.87;
  telactualtorque = SMCCAN.actual_torque_percent[3];
  torquecur = ftelruncurrent * telactualtorque / 100.0;
  elrr = SMCCAN.follow_err[3];
  pos2 = SMCCAN.encoder_Pos[3];
  if (elrr < 0) {
    torquecur *= -1;
  }
  Serial.print(torquecur);
  Serial.print(" ");
  ppos2 = pos2 * 0.0018;
  Serial.println(ppos2);
  */
  //  Serial.print(" ");
  //  Serial.println(elrr / 100.0);
}

bool pasflag = true;


void checkTorqueV(uint8_t nodeid) {
  float pertor = SMCCAN.getActualTorquePercent(nodeid);
  //  Serial.println(pertor);
  if (pertor < 2) {
    //    Serial.println("zero torque");
    SMCCAN.setPassiveMode(nodeid);
    Serial.println("torquereset");
    SMCCAN.setVelocityMode(nodeid);
  }
}


void setTorqueV(uint8_t nodeid, double current_t) {
  // receives the torque in amps
  float runncurrent = current_t; // * 1000;
  int32_t velocity = 0;
  float percent = 0;
  //  Serial.print("cur: ");
  //  Serial.println(runncurrent);
  /////
  checkTorqueV(nodeid);

  //////
  if (abs(runncurrent) > 2000)
    runncurrent = 2000;
  SMCCAN.setRunCurrent(nodeid, abs(runncurrent));
  int32_t encopos = SMCCAN.getEncoderPosition(nodeid);
  int32_t psoll = encopos;
  if (abs(runncurrent) < 5.9) {
    velocity = 0;
    percent = abs(runcurrent) * 17.035775; //100/5.87
    percent *= 20.48;
  }
  else {
    percent = 2048 * 10; // 2048 * 4; //3000
    velocity = 50000;
  }
  if (runncurrent < 0) {
    psoll -= percent;
    velocity *= -1;
  }
  else {
    psoll += percent;
  }
  if (runncurrent == 0) {
    psoll = encopos;
  }
  SMCCAN.writeToRegister(nodeid, V_SOLL, velocity);
  SMCCAN.waitForReply(nodeid, V_SOLL, false);
}


void homePosition(float hcurrent, float speedo ) {
  //float hcurrent = 200;
  //Set a low
  for (int i = 0; i < 4; i++ ) SMCCAN.setRunCurrent(nodesid[i], hcurrent);
  //  delay(1);
  int32_t vel = (int32_t)speedo / 0.01; //this corresponds to 5 rpm
  for (int i = 0; i < 4; i++) SMCCAN.setVelocityInt32(nodesid[i], vel);
  //  delay(1);
  int32_t homepos[] = { 0,0,0,0}; //imaginary home position
  for (int i = 0; i < 4; i++) SMCCAN.setPSOLL(nodesid[i], homepos[i]);
  // Add something for checking the correct home position or arrival to home position
  // before returning
}
void sendFeedback() {
  int32_t feedback_pos[] = {0, 0, 0, 0};
  for (int i = 0; i < 4; i++) {
    feedback_pos[i] = SMCCAN.getEncoderPosition(nodesid[i]);
    //    delayMicroseconds(100);
  }
  Serial.print("X");
  Serial.print(feedback_pos[0]);
  Serial.print(" Y");
  Serial.print(feedback_pos[1]);
  Serial.print(" Z");
  Serial.print(feedback_pos[2]);
  Serial.print(" T");
  Serial.println(feedback_pos[3]);
}
// ------------------------------------------------------------
void OnReceived() {
  uint8_t len, d0, d1, d2, d3, d4, d5, d6, d7;

  if (myMessenger.checkString("CAN")) {
    uint8_t nodoid = myMessenger.readInt();
    uint32_t comobject_id = myMessenger.readInt();
    len = myMessenger.readInt();
    d0 = myMessenger.readInt();
    d1 = myMessenger.readInt();
    d2 = myMessenger.readInt();
    d3 = myMessenger.readInt();
    d4 = myMessenger.readInt();
    d5 = myMessenger.readInt();
    d6 = myMessenger.readInt();
    d7 = myMessenger.readInt();
    uint32_t cobid;
    cobid = nodoid + comobject_id;
    wrMsg(&CANbus1, cobid, len, d0, d1, d2, d3, d4, d5, d6, d7);
  }
  else if (myMessenger.checkString("STR")) {
    Serial.println("OK");
    stream_data = true;
  }
  else if (myMessenger.checkString("SSTR")) {
    Serial.println("OK");
    stream_data = false;
  }
  else if (myMessenger.checkString("ME")) {
    Serial.println("OK");
    motion_enable = true;
  }
  else if (myMessenger.checkString("NME")) {
    Serial.println("OK");
    motion_enable = false;
  }
  else if (myMessenger.checkString("G28")) {
    Serial.println("OK");
    homePosition(900, 5);
  }
  else if (myMessenger.checkString("M114")) {
    sendFeedback();
  }
  else if (myMessenger.checkString("CUR")) {
    double received_current = myMessenger.readDouble();
    for (int i = 0; i < 4; i++) SMCCAN.setRunCurrent(nodesid[i], received_current);
    Serial.println("OK");
  }
  else if (myMessenger.checkString("node")) {
    int received_node = myMessenger.readInt();
    SMCCAN.setPassiveMode(nodesid[nodeintest]);
    nodeintest = received_node;
    // SMCCAN.setVelocityInt32(nodesid[nodeintest], 0);
    SMCCAN.setPositionMode(nodesid[nodeintest]);
    Serial.println("OK");
  }
  else if (myMessenger.checkString("psm")) {
    int received_node = myMessenger.readInt();
    SMCCAN.setPositionMode(nodesid[received_node]); 
    Serial.println("OK");
  }
  else if (myMessenger.checkString("pssm")) {
    int received_node = myMessenger.readInt();
    SMCCAN.setPassiveMode(nodesid[received_node]); 
    Serial.println("OK");
  }
  else if (myMessenger.checkString("asoll")) {
    uint32_t received = myMessenger.readUInt();
    uint32_t asoll ; //rpm/s
    asoll = received; 
    for (int i = 0; i < 4; i++) {
      SMCCAN.writeToRegister(nodesid[i], A_SOLL, asoll);
      SMCCAN.waitForReply(nodesid[i], A_SOLL, true);
    }
    Serial.println("OK"); 
  }
  else if (myMessenger.checkString("vsoll")) {
    double received_vsoll = myMessenger.readDouble();
    SMCCAN.setVelocityInt32(nodesid[nodeintest], (int32_t) (received_vsoll * 100));
    Serial.println("OK");
  } 

  else if (myMessenger.checkString("VSOLLall")) {
    double received_vsoll = myMessenger.readDouble();
    for (int i = 0; i < 4; i++) SMCCAN.setVelocityInt32(nodesid[i], (int32_t) (received_vsoll * 100));
    Serial.println("OK");
  }
  else if (myMessenger.checkString("G0")) {
    double received_pos[] = {0, 0, 0, 0};
    if (myMessenger.checkString("M1"))
      received_pos[0] = myMessenger.readInt();
    if (myMessenger.checkString("M2"))
      received_pos[1] = myMessenger.readInt();
    if (myMessenger.checkString("M3"))
      received_pos[2] = myMessenger.readInt();
    if (myMessenger.checkString("M4"))
      received_pos[3] = myMessenger.readInt();

    for (int i = 0; i < 4; i++) {
      SMCCAN.setPSOLL(nodesid[i], received_pos[i]);
      //      delayMicroseconds(300);
    }
    Serial.println("OK");
  }
  else if (myMessenger.checkString("c")) {
        uint32_t rcu = (uint32_t)myMessenger.readInt();
        r_current = rcu;
        Serial.print("Set current mA: ");
        Serial.println(rcu);
        for (int i = 0; i < 4; i++) SMCCAN.setRunCurrent(nodesid[i], rcu);
        Serial.println("OK");
  }
  else if (myMessenger.checkString("pas")) {
    for (int i = 0; i < 4; i++) SMCCAN.setPassiveMode(nodesid[i]);
    Serial.println("OK");
    //    SMCCAN.writeToRegister(MODE_REG, passive_mode);
  }
  else if (myMessenger.checkString("pos")) {
    for (int i = 0; i < 4; i++) SMCCAN.setPositionMode(nodesid[i]);
    Serial.println("OK");
  }
  else if (myMessenger.checkString("vel")) {
    for (int i = 0; i < 4; i++){
      SMCCAN.setVelocityInt32(nodesid[i], 0);
      SMCCAN.setVelocityMode(nodesid[i]);
    } 
    Serial.println("OK");
  }
  else if (myMessenger.checkString("ctv")) {
    ctvelstart = true; 
    Serial.println("OK");
  }
  else if (myMessenger.checkString("nctv")) {
    ctvelstart = false;
    SMCCAN.setVelocityInt32(nodesid[nodeintest], 0);
    SMCCAN.setPassiveMode(nodesid[nodeintest]);   
    delay(20); 
    SMCCAN.setVelocityMode(nodesid[nodeintest]);
    Serial.println("OK");
  }
  else if (myMessenger.checkString("ctt")) {
    cttorquestart = true; 
    SMCCAN.setPositionMode(nodesid[nodeintest]);   
    Serial.println("OK");
  }
  else if (myMessenger.checkString("nctt")) {
    cttorquestart = false;
    SMCCAN.setTorque(nodesid[nodeintest], 0);
    SMCCAN.setPassiveMode(nodesid[nodeintest]);   
    Serial.println("OK");
  }
  else if (myMessenger.checkString("ctto")) {
    cttorquestarto = true; 
    SMCCAN.setPositionMode(nodesid[nodeintest]);   
    Serial.println("OK");
  }
  else if (myMessenger.checkString("nctto")) {
    cttorquestarto = false;
    SMCCAN.setTorque(nodesid[nodeintest], 0);
    SMCCAN.setPassiveMode(nodesid[nodeintest]);   
    Serial.println("OK");
  }
  else if (myMessenger.checkString("tp")) {
    //    noInterrupts();
    //    torqueper = myMessenger.readInt();
    //    interrupts();
  }
  else if (myMessenger.checkString("torque")) {
    //    destorque = myMessenger.readDouble();
  }
  else if (myMessenger.checkString("write_pist")) {
    //    int32_t elposition;
    //    Serial.println("writing pist");
    //    elposition = myMessenger.readInt();
    //    SMCCAN.writeToRegister(P_IST, elposition);
  }
  else if (myMessenger.checkString("wps")) {
        int32_t elposition;
        
        int node = myMessenger.readInt();
        elposition = myMessenger.readInt();
        Serial.print("node ");
        Serial.print(node);
        Serial.print(" pos: ");
        Serial.println(elposition);
        SMCCAN.setPSOLL(nodesid[node], elposition);
        Serial.println("OK");
  }
  else if (myMessenger.checkString("maivel")) {
    maivelval = myMessenger.readDouble();
    Serial.println("OK");
  }
  else if (myMessenger.checkString("maitorval")) {
    maitorval = myMessenger.readDouble();
    Serial.println("OK");
  }
  else if (myMessenger.checkString("write_enc")) {
    //    int32_t elposition;
    //    elposition = myMessenger.readInt();
    //    SMCCAN.writeToRegister(ENCODER_POS, elposition);
  }
  else if (myMessenger.checkString("FS")) {
    fricstart = true; 
    Serial.println("OK"); 
  }
  else if (myMessenger.checkString("FT")) {
    fricstart = false;  
    Serial.println("OK"); 
  }
  else if (myMessenger.checkString("sine")) {
    sinewavestart = true; 
    Serial.println("OK"); 
  }

  else if (myMessenger.checkString("nsine")) {
    sinewavestart = false;
    Serial.println("OK");   
  }
  else if (myMessenger.checkString("sinep")) {
    sinewavestartpos = true; 
    Serial.println("OK"); 
  }
  else if (myMessenger.checkString("compt")) {
    computedtorque = true; 
    Serial.println("OK"); 
  }
  else if (myMessenger.checkString("ncompt")) {
    computedtorque = false; 
    Serial.println("OK"); 
  }
  else if (myMessenger.checkString("nsinep")) {
    sinewavestartpos = false;
    Serial.println("OK");   
  }
  else if (myMessenger.checkString("clcerr")) {
    for (int i = 0; i < 4; i++) SMCCAN.clearErrors(nodesid[i], true);
    Serial.println("OK");
  }
  else if (myMessenger.checkString("pnew")) {
    //    int32_t elposition;
    //    elposition = myMessenger.readInt();
    //    SMCCAN.writeToRegister(P_NEW, elposition);
  }
  else if (myMessenger.checkString("SP")){
    streampos = true; 
    Serial.println("OK"); 
  }
  else if (myMessenger.checkString("STP")) {
    streampos = false; 
    Serial.println("OK"); 
  }
  while (myMessenger.available()) {
    myMessenger.readInt();
  }
}
// -------------------------------------
//Read the serial for the Messenger
void ReadSerial() {
  while (Serial.available()) {
    myMessenger.process(Serial.read());
  }
}

