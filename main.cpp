#include "mbed.h"
#include "nRF24L01.h"
#include "RF24.h"
#include "RF24_config.h"


const float PI = 3.14159265359;
const float R2D = 180/PI;

PwmOut PWM1(p22);
PwmOut PWM2(p23);
PwmOut PWM3(p24);
PwmOut PWM4(p21);

DigitalIn switch_on(p5, PullDown);
DigitalOut LED_b(p26);

Timer t;

I2C i2c(p28,p27); //(I2C_SDA,I2C_SCL); //sda, scl I2C_SDA,I2C_SCL D4, D5 A4, A5
RF24 NRF24L01(p5, p6, p7, p15, p8);
Serial pc(USBTX, USBRX); //D1D0


//------------------------------------------------------------------------------------------------
//RF
const uint64_t pipe = 0x1212121212LL;
int8_t recv[30];
int16_t PITCH = 0, ROLL = 0, YAW = 0, THROTTLE = 0;
int8_t ackData[30];
int8_t flip1 = 1;

// ETC
char BUT1 = 1, BUT2 = 1, prev_BUT2 = 1;
int rf_fail_count = 10;

//-----------------------------------------------------------------------------------------
//mpu9250 register      
int MPU9250_ADDRESS     = (0x68<<1), // 0b11010000
    WHO_AM_I_MPU9250    = 0x75,
    FIFO_EN             = 0x23,
    SMPLRT_DIV          = 0x19,
    CONFIG              = 0x1A,     // 0b11010
    GYRO_CONFIG         = 0x1B,     // 0b11011
    ACCEL_CONFIG        = 0x1C,     // 0b11100
    ACCEL_CONFIG2       = 0x1D,     // 0b11101
    ACCEL_XOUT_H        = 0x3B,
    TEMP_OUT_H          = 0x41,
    GYRO_XOUT_H         = 0x43,

    PWR_MGMT_1          = 0x6B,
    PWR_MGMT_2          = 0x6C,  
    INT_PIN_CFG         = 0x37,     
    
    AK8963_ADDRESS      = (0x0C<<1),    // 0b00001100 0x0C<<1
    WHO_AM_I_AK8963     = 0x00;         // should return 0x48


float gyro_bias[3] = {0.0, 0.0, 0.0};
float accel_bias[3] = {0.0, 0.0, 0.0};
//------------------------------------------------------------------------------------------------
// taken from MPU9250 library

bool armed = false;

int l_count = 0;

float dt = 0.002;
int dt_us = 2000;

float accel_f[3], gyro_f[3];
float gyro_angle[3];
float roll_f, roll_accel, roll_g, roll_a;
float pitch_f, pitch_accel, pitch_g, pitch_a;
float yaw_f;

float tau = 1.0/3.0;
float alpha = 0.99;

float K_scale   = 1.0;
float Kp_roll   = 0.9, Kd_roll = 0.3;    // p=30: 0.03
float Kp_pitch  = 0.9, Kd_pitch = 0.3;   // p=50: 0.02,  p=100: 0.06,  p>100: 0.15
float Kd_yaw    = 0.5;

float roll_cont = 0.0, pitch_cont = 0.0, yaw_cont = 0.0;
float roll_c = 0.0, pitch_c = 0.0, yaw_c = 0.0, thrust_c = 0.0;
float sum_c[4];

float roll_err, roll_rate_err;
float roll_ref = 0.0, roll_rate_ref = 0.0;
float pitch_err, pitch_rate_err;
float pitch_ref = 0.0, pitch_rate_ref=0.0;
float yaw_err, yaw_rate_err;
float yaw_ref = 0.0, yaw_rate_ref = 0.0;

float thrust_in = 0.0;

float ROLL_ERR_MAX = 50.0, ROLL_RATE_ERR_MAX = 100.0;    // maximum errors considered
float PITCH_ERR_MAX = 50.0, PITCH_RATE_ERR_MAX = 100.0;
float YAW_RATE_ERR_MAX = 100.0;

float ROLL_CONT_MAX = 40.0, PITCH_CONT_MAX = 40.0, YAW_CONT_MAX = 40.0;     // maximum control gain considered

float roll_controller = 30.0, pitch_controller = 30.0, yaw_controller = 100.0;
float pwm1_pw, pwm2_pw, pwm3_pw, pwm4_pw;


// FUNC
void control(void);
void pwm_drive(void);
void control_loop();
void WHO_AM_I(void);
void MPU9250_INIT(void);
void MPU9250_RESET(void);
void MPU9250_GET_GYRO(int16_t * destination);
void MPU9250_GET_ACCEL(int16_t * destination);
void gyro_bias_f(void);
void accel_bias_f(void);
void RF_READ();
void RF_init(int nchannel);
int constrain_int16(int16_t x, int min, int max);
float constrain_float(float x, float min, float max);


// MAIN
int main()
{
    RF_init(30);             //channel number

    PWM1.period(0.0001);    // 10 kHz PWM for PWM1~PWM4
    pc.baud(115200);

    WHO_AM_I();
    MPU9250_INIT();
    LED_b = 1;
    wait_us(3000000);

    LED_b = 0;
    gyro_angle[0]=0.0;
    gyro_angle[1]=0.0;
    gyro_angle[2]=0.0;

    gyro_bias_f();
    pc.printf("gyro biases(deg/sec) %f %f %f \n\r", gyro_bias[1], gyro_bias[0], -gyro_bias[2]); 
    LED_b = 1;
    wait_us(1000000);

    // LED_b = 0;
    // accel_bias_f();
    // pc.printf("accel biases(deg/seg) %f %f %f \n\r", accel_bias[1], -accel_bias[0], accel_bias[2]);
    // LED_b = 1;
    // wait_us(1000000);

    int T = 0;

    t.start();

    while(true) {

        T = t.read_us();

        if ((int)prev_BUT2 == 0 && (int)BUT2 == 1) {
            armed = !armed;
            prev_BUT2 = 1;
        }
        else { prev_BUT2 = BUT2; }

        if ((int)BUT1 == 1) { LED_b = 1; }
        else { LED_b = 0; }

        control_loop();

        // pc.printf("dt: %d \r", dt_us);

        // pc.printf("dt: %d, RF Fail CNT: %d, armed: %d, BUT2: %d, Roll: %d, Pitch: %d, Yaw: %d, Throttle: %d \r", 
        // dt_us, rf_fail_count, armed, BUT2, ROLL, PITCH, YAW, THROTTLE);

        // pc.printf("dt: %d, RF Fail: %d, armed: %d, BUT2: %d, Ref - Roll: %.2f, Pitch: %.2f, Yaw: %.2f, Throttle: %.2f \r",
        // dt_us, rf_fail_count, armed, BUT2, roll_ref, pitch_ref, yaw_rate_ref, thrust_in);

        // pc.printf("dt: %d, armed: %d, BUT1: %d, BUT2: %d, Roll: %.2f, \tPitch: %.2f, \tYaw: %.2f, \tThrottle: %.2f \r", 
        // dt_us, armed, BUT1, BUT2, roll_c, pitch_c, yaw_c, thrust_c);

        // pc.printf("ARMED: %d, BUT1: %d, BUT2: %d, ROLL: %.2f \tPITCH: %.2f \tYAW RATE: %.2f \r", 
        // armed, BUT1, BUT2, roll_f, pitch_f, gyro_f[2]);

        // pc.printf("dt: %d, RF Fail: %d, ARMED: %d, BUT1: %d, BUT2: %d, PWM1: %f, PWM2: %f, PWM3: %f, PWM4: %f \r", 
        // dt_us, rf_fail_count, armed, BUT1, BUT2, pwm1_pw, pwm2_pw, pwm3_pw, pwm4_pw);

        dt_us = t.read_us() - T;
        dt = dt_us * 1E-6;
    }
}


void control_loop()
{
    int16_t gyro[3];
    int16_t accel[3]; 
    float roll_accel_1;
    float pitch_accel_1;

    RF_READ();

    roll_ref =     (float)(ROLL/255.0 * roll_controller);
    pitch_ref =   -(float)(PITCH/255.0 * pitch_controller);
    yaw_rate_ref = (float)(YAW/80.0 * yaw_controller);
    thrust_in =    (float)THROTTLE;

    l_count = 0;

    MPU9250_GET_GYRO(gyro);
    gyro_f[0] =    gyro[1]/32.8 - gyro_bias[1];     // deg/sec
    gyro_f[1] =    gyro[0]/32.8 - gyro_bias[0];
    gyro_f[2] = - (gyro[2]/32.8 - gyro_bias[2]);

    // gyro_angle[0] = gyro_angle[0] + (gyro_f[0])*dt;
    // gyro_angle[1] = gyro_angle[1] + (gyro_f[1])*dt;
    // gyro_angle[2] = gyro_angle[2] + (gyro_f[2])*dt;

    MPU9250_GET_ACCEL(accel);
    accel_f[0] =   accel[1]/8192.0;                 // Navigation frame reference (NED) unit in G (9.8 m/sec^2)
    accel_f[1] =   accel[0]/8192.0*(-1);
    accel_f[2] = - accel[2]/8192.0*(-1);

    roll_accel_1 = atan2(accel_f[1], accel_f[2]) * R2D;
    pitch_accel_1 = atan2(accel_f[0], sqrt(accel_f[1]*accel_f[1] + accel_f[2]*accel_f[2])) * R2D;


    alpha = tau/(tau + dt);

    roll_a = (1 - alpha) * roll_accel_1;
    roll_g = alpha * (roll_f + gyro_f[0]*dt);

    pitch_a = (1 - alpha) * pitch_accel_1;
    pitch_g = alpha * (pitch_f + gyro_f[1]*dt);

    roll_f = roll_a + roll_g;
    pitch_f = pitch_g + pitch_a;

    control();
    pwm_drive();
    l_count += 1;
}


void control(void)
{
    roll_err = roll_ref - roll_f;
    roll_rate_err = roll_rate_ref - gyro_f[0];

    pitch_err = pitch_ref - pitch_f;
    pitch_rate_err = pitch_rate_ref - gyro_f[1];

    // yaw_err = yaw_ref - yaw_f;
    yaw_rate_err = yaw_rate_ref - gyro_f[2];


    //roll control
    if (roll_err > ROLL_ERR_MAX) {
        roll_err = ROLL_ERR_MAX;
    }
    else if (roll_err < -(ROLL_ERR_MAX)) {
        roll_err = -(ROLL_ERR_MAX);
    }

    if (roll_rate_err > ROLL_RATE_ERR_MAX) {
        roll_rate_err = ROLL_RATE_ERR_MAX;
    }
    else if (roll_rate_err < -(ROLL_RATE_ERR_MAX)) {
        roll_rate_err = -(ROLL_RATE_ERR_MAX);
    }

    roll_cont = K_scale*(Kp_roll*roll_err + Kd_roll*roll_rate_err); 

    if (roll_cont > ROLL_CONT_MAX) {
        roll_cont = ROLL_CONT_MAX;
    }
    else if (roll_cont < -(ROLL_CONT_MAX)) {
        roll_cont = -(ROLL_CONT_MAX);
    }

    roll_c = roll_cont/100.0;


    //pitch control
    if (pitch_err > PITCH_ERR_MAX) {
        pitch_err = PITCH_ERR_MAX;
    } 
    else if (pitch_err < -(PITCH_ERR_MAX)) {
        pitch_err = -(PITCH_ERR_MAX);
    }

    if (pitch_rate_err > PITCH_RATE_ERR_MAX) {
        pitch_rate_err = PITCH_RATE_ERR_MAX;
    } 
    else if (pitch_rate_err < -(PITCH_RATE_ERR_MAX)) {
        pitch_rate_err = -(PITCH_RATE_ERR_MAX);
    }

    pitch_cont = K_scale * (Kp_pitch*pitch_err + Kd_pitch*pitch_rate_err); 

    if (pitch_cont > PITCH_CONT_MAX) {
        pitch_cont = PITCH_CONT_MAX;
    }
    else if (pitch_cont < -(PITCH_CONT_MAX)) {
        pitch_cont = -(PITCH_CONT_MAX);
    }

    pitch_c = pitch_cont/100.0;


    // yaw control
    if (yaw_rate_err > YAW_RATE_ERR_MAX) {
        yaw_rate_err = YAW_RATE_ERR_MAX;
    }
    else if (yaw_rate_err < -YAW_RATE_ERR_MAX) {
        yaw_rate_err = - YAW_RATE_ERR_MAX;
    }

    yaw_cont = K_scale*(Kd_yaw * yaw_rate_err);

    if (yaw_cont > YAW_CONT_MAX) {
        yaw_cont = YAW_CONT_MAX;
    }
    else if (yaw_cont < -(YAW_CONT_MAX)) {
        yaw_cont = -(YAW_CONT_MAX);
    }

    yaw_c = yaw_cont/100.0;


    // thrust control
    thrust_c = 1 - THROTTLE/1159.0;
}


void pwm_drive(void)
{
    sum_c[0] = ( + roll_c + pitch_c - yaw_c + thrust_c );
    sum_c[1] = ( - roll_c + pitch_c + yaw_c + thrust_c );
    sum_c[2] = ( - roll_c - pitch_c - yaw_c + thrust_c );
    sum_c[3] = ( + roll_c - pitch_c + yaw_c + thrust_c );

    pwm1_pw = constrain_float(sum_c[0], 0.0, 1.0);
    pwm2_pw = constrain_float(sum_c[1], 0.0, 1.0);
    pwm3_pw = constrain_float(sum_c[2], 0.0, 1.0);
    pwm4_pw = constrain_float(sum_c[3], 0.0, 1.0);

    if (armed == true) {
        PWM1 = pwm1_pw;
        PWM2 = pwm2_pw;
        PWM3 = pwm3_pw;
        PWM4 = pwm4_pw;
    }
    else {
        PWM1 = 0.0;
        PWM2 = 0.0;
        PWM3 = 0.0;
        PWM4 = 0.0;
    }
}


void MPU9250_RESET(void)
{
  // reset device
  //writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device

  char cmd[2];
  cmd[0] = PWR_MGMT_1; //status
  cmd[1] = 0x80;
  i2c.write(MPU9250_ADDRESS, cmd, 2, 0);
  wait_us(100000);
}


void WHO_AM_I(void)
{
    char cmd[2]; 
    cmd[0] = WHO_AM_I_MPU9250; 
    i2c.write(MPU9250_ADDRESS, cmd, 1, 1);
    i2c.read(MPU9250_ADDRESS, cmd, 1, 0);
    wait_us(100000);
    uint8_t DEVICE_ID = cmd[0];
    pc.printf("IMU device id is  %x \n\r", DEVICE_ID);
    wait_us(300000);

    /* cmd[0] = WHO_AM_I_AK8963; 
    i2c.write(AK8963_ADDRESS, cmd, 1);
    i2c.read(AK8963_ADDRESS, cmd, 0);//
    wait(0.1);
    uint8_t DEVICE_ID2 = cmd[0];
    pc.printf("MAG  id is %d \n\r", DEVICE_ID2);*/
}


void MPU9250_INIT(void)
{  
    // Initialize MPU9250 device
    // wake up device
    //and clear sleep mode bit (6), enable all sensors 
    //wait(0.1); // Delay 100 ms for PLL to get established on x-axis gyro; should check for PLL ready interrupt  
  
    // get stable time source
    // writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01); // Auto selects the best available clock source PLL if ready, else use the Internal oscillator

    // Configure Gyro and Accelerometer
    // Disable FSYNC and set gyro bandwidth to 41 Hz, delay 5.9 ms
    // DLPF_CFG = bits 2:0 = 011; this sets gyro bandwidth to 41 Hz, delay 5.9 ms and internal sampling rate at 1 kHz
    // Maximum delay is 4.9 ms which is just over a 200 Hz maximum rate
    // writeByte(MPU9250_ADDRESS, CONFIG, 0x03); //page 13 register map  
 
    // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
    // writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x04);  // Use a 200 Hz rate; the same rate set in CONFIG above

    char cmd[3];
    cmd[0] = PWR_MGMT_1; //reset
    cmd[1] = 0x80;
    i2c.write(MPU9250_ADDRESS, cmd, 2);
    pc.printf("MPU 1 \n\r");
    wait_us(10000);

    cmd[0] = PWR_MGMT_1; // Auto selects the best available clock source PLL if ready, else use the Internal oscillator
    cmd[1] = 0x01;
    i2c.write(MPU9250_ADDRESS, cmd, 2, 0);
    pc.printf("MPU 2 \n\r");
    wait_us(10000);
  
    cmd[0] = CONFIG;
    cmd[1] = 0x03;// 41Hz gyro bandwidth, 1kHz internal sampling
    i2c.write(MPU9250_ADDRESS, cmd, 2, 0);
    pc.printf("MPU 3 \n\r");
    wait_us(10000);
 
    // sample rate divisor for all sensors, 1000/(1+4)=200 Hz for Gyro 
    cmd[0] = SMPLRT_DIV; 
    cmd[1] = 0x04;//0x04 
    i2c.write(MPU9250_ADDRESS, cmd, 2, 0);
    pc.printf("MPU 4 \n\r");
    wait_us(10000);
 
    /*// Set gyroscope full scale range //page 14
    // Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
    uint8_t c;
    cmd[0] = GYRO_CONFIG; //status
    i2c.write(MPU9250_ADDRESS, cmd, 1, 1);
    i2c.read(MPU9250_ADDRESS, cmd, 1, 0);
    pc.printf("MPU 5 \n\r");
    c = cmd[0];
    */
  
    //writeByte(MPU9250_ADDRESS, GYRO_CONFIG, c & ~0xE0); // Clear self-test bits [7:5] 0b1110000
    //writeByte(MPU9250_ADDRESS, GYRO_CONFIG, c & ~0x18); // Clear AFS bits [4:3] 0b00011000
    //writeByte(MPU9250_ADDRESS, GYRO_CONFIG, c | 0b00010011); // Set scale range for the gyro

    cmd[0] = GYRO_CONFIG; 
    cmd[1] = 0b00010000;    // Gyro full scale 1000 deg/sec; Gyro DLPF Enable
    i2c.write(MPU9250_ADDRESS, cmd, 2, 0);
    pc.printf("MPU 6 \n\r");
    wait_us(10000);

    // Set accelerometer configuration
    // Accel fulll sacle range +/- 4g
    cmd[0] = ACCEL_CONFIG; 
    cmd[1] = 0b00001000;        // Accel 
    i2c.write(MPU9250_ADDRESS, cmd, 2, 0);
    pc.printf("MPU 8 \n\r");
    wait_us(10000);

    // Set accelerometer sample rate configuration (Fast sampling)
    // It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
    // accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz

    cmd[0] = ACCEL_CONFIG2; 
    cmd[1] = 0b00001100;
    i2c.write(MPU9250_ADDRESS, cmd, 2, 0);
    pc.printf("MPU 10 \n\r");
    wait_us(10000);
    // XYZ Gyro accel enable (default)
    cmd[0] = PWR_MGMT_2; 
    cmd[1] = 0x00;
    i2c.write(MPU9250_ADDRESS, cmd, 2, 0);
    pc.printf("MPU 11 \n\r");
    wait_us(10000);

    // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates, 
    // but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

    // Configure Interrupts and Bypass Enable
    // Set interrupt pin active high, push-pull, and clear on read of INT_STATUS, enable I2C_BYPASS_EN so additional chips 
    // can join the I2C bus and all can be controlled by the mbed as master
    //writeByte(MPU9250_ADDRESS, INT_PIN_CFG, 0x22);    
    //writeByte(MPU9250_ADDRESS, INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt
    //page 29

    //I2c bypass mode
    cmd[0] = INT_PIN_CFG; 
    cmd[1] = 0x22; //0x02  
    i2c.write(MPU9250_ADDRESS, cmd, 2, 0);
    pc.printf("MPU 12 \n\r");
    wait_us(10000);
}


void gyro_bias_f(void)
{    
    int16_t gyro1[3];
    pc.printf("Gyro bias calculate: Please keep still \n\r");

    for (int i=0; i<100; ++i) {
        MPU9250_GET_GYRO(gyro1);
        gyro_bias[0] = gyro_bias[0] + gyro1[0]/32.8;
        gyro_bias[1] = gyro_bias[1] + gyro1[1]/32.8;
        gyro_bias[2] = gyro_bias[2] + gyro1[2]/32.8;
        pc.printf("bias finding i = %d \r", i);
        wait_us(10000);
    }

    gyro_bias[0] = gyro_bias[0]/100.0;
    gyro_bias[1] = gyro_bias[1]/100.0; 
    gyro_bias[2] = gyro_bias[2]/100.0;   
    pc.printf("Gyroscope bias finding completed \n\r");
}


void accel_bias_f(void)
{
    int16_t accel1[3];
    pc.printf("Accel bias calculate: Please keep still \n\r");

    for (int i=0; i<100; ++i) {
        MPU9250_GET_ACCEL(accel1);
        accel_bias[0] = accel_bias[0] + accel1[0]/8192.0;
        accel_bias[1] = accel_bias[1] + accel1[1]/8192.0;
        accel_bias[2] = accel_bias[2] + accel1[2]/8192.0;
        pc.printf("bias finding i = %d \r", i);
        wait_us(10000);
    }

    accel_bias[0] = accel_bias[0]/100.0;
    accel_bias[1] = accel_bias[1]/100.0;
    accel_bias[2] = accel_bias[2]/100.0;
    pc.printf("Accelerometer bias finding completed \n\r");
}


void MPU9250_GET_GYRO(int16_t * destination)
{
    char cmd[6];
    cmd[0] = GYRO_XOUT_H;
    i2c.write(MPU9250_ADDRESS, cmd, 1, 1);
    i2c.read(MPU9250_ADDRESS, cmd, 6, 0);
    destination[0] = (int16_t)(((int16_t)cmd[0] << 8) | cmd[1]);
    destination[1] = (int16_t)(((int16_t)cmd[2] << 8) | cmd[3]);
    destination[2] = (int16_t)(((int16_t)cmd[4] << 8) | cmd[5]);
  
}


void MPU9250_GET_ACCEL(int16_t * destination)
{    
    char cmd[6];
    cmd[0] = ACCEL_XOUT_H;
    i2c.write(MPU9250_ADDRESS, cmd, 1, 1);
    i2c.read(MPU9250_ADDRESS, cmd, 6, 0);
    destination[0] = (int16_t)(((int16_t)cmd[0] << 8) | cmd[1]);
    destination[1] = (int16_t)(((int16_t)cmd[2] << 8) | cmd[3]);
    destination[2] = (int16_t)(((int16_t)cmd[4] << 8) | cmd[5]);
  
    //pc.printf("gyro_raw %d, %d, %d \n\r", destination[0], destination[1], destination[2]); 
}


void RF_READ()
{
    if (NRF24L01.available()) {

        NRF24L01.read(recv, 12);
        
        ROLL = *(int16_t*)(&recv[0]);       //ROLL = - ROLL;
        PITCH = *(int16_t*)(&recv[2]);      //flip pitch and roll
        YAW = *(int16_t*)(&recv[4]);
        THROTTLE = *(int16_t*)(&recv[6]);
        BUT1 = recv[8];
        BUT2 = recv[9];                     //should hold value here

        if (abs(ROLL) <= 5) { ROLL = 0; }
        if (abs(PITCH) <= 5) { PITCH = 0; }
        if (abs(YAW) <= 5) { YAW = 0; }
        if (THROTTLE >= 1000) { THROTTLE = 1159; }

        // Offset
        ROLL = ROLL - 89;
        PITCH = PITCH + 15;
        YAW = YAW - 5;

        rf_fail_count = 0;
    }
    else {
        rf_fail_count = rf_fail_count + 1;
        
        if(rf_fail_count >= 20 && rf_fail_count < 100) {
            THROTTLE = THROTTLE - 2;
            THROTTLE = constrain_int16(THROTTLE, 0, 1023);
        }
        else if(rf_fail_count >= 100) {
            ROLL = 0;
            PITCH = 0;
            YAW = 0;
            THROTTLE = 1159;

            rf_fail_count = 100;
        }
    }
}


void RF_init(int nchannel)
{   
    NRF24L01.begin();
    NRF24L01.setDataRate(RF24_2MBPS);   //RF24_2MBPS
    NRF24L01.setChannel(nchannel);      // set channel 10 20 30
    NRF24L01.setPayloadSize(28);
    NRF24L01.setAddressWidth(5);
    NRF24L01.setRetries(2,4);           // 1,3 2,8 1,8
    NRF24L01.enableAckPayload();
    NRF24L01.openReadingPipe(0, pipe);
    NRF24L01.startListening();
}


int constrain_int16(int16_t x, int min, int max)
{
    if (x > max) x = max;
    else if (x < min) x = min;
    
    return x;
}


float constrain_float(float x, float min, float max)
{
    if (x > max) x = max;
    else if (x < min) x = min;

    return x;
}