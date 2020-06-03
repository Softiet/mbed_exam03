// INCLDE :: GENERAL
#include "mbed.h"
#include <vector>

// INCLUDE :: WIFI
#include "MQTTNetwork.h"
#include "MQTTmbed.h"
#include "MQTTClient.h"

// INCLUDE :: ACCELORO
#include "fsl_port.h"
#include "fsl_gpio.h"
#define UINT14_MAX        16383
      // FXOS8700CQ I2C address
#define FXOS8700CQ_SLAVE_ADDR0 (0x1E<<1) // with pins SA0=0, SA1=0
#define FXOS8700CQ_SLAVE_ADDR1 (0x1D<<1) // with pins SA0=1, SA1=0
#define FXOS8700CQ_SLAVE_ADDR2 (0x1C<<1) // with pins SA0=0, SA1=1
#define FXOS8700CQ_SLAVE_ADDR3 (0x1F<<1) // with pins SA0=1, SA1=1
      // FXOS8700CQ internal register addresses
#define FXOS8700Q_STATUS 0x00
#define FXOS8700Q_OUT_X_MSB 0x01
#define FXOS8700Q_OUT_Y_MSB 0x03
#define FXOS8700Q_OUT_Z_MSB 0x05
#define FXOS8700Q_M_OUT_X_MSB 0x33
#define FXOS8700Q_M_OUT_Y_MSB 0x35
#define FXOS8700Q_M_OUT_Z_MSB 0x37
#define FXOS8700Q_WHOAMI 0x0D
#define FXOS8700Q_XYZ_DATA_CFG 0x0E
#define FXOS8700Q_CTRL_REG1 0x2A
#define FXOS8700Q_M_CTRL_REG1 0x5B
#define FXOS8700Q_M_CTRL_REG2 0x5C
#define FXOS8700Q_WHOAMI_VAL 0xC7

// INCLUDE :: RPC
#include "mbed_rpc.h"

// GLOBAL VARIABLES :: GENERAL
InterruptIn btn2(SW2);
InterruptIn btn3(SW3);
Serial pc(USBTX, USBRX);

float x_velocity = 0;
float y_velocity = 0;
int timestamp = 0;

std::vector<float> vx_data;
std::vector<float> xy_data;
std::vector<int> timestamp_data;


Thread ctrl_thread(osPriorityHigh);
EventQueue ctrl_queue;
// GLOBAL VARIABLES :: ACCELEROMETER
I2C i2c( PTD9,PTD8);
int m_addr = FXOS8700CQ_SLAVE_ADDR1;
uint8_t who_am_i, acc_data[2], acc_res[6];
int16_t acc16;
float acc_t[3];

Thread acc_thread;
EventQueue acc_queue(32 * EVENTS_EVENT_SIZE);
 
// GLOBAL VARIABLE :: XBEE
RawSerial xbee(D12, D11);
EventQueue xbee_queue(32 * EVENTS_EVENT_SIZE);
Thread xbee_t;
char xbee_reply[4];
// FUNCTIONS DECLARE :: GENERAL
void publish_message_acc(MQTT::Client<MQTTNetwork, Countdown>* client);
void velo_update();


// FUNCTIONS DECLARE :: ACCELEROMETER
void FXOS8700CQ_readRegs(int addr, uint8_t * data, int len);
void FXOS8700CQ_writeRegs(uint8_t * data, int len);
void acc_update();

// FUNCRION DECLARE :: XBEE
void xbee_rx_interrupt(void);
void xbee_rx(void);
void reply_messange(char *xbee_reply, char *messange);
void check_addr(char *xbee_reply, char *messenger);

// FUNCTION DECARE :: RPC
void getVelo(Arguments *in , Reply *out);
RPCFunction rpcGetVelo(&getVelo, "getVelo");

int main() {
      // ACCELERO INITIALIZING
      pc.baud(115200);
            // Enable the FXOS8700Q
      FXOS8700CQ_readRegs( FXOS8700Q_CTRL_REG1, &acc_data[1], 1);
      acc_data[1] |= 0x01;
      acc_data[0] = FXOS8700Q_CTRL_REG1;
      FXOS8700CQ_writeRegs(acc_data, 2);
            // Get the slave address
      FXOS8700CQ_readRegs(FXOS8700Q_WHOAMI, &who_am_i, 1);
      pc.printf("Here is %x,\r\nACC ALL SYSTEM GO\r\n", who_am_i);

      // XBEE INITIALIZING
            // XBee setting
      printf("INITIALIZING XBEE\r\n");
      xbee.baud(9600);
      xbee.printf("+++");
      xbee_reply[0] = xbee.getc();
      xbee_reply[1] = xbee.getc();
      if(xbee_reply[0] == 'O' && xbee_reply[1] == 'K'){
            printf("enter AT mode.\r\n");
            xbee_reply[0] = '\0';
            xbee_reply[1] = '\0';
      }
      xbee.printf("ATMY 0x240\r\n");
      reply_messange(xbee_reply, "setting MY : 0x240");
      xbee.printf("ATDL 0x140\r\n");
      reply_messange(xbee_reply, "setting DL : 0x140");
      xbee.printf("ATID 0x26\r\n");
      reply_messange(xbee_reply, "setting PAN ID : 0x26");
      xbee.printf("ATWR\r\n");
      reply_messange(xbee_reply, "write config");
      xbee.printf("ATMY\r\n");
      check_addr(xbee_reply, "MY");
      xbee.printf("ATDL\r\n");
      check_addr(xbee_reply, "DL");
      xbee.printf("ATCN\r\n");
      reply_messange(xbee_reply, "exit AT mode");
      xbee.getc();
      // start
      pc.printf("start\r\n");
      
      while(xbee.readable()){
            xbee.getc();
      }
      xbee_t.start(callback(&xbee_queue, &EventQueue::dispatch_forever));
      // Setup a serial interrupt function of receiving data from xbee
      xbee.attach(xbee_rx_interrupt, Serial::RxIrq);
      
      acc_queue.call_every(100,velo_update);
      acc_thread.start(callback(&acc_queue, &EventQueue::dispatch_forever));
      // GENERAL CODE
      
      
      // ctrl_queue.call_every(1000,publish_message_acc,&client);
      // ctrl_thread.start(callback(&ctrl_queue, &EventQueue::dispatch_forever));

      

      return 0;

}

// FUNCTIONS :: GENERAL 
void getVelo(Arguments *in, Reply *out){
      printf("RPC get called\n\r");
      int temp = 0;
      out->putData(x_velocity);
      out->putData(y_velocity);
      out->putData(timestamp);
}

void velo_update(){
    timestamp = timestamp + 100;
    FXOS8700CQ_readRegs(FXOS8700Q_OUT_X_MSB, acc_res, 6);
    acc16 = (acc_res[0] << 6) | (acc_res[1] >> 2);
    if (acc16 > UINT14_MAX/2)
        acc16 -= UINT14_MAX;
    acc_t[0] = ((float)acc16) / 4096.0f;
    acc16 = (acc_res[2] << 6) | (acc_res[3] >> 2);
    if (acc16 > UINT14_MAX/2)
        acc16 -= UINT14_MAX;
    acc_t[1] = ((float)acc16) / 4096.0f;
    acc16 = (acc_res[4] << 6) | (acc_res[5] >> 2);
    if (acc16 > UINT14_MAX/2)
        acc16 -= UINT14_MAX;
    acc_t[2] = ((float)acc16) / 4096.0f;
    x_velocity = x_velocity + 9.8 * acc_t[0] * 0.1;
    y_velocity = y_velocity + 9.8 * acc_t[1] * 0.1;

    vx_data.push_back(x_velocity);
    vy_data.push_back(y_velocity);
    timestamp_data.push_bak(timestamp);
    
    // printf("velo_updated!\n\rvx:%f\n\rvy:%f\n\rt:%d\n\r",x_velocity,y_velocity,timestamp);
}


// FUNCTIONS :: ACCELERO
void FXOS8700CQ_readRegs(int addr, uint8_t * data, int len) {
   char t = addr;
   i2c.write(m_addr, &t, 1, true);
   i2c.read(m_addr, (char *)data, len);
}

void FXOS8700CQ_writeRegs(uint8_t * data, int len) {
   i2c.write(m_addr, (char *)data, len);
}

void acc_update(){
      FXOS8700CQ_readRegs(FXOS8700Q_OUT_X_MSB, acc_res, 6);
      acc16 = (acc_res[0] << 6) | (acc_res[1] >> 2);
      if (acc16 > UINT14_MAX/2)
         acc16 -= UINT14_MAX;
      acc_t[0] = ((float)acc16) / 4096.0f;
      acc16 = (acc_res[2] << 6) | (acc_res[3] >> 2);
      if (acc16 > UINT14_MAX/2)
         acc16 -= UINT14_MAX;
      acc_t[1] = ((float)acc16) / 4096.0f;
      acc16 = (acc_res[4] << 6) | (acc_res[5] >> 2);
      if (acc16 > UINT14_MAX/2)
         acc16 -= UINT14_MAX;
      acc_t[2] = ((float)acc16) / 4096.0f;
}

// FUNCTIONS :: XBEE
void xbee_rx_interrupt(void){
  xbee.attach(NULL, Serial::RxIrq); // detach interrupt
  xbee_queue.call(&xbee_rx);
}

void xbee_rx(void){
  char buf[100] = {0};
  char outbuf[100] = {0};
  while(xbee.readable()){
    for (int i=0; ; i++) {
      char recv = xbee.getc();
      if (recv == '\r') {
        break;
      }
      buf[i] = pc.putc(recv);
    }
    RPC::call(buf, outbuf);
    printf("we get something: %s\n\r", buf);
    printf("we replied: %s\n\r",outbuf);
    wait(0.05);  
  }
  xbee.printf("%s\n\r",outbuf);
  xbee.attach(xbee_rx_interrupt, Serial::RxIrq); // reattach interrupt
}

void reply_messange(char *xbee_reply, char *messange){
  xbee_reply[0] = xbee.getc();
  xbee_reply[1] = xbee.getc();
  xbee_reply[2] = xbee.getc();
  if(xbee_reply[1] == 'O' && xbee_reply[2] == 'K'){
    printf("%s\r\n", messange);
    xbee_reply[0] = '\0';
    xbee_reply[1] = '\0';
    xbee_reply[2] = '\0';
  }
}

void check_addr(char *xbee_reply, char *messenger){
  xbee_reply[0] = xbee.getc();
  xbee_reply[1] = xbee.getc();
  xbee_reply[2] = xbee.getc();
  xbee_reply[3] = xbee.getc();
  printf("%s = %c%c%c\r\n", messenger, xbee_reply[1], xbee_reply[2], xbee_reply[3]);
  xbee_reply[0] = '\0';
  xbee_reply[1] = '\0';
  xbee_reply[2] = '\0';
  xbee_reply[3] = '\0';
}