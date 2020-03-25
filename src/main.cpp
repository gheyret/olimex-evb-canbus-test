#include <Arduino.h>
#include <WiFi.h>
#include <CAN_config.h>
#include <CAN.h>

#define OLIMEX_REL1_PIN 32
#define OLIMEX_REL2_PIN 33
#define OLIMEX_BUT_PIN 34

CAN_device_t CAN_cfg;

int CAN_init(void);
void setup()
{
    Serial.begin(115200);
    Serial.println("");
    delay(1500);

    pinMode(OLIMEX_REL1_PIN, OUTPUT);
    pinMode(OLIMEX_REL2_PIN, OUTPUT);
    pinMode(OLIMEX_BUT_PIN, INPUT);

    CAN_cfg.tx_pin_id = GPIO_NUM_5;                           // CAN TX pin example menuconfig GPIO_NUM_5
    CAN_cfg.rx_pin_id = GPIO_NUM_35;                          // CAN RX pin example menuconfig GPIO_NUM_35 ( Olimex )
    CAN_cfg.rx_queue = xQueueCreate(20, sizeof(CAN_frame_t)); // FreeRTOS queue for RX frames
    CAN_cfg.speed = CAN_SPEED_250KBPS; // CAN Node baudrade
    CAN_init();
}

void loop() 
{
    CAN_frame_t rx_frame;
    //receive next CAN frame from queue
    if(xQueueReceive(CAN_cfg.rx_queue,&rx_frame, 3*portTICK_PERIOD_MS)==pdTRUE){

      //do stuff!
      if(rx_frame.FIR.B.FF==CAN_frame_std)
        printf("New standard frame");
      else
        printf("New extended frame");

      if(rx_frame.FIR.B.RTR==CAN_RTR)
        printf(" RTR from 0x%08x, DLC %d\r\n",rx_frame.MsgID,  rx_frame.FIR.B.DLC);
      else{
        printf(" from 0x%08x, DLC %d\n",rx_frame.MsgID,  rx_frame.FIR.B.DLC);
      }
    }
}
