#include <PulsePosition.h>
#define NUM_PPM 3
PulsePositionOutput* ppm_output[NUM_PPM];
int ppm_pins[] = {5, 6, 9, 10, 20, 21, 22, 23};

#define LED 13

void setup() {                

  pinMode(LED,OUTPUT);
  
  Serial.begin(115200);

  for(int i=0;i<NUM_PPM;i++)
  {
    ppm_output[i] = new PulsePositionOutput(FALLING);
    ppm_output[i]->begin(ppm_pins[i]);
  }
}


#define PACKET_SIZE 23

union {
  struct{
    unsigned short id; 
    unsigned short ppm_chan[8];
    }data;
    unsigned char str[18];
} RcvPacket;


int receive_packet()
{
  if( (Serial.read()==0x12 && Serial.peek()==0x34) == 0)
    return -1;
  Serial.read();

  unsigned char cs = 0;
  unsigned char ecs = 0;
  for(int i=0;i<18;i++)
  {
    unsigned char rcv = Serial.read();
    RcvPacket.str[i] = rcv;
    cs += rcv;
  }
  ecs = Serial.read();

  
  if( (Serial.read()==0x56 && Serial.read()==0x78) == 0)
    return -2;
  if(ecs!=cs)
    return -3;

  Serial.println(RcvPacket.data.id);
  for(int i=0;i<8;i++)
  {
    Serial.print(RcvPacket.data.ppm_chan[i]);
    Serial.print(" ");
  }
    Serial.println("");

  
  for(int i=0;i<18;i++)
  {
    Serial.print(RcvPacket.str[i],HEX);
    Serial.print(" ");
  }
    Serial.println("");

  return 0;
}

unsigned long t=millis();
volatile bool received;

void set_received_ppm()
{
  received = true;
  if(RcvPacket.data.id<NUM_PPM)
  {
    ppm_output[RcvPacket.data.id]->write(0,0);
    for(int i=0;i<8;i++){
      float ppm = RcvPacket.data.ppm_chan[i]-3;
      ppm_output[RcvPacket.data.id]->write(i+1,ppm);
      Serial.print(ppm);
      Serial.print(" ");
    }
  }
  Serial.println(RcvPacket.data.id);
}

void loop() {
  if(Serial.available()>=PACKET_SIZE)
  {
    int ret = receive_packet();
    if(ret>=0)
    {
      set_received_ppm();
    }
    else{
      //Serial.println(ret);
    }
  }

  if(received && millis()-t<500){
    received = false;
    if(millis()-t<250)
      digitalWrite(LED,HIGH);
    else
      digitalWrite(LED,LOW);
  }
  else if(millis()-t>500){
    t = millis();
    digitalWrite(LED,LOW);
  }
  
}
