#include "Controller.h"
#include "XBee.h"
#include "define.h"

XBee xbee = XBee();
XBeeResponse response = XBeeResponse();
// create reusable response objects for responses we expect to handle 
ZBRxResponse rx = ZBRxResponse();
//ModemStatusResponse msr = ModemStatusResponse();

Controller::Controller(HardwareSerial *_Ser)
{
    Ser = _Ser;

    ButtonState = 0;
    LJoyX = 127;
    LJoyY = 127;
    RJoyX = 127;
    RJoyY = 127;

    ConAvailable = true;
}

void Controller::begin(int baudrate){
    Ser->begin(baudrate);
}
void Controller::begin_api(int baudrate){
    SERIAL_XBEE.begin(baudrate);
    xbee.begin(SERIAL_XBEE);
}

bool Controller::update(byte PinName)
{
  unsigned int checksum;
  //uint8_t tmp;
  char c;
  while (Ser->available())
  {
    c = Ser->read();
    if (c == '\n')
    {
      if (recv_num == 10)
      {
        checksum = 0;

        for (int i = 0; i < 9; i++)
        {
          raww_recv_msgs[i] = recv_msgs[i];
          recv_msgs[i] = recv_msgs[i] - 0x20;
          checksum += (unsigned int)recv_msgs[i];
        }

        if ((checksum & 0x3F) == (recv_msgs[9] - 0x20))
        {
          digitalWrite(PinName, !digitalRead(PinName));

          ButtonState = 0;//, LJoyX = 0, LJoyY = 0, RJoyX = 0, RJoyY = 0;

          ButtonState |= recv_msgs[0] & 0x3F;
          ButtonState |= (recv_msgs[1] & 0x3F) << 6;
          ButtonState |= (recv_msgs[2] & 0x0F) << 12;

          LJoyX = recv_msgs[3];
          LJoyX |= (recv_msgs[4] & 0x03) << 6;

          LJoyY = (recv_msgs[4] & 0x3C) >> 2;
          LJoyY |= (recv_msgs[5] & 0x0F) << 4;

          RJoyX = (recv_msgs[5] & 0x30) >> 4;
          RJoyX |= (recv_msgs[6] & 0x3F) << 2;

          RJoyY = recv_msgs[7];
          RJoyY |= (recv_msgs[8] & 0x03) << 6;
          //preButtonState = ButtonState;
          
          return true;
        }
      }
      recv_num = 0;
    }
    else
    {
      recv_msgs[recv_num++] = c;
    }
  }
          preButtonState = ButtonState;
  return false;
}

void Controller::update_api(byte PinName)
{
    
  xbee.readPacket();

  if (xbee.getResponse().isAvailable())
  {
    // got something
    digitalWrite(PinName,HIGH);
    if (xbee.getResponse().getApiId() == ZB_RX_RESPONSE)
    {
      // got a zb rx packet
      // now fill our zb rx class
      xbee.getResponse().getZBRxResponse(rx);
        static int recive_num[9] = {};
      for(int i = 0; i < 9; i++){
        recive_num[i] = rx.getData(i);
      }
      preButtonState = ButtonState;
      ButtonState = 0;          
      ButtonState |= recive_num[0] & 0x3F;
      ButtonState |= (recive_num[1] & 0x3F) << 6;
      ButtonState |= (recive_num[2] & 0x0F) << 12;

      LJoyX = recive_num[3];
      LJoyX |= (recive_num[4] & 0x03) << 6;

      LJoyY = (recive_num[4] & 0x3C) >> 2;
      LJoyY |= (recive_num[5] & 0x0F) << 4;

      RJoyX = (recive_num[5] & 0x30) >> 4;
      RJoyX |= (recive_num[6] & 0x3F) << 2;

      RJoyY = recive_num[7];
      RJoyY |= (recive_num[8] & 0x03) << 6;
    }
  }
  else
  {
    digitalWrite(PinName,LOW); 
  } 
}

void Controller::update_api_DR(byte PinName)
{
    
  xbee.readPacket();

  if (xbee.getResponse().isAvailable())
  {
    // got something
    digitalWrite(PinName,HIGH);
    if (xbee.getResponse().getApiId() == ZB_RX_RESPONSE)
    {
      // got a zb rx packet
      // now fill our zb rx class
      xbee.getResponse().getZBRxResponse(rx);
        static int recive_num_DR[5] = {};
      for(int i = 0; i < 5; i++)
      {
        recive_num_DR[i] = rx.getData(i);
      }
      ButtonState = recive_num_DR[0] & 0xFF;

      LJoyX = recive_num_DR[1] & 0xFF;
      LJoyY = recive_num_DR[2] & 0xFF;
      RJoyX = recive_num_DR[3] & 0xFF;
      RJoyY = recive_num_DR[4] & 0xFF;
    }
  }
  else
  {
    digitalWrite(PinName,LOW); 
  } 
}

int Controller::setAvailable(bool choose)
{
  static int count_PAD = 0;
  int num = 0;
  if(choose) 
  {
    ConAvailable = true;
  
    if(ButtonState & BUTTON_PAD) num += 1;
    if(~preButtonState & BUTTON_PAD) num += 1;
    if(num == PUSHED) count_PAD++;
    if(count_PAD % 2 == 0) ConAvailable = true;
    else ConAvailable = false;  
  }
  else
  {
    ConAvailable = false;
  }
  //preButtonState = ButtonState;
  return count_PAD;
}
 
bool Controller::readButton(unsigned int button,int status)
{
  int8_t num = 0;
  if(!ConAvailable) return false;
  if(getButtonState() & button)  num += 2;
  if(getpreButtonState() & button) num -= 1;
  if(num == status) return true;
  else return false;
  //preButtonState = ButtonState;
}

unsigned int Controller::getButtonState() const
{
  if(ConAvailable) return ButtonState;
  else return 0;
}
unsigned int Controller::getpreButtonState() const
{
  if(ConAvailable) return preButtonState;
  else return 0;
}

/**コントローラの入力に変化があったらtrueを返す**/
bool Controller::getButtonChanged() const
{
  if(ConAvailable) return buttonChanged;
  else return false;
}

unsigned int Controller::readJoy(int joy)
{
  unsigned int out;
  switch (joy)
  {
  case LX: out = LJoyX; break;
  case LY: out = LJoyY; break;
  case RX: out = RJoyX; break;
  case RY: out = RJoyY; break;
  default: out = 127;   break;
  }
  if(ConAvailable) return out;
  else return 127;
}