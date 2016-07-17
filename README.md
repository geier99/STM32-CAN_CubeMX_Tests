# STM32-CAN_CubeMX_Tests
Demonstrate the  use from STM32CubeMX for my STM32-CAN hardware

The following is implemented V0.1:
* LED port handling (blue Led toggles everytime then a CAN message is received)
* Intialize CAN1 for transmitting and receiving CAN-Frames at 500 kBit
* USB communication as VCP  (Virtual Com Port), green led toggles on receiving a lawicel line
* Skeloton for the Lawicel Protocol: 


V0.2:
- add Lawicel commands:
  -  baudrate settings (and some custom baudrate)  ('s' and 'S')
  -  open and close CAN channel ('O' and 'C')
     if the Channel is opened, then teh red led ist permanent on.
  -  open channel in listen only mode ('L')   
  -  settings for timestamp  ('Z')
- the testmessage is send after power up, but now then the channel is closed by lawicel protocol, the transmission is stopped.
  Only then the channel is opened again by lawicel protocol, then the transmission is started again.
- Remember: on receiving CAN frames only the blue led is toggeled. 
  
Todos:
* CAN2 
* DIP switch
* complete the Lawicel Protocol (receiving and sending CAN frames by the lawicel protocol)
