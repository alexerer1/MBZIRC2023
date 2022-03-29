#ifndef DYNAMIXEL_H_
#define DYNAMIXEL_H_

typedef unsigned char byte;

#include <map>
#include <vector>
#include <string>
#include "SerialPort.h"
#include "Utils.h"

static const int BufferSize=1024;

class Dynamixel {
 public:
  Dynamixel();
  Dynamixel(byte id, SerialPort* port);

  void Configure();
  void SetSerialFeedback(bool fb);
  
  byte GetAddress(std::string address);
  byte GetCommand(std::string command);

  int SendReceiveCommand(std::string command, 
			 std::string address, 
			 std::vector<byte> data,
			 std::vector<byte>* outData);

  int FormatCommand(byte command, byte address, 
		    std::vector<byte>, byte* buffer);

  int setID(byte id);
  int setBaudRate(byte baudRate);
  int getPosition();
  int setGoalPosition(int goal);
  int setMovingSpeed(int speed);
  int setCCWAngleLimit(int limit);
  int setCWAngleLimit(int limit);

 private:

  byte _id;
  SerialPort* _port;
  int _recvWaitTimeMS;
  bool _serialFeedback;

 protected:
  std::map<std::string, byte> Addresses;
  std::map<std::string, byte> Commands;
};

class AX12 : public Dynamixel {
 public:
  AX12();
  AX12(byte id, SerialPort* port);

  void Configure();

  static float posToAngle(short pos);
  static short angleToPos(float angle);

  int setCCWComplianceMargin(byte margin);
  int setCWComplianceMargin(byte margin);

  int setCCWComplianceSlope(byte slope);
  int setCWComplianceSlope(byte slope);
};

class MX28 : public Dynamixel {
 public:
  MX28();
  MX28(byte id, SerialPort* port);

  void Configure();

  static float posToAngle(short pos);
  static short angleToPos(float angle);

  int setPGain(byte pGain);
  int setIGain(byte iGain);
  int setDGain(byte dGain);
};

#endif
