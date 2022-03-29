#include <string>
#include <iostream>
#include "dynamixel/Dynamixel.h"

int main(int argc, char** argv) {
  bool serialFeedback = false;
  int motorId = 6;
  int numBytes = 1;
  int iData = 2;
  std::string command = "Get";
  std::string address = "Position";
  std::string motorType = "AX12";
  std::string portName = "/dev/ttyUSB0";
  int baudRate = 9600;

  // motor objects
  Dynamixel* motor;
  AX12 ax12;
  MX28 mx28;

  std::vector<byte> data;
  std::vector<byte> recvData;

  // parse command line args
  for (int i=1; i<argc; i++) {
    if (!strcmp(argv[i],"--baudRate")) {
      baudRate = atoi(argv[++i]);
    }
    else if (!strcmp(argv[i],"--serialFeedback")) {
      serialFeedback = true;
    }
    else if (!strcmp(argv[i],"--motorId")) {
      motorId = atoi(argv[++i]);
    }
    else if (!strcmp(argv[i],"--numBytes")) {
      numBytes = atoi(argv[++i]);
    }
    else if (!strcmp(argv[i],"--command")) {
      command = argv[++i];
    }
    else if (!strcmp(argv[i],"--address")) {
      address = argv[++i];
    }
    else if (!strcmp(argv[i],"--portName")) {
      portName = argv[++i];
    }
    else if (!strcmp(argv[i],"--motorType")) {
      motorType = argv[++i];
    }
    else if (!strcmp(argv[i],"--data")) {
      iData = std::strtoul(argv[++i], 0, 10);
    }
  }

  if (numBytes == 1) {
    data.push_back(iData);
  }
  else if (numBytes == 2) {
    byte h, l;
    Utils::ConvertToHL(iData, &h, &l);
    data.push_back(l);
    data.push_back(h);
  }

  SerialPort port;
  std::cout << "Connecting to: " << 
    portName << ":" << baudRate << std::endl;
  if (port.connect((char *)portName.c_str(), baudRate) != 0) {

    std::cout << "Success\n";

    // configure the motor objects
    if (motorType == "AX12") {
      ax12 = AX12(motorId, &port);
      std::cout << "new ax12 Success\n";
      motor = new AX12(motorId, &port);
      std::cout << "new motor Success\n";
    }
    else if (motorType == "MX28") {
      mx28 = MX28(motorId, &port);
      motor = new MX28(motorId, &port);
    }
    else {
      std::cout << "Error: motor type not supported!\n";
      return -1;
    }
    std::cout<< "initialize motor success";
    motor->Configure();
    motor->SetSerialFeedback(serialFeedback);
    std::cout<< "configure success: ";
  
    motor->setGoalPosition(50);
    std::cout<< "setGoalPosition success: ";
  }
  else {
    std::cout << "Couldn't open " << 
      portName << " at baudRate " << 
      baudRate << std::endl;
    return -1;
  }

  return 0;
}
