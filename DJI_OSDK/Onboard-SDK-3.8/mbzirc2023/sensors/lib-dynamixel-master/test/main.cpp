#include <string>
#include <iostream>
#include "dynamixel/Dynamixel.h"

int main(int argc, char** argv) {
  bool serialFeedback = false;
  int motorId = 1;
  int numBytes = 1;
  int iData = 2;
  std::string command = "Get";
  std::string address = "Position";
  std::string motorType = "MX28";
  std::string portName = "/dev/ttyO5";
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
      motor = new AX12(motorId, &port);
    }
    else if (motorType == "MX28") {
      mx28 = MX28(motorId, &port);
      motor = new MX28(motorId, &port);
    }
    else {
      std::cout << "Error: motor type not supported!\n";
      return -1;
    }

    motor->Configure();
    motor->SetSerialFeedback(serialFeedback);

    // For debugging only:
    byte buffer[1024];
    int length = motor->
      FormatCommand(motor->GetCommand(command),
		    motor->GetAddress(address),
		    data,
		    buffer);

    std::cout<< "buffer: " <<
      Utils::PrintBuffer(buffer, length) << std::endl;
    // end for debugging

    int retVal;
    retVal = motor->SendReceiveCommand(command,
				       address,
				       data,
				       &recvData);
				       
    int recvVal = 0;
    if (recvData.size() == 1) {
      recvVal = recvData[0];
    }
    else if (recvData.size() == 2) {
      recvVal = Utils::ConvertFromHL(recvData[0],recvData[1]);
    }
    std::cout << "received: " <<
      retVal << " : " << recvVal << std::endl;

    std::cout << "position: " << 
      motor->getPosition() << std::endl;
  }
  else {
    std::cout << "Couldn't open " << 
      portName << " at baudRate " << 
      baudRate << std::endl;
    return -1;
  }

  return 0;
}
