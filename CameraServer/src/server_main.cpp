#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <signal.h>
#include <unistd.h>

#include "CameraServer.hpp"

bool running = true;

void signal_handler(int s)
{
  printf("Caught signal %d\n", s);
  running = false;
}

int main(int argc, char const *argv[])
{

  std::cout << "Welcome to the camera server!!\n";

  std::cout << "REMBER SUDO IF FILES SHOULD BE SAVED!!!\n";
  std::cout << "\n#############################\n\n";
  std::cout << "REMBER SUDO IF FILES SHOULD BE SAVED!!!\n";
  std::cout << "\n#############################\n\n";
  std::cout << "REMBER SUDO IF FILES SHOULD BE SAVED!!!\n";
  std::cout << "\n#############################\n\n";
  std::cout << "REMBER SUDO IF FILES SHOULD BE SAVED!!!\n";
  std::cout << "\n#############################\n\n";
  std::cout << "REMBER SUDO IF FILES SHOULD BE SAVED!!!\n";

  signal(SIGINT, signal_handler);

  CameraServer server(640, 480);
  // CameraServer server(1280, 960);

  if (!server.init_camera())
    return 0;

  // server.set_loc_type(NONE);
  // server.set_loc_type(FIND_BALLOON);
  // server.set_loc_type(DRAW_CROSS);
  server.set_loc_type(FIND_NAVAL_BOX);

  server.angle_offset = 0.0 * 3.14159 / 180.0;

  // server.draw_cross = true;

  // server.draw_box = true;
  server.show_image = true;

  while (running)
  {

    //         std::cout << "\033[2J";
    //         std::cout << "\033[33;45m";
    std::cout << "Elapsed time = " << server.server_run_time << "  ||   FPS = " << server.fps << std::endl;
    usleep(500000);
  }

  std::cout << "Closing server\n";

  server.stop_server();

  return 0;
}
