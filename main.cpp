#include <iostream>
#include <string>
#include "V4l2Camera.hpp"
#include <iostream>

constexpr char CAM_DEV_PATH[] = "/dev/video0";

int main() {
  V4l2Camera camera{CAM_DEV_PATH};
  camera.open();
  auto mds = camera.getSensorModes();
  auto ctrls = camera.getControlParameters();
  camera.setSensorMode(mds[3]);
  camera.startCapturing();

  const auto &a = camera.getNewImage();
  std::cout << "w " << a.getWidth() << std::endl;
  std::cout << "h " << a.getHeight() << std::endl;

  camera.stopCapturing();
  camera.close();

  return 0;
}
