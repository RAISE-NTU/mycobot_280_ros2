#include <iostream>
#include <thread>
#include <chrono>
#include "mycobot_hardware_interface/mycobot_serial.hpp"

using namespace mycobot_hardware_interface;

int main() {
    std::cout << "Opening /dev/ttyACM0 in C++..." << std::endl;
    MyCobotSerial serial;
    try {
        serial.open("/dev/ttyACM0", 115200);
        std::cout << "Port opened successfully!" << std::endl;
    } catch (const std::exception &e) {
        std::cout << "Error opening port: " << e.what() << std::endl;
        return 1;
    }

    std::cout << "Waiting 3.5 seconds for M5Stack boot ('OK\\r\\n') to complete..." << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(3500));

    for (int i = 1; i <= 10; ++i) {
        std::cout << "Attempt " << i << ": testing get_angles()..." << std::endl;
        std::vector<double> angles;
        if (serial.get_angles(angles)) {
            std::cout << "  SUCCESS! Angles (deg): ";
            for (double a : angles) std::cout << a << " ";
            std::cout << std::endl;
            break;
        } else {
            std::cout << "  Failed to read angles." << std::endl;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    serial.close();
    return 0;
}
