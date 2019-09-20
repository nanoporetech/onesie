#include "minion_ioctl.h"

#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <cstdint>
#include <cstdio>

#include <stdexcept>
#include <string>
#include <iostream>

void read_temp_ioctl(
        const std::string& device,
        struct minion_temperature_command_s& temperature_cmd)
{
    int fd = open( device.c_str(), O_RDWR);
    if (fd < 0) {
        throw std::runtime_error("Failed to open device-node");
    }

    if (ioctl(fd, MINION_IOCTL_TEMP_CMD_READ, &temperature_cmd) < 0) {
        throw std::runtime_error(strerror(errno));
    }

    close(fd);
}

void write_temp_ioctl(
        const std::string& device,
        struct minion_temperature_command_s& temperature_cmd)
{
    int fd = open( device.c_str(), O_RDWR);
    if (fd < 0) {
        throw std::runtime_error("Failed to open device-node");
    }

    if (ioctl(fd, MINION_IOCTL_TEMP_CMD_WRITE, &temperature_cmd) < 0) {
        throw std::runtime_error(strerror(errno));
    }

    close(fd);
}

void usage()
{
    std::cerr
        << "usage: minion-temp [-s <temp>] [-f] <device>\n"
        << " -s, --set      Set the desired temperature\n"
        << " -f, --off      Disable temperature control\n\n"
        << "Examples:\n"
        << "    minion-temp <device>           -- read current state of TEC\n"
        << "    minion-temp -s <temp> <device> -- enable temperature control, with a specified target temperature\n"
        << "    minion-temp -f <device>        -- disable temperature control\n";
    exit(1);
}

int main(int argc, char* argv [])
{
    bool set = false;
    bool enable = false;
    double temperature;
    std::string device;

    // parse options
    if (argc == 1) {
        usage();
    }
    for (int index(1); index < argc; ++index) {
        std::string arg(argv[index]);
        if (arg == "-s" || arg == "--set") {
            set = true;
            enable = true;
            ++index;
            std::string field(argv[index]);
            try {
                temperature = std::stod(field,0);
            } catch(std::invalid_argument& e) {
                std::cerr << "couldn't convert '" << field << "'to a number" << std::endl;
                exit(1);
            } catch(std::out_of_range& e) {
                std::cerr << "'" << field << "'is too big" << std::endl;
                exit(1);
            }
            continue;
        }
        if (arg == "-f" || arg == "--off") {
            set = true;
            enable = false;
            continue;
        }

        // assume that arguments not starting with '-' may be device files
        if (arg[0] != '-') {
            // stat the file to see if it exists and is a character device
            struct stat buf;
            if (stat(arg.c_str(),&buf) < 0) {
                std::cerr << "Can't access '" << arg << "' " << strerror(errno) << std::endl;
                exit(errno);
            }
            if (S_ISCHR(buf.st_mode)) {
                device = arg;
                continue;
            } else {
                std::cerr << "File '" << arg << "' is not a character device-node" << std::endl;
                exit(1);
            }
        }

        usage();
    }
    if (device.empty()) {
        std::cerr << "Error: No device node specified." << std::endl;
        usage();
    }


    try {
        struct minion_temperature_command_s temp_command = {};
        if (set) {
            if (enable) {
                temp_command.desired_temperature = temperature * 256.0;
                temp_command.control_word = CTRL_EN_MASK;
            }

            write_temp_ioctl(device, temp_command);
        } else {
            read_temp_ioctl(device, temp_command);
        }

        std::cout << "desired temperature  : " << double(temp_command.desired_temperature) / 256.0
                  << (temp_command.control_word & CTRL_EN_MASK ? " (temperature control enabled)\n": " (temperature control disabled)\n")
                  << "heatsink temperature : " << double(temp_command.heatsink_temperature) / 256.0 << "\n"
                  << "flow-cell temperature: " << double(temp_command.flowcell_temperature) / 256.0 << std::endl;

        std::cout << "error codes: "
                  << (temp_command.error_word & FC_THERM_OPEN ? "flow-cell thermistor open-circuit " :"")
                  << (temp_command.error_word & FC_THERM_SHORT ? "flow-cell thermistor short-circuit " :"")
                  << (temp_command.error_word & FC_THERM_RANGE ? "flow-cell thermistor out of range " :"")
                  << (temp_command.error_word & HSINK_THERM_OPEN ? "heatsink thermistor open-circuit " :"")
                  << (temp_command.error_word & HSINK_THERM_SHORT ? "heatsink thermistor short-circuit " :"")
                  << (temp_command.error_word & HSINK_THERM_RANGE ? "heatsink thermistor out of range " :"") << std::endl;
        return temp_command.error_word ? -1 : 0;
    } catch (std::exception& e) {
        std::cerr << e.what() << std::endl;
        exit(1);
    }
}