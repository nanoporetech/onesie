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
        << " -s, --set      Set the desired temperature, enable temperature control\n"
        << " -f, --off      Disable temperature control\n"
        << " -r, --restart  Restart temperature controller\n"
        << "\n"
        << "Examples:\n"
        << "    minion-temp <device>           -- read current state of TEC\n"
        << "    minion-temp -s <temp> <device> -- enable temperature control, with a specified target temperature\n"
        << "    minion-temp -f <device>        -- disable temperature control\n"
        << "    minion-temp -r <device>        -- restart device temperature controller\n";
    exit(1);
}

int main(int argc, char* argv [])
{
    bool set_arg = false;
    double temperature = 34.0;
    bool off_arg = false;
    bool restart_arg = false;
    std::string device;

    // parse options
    if (argc == 1) {
        usage();
    }
    for (int index(1); index < argc; ++index) {
        std::string arg(argv[index]);
        if (arg == "-s" || arg == "--set") {
            set_arg = true;
            ++index;
            std::string field(argv[index]);
            try {
                temperature = std::stod(field, nullptr);
            } catch(std::invalid_argument&) {
                std::cerr << "couldn't convert '" << field << "'to a number" << std::endl;
                exit(1);
            } catch(std::out_of_range&) {
                std::cerr << "'" << field << "'is too big" << std::endl;
                exit(1);
            }
            continue;
        }
        if (arg == "-f" || arg == "--off") {
            off_arg = true;
            continue;
        }
        if (arg == "-r" || arg == "--restart") {
            restart_arg = true;
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
        if (set_arg || off_arg || restart_arg) {
            temp_command.desired_temperature = std::uint16_t(temperature * 256.0);
            if (set_arg) {
                temp_command.control_word = CTRL_EN_MASK;
            }
            if (off_arg) {
                // "off" overrides "set"
                temp_command.control_word = 0;
            }
            if (restart_arg) {
                // "restart" is compatible with both "off" and "set"
                temp_command.control_word |= CTRL_RESTART;
            }

            write_temp_ioctl(device, temp_command);
        } else {
            read_temp_ioctl(device, temp_command);
        }


        std::cout << "desired temperature  : " << double(temp_command.desired_temperature) / 256.0
                  << (temp_command.control_word & CTRL_EN_MASK ? " (temperature control enabled)\n": " (temperature control disabled)\n")
                  << "heatsink temperature : " << double(temp_command.heatsink_temperature) / 256.0 << "\n"
                  << "flow-cell temperature: " << double(temp_command.flowcell_temperature) / 256.0 << std::endl;

        if (temp_command.error_word) {

            auto check_error_bit = [error_word = temp_command.error_word](
                auto error_bit,
                char const * message
            ) {
                if (error_word & error_bit) {
                    std::cout << "\t" << message << "\n";
                }
            };

            std::cout << "error codes:\n";
            check_error_bit(SENS_I2C_ERR, "Sensor I2C error");
            check_error_bit(FC_SENS_ERR, "Flowcell sensor error");
            check_error_bit(FC_SENS_RANGE, "Flowcell sensor out of range");
            check_error_bit(HSINK_SENS_ERR, "Heatsink sensor error");
            check_error_bit(HSINK_SENS_RANGE, "Heatsink  sensor out of range");
            check_error_bit(FC_HSINK_DELTA, "Flowcell to heatsink difference out of range");
            check_error_bit(ADC_ERR, "Monitor ADC (U10) error");
            check_error_bit(REF_ERR, "2.5V reference out of range");
            check_error_bit(DAC_ERR, "DAC error");
            check_error_bit(SP_LOW_ERR, "Set point low out of range (0V)");
            check_error_bit(SP_HIGH_ERR, "Set point high out of range (2.5V)");
            check_error_bit(SP_MID_ERR, "Set point mid out of range (1.25V)");
            check_error_bit(READY, "Ready (thermal control probed)");
            std::cout << std::endl;
        }

        return temp_command.error_word ? -1 : 0;
    } catch (std::exception& e) {
        std::cerr << e.what() << std::endl;
        exit(1);
    }
}
