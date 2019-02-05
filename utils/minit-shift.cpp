#include "ont_minit_ioctl.h"

#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <cstdint>
#include <cstdio>
#include <iostream>
#include <iomanip>
#include <string>
#include <array>
#include <stdexcept>

const std::size_t asic_shift_reg_size(0x11a);

void shift_ioctl(
        const std::string& device,
        std::array<std::uint8_t, asic_shift_reg_size>& data,
        const bool start = 1,
        const bool enable = 1,
        const unsigned int speed = 50000)
{
    // try and open file
    int fd = open(device.c_str(), O_RDWR);
    if (fd < 0) {
        throw std::runtime_error("Failed to open device node");
    }

    struct minit_shift_reg_s shift_ioctl{
        nullptr,
        data.data(),
        speed,
        start,
        enable
    };
    const auto rc = ioctl(fd, MINIT_IOCTL_SHIFT_REG, &shift_ioctl);
    if (rc < 0) {
        throw std::runtime_error(strerror(-rc));
    }
}

void usage()
{
    std::cerr
        << "usage minit-shift [-x] <device>\n"
        << " -x, --hex        Output will be in hexadecimal csv\n";
    exit(1);
}

int main(int argc, char* argv[]) {
    bool hex = false;
    std::string device;

    // parse options
    if (argc == 1) {
        usage();
    }
    for (int index(1); index < argc; ++index) {
        std::string arg(argv[index]);
        if (arg == "-x" || arg == "--hex") {
            hex = true;
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

    std::array<std::uint8_t, asic_shift_reg_size> data{0};

    try {
        shift_ioctl(device, data);
    } catch (std::exception& e) {
        std::cerr << e.what() << std::endl;
        exit(1);
    }

    if (hex) {
        for (unsigned int i(0); i < data.size() ; ++i) {
            const std::uint8_t byte = data[i] ;
            if (i != 0) {
                printf(", ");
            }
            printf("0x%02x",byte);
        }
        std::cout << std::endl;
    } else {
        std::cout.write(reinterpret_cast<char*>(data.data()), data.size());
        // todo
    }
    return -1;
}
