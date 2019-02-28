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
        const unsigned int frequency = 50000)
{
    // try and open file
    int fd = open(device.c_str(), O_RDWR);
    if (fd < 0) {
        throw std::runtime_error("Failed to open device node");
    }

    struct minit_shift_reg_s shift_ioctl{
        POINTER_TO_U64(nullptr),
        POINTER_TO_U64(data.data()),
        frequency,
        start,
        enable
    };
    const auto rc = ioctl(fd, MINIT_IOCTL_SHIFT_REG, &shift_ioctl);
    if (rc < 0) {
        throw std::runtime_error(strerror(errno));
    }
}

void usage()
{
    std::cerr
        << "usage minit-shift [-x] [-e] [-s] [-f frequency ] <device>\n"
        << " -x, --hex        Output will be in hexadecimal csv\n"
        << " -e, --enable     Set the mod-enable bit\n"
        << " -s, --start      Set the start-bit\n"
        << " -f, --frequency  Set the interface clock-frequency in Hz (default 492,125.9 MHz)\n";
    exit(1);
}

int main(int argc, char* argv[]) {
    bool hex = false;
    bool enable = false;
    bool start = false;
    std::uint32_t frequency = 492125; // as slow as it'll go
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
        if (arg == "-e" || arg == "--enable") {
            enable = true;
            continue;
        }
        if (arg == "-s" || arg == "--start") {
            start = true;
            continue;
        }
        if (arg == "-f" || arg == "--frequency") {
            ++index;
            std::string field(argv[index]);
            try {
                frequency = (unsigned int)std::stoul(field,0,0);
            } catch(std::invalid_argument& e) {
                std::cerr << "couldn't convert '" << field << "'to a number" << std::endl;
                exit(1);
            } catch(std::out_of_range& e) {
                std::cerr << "'" << field << "'is too big or a negative number" << std::endl;
                exit(1);
            }
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
        shift_ioctl(device, data, start, enable, frequency);
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
