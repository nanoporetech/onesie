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

void hs_receiver_ioctl(
        const std::string& device,
        std::array<std::uint16_t, MINIT_IOCTL_HS_RECEIVER_REG_SIZE>& data,
        const bool write)
{
    // try and open file
    int fd = open(device.c_str(), O_RDWR);
    if (fd < 0) {
        throw std::runtime_error("Failed to open device node");
    }

    struct minit_hs_receiver_s hs_rx_ioctl;
    for (unsigned int i(0); i < MINIT_IOCTL_HS_RECEIVER_REG_SIZE ;++i) {
        hs_rx_ioctl.registers[i] = data.at(i);
    }
    hs_rx_ioctl.write = 1;
    const auto rc = ioctl(fd, MINIT_IOCTL_HS_RECIEVER, &hs_rx_ioctl);
    if (rc < 0) {
        throw std::runtime_error(strerror(rc));
    }
    for (unsigned int i(0); i < MINIT_IOCTL_HS_RECEIVER_REG_SIZE ;++i) {
        data.at(i) = hs_rx_ioctl.registers[i];
    }
}

void usage()
{
    std::cerr
        << "usage minit-hsrx [-wx] <device>\n"
        << " -e, --enable     Set write enable bit\n"
        << " -r, --sync-reset Set the sync-reset bit\n"
        << " -x, --hex        Output will be in hexadecimal csv\n";
    exit(1);
}


/*
 * usage minit-hsrx [-erx] <device>
 * -e, --enable     set write enable bit
 * -r, --sync-reset set the sync-reset bit
 * -x, --hex input and output will be in hexadecimal csv
 */

int main(int argc, char* argv[]) {
    bool enable = false;
    bool reset = false;
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
        if (arg == "-e" || arg == "--enable") {
            enable = true;
            continue;
        }
        if (arg == "-r" || arg == "--sync-reset") {
            reset = true;
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

    std::array<std::uint16_t, MINIT_IOCTL_HS_RECEIVER_REG_SIZE> regs{0};
    regs[0] |= enable ? 1 : 0;
    regs[0] |= reset  ? 2 : 0;

    try {
        hs_receiver_ioctl(device, regs, write);
    } catch (std::exception& e) {
        std::cerr << e.what() << std::endl;
        exit(1);
    }

    if (hex) {
        for (unsigned int i(0); i < regs.size() ; ++i) {
            const std::uint16_t word = regs[i] ;
            if (i != 0) {
                printf(", ");
            }
            printf("0x%04x",word);
        }
        std::cout << std::endl;
    } else {
        std::cout.write(reinterpret_cast<char*>(regs.data()),regs.size() * sizeof(std::uint16_t) );
        // todo
    }
    return -1;
}
