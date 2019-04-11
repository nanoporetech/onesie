#include "minion_ioctl.h"

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
        std::array<std::uint16_t, MINION_IOCTL_HS_RECEIVER_REG_SIZE>& data)
{
    // try and open file
    int fd = open(device.c_str(), O_RDWR);
    if (fd < 0) {
        throw std::runtime_error("Failed to open device node");
    }

    struct minion_hs_receiver_s hs_rx_ioctl;
    for (unsigned int i(0); i < MINION_IOCTL_HS_RECEIVER_REG_SIZE ;++i) {
        hs_rx_ioctl.registers[i] = data.at(i);
    }
    hs_rx_ioctl.write = 1;
    const auto rc = ioctl(fd, MINION_IOCTL_HS_RECIEVER, &hs_rx_ioctl);
    if (rc < 0) {
        throw std::runtime_error(strerror(rc));
    }
    for (unsigned int i(0); i < MINION_IOCTL_HS_RECEIVER_REG_SIZE ;++i) {
        data.at(i) = hs_rx_ioctl.registers[i];
    }
}

void usage()
{
    std::cerr
        << "usage minit-hsrx [-wx] <device>\n"
        << " -e, --enable     Set write enable bit\n"
        << " -r, --sync-reset Set the sync-reset bit\n"
        << " -x, --hex        Output will be in hexadecimal csv\n"
        << " -f, --frames     Number of frames in a packet (1-511)\n";
    exit(1);
}

int main(int argc, char* argv[]) {
    bool enable = false;
    bool reset = false;
    bool hex = false;
    unsigned int frames = 1;
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
        if (arg == "-f" || arg == "--frames") {
            ++index;
            std::string field(argv[index]);
            try {
                frames = (unsigned int)std::stoul(field,0,0);
                if (frames> 511) {
                    throw std::out_of_range("bigger than 511");
                }
            } catch(std::invalid_argument& e) {
                std::cerr << "couldn't convert '" << field << "'to a number" << std::endl;
                exit(1);
            } catch(std::out_of_range& e) {
                std::cerr << "'" << field << "'is " << e.what() << std::endl;
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

    std::array<std::uint16_t, MINION_IOCTL_HS_RECEIVER_REG_SIZE> regs{0};
    regs[0] |= enable ? 1 : 0;
    regs[0] |= reset  ? 2 : 0;
    regs[11] = frames;
    try {
        hs_receiver_ioctl(device, regs);
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
