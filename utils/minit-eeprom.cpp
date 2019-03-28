#include "ont_minit_ioctl.h"

#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <string>
#include <stdexcept>
#include <array>
#include <errno.h>
void read_eeprom(int fd, std::ostream& out, unsigned int start, unsigned int length)
{
    // prepare and get driver to do transfer
    std::array< char, 256 > buffer;
    struct minit_eeprom_transfer_s eeprom_transaction{
        reinterpret_cast<std::uintptr_t>(buffer.data()),
        start,
        length
    };
    const auto rc = ioctl(fd, MINIT_IOCTL_EEPROM_READ, &eeprom_transaction);
    if (rc < 0) {
        throw std::runtime_error(strerror(errno));
    }

    // dump data to output stream
    out.write(buffer.data(),std::min(length, (unsigned int)buffer.size()));
}

void write_eeprom(int fd, std::istream& in, unsigned int start, unsigned int length)
{
    std::array< char, 128 > buffer;

    // read data from stdin
    in.read(buffer.data(), std::min(length, (unsigned int)buffer.size()));

    // write to driver
    struct minit_eeprom_transfer_s eeprom_transaction{
        reinterpret_cast<std::uintptr_t>(buffer.data()),
        start,
        length
    };
    const auto rc = ioctl(fd, MINIT_IOCTL_EEPROM_WRITE, &eeprom_transaction);
    if (rc < 0) {
        throw std::runtime_error(strerror(errno));
    }
}


void usage()
{
    std::cerr << "usage: minit-eeprom [-rw] [-s start] [-l length] <device>\n"
              << " -r, --read   read from eeprom to stdout\n"
              << " -w, --write  write data from stdin to eeprom\n"
              << " -s, --start <offset bytes>\n"
              << "              start address in decimal or hex if starting with 0x. defaults to 0\n"
              << " -l, --length <size bytes>\n"
              << "              length of transfer in decimal or hex if starting with 0x. defaults to max length \n"
              << "\n"
              << "A read or a write must be requested, reads will be performed before writes." << std::endl;
    exit(1);
}

int main(int argc, char* argv[]) {
    bool read = false;
    bool write = false;
    bool hex = false;
    unsigned int start = 0;
    unsigned int length = 256;
    std::string device;

    // parse options
    if (argc == 1) {
        usage();
    }
    for (int index(1); index < argc; ++index) {
        std::string arg(argv[index]);
        if (arg == "-r" || arg == "--read") {
            read = true;
            continue;
        }

        if (arg == "-w" || arg == "--write") {
            write = true;
            continue;
        }

        if (arg == "-s" || arg == "--start") {
            ++index;
            std::string field(argv[index]);
            try {
                start = (unsigned int)std::stoul(field,0,0);
            } catch(std::invalid_argument& e) {
                std::cerr << "couldn't convert '" << field << "'to a number" << std::endl;
                exit(1);
            } catch(std::out_of_range& e) {
                std::cerr << "'" << field << "'is too big or a negative number" << std::endl;
                exit(1);
            }
            continue;
        }

        if (arg == "-l" || arg == "--length") {
            ++index;
            std::string field(argv[index]);
            try {
                length = (unsigned int)std::stoul(field,0,0);
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

    if (!(read || write)) {
        std::cerr << "Must request read and/or write operation." << std::endl;
        exit(1);
    }

    // open device file
    int fd = open(device.c_str(), O_RDWR);
    if (fd < 0) {
        throw std::runtime_error("Failed to open device node");
    }

    try {
        if (read) read_eeprom(fd, std::cout, start, length);
        if (write) write_eeprom(fd, std::cin, start, length);
    } catch (std::runtime_error& e) {
        std::cerr << e.what() << std::endl;
        exit(1);
    }

    return 0;
}
