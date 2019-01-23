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

using namespace std;

char* exe_name = NULL;


class bad_parameters : public exception
{
};

int reg_access_ioctl(
    char* filename,
    unsigned int bar,
    unsigned int offset,
    std::uint64_t value,
    unsigned int size,
    bool read,
    bool read_all)
{
    bool success = false;
    // try and open file
    int fd = open(filename, O_RDWR);
    if (fd < 0) {
        std::cerr << exe_name << " : Failed to open device node '" << filename << "'" << std::endl;
        return fd;
    }

    // perform ioctl
    int rc = 0;
    do {
        // make structure
        struct minit_register_s reg_ioctl = {
            offset,
            uint16_t(size),
            uint8_t(read ? 0 : 1),
            uint8_t(bar),
            value
        };

        rc = ioctl(fd, MINIT_IOCTL_REG_ACCESS, &reg_ioctl);
        if (rc < 0) {
            if (!success) {
                cerr << "Error " << rc << " (" << strerror(-rc) << ")" << endl;
            }
            break;
        }
        success = true;

        // if this is a read then output the value we've read
        if (read) {
            std::printf("bar %d offset 0x%08x value 0x%0*llx\n", bar, offset, size * 2, reg_ioctl.value);
        }
        offset += size;
        // if we want to read all the values in a bar then loop until something complains
    } while (read_all);

    // close file
    close(fd);

    return rc;
}


void help()
{
    std::cout <<
    "Read all registers in the specified bar\n"
    "    <device> <bar>\n"
    "Read or write the 32-bit register at specified byte-offset\n"
    "    <device> <bar> <offset> [value]\n"
    "Read the register at specified byte-offset with the specified length\n"
    "    <device> <bar> <offset> [read8/16/32/64]\n"
    "Write the register at specified byte-offset with the specified length\n"
    "    <device> <bar> <offset> [write8/16/32/64] <value>" << std::endl;
}

int main(int argc, char* argv[]) 
{
    char* device_filename;
    unsigned int bar = 0;
    unsigned int offset = 0;
    uint64_t value = 0;
    bool read = true;
    bool read_all = false;
    unsigned int size = 4;

    // parse options
    try {
        // if too few options then display help
        if (argc < 3) {
            throw bad_parameters();
        }
        device_filename = argv[1];
        bar = std::stol(std::string(argv[2]));

        // if no offset then read all registers
        if (argc < 4) {
            read_all = true;
        } else {
            read_all = false;
            offset = std::stol(std::string(argv[3]),NULL, 0);

            // is their a value or more options
            if (argc == 5) {
                std::string option4(argv[4]);
                if (option4 == "read8") {
                    read = true;
                    size = 1;
                } else if (option4 == "read16") {
                    read = true;
                    size = 2;
                } else if (option4 == "read32") {
                    read = true;
                    size = 4;
                } else if (option4 == "read64") {
                    read = true;
                    size = 8;
                } else {
                    // expect a value for a write if not a read of a defined size
                    read = false;
                    size = 4;
                    value = std::stol(option4,NULL, 0);
                }
            } else if (argc == 6) {
                std::string option4(argv[4]);
                std::string option5(argv[5]);
                if (option4 == "write8") {
                    read = false;
                    size = 1;
                } else if (option4 == "write16") {
                    read = false;
                    size = 2;
                } else if (option4 == "write32") {
                    read = false;
                    size = 4;
                } else if (option4 == "write64") {
                    read = false;
                    size = 8;
                } else {
                    throw bad_parameters();
                }
                value = std::stol(option5,NULL, 0);
            }
        }
    } catch (bad_parameters e) {
        help();
        return -1;
    } catch (...) {
        return -1;
    }

    return reg_access_ioctl(device_filename, bar, offset, value, size, read, read_all);
}
