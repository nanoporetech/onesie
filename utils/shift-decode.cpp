#include <vector>
#include <iostream>
#include <iomanip>
#include <cassert>

// writes a char of data to the end of a vector of bool
void char_to_bits_lsb(std::vector<bool>& vec, char data)
{
    for (unsigned int i = 0; i < 8; ++i) {
        vec.push_back(data & 0b0000'0001);
        data >>= 1;
    }
}

// extracts bits from vector of bool and converts to some kind of unsignde int
// length is start is 1-biased
template<typename T>
T bits_to_uint(std::vector<bool> const& vec, std::size_t start, std::size_t end = 0)
{
    if (end == 0) {
        end = start;
    }
    assert(start <= end);
    std::size_t length = end - start + 1;

    assert(sizeof(T) * 8 >= length);
    assert(vec.size() >= end);

    T out;
    --start;
    out = vec[start];
    for (unsigned int i = 1; i < length; ++i) {
        out |= vec[start + i] << i;
    }
    return out;
}

__attribute__((noreturn)) void usage()
{
    std::cerr
        << "usage shift-decode [-x][-o][-n]\n"
        << " -x, --hex        Input in hexadecimal csv\n"
        << " -o, --otp        Only decode OTP bits\n"
        << " -n, --not-input-select\n"
        << "                  Don't decode channel input-select bits\n";
    exit(1);
}

int main(int argc, char* argv[])
{
    bool hex = false;
    bool otp = false;
    bool not_input_select = false;

    for (int index(1); index < argc; ++index) {
        std::string arg(argv[index]);
        if (arg == "-x" || arg == "--hex") {
            hex = true;
            continue;
        }
        if (arg == "-o" || arg == "--otp") {
            otp = true;
            continue;
        }
        if (arg == "-n" || arg == "--not-input-select") {
            not_input_select = true;
            continue;
        }
        if (arg == "--help") {
            usage();
        }
    }

    std::vector<bool> shift_reg_raw;

    // read data
    char buffer[256];
    for (unsigned int i = 0; !std::cin.eof(); ++i) {
        unsigned char in;
        if (hex) {
            std::cin.getline(buffer, 256, ',');
            in = std::strtoul(buffer, nullptr, 0);
        } else {
            std::cin >> in;
        }
        char_to_bits_lsb(shift_reg_raw,in);
    }
    // output interpretation of bits
    std::cout << "ADC Reference Trim       " << bits_to_uint<unsigned int >(shift_reg_raw,  1,  5) << std::endl;
    std::cout << "Test Bias Trim           " << bits_to_uint<unsigned int >(shift_reg_raw,  6, 10) << std::endl;
    std::cout << "Sensor Bias Trim +'ve    " << bits_to_uint<unsigned int >(shift_reg_raw, 11, 15) << std::endl;
    std::cout << "Sensor Bias Trim -'ve    " << bits_to_uint<unsigned int >(shift_reg_raw, 16, 20) << std::endl;
    std::cout << "Sensor Bias Offset Trim  " << bits_to_uint<unsigned int >(shift_reg_raw, 21, 26) << std::endl;
    std::cout << "Temp Sensor Slope Trim   " << bits_to_uint<unsigned int >(shift_reg_raw, 27, 34) << std::endl;
    std::cout << "Bandgap Temp Coeff' Trim " << bits_to_uint<unsigned int >(shift_reg_raw, 35, 39) << std::endl;
    std::cout << "Serial Number            " << bits_to_uint<unsigned int >(shift_reg_raw, 40, 56) << std::endl;
    std::cout << "ADC Offset               " << bits_to_uint<unsigned int >(shift_reg_raw, 57, 65) << std::endl;
    std::cout << "Temp Offset              " << bits_to_uint<unsigned int >(shift_reg_raw, 66, 71) << std::endl;
    std::cout << "ASIC Revision            " <<(bits_to_uint<unsigned int >(shift_reg_raw, 72) ? "IA02D" : "IA02C") << std::endl;

    if (otp || shift_reg_raw.size() < 2259) {
        return 0;
    }

    std::cout << "OTP Margin               " << bits_to_uint<unsigned int >(shift_reg_raw, 73) << std::endl;
    std::cout << "OTP Arm to Blow          " << bits_to_uint<unsigned int >(shift_reg_raw, 74) << std::endl;
    std::cout << "OTP Bypass               " << bits_to_uint<unsigned int >(shift_reg_raw, 75) << std::endl;
    std::cout << "OTP Enable               " << bits_to_uint<unsigned int >(shift_reg_raw, 76) << std::endl;
    std::cout << "Channel mux override     " << bits_to_uint<unsigned int >(shift_reg_raw, 77) << std::endl;
    std::cout << "Channel MUX Select       " << bits_to_uint<unsigned int >(shift_reg_raw, 78, 80) << std::endl;
    std::cout << "Integrator Reset Override" << bits_to_uint<unsigned int >(shift_reg_raw, 81) << std::endl;
    std::cout << "Integrator Reset Force   " << bits_to_uint<unsigned int >(shift_reg_raw, 82) << std::endl;
    std::cout << "Select Test Buffer Input " << bits_to_uint<unsigned int >(shift_reg_raw, 83) << std::endl;
    std::cout << "Bypass TestBus Buffer    " << bits_to_uint<unsigned int >(shift_reg_raw, 84) << std::endl;
    std::cout << "EN_REF_SD_AGND_INPUT     " << bits_to_uint<unsigned int >(shift_reg_raw, 85) << std::endl;
    std::cout << "EN_REF_SD_TEST_INPUT     " << bits_to_uint<unsigned int >(shift_reg_raw, 86) << std::endl;
    std::cout << "ADC Stream Out           " << bits_to_uint<unsigned int >(shift_reg_raw, 87, 96) << std::endl;
    std::cout << "ADC Input Override       " << bits_to_uint<unsigned int >(shift_reg_raw, 97) << std::endl;
    std::cout << "ADC Test Input           " << bits_to_uint<unsigned int >(shift_reg_raw, 98, 105) << std::endl;
    std::cout << "Integration Time         " << bits_to_uint<unsigned int >(shift_reg_raw, 106, 115) << std::endl;
    std::cout << "SINC Decimation          " << bits_to_uint<unsigned int >(shift_reg_raw, 116) << std::endl;
    std::cout << "SYS_CLK Divider          " << bits_to_uint<unsigned int >(shift_reg_raw, 117, 121) << std::endl;
    std::cout << "Sensor Bias Adjust       " << bits_to_uint<unsigned int >(shift_reg_raw, 122, 130) << std::endl;
    std::cout << "Unblock Adjust           " << bits_to_uint<unsigned int >(shift_reg_raw, 131, 135) << std::endl;
    std::cout << "Integrator Reset Time    " << bits_to_uint<unsigned int >(shift_reg_raw, 136, 140) << std::endl;
    std::cout << "Integrator Gain          " << bits_to_uint<unsigned int >(shift_reg_raw, 141, 142) << std::endl;
    std::cout << "T/H Sample Time          " << bits_to_uint<unsigned int >(shift_reg_raw, 143, 146) << std::endl;
    std::cout << "SINC3 Delay              " << bits_to_uint<unsigned int >(shift_reg_raw, 147, 150) << std::endl;
    std::cout << "OS Correction Enable     " << bits_to_uint<unsigned int >(shift_reg_raw, 151) << std::endl;
    std::cout << "OS Correction Start      " << bits_to_uint<unsigned int >(shift_reg_raw, 152) << std::endl;
    std::cout << "Test Current Source Off  " << bits_to_uint<unsigned int >(shift_reg_raw, 153) << std::endl;
    std::cout << "Test Current Adjust      " << bits_to_uint<unsigned int >(shift_reg_raw, 154, 156) << std::endl;
    std::cout << "Test Current Duty Cycle  " << bits_to_uint<unsigned int >(shift_reg_raw, 157, 160) << std::endl;
    std::cout << "Overcurrent Disable      " << bits_to_uint<unsigned int >(shift_reg_raw, 161) << std::endl;
    std::cout << "T/H Gain                 " << bits_to_uint<unsigned int >(shift_reg_raw, 162) << std::endl;
    std::cout << "Select Integrator Test   " << bits_to_uint<unsigned int >(shift_reg_raw, 163, 172) << std::endl;
    std::cout << "Select T/H Test          " << bits_to_uint<unsigned int >(shift_reg_raw, 173, 182) << std::endl;
    std::cout << "Input Select Ref.        " << bits_to_uint<unsigned int >(shift_reg_raw, 183, 185) << std::endl;
    std::cout << "Non-Overlap Select       " << bits_to_uint<unsigned int >(shift_reg_raw, 186) << std::endl;
    std::cout << "SINC Test                " << bits_to_uint<unsigned int >(shift_reg_raw, 187, 188) << std::endl;
    std::cout << "EN_REF_INT_TEST_OUT      " << bits_to_uint<unsigned int >(shift_reg_raw, 189) << std::endl;
    std::cout << "EN_REF_T_H_TEST_OUT      " << bits_to_uint<unsigned int >(shift_reg_raw, 190) << std::endl;
    std::cout << "Add 5 micro-Amps         " << bits_to_uint<unsigned int >(shift_reg_raw, 191) << std::endl;
    std::cout << "Add 10 micro-Amps        " << bits_to_uint<unsigned int >(shift_reg_raw, 192) << std::endl;
    std::cout << "Select 7pF               " << bits_to_uint<unsigned int >(shift_reg_raw, 193) << std::endl;
    std::cout << "Select 14pF              " << bits_to_uint<unsigned int >(shift_reg_raw, 194) << std::endl;
    std::cout << "Select 28pF              " << bits_to_uint<unsigned int >(shift_reg_raw, 195) << std::endl;
    std::cout << "LPF Adjust               " << bits_to_uint<unsigned int >(shift_reg_raw, 196, 198) << std::endl;
    std::cout << "Samples to Resets        " << bits_to_uint<unsigned int >(shift_reg_raw, 199, 206) << std::endl;
    std::cout << "Spare! (SBZ)             " << bits_to_uint<unsigned int >(shift_reg_raw, 207) << std::endl;
    std::cout << "Sensor Bias Test Enable  " << bits_to_uint<unsigned int >(shift_reg_raw, 208) << std::endl;
    std::cout << "Bandgap Out Test Enable  " << bits_to_uint<unsigned int >(shift_reg_raw, 209) << std::endl;
    std::cout << "Spare! (SBZ)             " << bits_to_uint<unsigned int >(shift_reg_raw, 210, 211) << std::endl;

    if (not_input_select) {
        return 0;
    }

    for (unsigned int channel = 0; channel < 512; ++channel) {
        auto start = 212 + (channel * 4);
        std::cout << "Channel " << std::setw(3) << channel + 1
                  << " Input Select " << bits_to_uint<unsigned int >(shift_reg_raw, start, start + 3) << std::endl;
    }

    return 0;
}
