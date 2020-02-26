#include <vector>
#include <iostream>
#include <cassert>

// writes a char of data to the end of a vector of bool
void char_to_bits_lsb(std::vector<bool>& vec, char data)
{
    for (unsigned int i = 0; i < 8; ++i) {
        vec.push_back(data & 0b0000'0001);
        data >>= 1;
    }
}

// writes a char of data to the end of a vector of bool
void char_to_bits_msb(std::vector<bool>& vec, char data)
{
    for (unsigned int i = 0; i < 8; ++i) {
        vec.push_back(data & 0b1000'0000);
        data <<= 1;
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
    ++start;
    out = vec[start];
    for (unsigned int i = 1; i < length; ++i) {
        out <<=1;
        out |= vec[start + i];
    }
    return out;
}


int main(int, char**)
{
    std::vector<bool> otp_raw;

    // read data
    for (unsigned int i = 0; i < 9; ++i) {
        char in;
        std::cin >> in;
        char_to_bits_lsb(otp_raw,in);
    }

    // output interpretation of bits
    std::cout << "ADC Reference Trim       " << bits_to_uint<unsigned int >(otp_raw,  1,  5) << std::endl;
    std::cout << "Test Bias Trim           " << bits_to_uint<unsigned int >(otp_raw,  6, 10) << std::endl;
    std::cout << "Sensor Bias Trim +'ve    " << bits_to_uint<unsigned int >(otp_raw, 11, 15) << std::endl;
    std::cout << "Sensor Bias Trim -'ve    " << bits_to_uint<unsigned int >(otp_raw, 16, 20) << std::endl;
    std::cout << "Sensor Bias Offset Trim  " << bits_to_uint<unsigned int >(otp_raw, 21, 26) << std::endl;
    std::cout << "Temp Sensor Slope Trim   " << bits_to_uint<unsigned int >(otp_raw, 27, 34) << std::endl;
    std::cout << "Bandgap Temp Coeff' Trim " << bits_to_uint<unsigned int >(otp_raw, 35, 39) << std::endl;
    std::cout << "Serial Number            " << bits_to_uint<unsigned int >(otp_raw, 40, 56) << std::endl;
    std::cout << "ADC Offset               " << bits_to_uint<unsigned int >(otp_raw, 57, 65) << std::endl;
    std::cout << "Temp Offset              " << bits_to_uint<unsigned int >(otp_raw, 66, 71) << std::endl;
    std::cout << "ASIC Revision            " << (otp_raw[71] ? "IA02D" : "IA02C") << std::endl;

    return 0;
}
