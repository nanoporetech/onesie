/**
 * This should read in a binary file output from minion-dma via stdin and audit the data
 * checking for gaps in frame-numbers, consistent asic-id, sampling frequency, etc.
 */

#include <string>
#include <iostream>
#include <array>
#include <stdexcept>

static const std::size_t frame_length = 528; //522 D-words
typedef std::array<std::uint16_t, frame_length> frame_t;

int main(int argc, char* argv[])
{
    std::istream& in = std::cin;
    frame_t frame;
    std::uint32_t max_frame_no = (1 << 24) - 1;

    try {
        std::size_t pos = 0; // byte index
        std::uint32_t frame_no = 0;
        std::uint16_t sampling_freq;
        std::uint32_t asic_id;
        std::uint16_t bias_volt;
        std::uint16_t temp;
        std::uint16_t asic_config_id;

        // check a 16-bit value in the frame matches an expected value, update if different
        const auto check_and_update_word = [&](const unsigned int index, std::uint16_t& value, const std::string& name)->void
        {
            if (frame[index] != value) {
                if (frame_no > 0) {
                    std::cout << name << " has changed (expected " << value
                              << " now " << frame[index] << ") index " << pos + (index * 2)
                              << std::endl;
                } else {
                    std::cout << "Initial value of " << name << " is "
                              << frame[index] << std::endl;
                }
                value = frame[index];
            }
        };
        // check a 32-bit value in the frame matches an expected value, update if different
        const auto check_and_update_quad = [&](const unsigned int index, std::uint32_t& value, const std::string& name)->void
        {
            std::uint32_t quad = frame[index+1];
            quad <<= 16;
            quad |= frame[index];

            if (quad != value) {
                if (frame_no > 0) {
                    std::cout << name << " has changed (expected " << value
                              << " now " << quad << ") index " << pos + (index * 2)
                              << std::endl;

                } else {
                    std::cout << "Initial value of " << name << " is "
                              << quad << std::endl;
                }
                value = quad;
            }
        };

        // check a 32-bit value in the frame matches an expected value
        const auto check_quad = [&](const unsigned int index, std::uint32_t value, const std::string& name)->void
        {
            std::uint32_t quad = frame[index+1];
            quad <<= 16;
            quad |= frame[index];

            if (quad != value) {
                std::cout << name << " is incorrect (expected " << value
                          << " now " << quad << ") index " << pos + (index * 2)
                          << std::endl;
            }
        };

        std::cout << std::hex;
        while (!in.eof()) {
            in.read((char*)frame.data(), frame.size() * 2);

            check_and_update_quad(514, frame_no, "frame-no");
            check_and_update_word(516, sampling_freq, "sampling frequency");
            check_and_update_quad(517, asic_id, "ASIC-ID");
            check_and_update_word(519, bias_volt, "bias voltage");
            check_and_update_word(520, temp, "heatsink temp");
            check_and_update_word(521, asic_config_id, "asic_config_id");
            check_quad(           526, 0xbabecafe, "end of frame marker");

            // increment within 24-bit limits
            ++frame_no;
            if (frame_no == max_frame_no) {
                frame_no = 0;
            }

            // increment byte-index ready for next frame
            pos += frame_length * 2;
        }
    } catch (std::runtime_error& e) {
        std::cerr << e.what() << std::endl;
        return 1;
    }

    return 0;
}
