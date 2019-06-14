#include "minion_ioctl.h"

#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <csignal>
#include <iostream>
#include <string>
#include <stdexcept>
#include <array>
#include <deque>
#include <errno.h>
#include <future>
#include <atomic>
#include <vector>
#include <memory>
#include <chrono>
#include <algorithm>

static_assert(sizeof(struct minion_data_transfer_s) == MINION_DATA_TRANSFER_SIZE , "ioctl size wrong");

static const unsigned int frame_size = 1056;
static const unsigned int max_frames_per_packet = 511;
static const unsigned int MAX_QUEUE_SIZE = 128;

template<class T, std::size_t alignment>
struct AlignedAllocator {
    typedef std::size_t size_type;
    typedef std::ptrdiff_t difference_type;
    typedef T* pointer;
    typedef const T* const_pointer;
    typedef T& reference;
    typedef const T& const_reference;
    typedef T value_type;

    template <class U>
    struct rebind
    {
        typedef AlignedAllocator<U, alignment> other;
    };

    AlignedAllocator() noexcept {}
    template<class U, std::size_t x> AlignedAllocator( const AlignedAllocator<U,x>&) noexcept {}

    T* allocate (std::size_t n) {
        // allocate space for n objects, a pointer and wiggle-room
        T* mem = reinterpret_cast<T*>(::operator new(n*sizeof(T) + sizeof(std::uintptr_t) + alignment));
        // pointer will always be first, so introduce an offset for that
        std::uintptr_t p_mem = reinterpret_cast<std::uintptr_t>(mem) + sizeof(std::uintptr_t);

        // align the buffer
        if (p_mem & (alignment-1)) {
            p_mem += alignment - (p_mem & (alignment-1));
        }
        // a pointer to the original memory is placed before the buffer we return
        T** orig_mem =  reinterpret_cast<T**>(p_mem - sizeof(std::uintptr_t));

        *orig_mem = mem;
        return reinterpret_cast<T*>(p_mem);
    }
    void deallocate (T* p, std::size_t n) {
        auto mem = reinterpret_cast<std::uintptr_t>(p);
        // get a pointer to the original memory and free that
        auto orig_mem = reinterpret_cast<T**>(p - sizeof(std::uintptr_t));
        ::delete(*orig_mem);
    }
};

template <class T, class U, std::size_t a1, std::size_t a2>
constexpr bool operator== (const AlignedAllocator<T,a1>&, const AlignedAllocator<U,a2>&) noexcept
{return true;}

template <class T, class U, std::size_t a1, std::size_t a2>
constexpr bool operator!= (const AlignedAllocator<T,a1>&, const AlignedAllocator<U,a2>&) noexcept
{return false;}
class transfer {
public:
    typedef std::vector<char,AlignedAllocator<char,128>> buffer_t;
protected:
    std::size_t _size;
    buffer_t _buffer;
    std::promise<void> _complete;
    std::uint32_t _transfer_id;
    std::chrono::steady_clock::time_point _deadline;

public:
    transfer(const std::size_t size, std::uint32_t id)
        : _size(size)
        , _buffer(size, 0)
        , _transfer_id(id + 1)
    {
    }

    void submit_transfer(const int fd, const std::chrono::steady_clock::duration timeout = std::chrono::milliseconds(5000) )
    {
        minion_data_transfer_s data_transfer;
        data_transfer.buffer = reinterpret_cast<std::uintptr_t>(_buffer.data());
        data_transfer.buffer_size = _size;
        data_transfer.transfer_id = _transfer_id;
        data_transfer.signal_number = SIGUSR1;
        data_transfer.pid = getpid();
        _deadline = std::chrono::steady_clock::now() + timeout;

        const auto rc = ioctl(fd, MINION_IOCTL_SUBMIT_TRANSFER, &data_transfer);
        if (rc < 0) {
            throw std::runtime_error(strerror(errno));
        }
    }
    void completed(const std::uint32_t status) {
        try {
            if (status) {
                throw std::runtime_error(strerror(status));
            }
            _complete.set_value();
        } catch (...) {
            _complete.set_exception(std::current_exception());
        }
    }
    void wait_for_completion() {
        auto fut = _complete.get_future();
        if (fut.wait_until(_deadline) != std::future_status::ready) {
            throw std::runtime_error("timeout");
        }
        fut.get();
    }
    const buffer_t& get_buffer() const {return _buffer;}
    std::size_t get_size() const {return _size;}
    std::uint32_t get_transfer_id() const {return _transfer_id;}
};

/*
 * needs to:
 *  submit transfers and store them.
 *  receive signals
 *  query for finished transfers
 *  cancel timed-out transfers
 */
class transfer_manager {
protected:
    static transfer_manager* _instance ;
    std::size_t _max_queue_size;
    std::size_t _transfer_size;
    unsigned int _frames_per_packet;
    int _fd;
    std::deque< std::shared_ptr<transfer>> _transfers;
private:
    transfer_manager(
            const int fd,
            const std::size_t size,
            const std::size_t max_queue_size,
            const unsigned int fpp)
        :_max_queue_size(max_queue_size)
        ,_transfer_size(size)
        ,_frames_per_packet(fpp)
        ,_fd(fd)
    {
        // associate signals with a the signal_receiver() function
        struct sigaction usr_action;
        sigset_t block_mask;
        sigfillset (&block_mask);
        usr_action.sa_handler = signal_receiver;
        usr_action.sa_mask = block_mask; //block all signal inside signal handler.
        usr_action.sa_flags = SA_NODEFER;//do not block SIGUSR1 within sig_handler_int.
        sigaction (SIGUSR1, &usr_action, nullptr);
    }

    static void signal_receiver(int signal)
    {
        if (!_instance) {
            std::cerr << "signal handler called when not ready" << std::endl;
            return;
        }
        auto& transfers = _instance->_transfers;
        bool more;
        do {
            // get the status information for completed transfers
            std::array<minion_transfer_status_s, MAX_QUEUE_SIZE> statuses;
            minion_completed_transfers_s transfer_results;
            transfer_results.completed_transfers = reinterpret_cast<std::uintptr_t>(statuses.data());
            transfer_results.completed_transfers_size = statuses.size();
            transfer_results.no_completed_transfers = 0;

            const auto rc = ioctl(_instance->_fd, MINION_IOCTL_WHATS_COMPLETED, &transfer_results);
            if (rc < 0) {
                // error, but can't throw as we've been asynchronusly called
                std::cerr << "ioctl failed with error " << rc << std::endl;
                break;
            }

            for (int i(0); i < transfer_results.no_completed_transfers; ++i) {
                const auto& transfer_status = statuses[i];
                const auto& completed_id = transfer_status.transfer_id;
                // find transfer in queue
                auto it = std::find_if(transfers.begin(),transfers.end(),
                    [completed_id](std::shared_ptr<transfer>(candidate))->bool
                        {
                            return candidate->get_transfer_id() == completed_id;
                        });
                if (it == transfers.end()) {
                    std::cerr << "transfer-id " << transfer_status.transfer_id << " finished, but we have no record of it" << std::endl;
                    continue;
                }

                // mark the transfer as completed
                (*it)->completed(transfer_status.status);
            }

            // if the structure used to fetch the status information wasn't big enough
            // there will be more results
            more = (transfer_results.no_completed_transfers == transfer_results.completed_transfers_size) ;
        } while (more);
    }

    /**
     * @brief send ioctl to control hs-receiver core
     * @param enable set enable bit allowing data to be transferred
     * @param reset set reset bit resetting the hs-receiver core.
     */
    void hs_receiver(const bool enable, const bool reset, const unsigned int frames)
    {

        struct minion_hs_receiver_s hs_rx_ioctl;
        std::memset(&hs_rx_ioctl, 0, sizeof(hs_rx_ioctl));
        hs_rx_ioctl.write = 1;
        hs_rx_ioctl.registers[0] |= enable ? 1 : 0;
        hs_rx_ioctl.registers[0] |= reset  ? 2 : 0;
        hs_rx_ioctl.registers[11] = frames;

        std::cerr << " hs_rx "
                  << (enable ? "enable " : ". ")
                  << (reset  ? "reset " : ". ")
                  << frames << " frames per packet"
                  << std::endl;

        const auto rc = ioctl(_instance->_fd, MINION_IOCTL_HS_RECIEVER, &hs_rx_ioctl);
        if (rc < 0) {
            throw std::runtime_error(strerror(rc));
        }
    }

    void cancel_transfers() {
        const auto rc = ioctl(_instance->_fd, MINION_IOCTL_CANCEL_TRANSFERS);
        if (rc < 0) {
            throw std::runtime_error(strerror(rc));
        }
    }
public:
    static transfer_manager* setup(
        const int fd,
        const std::size_t size,
        const std::size_t max_queue_size,
        const unsigned int fpp)
    {
        if (_instance) {
            throw std::runtime_error("already setup");
        }
        const auto frames_per_packet = std::min(fpp, max_frames_per_packet);
        _instance = new transfer_manager(fd, size, max_queue_size, frames_per_packet);
        return _instance;
    }

    void stream_data(
            std::ostream& out,
            const std::size_t no_transfers,
            const bool stream,
            const bool reset = false
            )
    {
        std::size_t transfers_submitted(0);
        std::atomic<std::size_t> active_transfers(0);
        auto& transfers = _instance->_transfers;
        bool in_reset = false;

        // if resetting put hs-receiver into reset
        if (reset) {
            _instance->hs_receiver(false, false, _instance->_frames_per_packet);
            _instance->hs_receiver(false, true, _instance->_frames_per_packet);
            _instance->cancel_transfers();
            in_reset = true;
        }


        while (transfers_submitted < no_transfers || stream) {
            // loop until transfers active reaches limint
            while ((transfers.size() < _instance->_max_queue_size) &&
                   (transfers_submitted < no_transfers || stream))
            {
                // creating transfer objects on a queue (transfer manager)
                auto trans = std::make_shared<transfer>(_instance->_transfer_size,transfers_submitted);
                transfers.emplace_back(trans);

                // submit transfers
                trans->submit_transfer(_fd);
                ++transfers_submitted;
            }

            // if in reset, take the hs-receiver out of reset
            if (in_reset) {
                _instance->hs_receiver(false, false, _instance->_frames_per_packet);
                in_reset = false;
            }

            // wait for oldest transfer on queue to finish
            if (!transfers.empty()) {
                auto oldest = transfers.front();
                wait_and_stream(out, oldest);
                transfers.pop_front();
            }
        }

        // wait for remaining transfers to finish
        while (!transfers.empty()) {
            auto oldest = transfers.front();
            wait_and_stream(out, oldest);
            transfers.pop_front();
        }
    }
    void wait_and_stream(std::ostream& out, std::shared_ptr<transfer>& oldest)
    {
        oldest->wait_for_completion();
        const auto& buffer = oldest->get_buffer();
        out.write(buffer.data(), buffer.size());
    }
};

transfer_manager* transfer_manager::_instance(nullptr);

void usage()
{
    std::cerr << "usage: minion-dma [-s size] [-n number of transfers] [-q max queue size ] <device>\n"
              << " -s, --size size      Size of each transfer, defaults to 514*2-bytes\n"
              << " -n, --no-transfers   Number of transfers, (default 1)\n"
              << "     --stream         Transfer data repeatedly until further notice\n"
              << " -q, --max-queue      Maximum number of transfers to queue at once (default 8)\n"
              << " -p, --poll           Use polling rather than signals to detect when transfers have completed\n"
              << " -r, --reset          Reset the High-Speed receiver when starting\n";
    exit(1);
}

void number_or_quit(std::string const& argument, unsigned long& number)
{
    try {
        number = (unsigned int)std::stoul(argument,0);
    } catch(std::invalid_argument& ) {
        std::cerr << "couldn't convert '" << argument << "'to a number" << std::endl;
        exit(1);
    } catch(std::out_of_range& ) {
        std::cerr << "'" << argument << "'is too big or a negative number" << std::endl;
        exit(1);
    }
}

int main(int argc, char* argv[]) {
    std::string device;
    std::size_t size(514*2);
    std::size_t no_transfers(1);
    std::size_t max_queue_size(8);
    bool stream = false;
    bool poll = false;
    bool reset = false;

    // parse options
    if (argc == 1) {
        usage();
    }

    for (int index(1); index < argc; ++index) {
        std::string arg(argv[index]);

        if (arg == "-s" || arg == "--size" ) {
            ++index;
            number_or_quit(argv[index], size);
            continue;
        }
        if (arg == "-n" || arg == "--no-transfers" ) {
            ++index;
            number_or_quit(argv[index], no_transfers);
            continue;
        }
        if (arg == "-q" || arg == "--max-queue" ) {
            ++index;
            number_or_quit(argv[index], max_queue_size);
            if (max_queue_size > MAX_QUEUE_SIZE) {
                std::cerr << "maximum queue size supported is " << MAX_QUEUE_SIZE << std::endl;
                exit(1);
            }
            continue;
        }
        if (arg == "--stream") {
            stream = true;
            continue;
        }
        if (arg == "-p" || arg == "--poll") {
            poll = true;
            continue;
        }
        if (arg == "-r" || arg == "--reset") {
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

    // warn if the transfer size is not a multiple of the frame-size
    if (size % frame_size != 0) {
        std::cerr << "Warning: transfer-size of " << size << " is not a multiple of the frame-size (" << frame_size << ")\n";
    }

    // warn if the transfer size is more than the max packet size
    if (size > (frame_size * max_frames_per_packet)) {
        std::cerr << "Warning: transfer-size is larger that the maximum number of frames per packet (512)\n";
    }

    try {
        // open device file
        int fd = open(device.c_str(), O_RDWR);
        if (fd <= 0) {
            throw std::runtime_error("Failed to open device node");
        }

        auto* manager = transfer_manager::setup(fd, size, max_queue_size, size / frame_size);
        manager->stream_data(std::cout, no_transfers, stream, reset);
    } catch (std::runtime_error& e) {
        std::cerr << e.what() << std::endl;
        exit(1);
    }
    return 0;
}
