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

static_assert(sizeof(struct minit_data_transfer_s) == MINIT_DATA_TRANSFER_SIZE , "ioctl size wrong");

class transfer {
public:
    typedef std::vector<char> buffer_t;
protected:
    std::size_t _size;
    buffer_t _buffer;
    std::promise<void> _complete;
    std::uint32_t _transfer_id;
    std::chrono::steady_clock::time_point _deadline;

public:
    transfer(const std::size_t size, std::uint32_t id)
        : _size(size)
        , _buffer(size)
        , _transfer_id(id)
    {
    }

    void submit_transfer(const int fd, const std::chrono::steady_clock::duration timeout = std::chrono::milliseconds(5000) )
    {
        minit_data_transfer_s data_transfer;
        data_transfer.buffer = POINTER_TO_U64(_buffer.data());
        data_transfer.buffer_size = _size;
        data_transfer.transfer_id = _transfer_id;
        data_transfer.signal_number = SIGUSR1;
        data_transfer.pid = getpid();
        _deadline = std::chrono::steady_clock::now() + timeout;

        const auto rc = ioctl(fd, MINIT_IOCTL_SUBMIT_TRANSFER, &data_transfer);
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
    int _fd;
    std::deque< std::shared_ptr<transfer>> _transfers;
private:
    transfer_manager(
            const int fd,
            const std::size_t size,
            const std::size_t max_queue_size)
        :_max_queue_size(max_queue_size)
        ,_transfer_size(size)
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

        std::cerr << "file descriptor " << _fd << std::endl;
    }

    static void signal_receiver(int signal)
    {
        if (!_instance) {
            std::cerr << "signal handler called when not ready" << std::endl;
            return;
        }
        auto transfers = _instance->_transfers;
        bool more;
        do {
            // get the status information for completed transfers
            std::vector<minit_transfer_status_s> statuses(_instance->_max_queue_size);
            minit_completed_transfers_s transfer_results;
            transfer_results.completed_transfers = POINTER_TO_U64(statuses.data());
            transfer_results.completed_transfers_size = statuses.size();
            transfer_results.no_completed_transfers = 0;

            const auto rc = ioctl(_instance->_fd, MINIT_IOCTL_WHATS_COMPLETED, &transfer_results);
            if (rc < 0) {
                // error, but can't throw as we've been asynchronusly called
                std::cerr << "ioctl failed with error " << rc << std::endl;
                break;
            }

            for (const auto& transfer_status : statuses) {
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
public:
    static transfer_manager* setup(
        const int fd,
        const std::size_t size,
        const std::size_t max_queue_size)
    {
        if (_instance) {
            throw std::runtime_error("already setup");
        }
        _instance = new transfer_manager(fd, size, max_queue_size);
    }

    void stream_data(
            std::ostream& out,
            const std::size_t no_transfers,
            const bool stream
            )
    {
        std::size_t transfers_submitted(0);
        std::atomic<std::size_t> active_transfers(0);
        auto& transfers = _instance->_transfers;
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
            // wait for oldest transfer on queue to finish
            if (!transfers.empty()) {
                auto oldest = transfers.front();
                transfers.pop_front();
                wait_and_stream(out, oldest);
            }
        }

        // wait for remaining transfers to finish
        while (!transfers.empty()) {
            auto oldest = transfers.front();
            transfers.pop_front();
            wait_and_stream(out, oldest);
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
    std::cerr << "usage: minit-dma [-s size] [-n number of transfers] [-q max queue size ] <device>\n"
              << " -s, --size size      Size of each transfer, defaults to 514*2-bytes\n"
              << " -n, --no-transfers   Number of transfers, (default 1)\n"
              << "     --stream         Transfer data repeatedly until further notice\n"
              << " -q, --max-queue      Waximum number of transfers to queue at once (default 8)\n"
              << " -p, --poll           Use polling rather than signals to detect when transfers have completed"
              << std::endl;
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

    // open device file
    int fd = open(device.c_str(), O_RDWR);
    if (fd <= 0) {
        throw std::runtime_error("Failed to open device node");
    }

    try {
        auto* manager = transfer_manager::setup(fd, size, max_queue_size);
        manager->stream_data(std::cout, no_transfers, stream);
    } catch (std::runtime_error& e) {
        std::cerr << e.what() << std::endl;
        exit(1);
    }
    return 0;
}
