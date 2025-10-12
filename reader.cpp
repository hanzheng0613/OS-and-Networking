#include <boost/interprocess/shared_memory_object.hpp>

#include <boost/interprocess/sync/scoped_lock.hpp>
#include <cstring>
#include <iostream>

#include <boost/interprocess/mapped_region.hpp>

#include <boost/interprocess/sync/named_mutex.hpp>
#include <thread>

#include <chrono>

using namespace boost::interprocess;

const char* SHM_NAME = "Shared_Memory";
const char* MUTEX_NAME = "Mutex";

struct SharedData {
    int counter;
    char message[256];
};

std::string time() {
    auto now = std::chrono::system_clock::now();
    auto ms_total = std::chrono::duration_cast<std::chrono::milliseconds>(
                            now.time_since_epoch())
                            .count();
    std::time_t t = ms_total / 1000;  //  seconds
    int ms = ms_total % 1000;         //  milliseconds

    std::tm* bt = std::localtime(&t);

    char buffer[16];
    // HH:MM:SS.mmm
    std::snprintf(buffer, sizeof(buffer), "%02d:%02d:%02d.%03d",
                      bt->tm_hour, bt->tm_min, bt->tm_sec, ms);

    return std::string(buffer);
}

int main() {
    
    std::this_thread::sleep_for(std::chrono::seconds(1));

    try {
        shared_memory_object shm(open_only, SHM_NAME, read_write);
        mapped_region region(shm, read_write);
        void* address = region.get_address();
        SharedData* data = static_cast<SharedData*>(address);

        named_mutex mutex(open_only, MUTEX_NAME);

        int count = 0;

        while (count < 100) {
            {
                scoped_lock<named_mutex> lock(mutex);
                if (data->counter != count) {
                    count = data->counter;
                    std::cout << time() << " Read: " << data->message << std::endl;
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(120));
        }

        std::cout <<time() << " Finished." << std::endl;

    } catch (interprocess_exception &e) {
        std::cerr << "Exception: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
