#include <boost/interprocess/shared_memory_object.hpp>

#include <cstring>
#include <iostream>

#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/sync/named_mutex.hpp>
#include <thread>

#include <boost/interprocess/sync/scoped_lock.hpp>

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
        std::time_t t = ms_total / 1000;  // seconds
        int ms = ms_total % 1000;         // milliseconds

        std::tm* bt = std::localtime(&t);

        char buffer[16];
        // HH:MM:SS.mmm
        std::snprintf(buffer, sizeof(buffer), "%02d:%02d:%02d.%03d",
                      bt->tm_hour, bt->tm_min, bt->tm_sec, ms);

        return std::string(buffer);
}

int main() {
    // Reset if there exist old shared memory and mutex
    shared_memory_object::remove(SHM_NAME);
    named_mutex::remove(MUTEX_NAME);

    shared_memory_object shm(create_only, SHM_NAME, read_write);
    shm.truncate(sizeof(SharedData));

    mapped_region region(shm, read_write);
    void* address = region.get_address();
    SharedData* data = new (address) SharedData;
    data->counter = 0;
    std::strcpy(data->message, "Message: ");

    named_mutex mutex(create_only, MUTEX_NAME);

    std::cout << "Start Write at " << time() << std::endl;

    // Print the write messgae and time of writing
    for (int k = 1; k <= 100; ++k) {
        {
            scoped_lock<named_mutex> lock(mutex);
            data->counter = k;
            std::snprintf(data->message, sizeof(data->message),
                          "Message: Number %d", k);
            std::cout << time() << " Write: " << data->message << std::endl;
        }
        
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    std::cout <<time() << " Finished." << std::endl;

    
    shared_memory_object::remove(SHM_NAME);
    
    
    named_mutex::remove(MUTEX_NAME);

    return 0;
}
