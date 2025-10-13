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
    //std::strcpy(data->message, "Message: ");

    named_mutex mutex(create_only, MUTEX_NAME);

    std::cout << "Start Write at " << std::endl;

    // Print the write messgae and time of writing
    for (int k = 1; k <= 100; ++k) {
        {
            scoped_lock<named_mutex> lock(mutex);
            data->counter = k;
            std::snprintf(data->message, sizeof(data->message),
                          "Number %d", k);
            std::cout << " Write: " << data->message << std::endl;
        }
        
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    std::cout  << " Finished." << std::endl;

    
    shared_memory_object::remove(SHM_NAME);
    
    
    named_mutex::remove(MUTEX_NAME);

    return 0;
}
