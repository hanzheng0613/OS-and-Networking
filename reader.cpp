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


void read() {

    std::this_thread::sleep_for(std::chrono::seconds(1));

 
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
                //std::cout << " Read: " << data->message << std::endl;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    std::cout << "Finished reading." << std::endl;
}

int main() {
    try {
        read();
    } catch (interprocess_exception &e) {
        std::cerr << "Exception: " << e.what() << std::endl;
        return 1;
    }
    return 0;
}
