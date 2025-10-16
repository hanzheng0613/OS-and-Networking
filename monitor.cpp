#include <boost/interprocess/shared_memory_object.hpp>

#include <iostream>
#include <thread>


#include <chrono>

#include <cstring>
#include <boost/interprocess/sync/named_mutex.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>

using namespace boost::interprocess;
const char* SHM_NAME = "Shared_Memory";
const char* MUTEX_NAME = "Mutex";

struct SharedData {
    int counter;
    
    char message[100];
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
    try {
        shared_memory_object shm(open_only, "Shared_Memory", read_only);
        mapped_region region(shm, read_only);
        named_mutex mutex(open_only, MUTEX_NAME);
        const SharedData* data = static_cast<const SharedData*>(region.get_address());

        std::cout << "Start Profiling \n";

        int last_counter = data->counter;
        
        std::string last_message = data->message;
        
        boolean_t isReady(false);
        
        while (true) {
            // Catch Write()
            if (data->counter != last_counter ||
                std::strcmp(data->message, last_message.c_str()) != 0) {
                last_counter = data->counter;
                
                last_message = data->message;
                
                isReady = true;
                
                std::cout << time() << "Monitor: Write: counter=" << last_counter
                          << ", message = " << last_message << std::endl;
            }
            // Catch Read()
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            if(isReady&&mutex.try_lock()&&last_counter==data->counter&&last_message==data->message){
                std::cout << time() << "Monitor: read: counter=" << last_counter
                          << ", message = " << last_message << std::endl;
                isReady=false;
                
            }
            mutex.unlock();
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }

    } catch (const std::exception& e) {
        std::cerr << "Monitor error: " << e.what() << std::endl;
    }

    return 0;
}
