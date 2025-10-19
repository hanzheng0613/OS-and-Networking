#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/sync/named_mutex.hpp>
#include <iostream>
#include <cstring>
using namespace boost::interprocess;
// Define a struct to store data in shared memory
struct SharedData {
    char message[256]; // A simple char array to hold the message
};

// Function for the writer process
extern "C" void write_to_shared_memory() {
    // Find or construct a shared memory object
    shared_memory_object::remove("SharedMemory");
    named_mutex::remove("SharedMemoryMutex");
    
    
    named_mutex mutex(open_or_create, "SharedMemoryMutex");
    
   
	shared_memory_object shm(create_only, "SharedMemory", read_write);
	shm.truncate(sizeof(SharedData)); 
	 mapped_region region(shm, read_write);
    // Get pointer to memory
    void* addr = region.get_address();

    // Cast to your struct type
    SharedData* data = new (addr) SharedData;
    // Write a message to the shared memory
    scoped_lock<named_mutex> lock(mutex);
    std::strcpy(data->message, "Hello from writer process!");
    std::cout << "Message written to shared memory: " << data->message << std::endl;
}

// Function for the reader process
extern "C" void read_from_shared_memory() {
    // Find the shared memory object
    shared_memory_object shm(open_only, "SharedMemory", read_write);
    
    mapped_region region(shm, read_only);
	named_mutex mutex(open_only, "SharedMemoryMutex");
    // Get pointer to memory
    void* addr = region.get_address();

    // Cast to your struct type
    SharedData* data = static_cast<SharedData*>(addr);
    
    if (data) {
        // Read the message from shared memory
        scoped_lock<named_mutex> lock(mutex);
        std::cout << "Message read from shared memory: " << data->message << std::endl;
    } else {
        std::cerr << "Failed to find shared memory segment." << std::endl;
    }
}

int main() {
    // Create a managed shared memory segment
    

    // Writer process: Write to shared memory
    write_to_shared_memory();

    // Reader process: Read from shared memory
    read_from_shared_memory();

    return 0;
}

