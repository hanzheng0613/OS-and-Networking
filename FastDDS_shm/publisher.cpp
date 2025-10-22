#include <chrono>


#include "HelloMsg_idlPubSubTypes.hpp"
#include <fastdds/dds/publisher/qos/PublisherQos.hpp>
#include <thread>


#include <fastdds/dds/publisher/qos/DataWriterQos.hpp>

#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/rtps/transport/shared_mem/SharedMemTransportDescriptor.hpp>

#include <fastdds/dds/domain/DomainParticipantFactory.hpp>

#include <fastdds/dds/publisher/Publisher.hpp>

#include <fastdds/dds/publisher/DataWriter.hpp>
#include <fastdds/dds/topic/Topic.hpp>

#include <iostream>
using namespace eprosima::fastdds::dds;


using namespace eprosima::fastdds::rtps;

int count = 0;
extern "C" void publish(DataWriter* writer)
{
    HelloMsg msg;
    msg.id(count);
        msg.message("Message: Number " + std::to_string(count));
        writer->write(&msg);
        std::cout << "Send: id: " << msg.id() << " with message: " << msg.message() << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));

}

int main()
{

    DomainParticipantQos pqos;
    pqos.transport().use_builtin_transports = false;
    pqos.transport().user_transports.push_back(
        std::make_shared<SharedMemTransportDescriptor>());
    DomainParticipant* participant =
        DomainParticipantFactory::get_instance()->create_participant(0, pqos);
    if (!participant)
    {
        std::cerr << "Failed to create a new participant!" << std::endl;
        return 1;
    }


    HelloMsgPubSubType my_type;
    TypeSupport type(&my_type);
    participant->register_type(type);
    if (participant->register_type(type) != RETCODE_OK)
    {
        std::cerr << "Failed to register a new type!" << std::endl;
        return 1;
    }
 
    Topic* topic = participant->create_topic(
        "HelloMsgTopic",
        "HelloMsg",
        TOPIC_QOS_DEFAULT);
    if (!topic)
    {
        std::cerr << "Failed to create a new topic!" << std::endl;
        return 1;
    }
    Publisher* publisher = participant->create_publisher(PUBLISHER_QOS_DEFAULT, nullptr);
    if (!publisher)
    {
        std::cerr << "Failed to create a new publisher!" << std::endl;
        return 1;
    }
    
    DataWriter* writer = publisher->create_datawriter(topic, DATAWRITER_QOS_DEFAULT, nullptr);
    if (!writer)
    {
        std::cerr << "Failed to create a new DataWriter!" << std::endl;
        return 1;
    }
    for(int i=0;i<100;i++){
    	count=i;
    	publish(writer);
    }
    participant->delete_contained_entities();
    DomainParticipantFactory::get_instance()->delete_participant(participant);

    return 0;
}
