
#include <fastdds/dds/topic/Topic.hpp>
#include <fastdds/dds/domain/DomainParticipant.hpp>

#include <fastdds/rtps/transport/shared_mem/SharedMemTransportDescriptor.hpp>

#include "HelloMsg_idlPubSubTypes.hpp"

#include <fastdds/dds/subscriber/qos/DataReaderQos.hpp>
#include <chrono>
#include <fastdds/dds/subscriber/qos/SubscriberQos.hpp>

#include <iostream>

#include <thread>
#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/dds/subscriber/Subscriber.hpp>

#include <fastdds/dds/subscriber/DataReader.hpp>



using namespace eprosima::fastdds::dds;

using namespace eprosima::fastdds::rtps;

extern "C" void subscribe(DataReader* reader)
{
    std::cout << "Start running and waiting for messages..." << std::endl;

    SampleInfo info;
    HelloMsg msg;

    while (true)
    {
        if (reader->take_next_sample(&msg, &info) == RETCODE_OK && info.valid_data)
        {
            std::cout << "Received id: " << msg.id()
                      << " with message: " << msg.message() << std::endl;
        }

        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
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

    Subscriber* subscriber = participant->create_subscriber(SUBSCRIBER_QOS_DEFAULT, nullptr);
    if (!subscriber)
    {
        std::cerr << "Failed to create a new subscriber!" << std::endl;
        return 1;
    }
    
    DataReader* reader = subscriber->create_datareader(
        topic,
        DATAREADER_QOS_DEFAULT,
       	nullptr);

    if (!reader)
    {
        std::cerr << "Failed to create a new DataReader!" << std::endl;
        return 1;
    }

    	
    subscribe(reader);
   
    participant->delete_contained_entities();
    DomainParticipantFactory::get_instance()->delete_participant(participant);

    return 0;
}
