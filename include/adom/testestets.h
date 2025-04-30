#include <vector>
#include <string>
#include <array>

#include <adom/parameters.h>
#include <adom/translation.h>





/**
 * @brief Struct to hold configuration for a specific topic.
 */
struct TopicConfig {
    ObjectType topic_type;      // The type of the topic, using ObjectType enum
    const char* publisher_ip;   // IP address of the publisher
    std::vector<const char*> subscriber_ips; // List of subscriber IP addresses
    Structure data_structure;    // Data structure configuration

    TopicConfig(ObjectType type, const char* pub_ip, const std::vector<const char*>& subs, const Structure& ds)
        : topic_type(type), publisher_ip(pub_ip), subscriber_ips(subs), data_structure(ds) {}
};


/**
 * @brief Struct to hold the overall system configuration.
 */
struct SystemConfig {
#ifdef EXAMPLE_NOT_LOCALHOST

    static constexpr TopicConfig topics[] = {
        {
            IMAGE,              // Topic
            "192.168.3.100",    // IP address of Publisher
            {"192.168.3.101"},  // IP address of Subscriber(s)
            Structure(TWO_DIMENSIONAL, FULL_HD_BLOCK_ROWS, FULL_HD_BLOCKS_IN_ROW, FULL_HD_V_PIXEL_PER_IMAGE, FULL_HD_H_PIXEL_PER_IMAGE, CHANNEL_NUMBER)
        }
    // Add more topic configurations here if necessary, new topics also required to be defined in parameters.h
    };
    // The number of topics
    static constexpr size_t topic_count = sizeof(topics) / sizeof(topics[0]);
#else
    static constexpr TopicConfig topics[] = {
        {
            IMAGE,          // Topic
            "127.0.0.1"     // IP address of Publisher
            {"127.0.0.1"},  // Multiple Subscriber IDs are also possible
            Structure(TWO_DIMENSIONAL, FULL_HD_BLOCK_ROWS, FULL_HD_BLOCKS_IN_ROW, FULL_HD_V_PIXEL_PER_IMAGE, FULL_HD_H_PIXEL_PER_IMAGE, CHANNEL_NUMBER)
        }
        // Add more topic configurations here if necessary, new topics also required to be defined in parameters.h
    };
    // The number of topics
    static constexpr size_t topic_count = sizeof(topics) / sizeof(topics[0]);
#endif 
};