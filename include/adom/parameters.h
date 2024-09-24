#ifndef APPLICATION_DATA_OBJECT_MANAGEMENT_API__PARAMENTERS_H_
#define APPLICATION_DATA_OBJECT_MANAGEMENT_API__PARAMENTERS_H_

#include <unistd.h>
#include <string.h>
#include <string>
#include <boost/asio.hpp>

#define PROTOCOL_TEST 1

#define MAX_COUNT_MANAGED_OBJECTS_PER_TOPIC 3

#define PRINTF_BINARY_PATTERN_INT8 "%c%c%c%c%c%c%c%c"
#define PRINTF_BYTE_TO_BINARY_INT8(i)    \
    (((i) & 0x80ll) ? '1' : '0'), \
    (((i) & 0x40ll) ? '1' : '0'), \
    (((i) & 0x20ll) ? '1' : '0'), \
    (((i) & 0x10ll) ? '1' : '0'), \
    (((i) & 0x08ll) ? '1' : '0'), \
    (((i) & 0x04ll) ? '1' : '0'), \
    (((i) & 0x02ll) ? '1' : '0'), \
    (((i) & 0x01ll) ? '1' : '0')


/**
 * @brief Enum to describe different topics.
 * 
 */
enum Topic{
    NONE = 0,
    IMAGE = 1,
};


const std::string log_directory = "/ApplicationDataObjectManagement/examples/adom_logs/";

// Set the IP addresses to the host addresses on which you launch the ROS2 nodes
const char publisher_ip[20]    = "192.168.3.100";
const char subscriber_ip[20]  = "192.168.3.101";

// If you test the somple application on a single host, you can use the loopback interface
// const char publisher_ip[20]        = "127.0.0.1";
// const char subscriber_ip[20]        = "127.0.0.1";


const uint16_t home_dir_ingress_port    = 5400;
const uint16_t req_dir_ingress_port     = 5402;
const uint16_t string_length            = 20;
const uint16_t adom_message_overhead    = 8;
const uint16_t max_buffer_length        = 1400;

const uint16_t TOTAL_BLOCK_SIZE         = 1200;

// Resolution dependent parameters
const uint16_t HD_BLOCKS_IN_ROW            = 64; 
const uint16_t HD_BLOCK_ROWS               = 36;  
const uint16_t HD_V_PIXEL_PER_IMAGE        = 720;
const uint16_t HD_H_PIXEL_PER_IMAGE        = 1280;
const uint16_t FULL_HD_BLOCKS_IN_ROW       = 96; 
const uint16_t FULL_HD_BLOCK_ROWS          = 54;  
const uint16_t FULL_HD_V_PIXEL_PER_IMAGE   = 1080;
const uint16_t FULL_HD_H_PIXEL_PER_IMAGE   = 1920;

//Resolution independent parameters
const uint16_t V_PIXEL_PER_BLOCK        = 20;
const uint16_t H_PIXEL_PER_BLOCK        = 20;
const uint16_t CHANNEL_NUMBER           = 3;
const uint16_t MAX_NUM_TRACKED_SAMPLES  = 5;

const uint16_t EMPTY_QUEUE_ERROR        = 20;







#endif /*APPLICATION_DATA_OBJECT_MANAGEMENT_API__PARAMENTERS_H_*/