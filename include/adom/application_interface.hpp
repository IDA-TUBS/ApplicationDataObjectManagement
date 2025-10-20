#ifndef APPLICATION_DATA_OBJECT_MANAGEMENT_API__APPLICATION_INTERFACE_HPP_
#define APPLICATION_DATA_OBJECT_MANAGEMENT_API__APPLICATION_INTERFACE_HPP_


#include <unistd.h>
#include <string.h>
#include <string>
#include <vector>

#include "protocol.hpp"
#include "directory.hpp"
#include "log.hpp"


#include <adom/directory_config.hpp>
#include <adom/parameters.h>

using boost::asio::ip::udp;

class ApplicationInterface {
    public:

        /**
         * @brief Construct a new Application Interface object
         * 
         * @param associated_directory_descriptor 
         */
        ApplicationInterface(EntityDescriptor home_directory_descriptor):
            home_directory_descriptor_(home_directory_descriptor),
            home_directory_(home_directory_descriptor, directory_request_queue_, directory_reply_queue_, directory_announcement_queue_){
            
            std::string log_suffix = std::to_string(home_directory_descriptor_.entity_id);
            adom::init_file_log("log_adom_interface_entity-", log_suffix);         
            adom::init_console_log(); 
        }

        /**
         * @brief Construct a new Application Interface object from json config files
         * 
         * @param name          name of the directory in the json config file
         * @param cfg_path      path to the json participant json file
         * @param setup_path    path to the setup json file
         */
        ApplicationInterface(std::string name, std::string dir_cfg_path, std::string topic_cfg_path, std::string setup_path):
                home_dir_config_(name, dir_cfg_path, topic_cfg_path, setup_path),            
                #if(RTPS)
                home_directory_(home_dir_config_.host_id(), home_dir_config_.descriptor(), directory_request_queue_, directory_reply_queue_, directory_announcement_queue_){
                #else
                home_directory_(home_dir_config_.descriptor(), directory_request_queue_, directory_reply_queue_, directory_announcement_queue_){
                #endif

            home_directory_descriptor_ = home_dir_config_.descriptor();
            connected_directories_ = home_dir_config_.otherDescriptors();

            
            std::string log_suffix = std::to_string(home_directory_descriptor_.entity_id);
            adom::init_file_log("log_adom_interface_entity-", log_suffix);         
            adom::init_console_log(); 
        }


        /** 
         * @brief Read static definition of other directories and topic structures etc
         * 
         */
        void initialize(void);


        /**
         * @brief Send a Request to the associated Directory to: Write a new (complete) sample to the managed shared memory (called from a "Publisher" or "Writer")
         * 
         * @param type  name of the topic, whose object sample is written
         * @param structure structure of the data type required for parsing and specification of blocks
         */
        void registerNewTopic(Topic type, Structure structure);

        /**
         * @brief Register a new topic to be managed by the directory
         * 
         * @param topic_name name of the topic
         */
        void registerNewTopic(std::string topic_name);

        /**
         * @brief Send a Request to the associated Directory to: Write a new (complete) sample (called from a "Publisher" or "Writer")
         * 
         * @param type  name of the topic, whose object sample is written
         * @param data  data to be registered
         * @param structure_sample_data boolean to decide whether to track the object as a StructuredObjectSample (sample data will be reorganized to represent Object Block Structure)  or as a ObjectSample (sample data will beehld in memory as is)
         */
        void registerNewData(Topic type, unsigned char *data, bool structure_sample_data);


        /**
         * @brief Send a Request to the associated Directory to: Partially read data from a sample (called from a "Subscriber" or "Reader")
         * 
         * @param type 
         * @param associated_blocks 
         * @param output_data 
         * @param timeout
         * @return std::future<int> 
         */
        std::future<int> readPartialData(Topic type, std::vector<uint16_t> associated_blocks, unsigned char* output_data, std::chrono::milliseconds timeout = DATA_REQ_TIMEOUT);


        /**
         * @brief Send a Request to the associated Directory to: Partially read data from a sample (called from a "Subscriber" or "Reader")
         * 
         * @param topic_name            name of the topic, whose object sample is read partially
         * @param sequence_nr           sequence number of the sample 
         * @param associated_blocks     associated_blocks for this read access
         * @param output_data           data to write requested block into
         * @param timeout               waiting time for requested blocks (handle packet loss)
         */
        std::future<int> readPartialData(Topic type, int sequence_nr, std::vector<uint16_t> associated_blocks, unsigned char* output_data, std::chrono::milliseconds timeout = DATA_REQ_TIMEOUT);


        /**
         * @brief Stopping all processes within the interface
         * 
         */
        void stop();

        int getHomeDirEntityId(){
            return home_directory_descriptor_.entity_id;
        };



    private:

        EntityDescriptor home_directory_descriptor_;
        std::list<EntityDescriptor> connected_directories_;
        DirectoryCfg home_dir_config_; 
        Directory home_directory_;


        SafeQueue<std::pair<DataRequest, udp::endpoint>> directory_request_queue_; 
        SafeQueue<DataTransport> directory_reply_queue_;
        SafeQueue<std::pair<DataAnnouncement, udp::endpoint>> directory_announcement_queue_;

        /**
         * @brief List to track Structures for tracked topics
         * 
         */
        std::map<Topic, Structure> registered_topics_; 



};



#endif /*APPLICATION_DATA_OBJECT_MANAGEMENT_API__APPLICATION_INTERFACE_HPP_*/