#include <adom/application_interface.hpp>
#include <adom/opencv_helper.h>

#include <boost/log/trivial.hpp>
#include <boost/log/core.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/sources/severity_logger.hpp>
#include <boost/log/sources/record_ostream.hpp>
#include <boost/log/utility/setup/file.hpp>
#include <boost/log/utility/setup/common_attributes.hpp>
#include <boost/log/utility/setup/console.hpp>


/** 
 * @brief Read static definition of other directories and topic structures etc
 * 
 */
void ApplicationInterface::initialize(void){
    /** Temporary static definition of other directory */
    // setup parameter struct for other directory for test of subscribed application
    // ideally read this from config file
    if(home_directory_descriptor_.entity_id == 0){
        EntityDescriptor subscribed_directory;
        subscribed_directory.entity_id = 5;
        subscribed_directory.ingress_port = req_dir_ingress_port;
        strncpy(subscribed_directory.ip_address, subscriber_ip, sizeof(subscriber_ip));

        home_directory_.other_directory_descriptor_ = subscribed_directory; // @todo: make privat and make set with setter function
        home_directory_.initiate();
    } else {
        EntityDescriptor publisher_directory;
        publisher_directory.entity_id = 0;
        publisher_directory.ingress_port = home_dir_ingress_port;
        strncpy(publisher_directory.ip_address, publisher_ip, sizeof(publisher_ip));

        home_directory_.other_directory_descriptor_ = publisher_directory; // @todo: make privat and make set with setter function
        home_directory_.initiate();
    }

}


/**
 * @brief Send a Request to the associated Directory to: Write a new (complete) sample to the managed shared memory (called from a "Publisher" or "Writer")
 * 
 * @param type  name of the topic, whose object sample is written
 * @param structure structure of the data type required for parsing specification of blocks
 */
void ApplicationInterface::registerNewTopic(Topic type, Structure data_structure){
    // Validation: Check whether the dimensions specified in the data structure are valid
    if (data_structure.object_width % data_structure.block_cols != 0 || data_structure.object_height % data_structure.block_rows != 0) {
        throw std::runtime_error("Invalid data dimensions. Object width must be divisible by block_cols and object height by block_rows.");
        return; // structure not valid
    }

    // Check if entry for this type already exists
    auto it = registered_topics_.find(type);
    if (it == registered_topics_.end()) {
        // Add a new entry, when no entry for this type exists yet
        registered_topics_[type] = data_structure;
        logDebug("Added new Entry with Data Structure for this type: " + std::to_string(static_cast<int>(type)));
    } else {
        logDebug("An entry for the follwoing type already exists: " + std::to_string(static_cast<int>(type)));
    }
}


/**
 * @brief Send a Request to the associated Directory to: Write a new (complete) sample to the managed shared memory (called from a "Publisher" or "Writer")
 * 
 * @param type  name of the topic, whose object sample is written
 * @param data  data to be registered
 */
void ApplicationInterface::registerNewData(Topic type, unsigned char *data){

    Structure associated_structure = registered_topics_.find(type)->second;
    uint16_t sequence_number = home_directory_.getLatestSequencenumberOfTrackedTopic(type) + 1;

    // create object sample
    auto sample = std::make_shared<ObjectSample>(home_directory_descriptor_, type, sequence_number, data, associated_structure);

    //register object sample
    home_directory_.addNewObjectSample(sample);

}


/**
 * @brief Send a Request to the associated Directory to: Partially read data from a sample (called from a "Subscriber" or "Reader")
 * 
 * @param topic_name            name of the topic, whose object sample is read partially
 * @param associated_blocks     associated_blocks for this read access
 * @param output_data           data to write requested block into
 */
std::future<int> ApplicationInterface::readPartialData(Topic type, std::vector<uint16_t> associated_blocks, unsigned char* output_data) {
    return std::async(std::launch::async, [this, type, associated_blocks, output_data]() -> int {
        Structure associated_structure = registered_topics_.find(type)->second;

        // Wait until the directory has a sample of the requested topic
        while (home_directory_.getNumberOfTrackedObjects(type) == 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        // Get the latest tracked sample from the directory
        auto latest_sample = home_directory_.getLastObjectSample(type);

        // Check if individual requested blocks are already available
        auto updated_block_list = home_directory_.checkAvailability(latest_sample, associated_blocks);

        if (updated_block_list.empty()){
            logDebug("All associated Blocks are already present locally!");
            return 0;
        }

        // Send request, as a return, a request id is given
        int request_id = home_directory_.sendDataRequest(latest_sample->getHomeDirectory(), latest_sample, updated_block_list);


        // Be notified if all requested blocks were received
        {
            std::unique_lock<std::mutex> lock(home_directory_.request_tracking_lock_);
            auto& request_data = home_directory_.request_tracking_[request_id];
            request_data.cv.wait(lock, [&request_data]() {
                return request_data.received_blocks.size() == request_data.requested_blocks.size();
            });
        }

        // Translate data format to output format and write it into the output data
        translate_to_uchar(latest_sample->object_sample_data_, output_data, latest_sample->getStructure());

        return 0;
    });
}


/**
 * @brief Send a Request to the associated Directory to: Partially read data from a sample (called from a "Subscriber" or "Reader")
 * 
 * @param topic_name            name of the topic, whose object sample is read partially
 * @param sequence_nr           sequence number of the sample 
 * @param associated_blocks     associated_blocks for this read access
 * @param output_data           data to write requested block into
 */
std::future<int> ApplicationInterface::readPartialData(Topic type, int sequence_nr, std::vector<uint16_t> associated_blocks, unsigned char* output_data) {
    return std::async(std::launch::async, [this, type, sequence_nr, associated_blocks, output_data]() -> int {
        Structure associated_structure = registered_topics_.find(type)->second;

        // Wait until the directory has a sample of the requested topic
        while (home_directory_.getNumberOfTrackedObjects(type) == 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        // Wait until the directory has a sample of the requested sequence nr
        while (home_directory_.getLatestSequencenumberOfTrackedTopic(type) < sequence_nr){
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }

        // Get the latest tracked sample from the directory
        auto associated_sample = home_directory_.getSample(type, sequence_nr);

        if (associated_sample == nullptr){
            logError("Requested Sample is not available!");
            return 0;
        }

        // Check if individual requested blocks are already available
        auto updated_block_list = home_directory_.checkAvailability(associated_sample, associated_blocks);

        if (updated_block_list.empty()){
            logDebug("All associated Blocks are already present locally!");
            return 0;
        }

        // Send request, as a return, a request id is given
        int request_id = home_directory_.sendDataRequest(associated_sample->getHomeDirectory(), associated_sample, updated_block_list);


        // Be notified if all requested blocks were received
        {
            std::unique_lock<std::mutex> lock(home_directory_.request_tracking_lock_);
            auto& request_data = home_directory_.request_tracking_[request_id];
            request_data.cv.wait(lock, [&request_data]() {
                return request_data.received_blocks.size() == request_data.requested_blocks.size();
            });
        }

        // Translate data format to output format and write it into the output data
        translate_to_uchar(associated_sample->object_sample_data_, output_data, associated_sample->getStructure());

        return 0;
    });
}

/**
 * @brief Stopping all processes within directory
 * 
 */
void ApplicationInterface::stop(){
    home_directory_.stop();
}