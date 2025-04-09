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
    // home_directory_.other_directory_descriptor_ = other_directory_descriptor_; // @todo: make privat and make set with setter function
    home_directory_.connected_directories_ = connected_directories_;
    home_directory_.initiate();

}


/**
 * @brief Register a new topic to be managed by the directory
 *  
 * @param type  name of the topic
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
 * @brief Register a new topic to be managed by the directory
 * 
 * @param topic_name name of the topics
 */
void ApplicationInterface::registerNewTopic(std::string topic_name){

    if (topic_name !=  home_dir_config_.topic()){
        throw std::runtime_error("Invalid Topic. This Topic is not configured.");
        return; // topic not valid
    }

    Structure data_structure = home_dir_config_.dataStructure(topic_name);

    // Validation: Check whether the dimensions specified in the data structure are valid
    if (data_structure.object_width % data_structure.block_cols != 0 || data_structure.object_height % data_structure.block_rows != 0) {
        throw std::runtime_error("Invalid data dimensions. Object width must be divisible by block_cols and object height by block_rows.");
        return; // structure not valid
    }

    Topic type = (Topic) home_dir_config_.topicNr(topic_name);

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
 * @param timeout               waiting time for requested blocks (handle packet loss)
 */
std::future<int> ApplicationInterface::readPartialData(Topic type, std::vector<uint16_t> associated_blocks, unsigned char* output_data, std::chrono::milliseconds timeout) {
    return std::async(std::launch::async, [this, type, associated_blocks, output_data, timeout]() -> int {
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
            if(!request_data.cv.wait_for(lock, timeout, [&request_data]() {
                return request_data.received_blocks.size() == request_data.requested_blocks.size();
                })) {
                logError("Timeout while waiting for requested blocks.");
                return -1;
            }
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
 * @param timeout               waiting time for requested blocks (handle packet loss)
 */
std::future<int> ApplicationInterface::readPartialData(Topic type, int sequence_nr, std::vector<uint16_t> associated_blocks, unsigned char* output_data, std::chrono::milliseconds timeout) {
    return std::async(std::launch::async, [this, type, sequence_nr, associated_blocks, output_data, timeout]() -> int {
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
            if(!request_data.cv.wait_for(lock, timeout, [&request_data]() {
                return request_data.received_blocks.size() == request_data.requested_blocks.size();
                })) {
                logError("Timeout while waiting for requested blocks.");
                return -1;
            }
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