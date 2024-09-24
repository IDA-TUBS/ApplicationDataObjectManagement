#include <adom/directory.hpp>


#include <boost/log/trivial.hpp>
#include <boost/log/core.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/sources/severity_logger.hpp>
#include <boost/log/sources/record_ostream.hpp>
#include <boost/log/utility/setup/file.hpp>
#include <boost/log/utility/setup/common_attributes.hpp>
#include <boost/log/utility/setup/console.hpp>
#include <chrono>


/**
 * @brief Initialize the new directory.
 *          Handler threads are started and an ingress endpoint for communication is created
 * 
 */
void Directory::initiate(){

    recvMsgs = std::thread{&Directory::receiveMessages, this};
    handleReq = std::thread{&Directory::handleRequest, this};
    handleRep = std::thread{&Directory::handleReply, this};
    handleAnn = std::thread{&Directory::handleAnnouncements, this};

    udp::endpoint udp_ingress_endpoint(boost::asio::ip::address::from_string(directory_descriptor_.ip_address), directory_descriptor_.ingress_port);
}


/**
 * @brief This function is called at the application interface, as soon as new data is available.
 *          New data is either added by an user application directly, or through the reception of a data announcement.
 *          
 *          This function specifically is called, when the sample sequence number is not directly given, e.g. by the user application.
 *          Therefore, the last tracked sequence number has to be determined.
 * 
 * @param new_sample      shared Pointer to new ObjectSample to be tracked and distributed if requested
 */
void Directory::addNewObjectSample(std::shared_ptr<ObjectSample> new_sample){
    {  
        std::lock_guard<std::mutex> lock(data_access_lock_); // Mutex-Protection

        // insert a tuple containing the home directory, availability_matrix, block information and block header list into the list of tracked objects with a key containing the topic and sequence number)
        tracked_object_samples_.insert(std::make_pair(std::make_pair(new_sample->getTopic(), new_sample->getSequenceNumber()), new_sample));
    }
  
    // Update the tracker for the last received sequence number
    Topic topic = new_sample->getTopic();
    int new_sequence_number = new_sample->getSequenceNumber();
    auto it = last_received_sequencenumbers_.find(topic);

    if (it != last_received_sequencenumbers_.end()) {
        // There is already an entry for this topic
        int current_sequence_number = it->second;

        if (new_sequence_number < current_sequence_number) {
            // Error: new sequence number is smaller
            logError("Error: New sequence number " + std::to_string(new_sequence_number) + " is smaller than the current sequence number " + std::to_string(current_sequence_number) + " for topic " + std::to_string(topic) + ".");
        } else {
            // Update the sequence number if the new one is larger
            last_received_sequencenumbers_[topic] = new_sequence_number;
        }
    } else {
        // No entry available - add the new sequence number
        last_received_sequencenumbers_.insert(std::make_pair(topic, new_sequence_number));
    }

    if (new_sample->getHomeDirectory() == directory_descriptor_) {
        distributeAnnouncements(new_sample);
    }   

    int num = getNumberOfTrackedObjects(new_sample->getTopic());

    logDebug("Number of tracked Samples for Topic " + std::to_string(new_sample->getTopic()) + " is " + std::to_string(num) + "...");

    if (getNumberOfTrackedObjects(new_sample->getTopic()) > MAX_NUM_TRACKED_SAMPLES){
        removeOldestObjectSample(new_sample->getTopic());
    }
}


 /**
 * @brief This function is called after a new sample has been registered at the topic's home directory and data announcements have to be distributed to subscribed/ interested remote directories.
 *          A list of tracked remote directories is checked and announcements are sent to those, who are marked as subscribing directories for the associated topic
 * 
 * @param new_sample Pointer to new sample that needs to be announced to interested remote directories
 */
void Directory::distributeAnnouncements(std::shared_ptr<ObjectSample> new_sample){
    /**
     * @todo
     * o create list for subscribed directories and their ingress ports or endpoints
     * o go through that list and send annoucnement
     * 
     */
    
    DataAnnouncement announcement(directory_descriptor_, new_sample->getTopic(), new_sample->getSequenceNumber(), new_sample->getStructure());

    // serialize announcement for transport
    char serialized_announcement[max_buffer_length-adom_message_overhead];
    memset(serialized_announcement, 0, max_buffer_length-adom_message_overhead);
    announcement.dataAnnouncementToNet(serialized_announcement);

    // wrap serialized announcement in ADOM message for transport
    AdomMessage message(ANNOUNCEMENT, announcement.message_length_, serialized_announcement);

        
    // for (/** for each subscribed directory to the given topic*/){
    //     EntityDescriptor subscribed_dir;

    //     ...

    //     // create endpoint from subscribed_directory
    //     udp::endpoint subscribed_dir_endpoint(boost::asio::ip::address::from_string(subscribed_dir.ip_address), subscribed_dir.ingress_port); 

    //     send_msg(message, subscribed_dir_endpoint);
        
    // }
 
#ifdef PROTOCOL_TEST
    // for testing:  other_directory_descriptor is tracked with port info, this other directory is directly approached for now 
    logDebug("Sending announcement of sample " + std::to_string(new_sample->getSequenceNumber()) + " to directory " + std::to_string(other_directory_descriptor_.entity_id) + "...");

    
    // create endpoint from home_directory
    udp::endpoint other_dir_endpoint(boost::asio::ip::address::from_string(other_directory_descriptor_.ip_address), other_directory_descriptor_.ingress_port); 

    send_msg(message, other_dir_endpoint);
#endif
    
}


/**
 * @brief This function is called at a remote directory to write the data from a received DaTaTransport message into the remote object buffer.
 * 
 * 
 * @param data_block_message    Received data block message that holds block data
 */
void Directory::writeObjectBlock(DataTransport * data_block_message){

    // Ensure that data_block_message is not NULL
    if (data_block_message == nullptr) {
        logError("Received null data_block_message.");
        return; 
    }

    // get local header for received data block
    DataBlockHeader block_header = getHeader(data_block_message);

    {
        // Lock for accessing data
        std::lock_guard<std::mutex> lock(data_access_lock_); // Mutex-Protection

        // Create key for the search in the map
        auto key = std::make_pair(block_header.associated_object.object_type, block_header.associated_object.object_sequence_nr);
        auto it = tracked_object_samples_.find(key);

        // Does the key exist in the map?
        if (it != tracked_object_samples_.end()) {
            std::shared_ptr<ObjectSample>& associated_sample_ptr = it->second;

            // Check whether the shared_ptr is valid
            if (associated_sample_ptr) {
                memcpy(&associated_sample_ptr->object_sample_data_[0] + block_header.data_address_offset,
                    data_block_message->data_.data(),
                    data_block_message->block_size_);

                // Updating the availability
                associated_sample_ptr->makeAvailable(block_header.block_id);
            } else {
                logError("Affiliated sample pointer is null.");
            }
        } else {
            logError("No associated sample found for type: " + std::to_string(block_header.associated_object.object_type) + ", sequence number: " + std::to_string(block_header.associated_object.object_sequence_nr));
        }
    }

    // Track blocks for the request
    std::lock_guard<std::mutex> lock(request_tracking_lock_);
    auto& request_data = request_tracking_[data_block_message->associated_request_id_];
    request_data.received_blocks.insert(block_header.block_id);
    
    // Check whether all requested blocks have been received
    if (request_data.received_blocks.size() == request_data.requested_blocks.size()) { // todo, check for the right block ids
        // Receive all blocks, notification
        request_data.cv.notify_all();
        logDebug("Received All Requested Data! \n");
    }

}


/**
 * @brief This thread receives all incoming messages at the directories ADOM ingress socket and sorts them into their respective queue for REQUESTS, REPLIES  and ANNOUNCEMENTS for further processing
 * 
 */
void Directory::receiveMessages(void){
    /**
     *   @todo
     *   o requester endpoint oder home directory endpoint müssen noch nicht hier ermittelt werden, da die Info im übermittelten Request oder Announcement weiterhin vorhanden ist
     * 
     */

    DataRequest request;
    DataTransport reply;
    DataAnnouncement announcement;
    AdomMessage message;
    boost::system::error_code error;
 
    // Buffer for socket  message
    char udp_buffer[max_buffer_length]; 
    std::pair<DataRequest, udp::endpoint> current_request;
    std::pair<DataAnnouncement, udp::endpoint> current_announcement;
    udp::endpoint sender_endpoint;
    size_t ret_val = 0;

    while(directory_running_.load())
    {      
        while(directory_running_.load() && ret_val == 0){
            // receive serialized ADOM message wrapper
            ret_val = adom_socket_ingr_.receive_from(boost::asio::buffer(udp_buffer), sender_endpoint, 0, error);
        }

        if (ret_val == 0){
            logInfo("Stopped receiving Messages... \n");
            return;
        }
        
        logDebug("Received Msg on Ingress Port! \n");

        // deserialize ADOM message
        message.netToAdomMessage(udp_buffer);

        if (message.type_ == MessageType::REQUEST){

            // deserialize Request message
            request.netToDataRequest(message.payload_.data());

            logDebug("Received Data Request! \n")// + request.print());

            // determine Requester endpoint from the Request header
            const std::string &ip_str(request.reader_application_.ip_address);
            udp::endpoint udp_requester_endpoint(boost::asio::ip::address::from_string(ip_str), request.reader_application_.ingress_port); 

            // queue the request and the requester entpoint into the request queue for further processing
            current_request = {request, sender_endpoint};
            directory_request_queue_.enqueue(current_request);
            
            //clean up
            memset(udp_buffer, 0, max_buffer_length);
            request.clear();
            message.clear();
            ret_val = 0;

        } else if (message.type_ == MessageType::REPLY){
            reply.netToDataTransport(message.payload_.data());
            
            logDebug("Received Data Transport \n");

            // queue the reply into the reply queue for further processing
            directory_reply_queue_.enqueue(reply);

            
            //clean up
            memset(udp_buffer, 0, max_buffer_length);
            reply.clear();
            message.clear();
            ret_val = 0;

        } else if (message.type_ == MessageType::ANNOUNCEMENT){
            announcement.netToDataAnnouncement(message.payload_.data());

            logDebug("Received Data Announcement! \n");

            // determine home directory endpoint of the announcement and data from the announcement header
            const std::string &ip_str(announcement.home_directory_.ip_address);
            udp::endpoint udp_ann_home_directory_endpoint(boost::asio::ip::address::from_string(ip_str), announcement.home_directory_.ingress_port); 

            // queue the announcement and the home directory endpoint into the announcement queue for further processing
            current_announcement = {announcement, udp_ann_home_directory_endpoint};
            directory_announcement_queue_.enqueue(current_announcement);
            
            //clean up
            memset(udp_buffer, 0, max_buffer_length);
            announcement.clear();
            message.clear();
            ret_val = 0;

        } else {
            logError("Received not supported message type! \n");
        }

        
    }
    
    logInfo("Stopped receiving Messages... \n");

}


/**
 * @brief This thread polls the directory_request_queue_ to process all received and queued requests.
 *          For each enqueued request, the DirRequestCallback is called to process the request.
 * 
 */
void Directory::handleRequest(){

    std::pair<DataRequest, udp::endpoint> dequeued_request;
    while(directory_running_.load())
    { 
        try {
            dequeued_request = directory_request_queue_.dequeue(true, std::chrono::milliseconds(450));
        } catch (...) {
            continue;
        }

        Directory::DirRequestCallback(&dequeued_request.first, dequeued_request.second);

        dequeued_request.first.clear();

    }
    logInfo("Stopped handling Requests... \n");

}


/**
 * @brief This thread polls the directory_reply_queue_ to process all received and queued replies.
 *          For each enqueued reply, the DirReplyCallback is called to process the reply.
 * 
 */
void Directory::handleReply(){

    DataTransport dequeued_reply;
    while(directory_running_.load())
    {
        try {
            dequeued_reply = directory_reply_queue_.dequeue(true, std::chrono::milliseconds(450));
        } catch (...) {
            continue;;
        }

        Directory::DirReplyCallback(&dequeued_reply);

        dequeued_reply.clear();

    }
    logInfo("Stopped handling Replies... \n");
}


/**
 * @brief This thread polls the directory_announcement_queue_ to process all received and queued announcements
 *          For each enqueued announcement, the DirAnnouncementCallback is called to process the announcement.
 * 
 */
void Directory::handleAnnouncements(){

    std::pair<DataAnnouncement, udp::endpoint> dequeued_announcement;
    while(directory_running_.load())
    {   
        try {
            dequeued_announcement = directory_announcement_queue_.dequeue(true, std::chrono::milliseconds(450));
        } catch (...) {
            continue;
        }

        // dequeued annoucement is foreign announcement, that needs processed here 
        Directory::DirAnnouncementCallback(&dequeued_announcement.first);

        dequeued_announcement.first.clear();

    }
    logInfo("Stopped handling Announcements... \n");
}


/**
 * @brief This callback is called at the topic's/ data's home directory to answer data block requests.
 *          From the requested block list in the DataRequest, a list of all local associated data block headers is extracted.
 *          Then, the data behind these data blocks is transmitted. 
 * 
 * @param request   DataRequest received from a remote directory that holds information on the requested data blocks
 * @param target    Target destination for the requested data blocks
 */
void Directory::DirRequestCallback(DataRequest *request, udp::endpoint target){


    logDebug("Collecting relevant headers... \n");

    std::list<DataBlockHeader> header_list = getHeader(request);

    startDataTransport(header_list, target, request->request_id_);
}


/**
 * @brief This callback is called at a remote directory after requested data blocks are received.
 *          The data from the DataTransport messages is written into the locally prepared buffer
 * 
 * @param dequeued_reply   DataTransport received from a topic's/data's home directory that holds the block data on previously requested data blocks
 */
void Directory::DirReplyCallback(DataTransport *dequeued_reply){
    
    std::lock_guard<std::mutex> lock(list_lock_);

    logDebug("Received data block " + std::to_string(dequeued_reply->block_id_) + "... \n") // + dequeued_reply->print());

    writeObjectBlock(dequeued_reply);
}


/**
 * @brief This callback is called at a remote directory after an announcement for a new sample on a subscribed topic is received.
 *          When a new sample is avaiable at the topic's home directory, a new "empty" object with the new sequence number has to be added to the list of tracked objects
 * 
 * @param dequeued_announcement  DataAnnouncement received from a topic's/data's home directory 
 */
void Directory::DirAnnouncementCallback(DataAnnouncement *dequeued_announcement){

    logDebug("Received announcement for Sequ. Nr. " + std::to_string(dequeued_announcement->associated_object_.object_sequence_nr) + " for topic: " + std::to_string(dequeued_announcement->associated_object_.object_type) + "... \n") // + dequeued_announcement->print());

    size_t object_data_size = dequeued_announcement->structure_.object_width * dequeued_announcement->structure_.object_height * dequeued_announcement->structure_.object_channels;
    std::vector<unsigned char> data(object_data_size);

    auto new_object = std::make_shared<ObjectSample>(dequeued_announcement->home_directory_ , dequeued_announcement->associated_object_.object_type, dequeued_announcement->associated_object_.object_sequence_nr, data.data(), dequeued_announcement->structure_);
    addNewObjectSample(new_object);

}


/**
 * @brief This function is called to check whether a given set of blocks (identified by their IDs) for a given object sample is currently locally available 
 * 
 * @param sample        associated object sample
 * @param block_ids     given set of requested block ID
 */
std::vector<uint16_t> Directory::checkAvailability(std::shared_ptr<ObjectSample> sample, std::vector<uint16_t> block_ids){

    // Remove the invalid or unavailable IDs from block_ids
    block_ids.erase(std::remove_if(block_ids.begin(), block_ids.end(), 
        [sample](uint16_t block_id) {
            switch(sample->isAvailable(block_id)) {
                case (-1):
                    logError("Invalid Block ID!");
                    return true; // Remove invalid IDs
                case (0):
                    return false; // Keep unavailable IDs
                case (1):
                    return true; // Remove available IDs
                default:
                    return false; // Standard: Keep ID
            }
        }), block_ids.end());

    return block_ids; // Returns the updated vector
}


/**
 * @brief This function is called from within a read process triggered by the application.
 *          After the data to be read and the associated blocks are determined, ths function is called to iniate the process of sending the required data requests.
 * 
 * @todo make private
 * 
 * @param home_directory        Home directory for the data that is requested
 * @param sample                Affiliated sample the read process is directed to
 * @return                      Request ID
 */
int Directory::sendDataRequest(EntityDescriptor home_directory, std::shared_ptr<ObjectSample> sample, std::vector<uint16_t> block_ids){

    // create struct for associated object
    DataObjectInstance associated_object_instance;
    associated_object_instance.object_type = sample->getTopic();
    associated_object_instance.object_sequence_nr = sample->getSequenceNumber();
    uint16_t object_block_count = sample->getStructure().block_cols * sample->getStructure().block_rows;

    // create and fill object_block_validity_matrix
    auto object_block_validity_matrix = new int[object_block_count];
    memset(object_block_validity_matrix, 0, object_block_count);

    for (int i = 0; i < object_block_count; i++){
        object_block_validity_matrix[i] = 0;
        for (int j = 0; j < block_ids.size(); j++){
            if (block_ids[j] == i) {
                int matrix_index = block_ids[j] / 8;
                int bit_offset = block_ids[j]%8;
                uint8_t mask = 0b10000000;
                mask = mask >> bit_offset;
                object_block_validity_matrix[matrix_index] |= mask;
            }
        }
    }

    // construct and serialize request 
    int request_id;
    {    
        std::lock_guard<std::mutex> lock(request_tracking_lock_);
        request_id = findSmallestAvailableRequestId();
    
        // Set the expected blocks in the directory for this specific request ID
        std::set<uint16_t> blockSet(block_ids.begin(), block_ids.end());
        setRequestedBlocks(request_id, blockSet);
    }
    
    DataRequest request(directory_descriptor_, home_directory, associated_object_instance, object_block_count, object_block_validity_matrix, request_id);

    char serialized_request[max_buffer_length - adom_message_overhead];
    memset(serialized_request, 0, max_buffer_length - adom_message_overhead);
    request.dataRequestToNet(serialized_request);
    
    // create general ADOM message with serialized request as payload
    AdomMessage message(REQUEST, request.message_length_, serialized_request);


    // create endpoint from home_directory
    udp::endpoint home_dir_endpoint(boost::asio::ip::address::from_string(home_directory.ip_address), home_directory.ingress_port); 

    // send out ADOM message with request
    logDebug("Sending Data Request for blocks "  + std::to_string(*block_ids.begin()) + " - " + std::to_string(*(--block_ids.end())) + " ...");
    send_msg(message, home_dir_endpoint);
    

    delete[] object_block_validity_matrix;


    return request_id;
}


/** 
 * @brief This function is called at a topic's home directory, when data blocks need to be transmitted from the topic's home directory to a remote directory.
 *    
 * 
 * @param header_list       List of data block headers of the data blocks to be transmitted
 * @param udp_endpoint      Target UDP endpoint
 */
void Directory::startDataTransport(std::list<DataBlockHeader> header_list, udp::endpoint udp_endpoint, uint16_t request_id){

    auto associated_object_frame = header_list.front().associated_object.object_sequence_nr;

    // send out block data for every DataBlockHeader in the specified list
    while (!header_list.empty()) {
        auto key = std::make_pair(header_list.front().associated_object.object_type, header_list.front().associated_object.object_sequence_nr);
        auto it = tracked_object_samples_.find(key);

        if (it != tracked_object_samples_.end()) { // Check whether the key exists
            std::shared_ptr<ObjectSample> associated_sample = it->second; // Access to the shared_ptr

            if (associated_sample) { // Ensure that the shared_ptr is valid
                DataTransport block_reply(header_list.front(), &associated_sample->object_sample_data_[0] + header_list.front().data_address_offset, request_id);

                // serialize DataTransport message for transport
                char serialized_reply[max_buffer_length - adom_message_overhead];
                memset(serialized_reply, 0, max_buffer_length - adom_message_overhead);
                block_reply.dataTransportToNet(serialized_reply);

                logDebug( "Sending data block " + std::to_string(header_list.front().block_id) + " for Request "  + std::to_string(request_id) + "...\n");

                // wrap serialized DataTransport message within ADOM message and send out ADOM message
                AdomMessage message(REPLY, block_reply.message_length_, serialized_reply);
                send_msg(message, udp_endpoint);
            } else {
                logError("Affiliated sample is null for block ID " + std::to_string(header_list.front().block_id));
            }

        } else {
            logError("No affiliate sample found for object type: " + std::to_string(header_list.front().associated_object.object_type) + ", sequence number: " + std::to_string(header_list.front().associated_object.object_sequence_nr));
        }

        // remove processed DataBlockHeader from the list
        header_list.pop_front();

        // for flow control
        std::this_thread::sleep_for(std::chrono::microseconds(20));
    }
} 


/**
 * @brief Send ADOM message to the specified target.
 *          All messages, whether DataRequests, DataTransports or DataAnnouncements are are sent in a wrapper called ADOM message.
 * 
 * @param msg       ADOM message that contains DataRequests, DataTransports or DataAnnouncements
 * @param target    specified target destination of the message
 */
void Directory::send_msg(AdomMessage msg, udp::endpoint target){
    std::lock_guard<std::mutex> lock(send_lock_);   

    // serialize ADOM message for transport
    char udp_buffer_message[max_buffer_length] =  {0};
    msg.adomMessageToNet(udp_buffer_message);

    // send out serialized ADOM message via the directory's dedicated socket
    size_t bytes_sent = adom_socket_ingr_.send_to(boost::asio::buffer(udp_buffer_message, msg.adom_message_length_), target);

}


/**
 * @brief This function checks the associated object of the received DataTransport message to find the related local DataBlockHeader.
 * 
 * @param response             Received DataTransport message whose block data has to be written into the associated local buffers
 * @return DataBlockHeader     DataBlockheaders that links to the local data buffer reserved for the associated object block
 */
DataBlockHeader Directory::getHeader(DataTransport *response){

    // Check whether the incoming message is null
    if (!response) {
        logError("Received null DataTransport pointer.");
        return DataBlockHeader();  // Return of a standard DataBlockHeader object
    }

    auto key = std::make_pair(response->associated_object_.object_type, response->associated_object_.object_sequence_nr);
    auto it = tracked_object_samples_.find(key);

    if (it != tracked_object_samples_.end()) {
        std::shared_ptr<ObjectSample> associated_sample = it->second;

        if (associated_sample) {
            // Return of the DataBlockHeader if the associated_sample is valid
            return associated_sample->getHeader(response->block_id_);
        }
    } else {
            logError("Affiliated sample is null for object type: " + std::to_string(response->associated_object_.object_type) + ", sequence number: " + std::to_string(response->associated_object_.object_sequence_nr));
    }

    // Return of a standard DataBlockHeader object if not found
    return DataBlockHeader(); 

}



/**
 * @brief This function checks the associated object of the received DataRequest message to find the related local list of DataBlockHeaders.
 * 
 * @param request                       Received DataRequest message whose block data has to be send to a remote directory 
 * @return std::list<DataBlockHeader>   List of DataBlockheaders that link to the local object data of the associated object
 */
std::list<DataBlockHeader> Directory::getHeader(DataRequest *request){

    std::list<DataBlockHeader> header_list;
    std::vector<uint8_t> block_list;

    // Ensure that the request pointer is not NULL
    if (request == nullptr) {
        logError("Received null DataRequest pointer.");
        return header_list; // Return of an empty list
    }

    // go through the object_block_validity_ matrix in the request to extract the IDs and DataBlockHeaders of requested data blocks
    for(int i = 0; i < (request->object_block_count_-1); i++){
        int matrix_index = i / 8;
        int bit_offset = 8 - (i % 8);

        uint8_t mask = request->object_block_validity_[matrix_index];
        uint8_t block_bit = (mask >> (bit_offset-1)) & 0b00000001; 


        if (block_bit == 1){
            block_list.push_back(i);

            // Create key for the search in the map
            auto key = std::make_pair(request->object_instance_.object_type, request->object_instance_.object_sequence_nr);
            auto it = tracked_object_samples_.find(key);

            // Check whether the key exists in the map
            if (it != tracked_object_samples_.end()) {
                std::shared_ptr<ObjectSample> associated_sample = it->second;

                // Ensure that the shared_ptr is valid
                if (associated_sample) {
                    header_list.push_back(associated_sample->getHeader(i));
                } else {
                    logError("Affiliated sample pointer is null for key: (" + std::to_string(request->object_instance_.object_type) + ", " + std::to_string(request->object_instance_.object_sequence_nr) + ")");
                }
            } else {
                logError("No associated sample found for key: (" + std::to_string(request->object_instance_.object_type) + ", " + std::to_string(request->object_instance_.object_sequence_nr) + ")");
            }
        }
    }
    
    return header_list;
}

/**
 * @brief Set the Expected Blocks object for a specific request
 * 
 * @param request_id        associated request ID
 * @param expected_blocks   given requested and therefore expected blocks
 */
void Directory::setRequestedBlocks(int request_id, const std::set<uint16_t>& requested_blocks) {

    request_tracking_[request_id].requested_blocks = requested_blocks;
}


/**
 * @brief Find the smallest Int value that does not exist as a request ID
 * 
 * @return int  available request ID
 */
int Directory::findSmallestAvailableRequestId() {
    
    std::vector<int> existing_ids;

    // Fill the vector with existing request IDs
    for (const auto& pair : request_tracking_) {
        existing_ids.push_back(pair.first);
    }

    // Sort IDs
    std::sort(existing_ids.begin(), existing_ids.end());

    // Search for the smallest non-existent value
    int smallest_available = 0; // Start bei 0

    for (const auto& id : existing_ids) {
        if (id == smallest_available) {
            smallest_available++; // If the value is available, go to the next
        } else if (id > smallest_available) {
            // If the ID is greater than the smallest available value, then the current value is available
            break;
        }
    }

    return smallest_available;
}


/**
 * @brief This function is called to obtain a shared pointer to the most up-to-date sample for a specified topic that is managed by the directory.
 * 
 * @param type                              specified object sample topic
 * @return std::shared_ptr<ObjectSample>    shared pointer to most up-to-date sample
 */
std::shared_ptr<ObjectSample> Directory::getLastObjectSample(Topic type){

    int sequence_number = getLatestSequencenumberOfTrackedTopic(type);

    return getSample(type, sequence_number);
}

/**
 * @brief This function is called to obtain a shared pointer to the sample for a specified topic and a specified sequence number that is managed by the directory.
 * 
 * @param type                              specified object sample topic
 * @param sequence_nr                       specified sequence number
 * @return std::shared_ptr<ObjectSample>    shared pointer to most up-to-date sample
 */
std::shared_ptr<ObjectSample> Directory::getSample(Topic type, int sequence_nr){

    // Creating the key for the map
    auto key = std::make_pair(type, sequence_nr);
    auto it = tracked_object_samples_.find(key);

    // Check whether the element was found in the map
    if (it != tracked_object_samples_.end()) {
        std::shared_ptr<ObjectSample> object_sample = it->second;

        // Return of the found object pointer
        return object_sample; // Sharing the smart pointer is safe because we get it from the map
    } else {
        // Error logging or handling if the object is not found
        logError("No ObjectSample found for type: " + std::to_string(type) + ", sequence number: " + std::to_string(sequence_nr));
        
        // Return of an empty pointer (nullptr)
        return nullptr;
    }
}

/**
 * @brief This function is called to remove the oldest sample for a specified topic that is managedby the directory. 
 *          This funtion is called, when the maximum number of samples to track is reached.
 * 
 * @param type   specified object sample topic
 * @return int   result
 */
int Directory::removeOldestObjectSample(Topic type){
    uint16_t oldestSequnceNumber = std::numeric_limits<uint16_t>::max();
    std::pair<Topic, uint16_t> keyToRemove = {type, 0};

    std::lock_guard<std::mutex> lock(data_access_lock_); // Mutex protection
    
    // Go through the map to find the oldest element
    for (const auto& entry : tracked_object_samples_) {
        const std::pair<Topic, uint16_t>& key = entry.first;
        const std::shared_ptr<ObjectSample>& sample = entry.second;

        if (key.first == type && key.second < oldestSequnceNumber) {
            oldestSequnceNumber = key.second;
            keyToRemove = key;
        }
    }

    // Remove element if it is found
    if (oldestSequnceNumber != std::numeric_limits<uint16_t>::max()) {
        tracked_object_samples_.erase(keyToRemove);
        return 0;
    } else {
        logError("Could not remove old Object. No element found for the specified Topic.\n");
        return -1;
    }

}


/**
 * @brief Print out all objects tracked and cached by the directory
 * 
 */
void Directory::printTrackedObjectSamples(){

    if (tracked_object_samples_.empty()){
        std::cout << "There are no tracked Objects" << "\n\n\n";
    } else {
    
        std::cout << "Tracked Objects:" << "\n";

        for (auto it = tracked_object_samples_.begin(); it != tracked_object_samples_.end(); it++){
            it->second->printObjectSample();
            std::cout << "\tObject Block Address List skipped ";
            std::cout << "\n";
            std::cout << "\n";
        }
    }

}


/**
 * @brief This function returns the number of tracked samples for a given topic.
 * 
 * @param topic     Topic to check the tracked sample number on
 * @return int      Number of tracked samples for the given topic
 */
int Directory::getNumberOfTrackedObjects(Topic topic){

    if (tracked_object_samples_.empty()){
        return 0;
    } else {
        int count = 0;
        for (auto it = tracked_object_samples_.begin(); it != tracked_object_samples_.end(); it++){
            if (it->first.first == topic) {
                count++;
            }
        }
        return count;
    }
}
 

/**
 * @brief This funtion returns the sequnce number of the most up-to-date sample of a specified topic
 * 
 * @param type          specified object sample topic
 * @return uint16_t     latest sequence number
 */
uint16_t Directory::getLatestSequencenumberOfTrackedTopic(Topic type){

    return last_received_sequencenumbers_.find(type)->second;

}


/**
 * @brief Joining all threads for smooth exit
 * 
 */
void Directory::join(){
    recvMsgs.join();
    handleReq.join();
    handleRep.join();
    handleAnn.join();
}

/**
 * @brief Joining all threads for smooth exit
 * 
 */
void Directory::terminate(){
    pthread_cancel(recvMsgs.native_handle());
    pthread_cancel(handleReq.native_handle());
    pthread_cancel(handleRep.native_handle());
    pthread_cancel(handleAnn.native_handle());
}

/**
 * @brief Freeing all Objects
 * 
 */
void Directory::freeAllObjects(){
    tracked_object_samples_.clear();
    last_received_sequencenumbers_.clear();

};

/**
 * @brief Stopping all processes within directory
 * 
 */
void Directory::stop(){
    directory_running_.store(false);
    join();
}