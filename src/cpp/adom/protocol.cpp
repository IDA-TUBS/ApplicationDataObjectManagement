#include <adom/protocol.hpp>

/**
 * @brief This function is called to serialize Application Data Object Management Messages for transport
 * 
 * @param msg       Pointer to buffer to write the serialized message into
 */
void AdomMessage::adomMessageToNet(char* msg){

    auto msg_start = msg;

    memcpy(msg, &type_, sizeof(MessageType));
    auto offset = msg + sizeof(MessageType);

    memcpy(offset, &payload_length_, sizeof(payload_length_));
    offset = offset + sizeof(payload_length_);

    memcpy(offset, &payload_[0], sizeof(char)*payload_length_);
    offset = offset + sizeof(char)*payload_length_;

    uint16_t calc_message_length = offset - msg_start + sizeof(adom_message_length_);
    adom_message_length_ = calc_message_length; 

    memcpy(offset, &calc_message_length, sizeof(calc_message_length));
    offset = offset + sizeof(calc_message_length);

} 


/**
 * @brief This function is called to deserialize received Application Data Object Management Messages 
 * 
 * @param msg       Pointer to buffer to the serialized message
 */
void AdomMessage::netToAdomMessage(char* msg){

    memcpy(&type_, msg, sizeof(MessageType));
    auto offset = msg + sizeof(MessageType);

    memcpy(&payload_length_, offset, sizeof(payload_length_));
    offset = offset + sizeof(payload_length_);

    // Make sure that payload_ has the correct size here
    payload_.resize(payload_length_);
    memcpy(&payload_[0], offset, sizeof(char)*payload_length_);
    offset = offset + sizeof(char)*payload_length_;

    memcpy(&adom_message_length_, offset, sizeof(adom_message_length_));

} 

/**
 * @brief This function is called to deserialize received Application Data Object Management Messages 
 * 
 * @param msg       Pointer to buffer to the serialized message
 */
void AdomMessage::clear(){
    type_ = UNKNOWN;
    payload_length_ = 0;
    payload_.resize(payload_length_);
    adom_message_length_ = 0;

} 


/**
 * @brief This function is called to serialize DataRequest for transport
 * 
 * @param msg       Pointer to buffer to write the serialized request into
 */
void DataRequest::dataRequestToNet(char* msg){

    auto msg_start = msg;

    memcpy(msg, &request_id_, sizeof(request_id_));
    auto offset = msg + sizeof(request_id_);

    memcpy(offset, &reader_application_.entity_id, sizeof(reader_application_.entity_id));
    offset = offset + sizeof(reader_application_.entity_id);

    memcpy(offset, &reader_application_.ip_address, sizeof(char)*string_length);
    offset = offset + sizeof(char)*string_length;

    memcpy(offset, &reader_application_.ingress_port, sizeof(reader_application_.ingress_port));
    offset = offset + sizeof(reader_application_.ingress_port);

    memcpy(offset, &home_directory_.entity_id, sizeof(home_directory_.entity_id));
    offset = offset + sizeof(home_directory_.entity_id);

    memcpy(offset, &home_directory_.ip_address, sizeof(char)*string_length);
    offset = offset + sizeof(char)*string_length;

    memcpy(offset, &home_directory_.ingress_port, sizeof(home_directory_.ingress_port));
    offset = offset + sizeof(home_directory_.ingress_port);

    memcpy(offset, &object_instance_.object_type, sizeof(Topic));
    offset = offset + sizeof(Topic);

    memcpy(offset, &object_instance_.object_sequence_nr, sizeof(object_instance_.object_sequence_nr));
    offset = offset + sizeof(object_instance_.object_sequence_nr);

    memcpy(offset, &object_block_count_, sizeof(object_block_count_));
    offset = offset + sizeof(object_block_count_);

    memcpy(offset, &object_block_validity_[0], object_block_validity_.size() * sizeof(uint8_t));
    offset = offset + object_block_validity_.size() * sizeof(uint8_t);

    uint16_t calc_message_length = offset - msg_start + sizeof(message_length_);

    memcpy(offset, &calc_message_length, sizeof(calc_message_length));
    offset = offset + sizeof(calc_message_length);


}    


/**
 * @brief This function is called to deserialize received DataRequest 
 * 
 * @param msg       Pointer to buffer to the serialized request
 */
void DataRequest::netToDataRequest(char* msg){

    memcpy(&request_id_, msg, sizeof(request_id_));
    auto offset = msg + sizeof(request_id_);
    
    memcpy(&reader_application_.entity_id, offset, sizeof(reader_application_.entity_id));
    offset = offset + sizeof(reader_application_.entity_id);

    memcpy(&reader_application_.ip_address, offset, sizeof(char)*string_length);
    offset = offset + sizeof(char)*string_length;

    memcpy(&reader_application_.ingress_port, offset, sizeof(reader_application_.ingress_port));
    offset = offset + sizeof(reader_application_.ingress_port);


    memcpy(&home_directory_.entity_id, offset, sizeof(home_directory_.entity_id));
    offset = offset + sizeof(home_directory_.entity_id);

    memcpy(&home_directory_.ip_address, offset, sizeof(char)*string_length);
    offset = offset + sizeof(char)*string_length;

    memcpy(&home_directory_.ingress_port, offset, sizeof(home_directory_.ingress_port));
    offset = offset + sizeof(home_directory_.ingress_port);

    memcpy(&object_instance_.object_type, offset, sizeof(Topic));
    offset = offset + sizeof(Topic);

    memcpy(&object_instance_.object_sequence_nr, offset, sizeof(object_instance_.object_sequence_nr));
    offset = offset + sizeof(object_instance_.object_sequence_nr);

    memcpy(&object_block_count_, offset, sizeof(object_block_count_));
    offset = offset + sizeof(object_block_count_);

    // Make sure that object_block_validity_ has the correct size here
    object_block_validity_.resize(object_block_count_ / 8);
    memcpy(&object_block_validity_[0], offset, object_block_validity_.size() * sizeof(uint8_t));
    offset = offset + object_block_validity_.size() * sizeof(uint8_t);
    
    memcpy(&message_length_, offset, sizeof(message_length_));

    
}


/**
 * @brief This function is called to clear all attributes of the DataRequest for reuse
 * 
 */
void DataRequest::clear(){

    request_id_ = 0;

    //clear reader entity
    reader_application_.entity_id = 0;
    memset(reader_application_.ip_address, 0, 20);
    reader_application_.ingress_port = 0;

    //clear home dir entity
    home_directory_.entity_id = 0;
    memset(home_directory_.ip_address, 0, 20);
    home_directory_.ingress_port = 0;

    //clear object instance
    object_instance_.object_sequence_nr = 0;
    object_instance_.object_type = NONE;

    //clear object validity matrix
    object_block_validity_.clear(); 

    //clear block count
    object_block_count_ = 0;

    //clear message length
    message_length_ = 0;

}


/**
 * @brief This function prints out the DataRequest
 * 
 */
void DataRequest::print(){
    std::cout << "Content of Data Request #" << request_id_ << " :\n";
    std::cout << "Reader Application" << "\n";
    std::cout << "\tEntity ID: " << reader_application_.entity_id << "\n";
    std::cout << "\tIP Address: " << reader_application_.ip_address << "\n";
    std::cout << "\tIngr. Port: " << reader_application_.ingress_port << "\n";
    std::cout << "Home Directory" << "\n";
    std::cout << "\tEntity ID: " << home_directory_.entity_id << "\n";
    std::cout << "\tIP Address: " << home_directory_.ip_address << "\n";
    std::cout << "\tIngr. Port: " << home_directory_.ingress_port << "\n";
    std::cout << "Object Instance" << "\n";
    switch(object_instance_.object_type){
        case 0:
            std::cout << "\tType: NONE\n";
            break;
        case 1:
            std::cout << "\tType: IMAGE\n";
            break;
    }
    std::cout << "\tSequence Number: " << object_instance_.object_sequence_nr << "\n";
    std::cout << "Object Block Count: " << object_block_count_ << "\n";
    std::cout << "Object Block Validity Matrix: ";
    for (int i = 0; i < object_block_count_ / 8 ; i++)
        printf(PRINTF_BINARY_PATTERN_INT8, PRINTF_BYTE_TO_BINARY_INT8(object_block_validity_[i]));
    std::cout << "\n";
    std::cout << "Message Length: " << message_length_ << "\n";
    std::cout << "\n";
}


/**
 * @brief This function is called to serialize DataTransport for transport
 * 
 * @param msg       Pointer to buffer to write the serialized DataTransport into
 */
void DataTransport::dataTransportToNet(char* msg){

    auto msg_start = msg;

    memcpy(msg, &associated_request_id_, sizeof(associated_request_id_));
    auto offset = msg + sizeof(associated_request_id_);

    memcpy(offset, &associated_object_.object_type, sizeof(Topic));
    offset = offset + sizeof(Topic);

    memcpy(offset, &associated_object_.object_sequence_nr, sizeof(associated_object_.object_sequence_nr));
    offset = offset + sizeof(associated_object_.object_sequence_nr);

    memcpy(offset, &block_id_, sizeof(block_id_));
    offset = offset + sizeof(block_id_);

    memcpy(offset, &block_size_, sizeof(block_size_));
    offset = offset + sizeof(block_size_);

    memcpy(offset, &data_[0], block_size_);
    offset = offset + block_size_;

    uint16_t calc_message_length = offset - msg_start + sizeof(message_length_);
    message_length_ = calc_message_length;

    memcpy(offset, &calc_message_length, sizeof(calc_message_length));
    offset = offset + sizeof(calc_message_length);

   

}    


/**
 * @brief This function is called to deserialize received DataTransport 
 * 
 * @param msg       Pointer to buffer to the serialized DataTransport
 */
void DataTransport::netToDataTransport(char* msg){

    memcpy(&associated_request_id_, msg, sizeof(associated_request_id_));
    auto offset = msg + sizeof(associated_request_id_);

    memcpy(&associated_object_.object_type, offset, sizeof(Topic));
    offset = offset + sizeof(Topic);

    memcpy(&associated_object_.object_sequence_nr, offset, sizeof(associated_object_.object_sequence_nr));
    offset = offset + sizeof(associated_object_.object_sequence_nr); 

    memcpy(&block_id_, offset, sizeof(block_id_));
    offset = offset + sizeof(block_id_);

    memcpy(&block_size_, offset, sizeof(block_size_));
    offset = offset + sizeof(block_size_);

    // Make sure that data_ has the correct size here
    data_.resize(block_size_);
    memcpy(&data_[0], offset, block_size_);
    offset = offset + block_size_;

    memcpy(&message_length_, offset, sizeof(message_length_));    
}


/**
 * @brief This function is called to clear all attributes of the DataTransport for reuse
 * 
 */
void DataTransport::clear(){

    associated_request_id_ = 0;

    //clear object instance
    associated_object_.object_sequence_nr = 0;
    associated_object_.object_type = NONE;

    //clear block id
    block_id_ = 0;

    //clear data
    data_.assign(block_size_,0); 

    //clear block size
    block_size_ = 0;
    
    //clear message length
    message_length_ = 0;

}


/**
 * @brief This function prints out the DataTransport
 * 
 */
void DataTransport::print(){
    std::cout << "Content of Data Transport for Request #" << associated_request_id_ << " :\n";
    std::cout << "Affiliated Object Instance" << "\n";
    switch(associated_object_.object_type){
        case 0:
            std::cout << "\tType: NONE\n";
            break;
        case 1:
            std::cout << "\tType: IMAGE\n";
            break;
    }
    std::cout << "\tSequence Number: " << associated_object_.object_sequence_nr << "\n";
    std::cout << "Block ID: " << block_id_ << "\n";
    std::cout << "Block Size: " << block_size_ << "\n";
    std::cout << "Message Length: " << message_length_ << "\n";
    std::cout.write(data_.data(), block_size_); 
    std::cout << "\n" << "\n";
}


/**
 * @brief This function is called to serialize DataAnnouncement for transport
 * 
 * @param msg       Pointer to buffer to write the serialized DataAnnouncement into
 */
void DataAnnouncement::dataAnnouncementToNet(char* msg){

    auto msg_start = msg;

    memcpy(msg, &home_directory_.entity_id, sizeof(home_directory_.entity_id));
    auto offset = msg + sizeof(home_directory_.entity_id);

    memcpy(offset, &home_directory_.ip_address, sizeof(char)*string_length);
    offset = offset + sizeof(char)*string_length;

    memcpy(offset, &home_directory_.ingress_port, sizeof(home_directory_.ingress_port));
    offset = offset + sizeof(home_directory_.ingress_port);

    memcpy(offset, &associated_object_.object_type, sizeof(Topic));
    offset = offset + sizeof(Topic);

    memcpy(offset, &associated_object_.object_sequence_nr, sizeof(associated_object_.object_sequence_nr));
    offset = offset + sizeof(associated_object_.object_sequence_nr);

    memcpy(offset, &structure_.type, sizeof(structure_.type));
    offset = offset + sizeof(structure_.type);

    memcpy(offset, &structure_.block_rows, sizeof(structure_.block_rows));
    offset = offset + sizeof(structure_.block_rows);

    memcpy(offset, &structure_.block_cols, sizeof(structure_.block_cols));
    offset = offset + sizeof(structure_.block_cols);

    memcpy(offset, &structure_.object_height, sizeof(structure_.object_height));
    offset = offset + sizeof(structure_.object_height);

    memcpy(offset, &structure_.object_width, sizeof(structure_.object_width));
    offset = offset + sizeof(structure_.object_width);

    memcpy(offset, &structure_.object_channels, sizeof(structure_.object_channels));
    offset = offset + sizeof(structure_.object_channels);

    uint16_t calc_message_length = offset - msg_start + sizeof(message_length_);

    memcpy(offset, &calc_message_length, sizeof(calc_message_length));
    offset = offset + sizeof(calc_message_length);


}    


/**
 * @brief This function is called to deserialize a received DataAnnouncement 
 * 
 * @param msg       Pointer to buffer to the serialized DataAnnouncement
 */
void DataAnnouncement::netToDataAnnouncement(char* msg){

    memcpy(&home_directory_.entity_id, msg, sizeof(home_directory_.entity_id));
    auto offset = msg + sizeof(home_directory_.entity_id);

    memcpy(&home_directory_.ip_address, offset, sizeof(char)*string_length);
    offset = offset + sizeof(char)*string_length;

    memcpy(&home_directory_.ingress_port, offset, sizeof(home_directory_.ingress_port));
    offset = offset + sizeof(home_directory_.ingress_port);

    memcpy(&associated_object_.object_type, offset, sizeof(Topic));
    offset = offset + sizeof(Topic);

    memcpy(&associated_object_.object_sequence_nr, offset, sizeof(associated_object_.object_sequence_nr));
    offset = offset + sizeof(associated_object_.object_sequence_nr); 

    memcpy(&structure_.type, offset, sizeof(structure_.type));
    offset = offset + sizeof(structure_.type);

    memcpy(&structure_.block_rows, offset, sizeof(structure_.block_rows));
    offset = offset + sizeof(structure_.block_rows);

    memcpy(&structure_.block_cols, offset, sizeof(structure_.block_cols));
    offset = offset + sizeof(structure_.block_cols);

    memcpy(&structure_.object_height, offset, sizeof(structure_.object_height));
    offset = offset + sizeof(structure_.object_height);

    memcpy(&structure_.object_width, offset, sizeof(structure_.object_width));
    offset = offset + sizeof(structure_.object_width);

    memcpy(&structure_.object_channels, offset, sizeof(structure_.object_channels));
    offset = offset + sizeof(structure_.object_channels);

    memcpy(&message_length_, offset, sizeof(message_length_));
    
}


/**
 * @brief This function is called to clear all attributes of the DataAnnouncement for reuse
 * 
 */
void DataAnnouncement::clear(){

    //clear home dir entity
    home_directory_.entity_id = 0;
    memset(home_directory_.ip_address, 0, 20);
    home_directory_.ingress_port = 0;

    //clear object instance
    associated_object_.object_sequence_nr = 0;
    associated_object_.object_type = NONE;

    //clear structure
    structure_.block_cols = structure_.block_rows = structure_.object_channels = structure_.object_height = structure_.object_width = 0;
    structure_.type = ONE_DIMENSIONAL;

    //clear message length
    message_length_ = 0;

}


/**
 * @brief This function prints out the DataAnnouncement
 * 
 */
void DataAnnouncement::print(){
    std::cout << "Content of Data Announcement:" << "\n";
    std::cout << "Home Directory" << "\n";
    std::cout << "\tEntity ID: " << home_directory_.entity_id << "\n";
    std::cout << "\tIP Address: " << home_directory_.ip_address << "\n";
    std::cout << "\tIngr. Port: " << home_directory_.ingress_port << "\n";
    std::cout << "Affiliated Object Instance" << "\n";
    switch(associated_object_.object_type){
        case 0:
            std::cout << "\tType: NONE\n";
            break;
        case 1:
            std::cout << "\tType: IMAGE\n";
            break;
    }
    std::cout << "\tSequence Number: " << associated_object_.object_sequence_nr << "\n";
    std::cout << "Data Structure" << "\n";
    std::cout << "\tStructure Type: " << structure_.type << "\n";
    std::cout << "\tBlock Rows: " << structure_.block_rows << "\n";
    std::cout << "\tBlock Columns: " << structure_.block_cols << "\n";
    std::cout << "\tObject Height: " << structure_.object_height << "\n";
    std::cout << "\tObject Width: " << structure_.object_width << "\n";
    std::cout << "\tObject Channels: " << structure_.object_channels << "\n";
    std::cout << "Message Length: " << message_length_ << "\n";
    std::cout << "\n" << "\n";
}
