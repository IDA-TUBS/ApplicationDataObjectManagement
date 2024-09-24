#ifndef APPLICATION_DATA_OBJECT_MANAGEMENT_API__PROTOCOL_H_
#define APPLICATION_DATA_OBJECT_MANAGEMENT_API__PROTOCOL_H_

#include <unistd.h>
#include <string.h>
#include <string>
#include <iostream>
#include <vector>

#include <adom/parameters.h>
#include <adom/translation.h>
#include <adom/debug.h>

using boost::asio::ip::udp;


/**
 * @brief Enum to describe the different protocol message types.
 * 
 */
enum MessageType{
    UNKNOWN = 0,
    REQUEST = 1,
    REPLY = 2,
    ANNOUNCEMENT,
};



/**
 * @brief Enum to describe the status of a block within a validity matrix of an object.
 * 
 */
enum BlockValidity
{
    INVALID = 0,
    REQUESTED,
    VALID,
    SHARED  // shared status is not currently used
};




/**
 * @brief This struct is utilized to describe an Entity of a directory. 
 *          This struct must be filled prior to setting up the directory struct itself.
 * 
 */
struct EntityDescriptor
{
    uint16_t entity_id;
    char ip_address[string_length]; //@todo ersetzen mit boost struct ?
    uint16_t ingress_port; //@todo ersetzen mit boost struct ?

    EntityDescriptor() 
        : entity_id(0), ingress_port(0) {
        std::memset(ip_address, 0, sizeof(ip_address));
    }
    

};

inline bool operator==(const EntityDescriptor& lhs, const EntityDescriptor& rhs)
{
    return lhs.entity_id == rhs.entity_id &&
        std::memcmp(lhs.ip_address, rhs.ip_address, string_length) == 0 &&
        lhs.ingress_port     == rhs.ingress_port;
}


/**
 * @brief Struct to describe an Object instance with its sequence number within a topic or object type
 * 
 * @todo 
 *  o Umbenennen, eher DataObjectIdentifier
 * 
 */
struct DataObjectInstance{
    enum Topic object_type;
    uint16_t object_sequence_nr;

    DataObjectInstance() : object_type(static_cast<Topic>(0)), object_sequence_nr(0) {}
};


/**
 * @brief Struct to describe a Data Block, that is part of an Object instance 
 * 
 */
struct DataBlockHeader{
    DataObjectInstance associated_object;
    uint16_t block_id;
    uint16_t block_size;
    long data_address_offset;

    DataBlockHeader() 
        : associated_object(), block_id(0), block_size(0), data_address_offset(0) {}


    DataBlockHeader(Topic topic, uint16_t updated_sequence_nr, uint16_t block_id, uint16_t block_size, long data_address_offset):
        associated_object(),
        block_id(block_id),
        block_size(block_size),
        data_address_offset(data_address_offset)
        {
            associated_object.object_sequence_nr = updated_sequence_nr;
            associated_object.object_type = topic;
        }
};


/**
 * @brief Application Data Object Management Messages are used as Wrapper Data Announcements, for Data Request, and Date Transports
 * 
 */
class AdomMessage{

public:
    enum MessageType type_;
    uint16_t payload_length_;
    std::vector<char> payload_;
    uint16_t adom_message_length_;

    
    AdomMessage() : type_(), payload_length_(0), payload_(0), adom_message_length_(0) {};


    AdomMessage(
        MessageType type, 
        uint16_t payload_length,
        char* msg
        ){
        type_ = type;
        payload_length_= payload_length;
        payload_.assign(payload_length_, 0); 
        memcpy(&payload_[0], msg, payload_length_);
    };

    /**
     * @brief This function is called to serialize Application Data Object Management Messages for transport
     * 
     * @param msg       Pointer to buffer to write the serialized message into
     */
    void adomMessageToNet(char* msg);

    /**
     * @brief This function is called to deserialize received Application Data Object Management Messages 
     * 
     * @param msg       Pointer to buffer to the serialized message
     */
    void netToAdomMessage(char* msg);

    /**
     * @brief This function is called to clear all attributes of the Application Data Object Management Messages for reuse
     * 
     */
    void clear();

};


/**
 * @brief DataRequest is used by a remote (subscribed) directory (reader_application_) to request specific blocks of a sample object managed by the data's (publihser/) home directory (home_directory_) via a block_validity_matrix.
 * 
 */
class DataRequest{

public:
    uint16_t request_id_;
    EntityDescriptor reader_application_;
    EntityDescriptor home_directory_;
    DataObjectInstance object_instance_;
    uint16_t object_block_count_;
    std::vector<uint8_t> object_block_validity_;

    uint16_t message_length_;

    DataRequest() : request_id_(), reader_application_(), home_directory_(), object_instance_(), object_block_count_(0), object_block_validity_(0), message_length_(0){};
    
    DataRequest(EntityDescriptor reader_application, 
                EntityDescriptor home_directory, 
                DataObjectInstance object_instance,
                uint16_t object_block_count,
                int* object_block_validity,
                int request_id
                ){
        request_id_ = request_id;
        reader_application_ = reader_application;
        home_directory_ = home_directory;
        object_instance_ = object_instance;
        object_block_count_ = object_block_count;
        object_block_validity_.assign(object_block_count_ / 8, 0); 
        for (int i = object_block_count_ / 8 - 1; i >= 0; i--)
            object_block_validity_[i] = object_block_validity[i];

        // size_t validity_size = (object_block_count_ + 7) / 8;  // +7, um sicherzustellen, dass wir auf volle Bytes aufrunden
        // object_block_validity_.assign(validity_size, 0);

        //         // Kopiere Werte aus dem übergebenen integer Array
        // for (size_t i = 0; i < validity_size; i++) {
        //     if (i < validity_size && i < object_block_count_ / 8) {
        //         object_block_validity_[i] = object_block_validity[i];
        //     } else {
        //         object_block_validity_[i] = 0; // Fallback-Wert
        //     }
        // }

        auto offset = sizeof(request_id_);
        offset = offset + sizeof(reader_application_.entity_id);
        offset = offset + sizeof(char)*string_length;
        offset = offset + sizeof(reader_application_.ingress_port);
        offset = offset + sizeof(home_directory_.entity_id);
        offset = offset + sizeof(char)*string_length;
        offset = offset + sizeof(home_directory_.ingress_port);
        offset = offset + sizeof(char)*string_length;
        offset = offset + sizeof(object_instance_.object_sequence_nr);
        offset = offset + sizeof(object_block_count_);
        offset = offset + object_block_validity_.size() * sizeof(uint8_t); 
        offset = offset + sizeof(message_length_);
        message_length_ = offset;

    
    
    };

    /**
     * @brief This function is called to serialize DataRequest for transport
     * 
     * @param msg       Pointer to buffer to write the serialized request into
     */
    void dataRequestToNet(char* msg);
    
    /**
     * @brief This function is called to deserialize received DataRequest 
     * 
     * @param msg       Pointer to buffer to the serialized request
     */
    void netToDataRequest(char* msg);

    /**
     * @brief This function is called to clear all attributes of the DataRequest for reuse
     * 
     */
    void clear();

    /**
     * @brief This function prints out the DataRequest
     * 
     */
    void print();
};


/**
 * @brief DataTransport is used by a home directory (publisher) to transport requested blocks of a sample object to a remote (subscribed) directory.

 * 
 */
class DataTransport{

public:
    uint16_t associated_request_id_;
    DataObjectInstance associated_object_;
    uint16_t block_id_;
    uint16_t block_size_;
    std::vector<char> data_;
    uint16_t message_length_;

    DataTransport() : associated_request_id_(0), block_id_(0), block_size_(0), message_length_(0) {
        data_.assign(block_size_,0); 
        associated_object_ = DataObjectInstance();
    }

    DataTransport(DataBlockHeader block_header, unsigned char* data, uint16_t associated_request_id) 
        : associated_request_id_(associated_request_id),
          block_id_(block_header.block_id), 
          block_size_(block_header.block_size) {

        associated_object_ = block_header.associated_object;
        data_.assign(block_size_,0); 
        memcpy(&data_[0], data, block_size_);

        auto offset = sizeof(associated_request_id_);
        offset = offset + sizeof(Topic);
        offset = offset + sizeof(associated_object_.object_sequence_nr);
        offset = offset + sizeof(block_id_);
        offset = offset + sizeof(block_size_);
        offset = offset + block_size_;
        offset = offset + sizeof(message_length_);
        message_length_ = offset;
    };

    /**
     * @brief This function is called to serialize DataTransport for transport
     * 
     * @param msg       Pointer to buffer to write the serialized DataTransport into
     */
    void dataTransportToNet(char* msg);
    
    /**
     * @brief This function is called to deserialize received DataTransport 
     * 
     * @param msg       Pointer to buffer to the serialized DataTransport
     */
    void netToDataTransport(char* msg);

    /**
     * @brief This function is called to clear all attributes of the DataTransport for reuse
     * 
     */
    void clear();

    /**
     * @brief This function prints out the DataTransport
     * 
     */
    void print();
};


/**
 * @brief DataAnnouncement is used by a home directory (publisher) to inform a remote (subscribed) directory of a new available sample.

 * 
 */
class DataAnnouncement{

public:
    EntityDescriptor home_directory_;
    DataObjectInstance associated_object_;
    Structure structure_;
    uint16_t message_length_;

    DataAnnouncement() {};
    DataAnnouncement(EntityDescriptor home_directory,   // to be deleted
                Topic object_type,
                uint16_t object_sequence_nr){
        home_directory_ = home_directory;
        associated_object_.object_type = object_type;
        associated_object_.object_sequence_nr = object_sequence_nr;


        structure_.block_cols = structure_.block_rows = structure_.object_channels = structure_.object_height = structure_.object_width = 0;
        structure_.type = ONE_DIMENSIONAL;

        auto offset = sizeof(home_directory_.entity_id);
        offset = offset + sizeof(char)*string_length; // length of string that holds ip_address
        offset = offset + sizeof(home_directory_.ingress_port); 
        offset = offset + sizeof(Topic); // object_type to be parsed as string for future extensions??
        offset = offset + sizeof(associated_object_.object_sequence_nr);
        offset = offset + sizeof(structure_.type);
        offset = offset + sizeof(structure_.block_rows);
        offset = offset + sizeof(structure_.block_cols);
        offset = offset + sizeof(structure_.object_height);
        offset = offset + sizeof(structure_.object_width);
        offset = offset + sizeof(structure_.object_channels);
        offset = offset + sizeof(message_length_);
        message_length_ = offset;
    };

    DataAnnouncement(EntityDescriptor home_directory, 
                Topic object_type,
                uint16_t object_sequence_nr,
                Structure structure){
        home_directory_ = home_directory;
        associated_object_.object_type = object_type;
        associated_object_.object_sequence_nr = object_sequence_nr;
        structure_ = structure;

        auto offset = sizeof(home_directory_.entity_id);
        offset = offset + sizeof(char)*string_length; // length of string that holds ip_address
        offset = offset + sizeof(home_directory_.ingress_port); 
        offset = offset + sizeof(Topic); // object_type to be parsed as string for future extensions??
        offset = offset + sizeof(associated_object_.object_sequence_nr);
        offset = offset + sizeof(structure_.type);
        offset = offset + sizeof(structure_.block_rows);
        offset = offset + sizeof(structure_.block_cols);
        offset = offset + sizeof(structure_.object_height);
        offset = offset + sizeof(structure_.object_width);
        offset = offset + sizeof(structure_.object_channels);
        offset = offset + sizeof(message_length_);
        message_length_ = offset;
    };

    /**
     * @brief This function is called to serialize DataAnnouncement for transport
     * 
     * @param msg       Pointer to buffer to write the serialized DataAnnouncement into
     */
    void dataAnnouncementToNet(char* msg);
    
    /**
     * @brief This function is called to deserialize a received DataAnnouncement 
     * 
     * @param msg       Pointer to buffer to the serialized DataAnnouncement
     */
    void netToDataAnnouncement(char* msg);

    /**
     * @brief This function is called to clear all attributes of the DataAnnouncement for reuse
     * 
     */
    void clear();

    /**
     * @brief This function prints out the DataAnnouncement
     * 
     */
    void print();
};


struct AppRequest{

    enum Topic topic;
    uint16_t sequence_nr;
    void* object_address;
    void* access_start_address;
    void* access_end_address;

};

#endif /*APPLICATION_DATA_OBJECT_MANAGEMENT_API__PROTOCOL_H_*/