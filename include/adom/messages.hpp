#ifndef SCPS_MESSAGES_H
#define SCPS_MESSAGES_H

#include <cstring>
#include <chrono>
#include <vector>
#include <adom/message_net.hpp>
#include <adom/config.hpp>
#include <adom/log.hpp>
#include <adom/guid/guid.hpp>
#include <adom/guid/guidPrefix.hpp>


namespace scps {

enum SUBMESSAGES
{
    // basic SCPS messages needed for ADOM
    DATA_ANNOUNCEMENT = 0x20,
    DATA_REQUEST = 0x21,
    DATA_TRANSPORT = 0x22,

    // new submessane required for BEC
    NACK_TRANSPORT = 0x23
    };



class SCPSHeader
{
  public:
    /**
     * @brief constructor for empty submessage object
     */
    SCPSHeader()
    {
        this->length = sizeof(this->protocol) + 
                        sizeof(this->version) + 
                        sizeof(this->vendorID) + 
                        sizeof(this->guidPrefix);
    };


    /**
     * @brief constructor
     *
     * @param 12 byte guid prefix
     */
    SCPSHeader(GuidPrefix_t &prefix)
    {
        this->protocol = this->encode("SCPS");
        this->version = 1;
        this->vendorID = vendorID_IDA;

        guidPrefix = prefix;

        this->length = sizeof(this->protocol) + 
                        sizeof(this->version) + 
                        sizeof(this->vendorID) + 
                        sizeof(this->guidPrefix);
    };

    /**
     * @brief copy constructor
     */
    SCPSHeader(SCPSHeader &header)
    {
        this->protocol = header.protocol;
        this->version = header.version;
        this->vendorID = header.vendorID;
        this->guidPrefix = header.guidPrefix;

        this->length = header.length;
    };

    /**
     * @brief default empty destructor
     */
    ~SCPSHeader(){};

    // message contents
    uint32_t protocol;              // Identifies the message as an W2RP message.
    uint16_t version;               // Identifies the version of the W2RP protocol.
    uint16_t vendorID;              // Indicates the vendor that provides the implementation of the W2RP protocol.
    GuidPrefix_t guidPrefix;   // Defines a default prefix to use for all GUIDs that appear in the message.

    // misc information
    uint32_t length;    

    /**
     * @brief encode protocol name in uint32_t
     *
     * @param string containing 4 letter protocol name
     * @return protocol name encoded in uint32_t
     */
    uint32_t encode(const char* str) {
        uint32_t result = 0;
        for (int i = 0; i < 4; ++i) {
            result <<= 8; // Shift the bits to make space for the next character
            result |= str[i]; // Add the ASCII value of the character to the result
        }
        return result;
    };


    /**
     * @brief Convert header to char array
     * 
     * @param msg char array to store the byte stream
     */
    void headerToNet(MessageNet_t* msg);

    /**
     * @brief Convert char array to header
     * 
     * @param msg char array to store the byte stream
     */
    void netToHeader(MessageNet_t* msg);

};



class SubmessageHeader
{
  public:
    /**
     * @brief constructor for empty submessage object
     */
    SubmessageHeader()
    {
        this->length = sizeof(this->submessageId) + 
                        sizeof(this->submessageLength) + 
                        sizeof(this->flags) + 
                        sizeof(this->is_last);
    };

    /**
     * @brief constructor
     *
     * @param id of submessage
     * @param length of submessage
     * @param true if last submsg, else false
     */
    SubmessageHeader(uint8_t subMsgId, uint32_t subMsgLength, bool isLast): 
        submessageId(subMsgId),
        submessageLength(subMsgLength),
        flags(0),
        is_last(isLast)
    {
        this->length = sizeof(this->submessageId) + 
                        sizeof(this->submessageLength) + 
                        sizeof(this->flags) + 
                        sizeof(this->is_last);
    };

    /**
     * @brief copy constructor
     */
    SubmessageHeader(SubmessageHeader &header)
    {
        this->submessageId = header.submessageId;
        this->submessageLength = header.submessageLength;
        this->flags = header.flags;
        this->is_last = header.is_last;
        this->length = header.length;
    };

    /**
     * @brief destructor
     */
    ~SubmessageHeader()
    {};

    // contents
    uint8_t submessageId;
    uint32_t submessageLength;  // without header
    uint8_t flags;
    bool is_last;

    // misc information
    uint32_t length;

    /**
     * @brief Convert submsg header to char array
     * 
     * @param msg char array to store the byte stream
     */
    void headerToNet(MessageNet_t* msg);

    /**
     * @brief Convert char array to submsg header
     * 
     * @param msg char array to store the byte stream
     */
    void netToHeader(MessageNet_t* msg);
};

class SubmessageBase
{
  public:
    /**
     * @brief constructor for base msg object
     */
    SubmessageBase()
    {};

    /**
     * @brief destructor
     */
    ~SubmessageBase()
    {
        if(subMsgHeader)
        {
            delete subMsgHeader;
        }
    };

    // contents
    SubmessageHeader *subMsgHeader = nullptr;

};


class DataAnnouncementMsg: public SubmessageBase
{
  public:
    /**
     * @brief constructor for empty submessage object
     */
    DataAnnouncementMsg()
    {
        subMsgHeader = new SubmessageHeader(DATA_ANNOUNCEMENT, this->length, false);
    };

    /**
     * @brief constructor
     */
    DataAnnouncementMsg(uint32_t remote_dir_ID, uint32_t home_dir_ID,
             uint64_t object_type, uint64_t object_sequence_nr, 
             uint16_t payload_size, unsigned char *payload, std::chrono::system_clock::time_point sampleTimestamp
    ):
             remote_dir_ID(remote_dir_ID),
             home_dir_ID(home_dir_ID), 
             object_type(object_type),
             object_sequence_nr(object_sequence_nr),
             payload_size(payload_size),
             timestamp(sampleTimestamp)
    {
        this->serialized_payload = new unsigned char[payload_size]{0};
        memset(this->serialized_payload, 0, payload_size * sizeof(unsigned char));
        memcpy(this->serialized_payload, payload, payload_size);

        subMsgHeader = new SubmessageHeader(DATA_ANNOUNCEMENT, this->length, false);

        this->length = sizeof(remote_dir_ID) +
                        sizeof(home_dir_ID) +
                        sizeof(object_type) +
                        sizeof(object_sequence_nr) +
                        sizeof(payload_size) +
                        sizeof(timestamp) +
                        payload_size;
    };


    /**
     * @brief copy constructor
     */
    DataAnnouncementMsg(DataAnnouncementMsg &announce)
    {
        subMsgHeader = announce.subMsgHeader;

        this->remote_dir_ID = announce.remote_dir_ID;
        this->home_dir_ID = announce.home_dir_ID;
        this->object_type = announce.object_type;
        this->object_sequence_nr = announce.object_sequence_nr;
        this->payload_size = announce.payload_size;

        this->serialized_payload = new unsigned char[payload_size]{0};
        memset(this->serialized_payload, 0, payload_size * sizeof(unsigned char));
        memcpy(this->serialized_payload, announce.serialized_payload, announce.payload_size);
        this->timestamp = announce.timestamp;

        this->length = announce.length;
    };


    /**
     * @brief default destructor
     */
    ~DataAnnouncementMsg()
    {
        if(serialized_payload)
        {
            delete serialized_payload;
        }
    };


    uint32_t remote_dir_ID;             // Identifies the remote directory entity that is being informed of the new data-object. 
    uint32_t home_dir_ID;               // Identifies the home directory entity that made the change to the data- object.
    uint64_t object_type;               // Identifies the topic of the new data-object. 
    uint64_t object_sequence_nr;         // Uniquely identifies the change and the relative order for all changes made by the home directory.
    // parameterList inlineQos;
    uint16_t payload_size;               // The size of the optional payload
    unsigned char *serialized_payload;   // optional payload; can contain suggested structure for data-object
    std::chrono::system_clock::time_point timestamp; // timestamp signaling the arrival of the sample at the writer, required at the reader for determining deadline violations. Requires time sync between nodes hosting writer and reader 

    // misc information
    uint32_t length;

    /**
     * @brief Convert data frag to char array
     * 
     * @param msg char array to store the byte stream
     */
    void dataToNet(MessageNet_t* msg);

    /**
     * @brief Convert char array to data frag
     * 
     * @param msg char array to store the byte stream
     */
    void netToData(MessageNet_t* msg);

    void print();


};


class DataRequestMsg: public SubmessageBase
{
  public:
    /**
     * @brief constructor for empty submessage object
     */
    DataRequestMsg()
    {
        subMsgHeader = new SubmessageHeader(DATA_REQUEST, this->length, false);
    };

    /**
     * @brief constructor
     */
    DataRequestMsg(uint32_t remote_dir_ID, uint32_t home_dir_ID,
             uint64_t object_type, uint64_t object_sequence_nr, 
             uint64_t request_ID, uint16_t object_block_count, uint8_t *block_validity, 
             uint16_t payload_size, unsigned char *payload, std::chrono::system_clock::time_point sampleTimestamp
    ):
             remote_dir_ID(remote_dir_ID),
             home_dir_ID(home_dir_ID), 
             object_type(object_type),
             object_sequence_nr(object_sequence_nr),
             request_ID(request_ID),
             object_block_count(object_block_count),
             payload_size(payload_size),
             timestamp(sampleTimestamp)
    {
        this->object_block_validity = new uint8_t[(object_block_count + 7) / 8];
        memcpy(this->object_block_validity, block_validity, (object_block_count + 7) / 8);

        this->serialized_payload = new unsigned char[payload_size]{0};
        memset(this->serialized_payload, 0, payload_size * sizeof(unsigned char));
        memcpy(this->serialized_payload, payload, payload_size);

        subMsgHeader = new SubmessageHeader(DATA_REQUEST, this->length, false);

        this->length = sizeof(remote_dir_ID) +
                        sizeof(home_dir_ID) +
                        sizeof(request_ID) +
                        sizeof(object_block_count) +
                        sizeof(object_type) +
                        sizeof(object_sequence_nr) +
                        sizeof(timestamp) +
                        sizeof(payload_size) +
                        ((object_block_count + 7) / 8) +
                        payload_size;
    };


    /**
     * @brief copy constructor
     */
    DataRequestMsg(DataRequestMsg &request)
    {
        subMsgHeader = request.subMsgHeader;

        this->remote_dir_ID = request.remote_dir_ID;
        this->home_dir_ID = request.home_dir_ID;
        this->object_type = request.object_type;
        this->object_sequence_nr = request.object_sequence_nr;
        this->request_ID = request.request_ID;
        this->object_block_count = request.object_block_count;
        this->payload_size = request.payload_size;

        object_block_validity = new uint8_t[object_block_count / 8]();
        for (int i = object_block_count / 8 - 1; i >= 0; i--)
            this->object_block_validity[i] = request.object_block_validity[i];

        this->serialized_payload = new unsigned char[payload_size]{0};
        memset(this->serialized_payload, 0, payload_size * sizeof(unsigned char));
        memcpy(this->serialized_payload, request.serialized_payload, request.payload_size);
        this->timestamp = request.timestamp;

        this->length = request.length;
    };


    /**
     * @brief default destructor
     */
    ~DataRequestMsg()
    {
        if(serialized_payload)
        {
            delete serialized_payload;
        }
    };

    
    uint32_t remote_dir_ID;             // Identifies the remote directory entity that is being informed of the new data-object. 
    uint32_t home_dir_ID;               // Identifies the home directory entity that made the change to the data- object.
    uint64_t object_type;               // Identifies the topic of the new data-object. 
    uint64_t object_sequence_nr;         // Uniquely identifies the change and the relative order for all changes made by the home directory.
    uint64_t request_ID;                // Identifies the request ID with with a remote directory requests data of the new data-object. 
    // parameterList inlineQos;
    uint16_t object_block_count;        // Defines the overall amount of blocks the new data object consists of.
    uint8_t *object_block_validity;     // Array that defines which blocks are requested.
    uint16_t payload_size;               // The size of the optional payload.
    unsigned char *serialized_payload;   // optional payload; can contain suggested structure for data-object.
    std::chrono::system_clock::time_point timestamp; // timestamp signaling the arrival of the sample at the writer, required at the reader for determining deadline violations. Requires time sync between nodes hosting writer and reader 

    // misc information
    uint32_t length;

    /**
     * @brief Convert data frag to char array
     * 
     * @param msg char array to store the byte stream
     */
    void dataToNet(MessageNet_t* msg);

    /**
     * @brief Convert char array to data frag
     * 
     * @param msg char array to store the byte stream
     */
    void netToData(MessageNet_t* msg);

    void print();


};

class DataTransportMsg: public SubmessageBase
{
  public:
    /**
     * @brief constructor for empty submessage object
     */
    DataTransportMsg()
    {
        subMsgHeader = new SubmessageHeader(DATA_TRANSPORT, this->length, false);
    };

    /**
     * @brief constructor
     */
    DataTransportMsg(uint32_t remote_dir_ID, uint32_t home_dir_ID,
             uint64_t object_type, uint64_t object_sequence_nr, uint64_t request_ID, uint16_t block_ID,
             uint16_t block_size, 
             uint64_t data_address_offset,
             unsigned char *block_payload, std::chrono::system_clock::time_point sampleTimestamp
    ):
             remote_dir_ID(remote_dir_ID),
             home_dir_ID(home_dir_ID), 
             object_type(object_type),
             object_sequence_nr(object_sequence_nr),
             request_ID(request_ID),
             block_ID(block_ID),
             block_size(block_size),
             data_address_offset(data_address_offset),
             timestamp(sampleTimestamp)
    {
        this->serialized_block_payload = new unsigned char[block_size]{0};
        memset(this->serialized_block_payload, 0, block_size * sizeof(unsigned char));
        memcpy(this->serialized_block_payload, block_payload, block_size);

        subMsgHeader = new SubmessageHeader(DATA_TRANSPORT, this->length, false);

        this->length = sizeof(remote_dir_ID) +
                        sizeof(home_dir_ID) +
                        sizeof(object_type) +
                        sizeof(object_sequence_nr) +
                        sizeof(request_ID) +
                        sizeof(block_ID) +
                        sizeof(block_size) +
                        sizeof(data_address_offset) +
                        sizeof(timestamp) +
                        block_size;
    };


    /**
     * @brief copy constructor
     */
    DataTransportMsg(DataTransportMsg &transport)
    {
        subMsgHeader = transport.subMsgHeader;

        this->remote_dir_ID = transport.remote_dir_ID;
        this->home_dir_ID = transport.home_dir_ID;
        this->object_type = transport.object_type;
        this->object_sequence_nr = transport.object_sequence_nr;
        this->request_ID = transport.request_ID;
        this->block_size = transport.block_size;
        this->data_address_offset = transport.data_address_offset;

        this->serialized_block_payload = new unsigned char[block_size]{0};
        memset(this->serialized_block_payload, 0, block_size * sizeof(unsigned char));
        memcpy(this->serialized_block_payload, transport.serialized_block_payload, transport.block_size);
        this->timestamp = transport.timestamp;

        this->length = transport.length;
    };


    /**
     * @brief default destructor
     */
    ~DataTransportMsg()
    {
        if(serialized_block_payload)
        {
            delete serialized_block_payload;
        }
    };


    uint32_t remote_dir_ID;             // Identifies the remote directory entity that is being informed of the new data-object. 
    uint32_t home_dir_ID;               // Identifies the home directory entity that made the change to the data- object.
    uint64_t object_type;               // Identifies the topic of the new data-object. 
    uint64_t object_sequence_nr;         // Uniquely identifies the change and the relative order for all changes made by the home directory.
    uint64_t request_ID;                // Identifies the request ID with with a remote directory requests data of the new data-object. 
    uint16_t block_ID;                  // Identifies the block ID with of the block requested by a remote directory. 
    // parameterList inlineQos;
    uint16_t block_size;               // The size of the block that is transported.
    uint64_t data_address_offset;      // the offset of data block within the data object
    unsigned char *serialized_block_payload;   // payload contains the block data to be transported.
    std::chrono::system_clock::time_point timestamp; // timestamp signaling the arrival of the sample at the writer, required at the reader for determining deadline violations. Requires time sync between nodes hosting writer and reader 

    // misc information
    uint32_t length;

    /**
     * @brief Convert data frag to char array
     * 
     * @param msg char array to store the byte stream
     */
    void dataToNet(MessageNet_t* msg);

    /**
     * @brief Convert char array to data frag
     * 
     * @param msg char array to store the byte stream
     */
    void netToData(MessageNet_t* msg);

    void print();


};


class NetMessageParser
{
  public:
    /**
    * @brief constructor
    */
    NetMessageParser()
    {};

    /**
    * @brief destructor
    */
    ~NetMessageParser()
    {};


    void getSubmessages(MessageNet_t* msg, std::vector<SubmessageBase*> *res);
};


} //end namespace

#endif //MESSAGES_H