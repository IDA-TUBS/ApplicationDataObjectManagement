/*
 *
 */

#include <adom/messages.hpp>


namespace scps {

void SCPSHeader::headerToNet(MessageNet_t* msg)
{
    msg->add(&protocol, sizeof(protocol));
    msg->add(&version, sizeof(version));
    msg->add(&vendorID, sizeof(vendorID));
    msg->add(guidPrefix.value, guidPrefix.size);
}

void SCPSHeader::netToHeader(MessageNet_t* msg)
{
    msg->read(&protocol, sizeof(protocol));
    msg->read(&version, sizeof(version));
    msg->read(&vendorID, sizeof(vendorID));
    msg->read(guidPrefix.value, guidPrefix.size);
}


void SubmessageHeader::headerToNet(MessageNet_t* msg)
{   
    msg->add(&submessageId, sizeof(submessageId));
    msg->add(&submessageLength, sizeof(submessageLength));
    msg->add(&flags, sizeof(flags));
    msg->add(&is_last, sizeof(is_last));
}

void SubmessageHeader::netToHeader(MessageNet_t* msg)
{
    msg->read(&submessageId, sizeof(submessageId));
    msg->read(&submessageLength, sizeof(submessageLength));
    msg->read(&flags, sizeof(flags));
    msg->read(&is_last, sizeof(is_last));

    this->length = sizeof(this->submessageId) + 
                        sizeof(this->submessageLength) + 
                        sizeof(this->flags) + 
                        sizeof(this->is_last);
}

void DataAnnouncementMsg::dataToNet(MessageNet_t* msg)
{
    // add submsg header
    subMsgHeader->headerToNet(msg);

    // add contents
    msg->add(&remote_dir_ID, sizeof(remote_dir_ID));
    msg->add(&home_dir_ID, sizeof(home_dir_ID));
    msg->add(&object_type, sizeof(object_type));
    msg->add(&object_sequence_nr, sizeof(object_sequence_nr));
    msg->add(&payload_size, sizeof(payload_size));
    msg->add(serialized_payload, payload_size);
    msg->add(&timestamp, sizeof(timestamp));
}

void DataAnnouncementMsg::netToData(MessageNet_t* msg)
{
    // read submsg header
    subMsgHeader->netToHeader(msg);

    // read contents
    msg->read(&remote_dir_ID, sizeof(remote_dir_ID));
    msg->read(&home_dir_ID, sizeof(home_dir_ID));
    msg->read(&object_type, sizeof(object_type));
    msg->read(&object_sequence_nr, sizeof(object_sequence_nr));
    msg->read(&payload_size, sizeof(payload_size));
    // Allocate memory for the payload data
    this->serialized_payload = new unsigned char[this->payload_size];  
    msg->read(serialized_payload, payload_size);
    msg->read(&timestamp, sizeof(timestamp));

    this->length = sizeof(remote_dir_ID) +
                sizeof(home_dir_ID) +
                sizeof(object_type) +
                sizeof(object_sequence_nr) +
                sizeof(payload_size) +
                sizeof(timestamp) +
                payload_size;
}

void DataAnnouncementMsg::print()
{
    logDebug("[DataAnnouncementMsg] remote_dir_ID: " << remote_dir_ID)
    logDebug("[DataAnnouncementMsg] home_dir_ID: " << home_dir_ID)
    logDebug("[DataAnnouncementMsg] object_type: " << object_type)
    logDebug("[DataAnnouncementMsg] object_sequence_nr: " << object_sequence_nr)
}


void DataRequestMsg::dataToNet(MessageNet_t* msg)
{
    // add submsg header
    subMsgHeader->headerToNet(msg);

    // add contents
    msg->add(&remote_dir_ID, sizeof(remote_dir_ID));
    msg->add(&home_dir_ID, sizeof(home_dir_ID));
    msg->add(&object_type, sizeof(object_type));
    msg->add(&object_sequence_nr, sizeof(object_sequence_nr));
    msg->add(&request_ID, sizeof(request_ID));
    msg->add(&object_block_count, sizeof(object_block_count));
    msg->add(object_block_validity, (this->object_block_count + 7) / 8);    
    msg->add(&payload_size, sizeof(payload_size));
    msg->add(serialized_payload, payload_size);
    msg->add(&timestamp, sizeof(timestamp));
}

void DataRequestMsg::netToData(MessageNet_t* msg)
{
    // read submsg header
    subMsgHeader->netToHeader(msg);

    // read contents
    msg->read(&remote_dir_ID, sizeof(remote_dir_ID));
    msg->read(&home_dir_ID, sizeof(home_dir_ID));
    msg->read(&object_type, sizeof(object_type));
    msg->read(&object_sequence_nr, sizeof(object_sequence_nr));
    msg->read(&request_ID, sizeof(request_ID));
    msg->read(&object_block_count, sizeof(object_block_count));
    // Allocate memory for the validity matrix
    this->object_block_validity = new uint8_t[(this->object_block_count + 7) / 8];  
    msg->read(object_block_validity, (this->object_block_count + 7) / 8);
    msg->read(&payload_size, sizeof(payload_size));
    // Allocate memory for the payload data
    this->serialized_payload = new unsigned char[this->payload_size];  
    msg->read(serialized_payload, payload_size);
    msg->read(&timestamp, sizeof(timestamp));

    this->length = sizeof(remote_dir_ID) +
                        sizeof(home_dir_ID) +
                        sizeof(object_type) +
                        sizeof(object_sequence_nr) +
                        sizeof(request_ID) +
                        sizeof(object_block_count) +
                        ((this->object_block_count + 7) / 8) +
                        sizeof(timestamp) +
                        sizeof(payload_size) +
                        payload_size;
}

void DataRequestMsg::print()
{
    logDebug("[DataRequestMsg] remote_dir_ID: " << remote_dir_ID)
    logDebug("[DataRequestMsg] home_dir_ID: " << home_dir_ID)
    logDebug("[DataRequestMsg] object_type: " << object_type)
    logDebug("[DataRequestMsg] object_sequence_nr: " << object_sequence_nr)
    logDebug("[DataRequestMsg] request_ID: " << request_ID)
}


void DataTransportMsg::dataToNet(MessageNet_t* msg)
{
    // add submsg header
    subMsgHeader->headerToNet(msg);

    // add contents
    msg->add(&remote_dir_ID, sizeof(remote_dir_ID));
    msg->add(&home_dir_ID, sizeof(home_dir_ID));
    msg->add(&object_type, sizeof(object_type));
    msg->add(&object_sequence_nr, sizeof(object_sequence_nr));
    msg->add(&request_ID, sizeof(request_ID));
    msg->add(&block_ID, sizeof(block_ID));
    msg->add(&block_size, sizeof(block_size));
    msg->add(serialized_block_payload, block_size);
    msg->add(&timestamp, sizeof(timestamp));
}

void DataTransportMsg::netToData(MessageNet_t* msg)
{
    // read submsg header
    subMsgHeader->netToHeader(msg);

    // read contents
    msg->read(&remote_dir_ID, sizeof(remote_dir_ID));
    msg->read(&home_dir_ID, sizeof(home_dir_ID));
    msg->read(&object_type, sizeof(object_type));
    msg->read(&object_sequence_nr, sizeof(object_sequence_nr));
    msg->read(&request_ID, sizeof(request_ID));
    msg->read(&block_ID, sizeof(block_ID));
    msg->read(&block_size, sizeof(block_size));    
    // Allocate memory for the payload data
    this->serialized_block_payload = new unsigned char[this->block_size];  
    msg->read(serialized_block_payload, block_size);
    msg->read(&timestamp, sizeof(timestamp));

    this->length = sizeof(remote_dir_ID) +
                    sizeof(home_dir_ID) +
                    sizeof(object_type) +
                    sizeof(object_sequence_nr) +
                    sizeof(request_ID) +
                    sizeof(block_ID) +
                    sizeof(block_size) +
                    sizeof(timestamp) +
                    block_size;
}

void DataTransportMsg::print()
{
    logDebug("[DataTransportMsg] remote_dir_ID: " << remote_dir_ID)
    logDebug("[DataTransportMsg] home_dir_ID: " << home_dir_ID)
    logDebug("[DataTransportMsg] object_type: " << object_type)
    logDebug("[DataTransportMsg] object_sequence_nr: " << object_sequence_nr)
    logDebug("[DataTransportMsg] request_ID: " << request_ID)
    logDebug("[DataTransportMsg] block_ID: " << block_ID)
}



void NetMessageParser::getSubmessages(MessageNet_t* msg, std::vector<SubmessageBase*> *res)
{
    SubmessageHeader *subMsgHeader;
    subMsgHeader = new SubmessageHeader();

    
    // Move read head to submessage segment
    msg->reset();
    msg->movePos(SCPSHeader().length);

    while (true)
    {
        // check if end of message has been reached
        if(msg->pos >= (msg->length - 1))
        {
            // end of message reached
            break;
        }

        // parse submessage header
        subMsgHeader->netToHeader(msg);

        // based on id, parse corresponding submessage
        switch (subMsgHeader->submessageId)
        {
        case DATA_ANNOUNCEMENT:
            if(msg->movePos(-(subMsgHeader->length)))
            {
                DataAnnouncementMsg *announce;
                announce = new DataAnnouncementMsg();

                announce->netToData(msg);

                res->push_back(announce);
            }
            else
            {
                // something went wrong
                logDebug("[NetMessageParser] something went wrong")
            }
            break;
        case DATA_REQUEST:
            if(msg->movePos(-(subMsgHeader->length)))
            {
                DataRequestMsg *request;
                request = new DataRequestMsg();

                request->netToData(msg);

                res->push_back(request);
            }
            else
            {
                // something went wrong
                logDebug("[NetMessageParser] something went wrong")
            }
            break;
        case DATA_TRANSPORT:
            if(msg->movePos(-(subMsgHeader->length)))
            {
                DataTransportMsg *transport;
                transport = new DataTransportMsg();

                transport->netToData(msg);

                res->push_back(transport);
            }
            else
            {
                // something went wrong
                logDebug("[NetMessageParser] something went wrong")
            }
            break;
        default:
            logDebug("[NetMessageParser] Unknown sub-message")
            break;
        }
    }
    
}


} // end namespace