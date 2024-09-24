#ifndef APPLICATION_DATA_OBJECT_MANAGEMENT_API__DIRECTORY_HPP_
#define APPLICATION_DATA_OBJECT_MANAGEMENT_API__DIRECTORY_HPP_


#include <unistd.h>
#include <string.h>
#include <string>
#include <condition_variable>
#include <unordered_map>
#include <mutex>
#include <vector>
#include <set>
#include <list>
#include <thread>
#include <boost/asio.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <adom/safe_queue.hpp>
#include <adom/protocol.hpp>
#include <adom/parameters.h>
#include <adom/log.hpp>


using boost::asio::ip::udp;


// Structure for tracking each request
struct RequestTracking {
    std::set<uint16_t> received_blocks;
    std::set<uint16_t> requested_blocks;
    std::condition_variable cv;
};


/**
 * @brief Class to decribe an ObjectSample
 * 
 */
class ObjectSample {
    public:
    
    std::vector<unsigned char> object_sample_data_;

    ObjectSample(EntityDescriptor home_directory, Topic topic, uint16_t updated_sequence_nr, unsigned char *data, Structure structure){
        home_directory_descriptor_ = home_directory;
        topic_ = topic;
        sequence_nr_ = updated_sequence_nr;
        data_structure_ = structure;
        block_count_ = data_structure_.block_cols * data_structure_.block_rows;
        block_size_ = structure.object_width * structure.object_height * structure.object_channels / block_count_;

        object_sample_data_ = translate_from_uchar(data, data_structure_);

        block_header_list_.reserve(block_count_);   // reserve enough memory to make sure no reallocation and moving of data happens

        for (uint16_t i = 0; i < block_count_; i++){

            auto address_offset = &object_sample_data_[i*block_size_] - &object_sample_data_[0];
            DataBlockHeader header {topic, updated_sequence_nr, i, block_size_, address_offset};

            block_header_list_.push_back(header);


        }


        if (home_directory_descriptor_ == local_directory_descriptor_){
            availability_matrix_.assign(block_count_,1); 
        } else {
            availability_matrix_.assign(block_count_,0); 
        }


    }
    

    EntityDescriptor getHomeDirectory(){
        return home_directory_descriptor_;
    };

    Topic getTopic(){
        return topic_;
    };

    uint16_t getSequenceNumber(){
        return sequence_nr_;
    };

    uint16_t getBlockCount(){
        return block_count_;
    };

    Structure getStructure(){
        return data_structure_;
    };    
    
    DataBlockHeader getHeader(int block_id){
        return block_header_list_[block_id];
    };

    std::vector<DataBlockHeader> getHeader(){
        return block_header_list_;
    };

    int isAvailable(int block_id){
        if (block_id > block_count_) {
            throw std::runtime_error("Invalid Block ID!");
            return -1;
        }

        return availability_matrix_[block_id];
    };

    void makeAvailable(int block_id){
        if (block_id > block_count_) {
            throw std::runtime_error("Invalid Block ID!");
            return;
        }
        availability_matrix_[block_id] = 1;

        return ;
    };

    void printObjectSample(){
        std::cout << "Content of Object Sample:" << "\n";
        std::cout << "Home Directory" << "\n";
        std::cout << "\tEntity ID: " << home_directory_descriptor_.entity_id << "\n";
        std::cout << "\tIP Address: " << home_directory_descriptor_.ip_address << "\n";
        std::cout << "\tIngr. Port: " << home_directory_descriptor_.ingress_port << "\n";
        std::cout << "Affiliated Object Instance" << "\n";
        switch(topic_){
            case 0:
                std::cout << "\tType: NONE\n";
                break;
            case 1:
                std::cout << "\tType: IMAGE\n";
                break;
        }
        std::cout << "\tSequence Number: " << sequence_nr_ << "\n";
        std::cout << "Data Structure" << "\n";
        std::cout << "\tStructure Type: " << data_structure_.type << "\n";
        std::cout << "\tBlock Rows: " << data_structure_.block_rows << "\n";
        std::cout << "\tBlock Columns: " << data_structure_.block_cols << "\n";
        std::cout << "\tObject Height: " << data_structure_.object_height << "\n";
        std::cout << "\tObject Width: " << data_structure_.object_width << "\n";
        std::cout << "\tObject Channels: " << data_structure_.object_channels << "\n";
        std::cout << "Block Count: " << block_count_ << "\n";
        std::cout << "Block Size: " << block_size_ << "\n";
        std::cout << "\n" << "\n";
    }

    private:


    Topic topic_;
    uint16_t sequence_nr_;
    EntityDescriptor home_directory_descriptor_;
    EntityDescriptor local_directory_descriptor_;
    Structure data_structure_;

    uint16_t block_count_;
    uint16_t block_size_;

    std::vector<DataBlockHeader> block_header_list_;
    std::vector<int> availability_matrix_;


};



class Directory {
    public:
    
        /**
         * @brief Construct a new Directory object
         * 
         * @param entity_descriptor 
         */
        Directory(EntityDescriptor entity_descriptor, SafeQueue<std::pair<DataRequest, udp::endpoint>> &directory_request_queue, SafeQueue<DataTransport> &directory_reply_queue, SafeQueue<std::pair<DataAnnouncement, udp::endpoint>> &directory_announcement_queue):
            directory_descriptor_(entity_descriptor),
            directory_request_queue_(directory_request_queue),
            directory_reply_queue_(directory_reply_queue),
            directory_announcement_queue_(directory_announcement_queue),
            recvMsgs{},
            handleReq{},
            handleRep{},
            handleAnn{},
            adom_socket_ingr_(socket_context_),
            send_lock_(),
            list_lock_(),
            socket_endpoint_ingr_(boost::asio::ip::address::from_string(directory_descriptor_.ip_address), directory_descriptor_.ingress_port)
        {
            
                std::string log_suffix = std::to_string(directory_descriptor_.entity_id);

                adom_socket_ingr_.open(socket_endpoint_ingr_.protocol());
                adom_socket_ingr_.set_option(boost::asio::socket_base::reuse_address(true));
                adom_socket_ingr_.set_option(boost::asio::socket_base::broadcast(true));
                adom_socket_ingr_.non_blocking();
                adom_socket_ingr_.bind(socket_endpoint_ingr_);
                directory_running_.store(true);
        }



        /**
         * @brief Initialize the new directory.
         *          Handler threads are started and an ingress endpoint for communication is created
         * 
         */
        void initiate();


        /**
         * @brief This function is called at the application interface, as soon as new data is available.
         *          New data is either added by an user application directly, or through the reception of a data announcement.
         *          
         *          This function specifically is called, when the sample sequence number is not directly given, e.g. by the user application.
         *          Therefore, the last tracked sequence number has to be determined.
         * 
         * @param new_sample      shared Pointer to new ObjectSample to be tracked and distributed if requested
         */
        void addNewObjectSample(std::shared_ptr<ObjectSample> new_sample);


        /**
         * @brief This function is called to check whether a given set of blocks (identified by their IDs) for a given object sample is currently locally available 
         * 
         * @param sample        associated object sample
         * @param block_ids     given set of requested block ID
         */
        std::vector<uint16_t> checkAvailability(std::shared_ptr<ObjectSample> sample, std::vector<uint16_t> block_ids);

        
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
        int sendDataRequest(EntityDescriptor home_directory, std::shared_ptr<ObjectSample>, std::vector<uint16_t> block_ids);
        

        /**
         * @brief This function is called to obtain a shared pointer to the most up-to-date sample for a specified topic that is managed by the directory.
         * 
         * @param type                              specified object sample topic
         * @return std::shared_ptr<ObjectSample>    shared pointer to most up-to-date sample
         */
        std::shared_ptr<ObjectSample> getLastObjectSample(Topic type);


        /**
         * @brief This function is called to obtain a shared pointer to the sample for a specified topic and a specified sequence number that is managed by the directory.
         * 
         * @param type                              specified object sample topic
         * @param sequence_nr                       specified sequence number
         * @return std::shared_ptr<ObjectSample>    shared pointer to most up-to-date sample
         */
        std::shared_ptr<ObjectSample> getSample(Topic type, int sequence_nr);


        /**
         * @brief Print out all objects tracked and cached by the directory
         * 
         */
        void printTrackedObjectSamples();


        /**
         * @brief This function returns the number of tracked samples for a given topic.
         * 
         * @param topic     Topic to check the tracked sample number on
         * @return int      Number of tracked samples for the given topic
         */
        int getNumberOfTrackedObjects(Topic topic);

        /**
         * @brief This funtion returns the sequnce number of the most up-to-date sample of a specified topic
         * 
         * @param type          specified object sample topic
         * @return uint16_t     latest sequence number
         */
        uint16_t getLatestSequencenumberOfTrackedTopic(Topic topic);

        
        /**
         * @brief Set the Expected Blocks object for a specific request
         * 
         * @param request_id        associated request ID
         * @param expected_blocks   given requested and therefore expected blocks
         */
        void setRequestedBlocks(int request_id, const std::set<uint16_t>& requested_blocks);

        
        /**
         * @brief Joining all threads for smooth exit
         * 
         */
        void join();

        /**
         * @brief Terminating all threads
         * 
         */
        void terminate();

        /**
         * @brief Freeing all Objects
         * 
         */
        void freeAllObjects();

        /**
         * @brief Stopping all processes within directory
         * 
         */
        void stop();



        /**
         * @brief information/ descriptor on other directories.
         *          Temporary solution as compensatoin for a subscriber list, that is currently missing as no discovery process is implemented
         * 
         * @todo make private
         * 
         */
        EntityDescriptor other_directory_descriptor_;
        
        std::mutex request_tracking_lock_;

        /**
         * @brief List to track received Blocks for pending requests
         * 
         */
        std::unordered_map<int, RequestTracking> request_tracking_;



    private:

         /**
         * @brief This function is called after a new sample has been registered at the topic's home directory and data announcements have to be distributed to subscribed/ interested remote directories.
         *          A list of tracked remote directories is checked and announcements are sent to those, who are marked as subscribing directories for the associated topic
         * 
         * @param new_sample Pointer to new sample that needs to be announced to interested remote directories
         */
        void distributeAnnouncements(std::shared_ptr<ObjectSample> new_sample);


        /**
         * @brief deactivated function, not valid unless application object header in tracked_objects_ is saved as void ptr
         * 
         * 
         * @param data_block_message    Received data block message that holds block data
         */
        void writeObjectBlock(DataTransport * data_block_message);


        /**
         * @brief This thread receives all incoming messages at the directories ADOM ingress socket and sorts them into their respective queue for REQUESTS, REPLIES  and ANNOUNCEMENTS for further processing
         * 
         */
        void receiveMessages();


        /**
         * @brief This thread polls the directory_request_queue_ to process all received and queued requests.
         *          For each enqueued request, the DirRequestCallback is called to process the request.
         * 
         */
        void handleRequest();


        /**
         * @brief This thread polls the directory_reply_queue_ to process all received and queued replies.
         *          For each enqueued reply, the DirReplyCallback is called to process the reply.
         * 
         */
        void handleReply();

        /**
         * @brief This thread polls the directory_announcement_queue_ to process all received and queued announcements
         *          For each enqueued announcement, the DirAnnouncementCallback is called to process the announcement.
         * 
         */
        void handleAnnouncements();

        /**
         * @brief This callback is called at the topic's/ data's home directory to answer data block requests.
         *          From the requested block list in the DataRequest, a list of all local associated data block headers is extracted.
         *          Then, the data behind these data blocks is transmitted. 
         * 
         * @param request       DataRequest received from a remote directory that holds information on the requested data blocks
         * @param target        Target destination for the requested data blocks
         */
        void DirRequestCallback(DataRequest *request, udp::endpoint target);


        /**
         * @brief This callback is called at a remote directory after requested data blocks are received.
         *          The data from the DataTransport messages is written into the locally prepared buffer
         * 
         * @param dequeued_reply   DataTransport received from a topic's/data's home directory that holds the block data on previously requested data blocks
         */
        void DirReplyCallback(DataTransport* reply);


        /**
         * @brief This callback is called at a remote directory after an announcement for a new sample on a subscribed topic is received.
         *          When a new sample is avaiable at the topic's home directory, a new "empty" object with the new sequence number has to be added to the list of tracked objects
         * 
         * @param dequeued_announcement  DataAnnouncement received from a topic's/data's home directory 
         */
        void DirAnnouncementCallback(DataAnnouncement* dequeued_announcement);


        /**
         * @brief This function is called at a topic's home directory, when data blocks need to be transmitted from the topic's home directory to a remote directory.
         *    
         * 
         * @param header_list       List of data block headers of the data blocks to be transmitted
         * @param udp_endpoint      Target UDP endpoint
         * @param request_id        ID of the request
         */
        void startDataTransport(std::list<DataBlockHeader> header_list, udp::endpoint udp_endpoint, u_int16_t request_id);

        
        /**
         * @brief Send ADOM message to the specified target.
         *          All messages, whether DataRequests, DataTransports or DataAnnouncements are are sent in a wrapper called ADOM message.
         * 
         * @param msg       ADOM message that contains DataRequests, DataTransports or DataAnnouncements
         * @param target    specified target destination of the message
         */
        void send_msg(AdomMessage msg, udp::endpoint target);


        /**
         * @brief This function checks the associated object of the received DataTransport message to find the related local DataBlockHeader.
         * 
         * @param response             Received DataTransport message whose block data has to be written into the associated local buffers
         * @return DataBlockHeader     DataBlockheaders that links to the local data buffer reserved for the associated object block
         */
        DataBlockHeader getHeader(DataTransport *response);


        /**
         * @brief This function checks the associated object of the received DataRequest message to find the related local vector of DataBlockHeaders.
         * 
         * @param request                             Received DataRequest message whose block data has to be send to a remote directory 
         * @return std::list<DataBlockHeader>       List of DataBlockheaders that link to the local object data of the associated object
         */
        std::list<DataBlockHeader> getHeader(DataRequest *request);


        /**
         * @brief Find the smallest Int value that does not exist as a request ID
         * 
         * @return int  available request ID
         */
        int findSmallestAvailableRequestId();


        /**
         * @brief This function is called to remove the oldest sample for a specified topic that is managedby the directory. 
         *          This funtion is called, when the maximum number of samples to track is reached.
         * 
         * @param type   specified object sample topic
         * @return int   result
         */
        int removeOldestObjectSample(Topic type);

        
        
        


        /**
         * @brief List of all tracked objects SAMPLES of the directory (either managed locally or managed by another directory)
         *                                        
         */
        std::map<std::pair<Topic,uint16_t>, std::shared_ptr<ObjectSample> > tracked_object_samples_;
        

        /**
         * @brief List to track last received sequencenumbers for tracked topics
         * 
         */
        std::map<Topic, uint64_t> last_received_sequencenumbers_; 


        /**
         * @brief Information/ Descriptor on self (given via Constructor paramenters)
         * 
         */
        EntityDescriptor directory_descriptor_;


        /**
         * @brief internal structs for communcation, created from values from EntityDescriptor
         * 
         */
        boost::asio::io_context socket_context_;
        udp::endpoint socket_endpoint_ingr_;
        udp::socket adom_socket_ingr_;
        std::mutex send_lock_;
        std::mutex list_lock_;
        std::mutex data_access_lock_;

        SafeQueue<std::pair<DataRequest, udp::endpoint>> &directory_request_queue_;
        SafeQueue<DataTransport> &directory_reply_queue_;
        SafeQueue<std::pair<DataAnnouncement, udp::endpoint>> &directory_announcement_queue_;

        
        /**
         * @brief Thread for the receiveDirRequest() function 
         * 
         */
        std::thread recvMsgs;

        /**
         * @brief Thread for the handle_req() function
         * 
         */
        std::thread handleReq;


        /**
         * @brief Thread for the handle_msg() function
         * 
         */
        std::thread handleRep;

        /**
         * @brief Thread for the handle_announcements() function
         * 
         */
        std::thread handleAnn;

        std::atomic<bool> directory_running_;


};



#endif /*APPLICATION_DATA_OBJECT_MANAGEMENT_API__DIRECTORY_HPP_*/