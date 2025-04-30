#ifndef DIRECTORY_CONFIG_h
#define DIRECTORY_CONFIG_h

#include <adom/config.hpp>
#include <adom/parameters.h>
#include <adom/translation.h>
#include <adom/setup_config.hpp>
#include <adom/topic_config.hpp>
#include <adom/protocol.hpp>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp> 
#include <chrono>

#ifndef DEFAULT_CONFIG
#define DEFAULT_PARTICIPANTS "participants.json"
#define DEFAULT_TOPICS "topics.json"
#endif

class DirectoryCfg
{
    public:
    /*--------------------- Methods -----------------------*/
    
    /**
     * @brief Construct a new empty directory Cfg object
     * 
     */
    DirectoryCfg();

    /**
     * @brief Construct a new directory Cfg object
     * 
     * @param name directory name
     * @param participants_path path to participants config file
     * @param topics_path path to topic config  file
     * @param setup_path path to setup config  file
     */
    DirectoryCfg(
        std::string name, 
        std::string participants_path = DEFAULT_PARTICIPANTS, 
        std::string topics_path = DEFAULT_TOPICS,
        std::string setup_path = DEFAULT_SETUP
    );

    /**
     * @brief Destroy the reader Cfg object
     * 
     */
    ~DirectoryCfg();

    /**
     * @brief load a configuration
     * 
     * @param name reader name
     * @param cfg_path path to config file
     * @param setup_path path to setup file
     */
    void load(
        std::string name = DIRECTORY, 
        std::string participants_path = DEFAULT_PARTICIPANTS, 
        std::string topics_path = DEFAULT_TOPICS,
        std::string setup_path = DEFAULT_SETUP
    );
    
    /**
     * @brief print the configuration 
     * 
     */
    void print();

    /*--------------------- Attribute getter Methods -----------------------*/

    /**
     * @brief get entity descriptor
     * 
     * @return entity descriptor
     */
    struct EntityDescriptor descriptor();

    /**
     * @brief get entity descriptor
     * 
     * @return entity descriptor
     */
    struct std::list<EntityDescriptor> otherDescriptors();

    // /**
    //  * @brief get list of vectors
    //  * 
    //  * @return std::vector<std::string> 
    //  */
    // std::vector<std::string> topics();
    
    /**
     * @brief get topic
     * 
     * @return std::string
     */
    std::string topic();

    /**
     * @brief get topic nr
     * 
     * @return std::string
     */
    int topicNr(std::string topic_name);


    /**
     * @brief get role
     * 
     * @return std::string
     */
    std::string role();


    /**
     * @brief get host
     * 
     * @return std::string
     */
    std::string host();

    /**
     * @brief get host id
     * 
     * @return uint32_t
     */
    uint32_t host_id();

    /**
     * @brief get data structure of specific topic from config
     * 
     * @return std::string
     */
    struct Structure dataStructure(std::string topic_name);



    private:

    bool check();

    template<typename T>
    T getAttribute(std::string name)
    {
        return config.get<T>(dir_name + "." + name);
    }

    std::string dir_name;
    boost::property_tree::ptree config;
    setupConfig setup;
    TopicCfg topic_config;
};


#endif