#ifndef TOPIC_CONFIG_h
#define TOPIC_CONFIG_h

#include <adom/config.hpp>
#include <adom/parameters.h>
#include <adom/translation.h>
#include <adom/setup_config.hpp>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp> 
#include <chrono>

#ifndef DEFAULT_CONFIG
#define DEFAULT_PARTICIPANTS "participants.json"
#define DEFAULT_TOPICS "topics.json"
#endif


/********************** General ***************************/
#define HOST_ID "HOST_ID"

/**
 * @brief class for storing information about the underlying setup (hosts, etc.)
 * 
 */
class TopicCfg
{
    public:

    /**
     * @brief Construct a new empty setup Config object
     * 
     */
    TopicCfg();

    /**
     * @brief Construct a new setup Config object
     * 
     * @param name              name of the topic
     * @param topic_cfg_path    path to the topic config file
     */
    TopicCfg(std::string name, std::string topic_cfg_path);

    /**
     * @brief load a configuration
     * 
     * @param name              name of the topic
     * @param topic_cfg_path    path to the topic config file
     */
    void load(std::string name, std::string topic_cfg_path);

    /**
     * @brief
     * 
     * @return struct Structure 
     */
    struct Structure structure();

    /**
     * @brief Check if topic with specified name exists
     * 
     * @param name topic name
     * @return true configuration exists
     * @return false no configuration available
     */
    bool check(std::string name);

    template<typename T>
    T get(const std::string& key) {
        return config.get<T>(key); // Use Boost's ptree get method
    }

    boost::property_tree::ptree get_child(const std::string& key) {
        return config.get_child(key);
    }

    private:

    template<typename T>
    T getAttribute(std::string name, std::string attr)
    {
        auto node = config.find(name);
        if (node != config.not_found())
        {
            return config.get<T>(name + "." + attr);
        }
        else
        {
            logError("Host " << name << " does not exist");
            throw std::invalid_argument("Host " + name + " does not exist");
        }
    }

    boost::property_tree::ptree config;
    std::string topic_name;
};


#endif