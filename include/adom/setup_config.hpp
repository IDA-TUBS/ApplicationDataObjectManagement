#ifndef SETUP_CONFIG_h
#define SETUP_CONFIG_h

#define BOOST_BIND_GLOBAL_PLACEHOLDERS
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp> 

#include <adom/log.hpp>


#ifndef DEFAULT_CONFIG
#define DEFAULT_SETUP "setup_defines.json"
#endif


/********************** General ***************************/
#define HOST_ID "HOST_ID"

/**
 * @brief class for storing information about the underlying setup (hosts, etc.)
 * 
 */
class setupConfig
{
    public:

    /**
     * @brief Construct a new empty setup Config object
     * 
     */
    setupConfig();

    /**
     * @brief Construct a new setup Config object
     * 
     * @param cfg_path path to the setup file
     */
    setupConfig(std::string cfg_path);

    /**
     * @brief 
     * 
     * @param cfg_path 
     */
    void load(std::string cfg_path);

    /**
     * @brief Get the hostID object
     * 
     * @param name name of the host
     * @return uint32_t ID of the host
     */
    uint32_t get_hostID(std::string name);

    /**
     * @brief Check if host with specified name exists
     * 
     * @param name host name
     * @return true configuration exists
     * @return false no configuration available
     */
    bool check(std::string name);

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
};


#endif