#include <adom/directory_config.hpp>
#include <stdexcept>

DirectoryCfg::DirectoryCfg()
{

};

DirectoryCfg::DirectoryCfg(std::string name, std::string dir_cfg_path,  std::string topic_cfg_path, std::string setup_path)
{
    load(name, dir_cfg_path, topic_cfg_path, setup_path);
};

DirectoryCfg::~DirectoryCfg()
{

};

void DirectoryCfg::load(std::string name, std::string dir_cfg_path, std::string topic_cfg_path,  std::string setup_path)
{
    dir_name = name;
    
    boost::property_tree::read_json(dir_cfg_path, config); 
    setup.load(setup_path);  

    topic_config.load(topic(), topic_cfg_path);
    
    check();
};

void DirectoryCfg::print()
{
    logInfo("#---------- Directory Configuration -------------#")
    logInfo("# Entity ID: " << descriptor().entity_id)
    logInfo("# Address: " << descriptor().ip_address)
    logInfo("# Port: " << descriptor().ingress_port)
    logInfo("# Topic: " << topic())
    logInfo("# Role: " << role())
    logInfo("# Host: " << host())
    logInfo("#---------------------------------------------#")

    auto connected_dirs = otherDescriptors();
    for (auto it = connected_dirs.begin(); it != connected_dirs.end(); it++)
    {   
        logInfo("#---------- Other Directory Configuration -------------#")
        logInfo("# Entity ID: " << it->entity_id)
        logInfo("# Address: " << it->ip_address)
        logInfo("# Port: " << it->ingress_port)
        logInfo("#---------------------------------------------#")
    }
}

/*----------------------------------- Attribute getter Methods --------------------------------------*/

struct EntityDescriptor DirectoryCfg::descriptor()
{   
    uint16_t entity_id = getAttribute<int>(ENTITY);
    
    char ip_address[string_length]; // Corrected array declaration
    // Assuming that getAttribute for IP actually gives us a string or char array
    std::strncpy(ip_address, getAttribute<std::string>(ADDRESS).c_str(), string_length - 1); // Ensure null termination
    ip_address[string_length - 1] = '\0'; // Guarantee null-termination

    uint16_t ingress_port = getAttribute<int>(PORT);
    return EntityDescriptor(entity_id, ip_address, ingress_port);
}

struct std::list<EntityDescriptor> DirectoryCfg::otherDescriptors()
{
    std::list<EntityDescriptor> connected_dirs;

    auto pub_dir_names = topic_config.get_child(topic() + "." + PUBLISHER);
    auto subs_dir_names = topic_config.get_child(topic() + "." + SUBSCRIBER);

    if (role() == "PUBLISHER"){
        // other_dir_name = topic_config.get<std::string>(topic() + "." + SUBSCRIBER);
        // other_dir_names = topic_config.get_child(topic() + "." + SUBSCRIBER);

        for (auto it = subs_dir_names.begin(); it != subs_dir_names.end(); it++)
        {
            std::string dir_name = it->second.get_value<std::string>();
        
            uint16_t entity_id = config.get<int>(dir_name + "." + ENTITY);
            
            char ip_address[string_length]; // Corrected array declaration
            // Assuming that getAttribute for IP actually gives us a string or char array
            std::strncpy(ip_address, config.get<std::string>(dir_name + "." + ADDRESS).c_str(), string_length - 1); // Ensure null termination
            ip_address[string_length - 1] = '\0'; // Guarantee null-termination

            uint16_t ingress_port = config.get<int>(dir_name + "." + PORT);

            connected_dirs.push_back(EntityDescriptor(entity_id, ip_address, ingress_port));
        }


    } else {
        // other_dir_name = topic_config.get<std::string>(topic() + "." + PUBLISHER);
        // other_dir_names = topic_config.get_child(topic() + "." + PUBLISHER);


        for (auto it = pub_dir_names.begin(); it != pub_dir_names.end(); it++)
            {
                std::string dir_name = it->second.get_value<std::string>();
            
                uint16_t entity_id = config.get<int>(dir_name + "." + ENTITY);
                
                char ip_address[string_length]; // Corrected array declaration
                // Assuming that getAttribute for IP actually gives us a string or char array
                std::strncpy(ip_address, config.get<std::string>(dir_name + "." + ADDRESS).c_str(), string_length - 1); // Ensure null termination
                ip_address[string_length - 1] = '\0'; // Guarantee null-termination

                uint16_t ingress_port = config.get<int>(dir_name + "." + PORT);

                connected_dirs.push_back(EntityDescriptor(entity_id, ip_address, ingress_port));
            }

    }

    

    return connected_dirs;
}

// std::vector<std::string> DirectoryCfg::topics()
// {   
//     std::vector<std::string> topics;

//     // get readers child node
//     auto topics_names = config.get_child(id + "." + TOPICS);
//     for(auto it = topics_names.begin(); it != topics_names.end(); it++)
//     {
//         std::string topic = it->second.get_value<std::string>();
//         topics.push_back(topics);
//     }

//     return topics;

//     /** ODER */
//      auto topics = config.get_child(names + "." + TOPICS);
// }

std::string DirectoryCfg::topic(){
    return config.get<std::string>(dir_name + "." + TOPICS);
}

int DirectoryCfg::topicNr(std::string topic_name){
    return topic_config.get<int>(topic_name + "." + TOPIC_NR);
}

std::string DirectoryCfg::role(){
    return config.get<std::string>(dir_name + "." + ROLE);
}

std::string DirectoryCfg::host(){
    return config.get<std::string>(dir_name + "." + HOST);
}

struct Structure DirectoryCfg::dataStructure(std::string topic_name){

    struct Structure data_structure;

    data_structure.block_cols = topic_config.get<int>(topic_name + ".STRUCTURE." + BLOCK_COLS);
    data_structure.block_rows = topic_config.get<int>(topic_name + ".STRUCTURE." + BLOCK_ROWS);
    data_structure.object_channels = topic_config.get<int>(topic_name + ".STRUCTURE." + OBJECT_CHANNELS);
    data_structure.object_height = topic_config.get<int>(topic_name + ".STRUCTURE." + OBJECT_HEIGHT);
    data_structure.object_width = topic_config.get<int>(topic_name + ".STRUCTURE." + OBJECT_WIDTH);
    data_structure.type = (StructureType) topic_config.get<int>(topic_name + ".STRUCTURE." + TYPE);

    return data_structure;
}


/*------------------------------------- Private -----------------------------------------*/
bool DirectoryCfg::check()
{
    // Check Reader ID
    if(config.find(dir_name) == config.not_found())
    {
        logError("No configuration found for " << dir_name);
        throw std::invalid_argument(dir_name + " not found");
        return false;
    }

    // Check Host
    std::string hostName = getAttribute<std::string>(HOST);
    if(!setup.check(hostName))
    {
        logError("No configuration for assigned host " << hostName);
        throw std::invalid_argument(hostName + " not found");
        return false;
    }

    // Print attributes to validate configuration parameters (availability+type)
    print();

    return true;
}