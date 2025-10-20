#include<adom/topic_config.hpp>
#include <stdexcept>

TopicCfg::TopicCfg()
{
}

TopicCfg::TopicCfg(std::string name, std::string topic_cfg_path)
{
    boost::property_tree::read_json(topic_cfg_path, config);     
}


void TopicCfg::load(std::string name, std::string topic_cfg_path)
{
    topic_name = name;
    boost::property_tree::read_json(topic_cfg_path, config);    
    check(name); 
}

bool TopicCfg::check(std::string name)
{
    if(config.find(name) == config.not_found())
    {
        return false;
    }
    else
    {
        return true;
    }
}