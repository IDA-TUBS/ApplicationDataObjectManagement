#include<adom/setup_config.hpp>

setupConfig::setupConfig()
{
}

setupConfig::setupConfig(std::string cfg_path)
{
    boost::property_tree::read_json(cfg_path, config);     
}

uint32_t setupConfig::get_hostID(std::string name)
{
    std::string hex_string = getAttribute<std::string>(name, HOST_ID);

    if(!hex_string.empty())
    {
        // Remove the "0x" prefix if present
        if (hex_string.find("0x") == 0 || hex_string.find("0X") == 0) {
            hex_string = hex_string.substr(2);
        }
        return std::stoul(hex_string, nullptr, 16);
    }
    else
    {
        return 0;
    }
}

void setupConfig::load(std::string cfg_path)
{
    boost::property_tree::read_json(cfg_path, config);     
}

bool setupConfig::check(std::string name)
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