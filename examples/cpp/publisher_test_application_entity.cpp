#include <unistd.h>
#include <string.h>
#include <string>
#include <iostream>
#include <chrono>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>


#include <adom/application_interface.hpp>



using boost::asio::ip::udp;

int main(int argc, char* argv[])
{
    std::string setup_path = std::string(CONFIG_PATH) + "/setup_defines.json";
    std::string dir_cfg_path = std::string(CONFIG_PATH) + "/participants.json";
    std::string topic_cfg_path = std::string(CONFIG_PATH) + "/topics.json";


    ApplicationInterface adom_interface("DIRECTORY_01", dir_cfg_path, topic_cfg_path, setup_path);
    adom_interface.initialize(); 
    adom_interface.registerNewTopic("IMAGE");


    for(int i = 1; i<176; i++){
        auto start_time = std::chrono::steady_clock::now();

        auto filename = std::string(DATA_PATH) + "/full_image_" + std::to_string(i) + ".png";
        cv::Mat image_sample = cv::imread(filename, cv::IMREAD_ANYCOLOR);
        adom_interface.registerNewData(IMAGE, image_sample.data, false);

        std::this_thread::sleep_until(start_time += std::chrono::milliseconds{100});
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    adom_interface.stop();

}