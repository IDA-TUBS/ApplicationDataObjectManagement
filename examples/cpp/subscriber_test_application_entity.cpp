#include <unistd.h>
#include <string.h>
#include <string>
#include <iostream>
#include <fstream>
#include <chrono>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <adom/application_interface.hpp>
#include <adom/opencv_helper.h>



using boost::asio::ip::udp;


struct Roi{
    u_int16_t first_pixel_x;
    u_int16_t first_pixel_y;
    u_int16_t last_pixel_x;
    u_int16_t last_pixel_y;

    Roi(u_int16_t first_pixel_x, u_int16_t first_pixel_y, u_int16_t last_pixel_x, u_int16_t last_pixel_y) 
        : first_pixel_x(first_pixel_x), first_pixel_y(first_pixel_y), last_pixel_x(last_pixel_x), last_pixel_y(last_pixel_y) {
        
    }
};

struct Frame{
    uint32_t nr;
    std::vector<Roi> roi_list;
};

void addRoiToFrame(Frame* frame, u_int16_t first_pixel_x, u_int16_t first_pixel_y, u_int16_t last_pixel_x, u_int16_t last_pixel_y){
    Roi roi(first_pixel_x, first_pixel_y, last_pixel_x, last_pixel_y);
    frame->roi_list.push_back(roi);
}


void readRoiData(const std::string& filename, std::vector<Frame>& frame_list) {
    std::ifstream file(filename);
    std::string line;

    if (!file.is_open()) {
        std::cerr << "Could not open the file!" << std::endl;
        return;
    }

    // read line by line
    std::getline(file, line);  // Skip header

    Frame current_frame;
    while (std::getline(file, line)) {
        std::istringstream ss(line);
        std::string sample_id_str, x_value_str, y_value_str, width_str, height_str;

        // read values from csv row
        std::getline(ss, sample_id_str, ',');
        std::getline(ss, x_value_str, ',');
        std::getline(ss, y_value_str, ',');
        std::getline(ss, width_str, ',');
        std::getline(ss, height_str);

        uint32_t sample_id = std::stoi(sample_id_str);
        u_int16_t x_value = static_cast<u_int16_t>(std::stoi(x_value_str));
        u_int16_t y_value = static_cast<u_int16_t>(std::stoi(y_value_str));
        u_int16_t width = static_cast<u_int16_t>(std::stoi(width_str));
        u_int16_t height = static_cast<u_int16_t>(std::stoi(height_str));

        // When we have a new frame, we store the previous one
        if (current_frame.nr != sample_id) {
            if (!current_frame.roi_list.empty()) {
                frame_list.push_back(current_frame);
            }
            current_frame.nr = sample_id;
            current_frame.roi_list.clear();  // Delete old ROIs (if any)
        }

        // add ROI
        addRoiToFrame(&current_frame, x_value, y_value, x_value + width, y_value + height);
    }

    // Add last frame, if available
    if (!current_frame.roi_list.empty()) {
        frame_list.push_back(current_frame);
    }

    file.close();
}

int main(int argc, char* argv[])
{
    if(argv[1] == NULL){
        printf("Please provide the directroy name for the application!\n");
        printf("usage: %s [directory_name]\n\n", argv[0]);
        return 0;
    }

    std::string setup_path = std::string(CONFIG_PATH) + "/setup_defines.json";
    std::string dir_cfg_path = std::string(CONFIG_PATH) + "/participants.json";
    std::string topic_cfg_path = std::string(CONFIG_PATH) + "/topics.json";

    // create and initialize application interface
    ApplicationInterface adom_interface(argv[1], dir_cfg_path, topic_cfg_path, setup_path);
    adom_interface.initialize();

    // preload example ROIs
    std::vector<Frame> frame_list;
    readRoiData(std::string(DATA_PATH) + "/roi_data.csv", frame_list);

    // create data_structure for object IMAGE, this data_structure is required by the underlying layers to properly parse and deparse object samples
    Structure data_structure(TWO_DIMENSIONAL, FULL_HD_BLOCK_ROWS, FULL_HD_BLOCKS_IN_ROW, FULL_HD_V_PIXEL_PER_IMAGE, FULL_HD_H_PIXEL_PER_IMAGE, CHANNEL_NUMBER);

    // register IMAGE topic, so that the directory starts managing associated data samples
    adom_interface.registerNewTopic("IMAGE");
    

    for(const auto &frame : frame_list){
        auto start = std::chrono::steady_clock::now();

        // create "empty" image to fill with caching protocol
        cv::Mat cached_image = cv::Mat::zeros(FULL_HD_V_PIXEL_PER_IMAGE, FULL_HD_H_PIXEL_PER_IMAGE, CV_8UC3);

        // Define the list of block areas to read
        std::vector<std::vector<uint16_t>> block_lists;

        // Collect Info on required block per Roi
        for (const auto &roi : frame.roi_list)
        {
        // translate roi to block access
            block_lists.push_back(findBlocksForReadAccess(roi.first_pixel_x, roi.first_pixel_y, roi.last_pixel_x, roi.last_pixel_y, data_structure));
        }
        logInfo("Translated roi to block access");

        // request blocks
        std::vector<std::future<int>> futures;
        for (const auto& blocks : block_lists) {
            futures.push_back(adom_interface.readPartialData(IMAGE, frame.nr, blocks, cached_image.data));
        }
        logInfo("Requested rois");

        // wait for all blocks to arrive
        bool all_success = true;
        for (auto& future : futures) {
            int result = future.get();
            if (result != 0) {
                all_success = false;
            }
        }
        logInfo("Received all rois");

        if (all_success) {
        
            logInfo("Received all rois with success");
            

            // Save the image as a PNG file
            std::string filename = std::string(DATA_PATH) + "/partially_cached_image_data_dir-" + std::to_string(adom_interface.getHomeDirEntityId()) + "_frame-" + std::to_string(frame.nr) + ".png";
            if (cv::imwrite(filename, cached_image)) {
                logInfo("Saved image as " + filename);
            } else {
                logInfo("Failed to save image");
            }
        }

        std::this_thread::sleep_until(start += std::chrono::milliseconds{100});
    }
    
    adom_interface.stop();

}