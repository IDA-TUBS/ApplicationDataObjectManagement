#ifndef APPLICATION_DATA_OBJECT_MANAGEMENT_API__OPENCV_HELPER_H_
#define APPLICATION_DATA_OBJECT_MANAGEMENT_API__OPENCV_HELPER_H_

#include <unistd.h>
#include <string.h>
#include <string>
#include <vector>

#include <adom/translation.h>

/**
 * @brief Finds associated data block for a given data address and returns the number of the associated block
 * 
 * @param readDataAddress       given data address (through read process)
 * @param objectStartData       start address of data objects
 * @param data_structure        internal structure of the data object and how it is cached
 * @return int                  
 */
inline uint16_t findBlockFromAddress(const unsigned char* readDataAddress, const unsigned char* objectStartData, Structure data_structure) {

    if(data_structure.type == TWO_DIMENSIONAL){
        // Calculating the offset of the address within the data array
        uint16_t offset = readDataAddress - objectStartData;
        if (offset < 0 || offset >= data_structure.object_width * data_structure.object_height * data_structure.object_channels) {
            throw std::runtime_error("Invalid read data address.");
        }

        // Calculate the (x, y) coordinates based on the offset
        uint16_t elementIndex = offset / data_structure.object_channels; // Index of the element in the matrix (ignores the channels)
        uint16_t x = elementIndex % data_structure.object_width;
        uint16_t y = elementIndex / data_structure.object_width;

        // Calculation of the associated data block
        int blockHeight = data_structure.object_height / data_structure.block_rows;
        int blockWidth = data_structure.object_width / data_structure.block_cols;
        uint16_t blockRow = y / blockHeight;
        uint16_t blockCol = x / blockWidth;
        uint16_t blockIndex = blockRow * data_structure.block_cols + blockCol;

        return blockIndex;
    }
    return 0;
    
}

/**
 * @brief Finds associated data block for a given data address and returns the number of the associated block
 * 
 * @param readDataAddress       given data address (through read process)
 * @param objectStartData       start address of data objects
 * @param data_structure        internal structure of the data object and how it is cached
 * @return int                  
 */
inline uint16_t findBlock(int x, int y, Structure data_structure) {

    if(data_structure.type == TWO_DIMENSIONAL){

        // Calculation of the associated data block
        int blockHeight = data_structure.object_height / data_structure.block_rows;
        int blockWidth = data_structure.object_width / data_structure.block_cols;
        uint16_t blockRow = y / blockHeight;
        uint16_t blockCol = x / blockWidth;
        uint16_t blockIndex = blockRow * data_structure.block_cols + blockCol;

        return blockIndex;
    }
    return 0;
    
}

/**
 * @brief Finds associated data blocks for a read access to an image that is specified by the first pixel and the last pixel.
 * 
 * @param first_pixel_x              x coordindate of start pixel
 * @param first_pixel_y              y coordindate of start pixel
 * @param last_pixel_x               x coordindate of end pixel
 * @param last_pixel_y               y coordindate of end pixel
 * @param data_structure             structure of the image, e.g. number of columns and rows...
 * @return std::vector<uint16_t>     block IDs of the associated blocks
 */
inline std::vector<uint16_t> findBlocksForReadAccess(int first_pixel_x, int first_pixel_y, int last_pixel_x, int last_pixel_y, Structure data_structure) {
    
    std::vector<uint16_t> associated_blocks;

    //find the associated blocks to the first and last pixel
    int first_block = findBlock(first_pixel_x, first_pixel_y, data_structure);
    int last_block = findBlock(last_pixel_x, last_pixel_y, data_structure);

    
    uint16_t firstBlockRow = first_block / data_structure.block_cols;
    uint16_t firstBlockCol = first_block % data_structure.block_cols;
    uint16_t lastBlockRow = last_block / data_structure.block_cols;
    uint16_t lastBlockCol = last_block % data_structure.block_cols;

    // Iterate over all rows and columns in the defined range to get all the associated block for the specified region of interest
    for (uint16_t row = firstBlockRow; row <= lastBlockRow; ++row) {
        for (uint16_t col = firstBlockCol; col <= lastBlockCol; ++col) {
            uint16_t index = row * data_structure.block_cols + col; // Berechnung des Index
            associated_blocks.push_back(index); 
        }
    }

    return associated_blocks;
}


#endif /* APPLICATION_DATA_OBJECT_MANAGEMENT_API__OPENCV_HELPER_H_ */