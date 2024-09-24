#ifndef APPLICATION_DATA_OBJECT_MANAGEMENT_API__TRANSLATION_H_
#define APPLICATION_DATA_OBJECT_MANAGEMENT_API__TRANSLATION_H_

#include <unistd.h>
#include <string.h>
#include <string>
#include <iostream>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <adom/parameters.h>


/**
 * @brief Enum to describe different types of decomposition strategies for objects samples.
 * 
 */

enum StructureType{
    NOT_SPECIFIED = 0,
    ONE_DIMENSIONAL = 1,
    TWO_DIMENSIONAL = 2
};

/**
 * @brief To decompose the object sample into data blocks, that are beneficial to the data type or user application's use of the data,
 *          the user should specify the proper structure of the decomposed object sample.
 *          
 *          1D data structure: Data blocks are composed of one (or several successive) memory "lines" (equals standard hardware data caching)
 *              --For 1D data, block_line_width equals about 1kB to fill an etherent frame and block_line_count is 1.--
 *              For 1D data, object_height and block_rows is 1. 
 * 
 *          2D data structure: Data blocks are composed of several NON successive memory "lines"
 *              --For 2D data, the following applies: block_line_width * block_line_count <= 1kB and block_line_count > 1.--
 * 
 */
struct Structure{
    StructureType type;
    uint16_t block_rows;
    uint16_t block_cols;
    uint16_t object_height;
    uint16_t object_width;
    uint16_t object_channels;

    Structure(){};
    
    Structure(StructureType type, uint16_t block_rows, uint16_t block_cols, uint16_t object_height, uint16_t object_width, uint16_t object_channels):
        type(type),
        block_rows(block_rows),
        block_cols(block_cols),
        object_height(object_height),
        object_width(object_width),
        object_channels(object_channels)
    {};
};

/**
 * @brief This function takes data in form of unsigned char *data, e.g. unsigned char array, and translates in into the data format required to construct a cachable Object Sample.
 * 
 * @todo
 *  o check for allignement issues, is the new data buffer big enough etc
 * 
 */  
inline  std::vector<unsigned char> translate_from_uchar(unsigned char *data, Structure structure){
    int block_count = structure.block_cols;
    size_t object_data_size = structure.object_width * structure.object_height * structure.object_channels;
    int block_height = structure.object_height / structure.block_rows;
    int block_width = structure.object_width / structure.block_cols;
    
    std::vector<unsigned char> object_sample_data(object_data_size);

    if (structure.type == ONE_DIMENSIONAL){
        if (structure.object_width % structure.block_cols != 0 || structure.block_rows == 1) {
            throw std::runtime_error("Invalid image dimensions. Width must be divisible by block_cols and block_rows must be 1.");
        }
        
        int dest_index = 0;
        for(uint16_t i = 0; i < block_count; i++){
            dest_index =  i * block_width * structure.object_channels;
            memcpy(&object_sample_data[dest_index], &data[dest_index], block_width * structure.object_channels);
        }

        return object_sample_data;

    } else if(structure.type == TWO_DIMENSIONAL) {

        if (structure.object_width % structure.block_cols != 0 || structure.object_height % structure.block_rows != 0) {
            throw std::runtime_error("Invalid image dimensions. Width must be divisible by block_cols and height by block_rows.");
        } 

        int dest_index = 0;
        for (int row = 0; row < structure.block_rows; ++row) {
            for (int col = 0; col < structure.block_cols; ++col) {
                int start_row = row * block_height;
                int start_col = col * block_width;
                for (int r = 0; r < block_height; ++r) {
                    int src_index = ((start_row + r) * structure.object_width + start_col) * structure.object_channels;
                    memcpy(&object_sample_data[dest_index], &data[src_index], block_width * structure.object_channels);
                    dest_index += block_width * structure.object_channels;
                }
            }
        }

        return object_sample_data;

    } else {
        throw std::runtime_error("Invalid structure type.");
    }
};

inline  int translate_uchar_access(const unsigned char* pixel_address, const unsigned char* original_data, Structure structure){    
    // Validation: Check whether the dimensions of the screen are divisible by numCols and NumRows
    if (structure.object_width % structure.block_cols != 0 || structure.object_height % structure.block_rows != 0) {
        throw std::runtime_error( "Invalid image dimensions. Width must be divisible by numCols and height by numRows.");
    }

    int block_height = structure.object_height / structure.block_rows;
    int block_width = structure.object_width / structure.block_cols;

    // Calculating the offset of the address within the data array
    int offset = pixel_address - original_data;
    if (offset < 0 || offset >= structure.object_width * structure.object_height * structure.object_channels) {
        throw std::runtime_error("Invalid pixel address");
    }

    // Calculate the (x, y) coordinates based on the offset
    int pixel_index = offset / structure.object_channels; // Index des Pixels im Bild (ignoriert die Farbkanäle)
    int x = pixel_index % structure.object_width;
    int y = pixel_index / structure.object_width;

    // Calculation of the sub-image position
    int associated_block_row = y / block_height;
    int associated_block_col = x / block_width;

    // Calculate the index of the subimage based on the sequence in image.data
    int block_index = associated_block_row * structure.block_cols + associated_block_col;

    return block_index;

};

inline unsigned char * translate_to_uchar(std::vector<unsigned char> object_sample_data, Structure structure){
    int block_count = structure.block_cols;
    size_t object_data_size = structure.object_width * structure.object_height * structure.object_channels;
    int block_height = structure.object_height / structure.block_rows;
    int block_width = structure.object_width / structure.block_cols;
    
    std::vector<unsigned char> data(object_data_size);
    
    if (structure.type == ONE_DIMENSIONAL){
        if (structure.object_width % structure.block_cols != 0 || structure.block_rows == 1) {
            throw std::runtime_error("Invalid image dimensions. Width must be divisible by block_cols and block_rows must be 1.");
        }

        int dest_index = 0;
        for(uint16_t i = 0; i < block_count; i++){
            dest_index =  i * block_width * structure.object_channels;
            memcpy(&data[dest_index], &object_sample_data[dest_index], block_width * structure.object_channels);
        }

        return data.data();

    } else if(structure.type == TWO_DIMENSIONAL) {
        if (structure.object_width % structure.block_cols != 0 || structure.object_height % structure.block_rows != 0) {
            throw std::runtime_error("Invalid image dimensions. Width must be divisible by block_cols and height by block_rows.");
        }


        int src_index = 0;
        for (int row = 0; row < structure.block_rows; ++row) {
            for (int col = 0; col < structure.block_cols; ++col) {
                int start_row = row * block_height;
                int start_col = col * block_width;
                for (int r = 0; r < block_height; ++r) {
                    int dest_index = ((start_row + r) * structure.object_width + start_col) * structure.object_channels;
                    memcpy(&data[dest_index], &object_sample_data[src_index], block_width * structure.object_channels);
                    src_index += block_width * structure.object_channels;
                }
            }
        }

        return data.data();


    } else {
        throw std::runtime_error("Invalid structure type.");
    }        

};

inline void translate_to_uchar(std::vector<unsigned char> object_sample_data, unsigned char* output_data, Structure structure){
    int block_count = structure.block_cols;
    size_t object_data_size = structure.object_width * structure.object_height * structure.object_channels;
    int block_height = structure.object_height / structure.block_rows;
    int block_width = structure.object_width / structure.block_cols;

    std::vector<unsigned char> data(object_data_size);
    
    if (structure.type == ONE_DIMENSIONAL){
        if (structure.object_width % structure.block_cols != 0 || structure.block_rows == 1) {
            throw std::runtime_error("Invalid image dimensions. Width must be divisible by block_cols and block_rows must be 1.");
        }

        for(uint16_t i = 0; i < block_count; i++){
            memcpy(&data[block_count], &object_sample_data[block_count], block_width * structure.object_channels);
        }

        memcpy(output_data, data.data(), object_data_size);

    } else if(structure.type == TWO_DIMENSIONAL) {
        if (structure.object_width % structure.block_cols != 0 || structure.object_height % structure.block_rows != 0) {
            throw std::runtime_error("Invalid image dimensions. Width must be divisible by block_cols and height by block_rows.");
        }


        int src_index = 0;
        for (int row = 0; row < structure.block_rows; ++row) {
            for (int col = 0; col < structure.block_cols; ++col) {
                int start_row = row * block_height;
                int start_col = col * block_width;
                for (int r = 0; r < block_height; ++r) {
                    int dest_index = ((start_row + r) * structure.object_width + start_col) * structure.object_channels;
                    memcpy(&data[dest_index], &object_sample_data[src_index], block_width * structure.object_channels);
                    src_index += block_width * structure.object_channels;
                }
            }
        }

        memcpy(output_data, data.data(), object_data_size);


    } else {
        throw std::runtime_error("Invalid structure type.");
    }        


};


#endif /*APPLICATION_DATA_OBJECT_MANAGEMENT_API__TRANSLATION_H_*/