cmake_minimum_required(VERSION 3.0.0)
project(adom VERSION 0.1.0)
set (CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "-g")

include(GNUInstallDirs)

### cmake macros
include(${PROJECT_SOURCE_DIR}/cmake/dev/set_sources.cmake)
include(${PROJECT_SOURCE_DIR}/cmake/dev/generate_msvc_libraries.cmake)

### Build directory structure
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

### search for local boost
find_package(Boost REQUIRED COMPONENTS log log_setup system filesystem)
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )



### Define library content
set(
    LIB_SRC_FILES
    ${PROJECT_SOURCE_DIR}/src/cpp/adom/protocol.cpp
    ${PROJECT_SOURCE_DIR}/src/cpp/adom/application_interface.cpp
    ${PROJECT_SOURCE_DIR}/src/cpp/adom/directory.cpp
    ${PROJECT_SOURCE_DIR}/src/cpp/adom/setup_config.cpp
    ${PROJECT_SOURCE_DIR}/src/cpp/adom/directory_config.cpp
    ${PROJECT_SOURCE_DIR}/src/cpp/adom/topic_config.cpp
    ${PROJECT_SOURCE_DIR}/src/cpp/adom/log.cpp
)

## Define library content
add_library(adom SHARED ${LIB_SRC_FILES})

target_include_directories(adom
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)


include_directories(${Boost_INCLUDE_DIRS})

target_link_libraries(adom PUBLIC ${Boost_LIBRARIES} ${OpenCV_LIBS})

# Enable logging via -DLOG during build process
if(DEFINED LOG)
    message("Logging activated")
    target_compile_definitions(adom PUBLIC LOG_ON)
endif()

# Set logging level via -DLOG_LEVEL
if(DEFINED LOG_LEVEL)
    message("Logging level set to ${LOG_LEVEL}")
    target_compile_definitions(adom PUBLIC LOG_LEVEL=${LOG_LEVEL})
endif()

# Enable file log via -DFILE during build process
if(DEFINED FILE)
    message("File logging activated")
    target_compile_definitions(adom PUBLIC FILE_ON)
endif()

# Enable console log via -DCONSOLE during build process
if(DEFINED CONSOLE)
    message("Console logging activated")
    target_compile_definitions(adom PUBLIC CONSOLE_ON)
endif()



target_compile_options(adom PUBLIC -g)

include(CMakePackageConfigHelpers)

### Add headers to system include dir 
set_public_headers_directory(${PROJECT_SOURCE_DIR}/include adom
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    COMPONENT headers
    INSTALL
)

### Install lib
install(
    TARGETS adom
    EXPORT adomTargets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

install(
    EXPORT adomTargets
    NAMESPACE adom::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/adom
    FILE adomTargets.cmake
)

### Install template implementation files
install(
    DIRECTORY ${PROJECT_SOURCE_DIR}/include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR} 
    FILES_MATCHING PATTERN "*.tpp"
)

### Add package info for cmake
configure_package_config_file( 
    "adomConfig.cmake.in" 
    "adomConfig.cmake"
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/adom
    PATH_VARS
        CMAKE_INSTALL_LIBDIR
)

write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/adomConfigVersion.cmake
    VERSION 0.1.0
    COMPATIBILITY SameMajorVersion
)

### Install Config and ConfigVersion files
install(
    FILES "${CMAKE_CURRENT_BINARY_DIR}/adomConfig.cmake"
            "${CMAKE_CURRENT_BINARY_DIR}/adomConfigVersion.cmake"
    DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/adom"
)

### CPP example build Targets

# Define the data and config path
set(DATA_PATH "${CMAKE_SOURCE_DIR}/examples/cpp/data")
set(CONFIG_PATH "${CMAKE_SOURCE_DIR}/examples/cpp/config")

# Basic Application Entity emulating a Publisher
add_executable(publisher_test_application_entity examples/cpp/publisher_test_application_entity.cpp)
target_link_libraries(publisher_test_application_entity adom boost_log ${OpenCV_LIBS})
target_compile_definitions(publisher_test_application_entity PRIVATE
    DATA_PATH="${DATA_PATH}"
    CONFIG_PATH="${CONFIG_PATH}"
)

# Basic Application Entity emulating a Subscriber
add_executable(subscriber_test_application_entity examples/cpp/subscriber_test_application_entity.cpp)
target_link_libraries(subscriber_test_application_entity adom boost_log ${OpenCV_LIBS})
target_compile_definitions(subscriber_test_application_entity PRIVATE
    DATA_PATH="${DATA_PATH}"
    CONFIG_PATH="${CONFIG_PATH}"
)
