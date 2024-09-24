#ifndef SocketEndpoint_h
#define SocketEndpoint_h

#include <string>
#include <chrono>

/**
 * @brief Describes the parameters of an boost asio endpoint. Used for passing endpoints to functions.
 * 
 */
struct socket_endpoint
{
    std::string ip_addr;
    int port;

    socket_endpoint(){};
    socket_endpoint(    
        std::string ip, 
        int p
    ):
        ip_addr(ip),
        port(p)
    {};
};


#endif