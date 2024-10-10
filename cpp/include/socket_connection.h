#ifndef SOCKET_CONNECTION_H
#define SOCKET_CONNECTION_H

#include <iostream>
#include <cstring>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>

struct ReadResult {
    char* data;
    ssize_t length;
};

class SocketConnection{

    public: 
        SocketConnection();
        ~SocketConnection();
        ReadResult read();
        int connect(int port);
        void disconnect();

    private: 
        int _server_socket, _client_socket;
        struct sockaddr_in _server, _client;
        char _buffer[1024];
};
#endif