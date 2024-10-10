#include "socket_connection.h"

SocketConnection::SocketConnection(){}

int SocketConnection::connect(int port){
    socklen_t client_len = sizeof(_client);
     // Create socket
    _server_socket = socket(AF_INET, SOCK_STREAM, 0);
    if (_server_socket == -1) {
        std::cerr << "Could not create socket" << std::endl;
        return 1;
    }

	_server.sin_family = AF_INET;
    _server.sin_addr.s_addr = INADDR_ANY;  // Listen on all interfaces
    _server.sin_port = htons(62908);
	// Bind the socket
    if (bind(_server_socket, (struct sockaddr *)&_server, sizeof(_server)) < 0) {
        std::cerr << "Bind failed" << std::endl;
        return 1;
    }
	 // Listen for incoming connections
    listen(_server_socket, 1);
    std::cout << "Waiting for a connection..." << std::endl;
	// Accept a connection
    _client_socket = accept(_server_socket, (struct sockaddr *)&_client, &client_len);
    if (_client_socket < 0) {
        std::cerr << "Accept failed" << std::endl;
        return 1;
    }
    std::cout << "Connection accepted." << std::endl;
    return 0;
}

ReadResult SocketConnection::read() {
    ssize_t bytes_received = recv(_client_socket, _buffer, sizeof(_buffer) - 1, 0);
    if (bytes_received <= 0) {
        std::cerr << "Connection closed or error occurred." << std::endl;
        return { nullptr, 0 };  // Return nullptr if there's an error
    }

    _buffer[bytes_received] = '\0';  // Null-terminate the received string
    std::cout << "Received: " << _buffer << std::endl;  // Print received data

    return { _buffer, bytes_received };  // Return both the buffer and length
}


SocketConnection::~SocketConnection(){
    disconnect();
}

void SocketConnection::disconnect() {
    if (_client_socket >= 0) {
        close(_client_socket);
        _client_socket = -1; // Optional: set to -1 to indicate it's closed
    }
    if (_server_socket >= 0) {
        close(_server_socket);
        _server_socket = -1; // Optional: set to -1 to indicate it's closed
    }
}
