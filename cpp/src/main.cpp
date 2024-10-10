#include <iostream>
#include <cstring>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fstream>
#include "read_shared_data.h"
#include "wave_generator.h"

int main(){
	SharedData sd;
	sd.read_data();
}

// int main(){

//     int server_socket, client_socket;
// 	struct sockaddr_in server, client;
//     socklen_t client_len = sizeof(client);
//     char buffer[1024];  // Buffer to hold received data

//     // Create socket
//     server_socket = socket(AF_INET, SOCK_STREAM, 0);
//     if (server_socket == -1) {
//         std::cerr << "Could not create socket" << std::endl;
//         return 1;
//     }

// 	server.sin_family = AF_INET;
//     server.sin_addr.s_addr = INADDR_ANY;  // Listen on all interfaces
//     server.sin_port = htons(62908);
// 	// Bind the socket
//     if (bind(server_socket, (struct sockaddr *)&server, sizeof(server)) < 0) {
//         std::cerr << "Bind failed" << std::endl;
//         return 1;
//     }
// 	 // Listen for incoming connections
//     listen(server_socket, 1);
//     std::cout << "Waiting for a connection..." << std::endl;
// 	// Accept a connection
//     client_socket = accept(server_socket, (struct sockaddr *)&client, &client_len);
//     if (client_socket < 0) {
//         std::cerr << "Accept failed" << std::endl;
//         return 1;
//     }
//     std::cout << "Connection accepted." << std::endl;

//     // Receive data
//     while (true) {
//         ssize_t bytes_received = recv(client_socket, buffer, sizeof(buffer) - 1, 0);
//         if (bytes_received <= 0) {
//             std::cout << "Connection closed or error occurred." << std::endl;
//             break;  // Break the loop on error or client disconnect
//         }
//         buffer[bytes_received] = '\0';  // Null-terminate the received string
//         std::cout << "Received: " << buffer << std::endl;  // Print received data
//     }

//     // Clean up
//     close(client_socket);
//     close(server_socket);
//     return 0;
// }



// int main(){
// 	SharedData sd;
// 	std::vector<std::vector<uint16_t>> sequence1;
// 	try{
// 		sequence1 = sd.read_shared_data();
// 		sd.print();
// 	}
// 	catch(SharedDataException &e){
// 		if (e.error_code() == 21){
// 			//Do nothing this means the file is empty
// 		}
// 		else{std::cout << "[Error] " << e.what() << std::endl;}
// 	}
// 	WaveGenerator sounds(sequence1);
// 	sounds.play_sounds();
// 	std::string filename = "../dtmf_sounds/dtmf_sounds.wav";
// 	sounds.save_to_wav_file(filename);
// 	//lyde ended

// 	return 0;
// }
