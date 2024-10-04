#include <iostream>
#include <fstream>
#include "read_shared_data.h"

int main(){
	SharedData sd;
	while (1){
		try{
			sd.read_shared_data();
			sd.print();
		}
		catch(SharedDataException &e){
			if (e.error_code() == 21){

			}
			else{std::cout << "[Error] " << e.what() << std::endl;}
		}
	}
	return 0;
}