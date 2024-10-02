#include <iostream>
#include <fstream>
#include "read_shared_data.h"

int main(){
	SharedData sd;
	sd.read_shared_data();
	sd.print();
	return 0;
}