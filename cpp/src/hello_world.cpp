#include "hello_world.h"

HelloWorld::HelloWorld(): _hello("Hello World!"){}

void HelloWorld::print(){
    std::cout << _hello << std::endl;
}

