#include "hello_world.h"

HelloWorld::HelloWorld(): _hello("Hello World!"){}

void HelloWorld::print(){
    if(_hello.empty()){
        throw HelloWorldError("No message");
    }
    std::cout << _hello << std::endl;
}

