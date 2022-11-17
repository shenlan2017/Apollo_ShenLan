#include <iostream>
#include "example.hpp"

Example::Example(int input){
    std::cout << "Init Example Class." << std::endl;
}

void Example::Printer(){
    std::cout << "Execute Print Function." << std::endl;
}
