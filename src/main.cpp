#include <iostream>
#include "uvcacquisition.hpp"
#include "uvcvideoproducer.hpp"

int main(int argc, char *argv[])
{

    std::cout << "hello world." << std::endl;
    UvcAcquisition acq;
    UvcVideoProducer vp;
    vp.setUvc( & acq );

    std::cout << "goodbye world." << std::endl;
    return 1;
}

