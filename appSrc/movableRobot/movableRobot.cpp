#include <iostream>

#include "robots/hexanodes/hexanodesSimulator.h"
#include "robots/hexanodes/hexanodesBlockCode.h"

#include "movableRobotBlockCode.hpp"

using namespace std;
using namespace Hexanodes;

int main(int argc, char **argv) {
    try
    {
        createSimulator(argc, argv, MovableRobotBlockCode::buildNewBlockCode);
        getSimulator()->printInfo();
        BaseSimulator::getWorld()->printInfo();
        deleteSimulator();
    }
    catch(std::exception const& e)
    {
        cerr << "Uncaught exception: " << e.what();
    }

    return 0;
}
