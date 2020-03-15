#include <traj_gen2/TrajGen.hpp>
#include <iostream>
using namespace trajgen;

int main(){
    const int dim = 2;
    FixPin<float,dim> pin(0.0f,1,Vector2f(0,0));
    std::cout << pin.getType() << std::endl;
    

    return 0; 

}