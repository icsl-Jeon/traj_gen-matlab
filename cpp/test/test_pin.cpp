#include <traj_gen2/TrajGen.hpp>
#include <iostream>
using namespace trajgen;
int main(){
    const int dim = 1;
    typedef Vector<1> Vector1f ;

    time_knots ts{0,2,3};
    PolyParam pp(7,4,ALGORITHM::POLY_COEFF);
    Vector4f objWeights(10,1,1,10);

    FixPin<dim> pin1(0.0f,0,Vector1f(0));
    FixPin<dim> pin2(0.0f,2,Vector1f(1));
    FixPin<dim> pin3(0.0f,1,Vector1f(2));
    LoosePin<dim> pin4(0.3f,0,Vector1f(0.7),Vector1f(0.8));
    LoosePin<dim> pin5(2.0f,2,Vector1f(0.7),Vector1f(0.8));
    LoosePin<dim> pin6(2.4f,1,Vector1f(0.0),Vector1f(1.9));
    FixPin<dim> pin7(3.0f,0,Vector1f(1));

    std::vector<Pin<dim>*> pinSet{&pin1,&pin2,&pin3,&pin4,&pin5,&pin6,&pin7}; // to prevent downcasting slicing, vector of pointers

    PolyTrajGen<dim> pTraj(ts,pp);
    pTraj.setDerivativeObj(objWeights);
    pTraj.addPinSet(pinSet);



    return 0; 

}