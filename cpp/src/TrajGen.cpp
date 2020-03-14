#include <traj_gen2/TrajGen.hpp>

using namespace trajgen;


template<typename T,size_t dim> void TrajGen<T,dim>::appendPin(const Pin<T,dim> &pin){    
    assert ((pin.getType() == PIN_TYPE::FIX_PIN || pin.getType() == PIN_TYPE::LOOSE_PIN) && "Unknown pintype." )
    if (pin.getType() == PIN_TYPE::FIX_PIN)
        fixPinSet.push_back(pin);
    else 
        loosePinSet.push_back(pin);            
}

template<typename T,size_t dim> void TrajGen<T,dim>::addPinSet(const vector<Pin<T,dim>>& pinset){
    for (auto it = pinset.begin() ; it < pinset.end() ; it++)
        addPin(*it);    
}

template<typename T,size_t dim> void TrajGen<T,dim>::setDerivativeObj(VectorXf weights){
    weight_mask = weights;
}





