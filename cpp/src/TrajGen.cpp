#include <traj_gen2/TrajGen.hpp>



using namespace trajgen;

/////////////////////////////
// TrajGen as a base class //
/////////////////////////////

template <typename T,size_t dim> TrajGen<T,dim>::TrajGen(time_knots ts_): ts(ts_){
    // pinset per segment
    M = ts.size();
    this->fixPinSet = new vector<FixPin<T,dim>>[M];
    this->loosePinSet = new vector<FixPin<T,dim>>[M];
    fixPinOrderSet = new vector<d_order>[M];
}

/**
 * Add pin struct to corresponding segment
 * @tparam T
 * @tparam dim
 * @param pin
 */
template<typename T,size_t dim> void TrajGen<T,dim>::addPin(const Pin<T,dim> &pin){
    uint m;
    this->findSegInterval(pin.t,m); // which segment to add the pin
    if (pin.getType() == PIN_TYPE::FIX_PIN)
        fixPinSet[m].push_back(pin);
    else 
        loosePinSet[m].push_back(pin);

}

template<typename T,size_t dim> void TrajGen<T,dim>::addPinSet(const vector<Pin<T,dim>>& pinset){
    for (auto it = pinset.begin() ; it < pinset.end() ; it++)
        addPin(*it);     
}

template<typename T,size_t dim> void TrajGen<T,dim>::setDerivativeObj(VectorXf weights){
    weight_mask = weights; 

}

///////////////////////////////////////////
// PolyTrajGen for piecewise polynomials //
///////////////////////////////////////////

template<typename T,size_t dim> PolyTrajGen<T,dim>::PolyTrajGen(time_knots ts_,Param param_)  : TrajGen<T,dim>(ts_), param(param_),N(param_.poly_order){
    
    printf ("Initialized %d segments ",ts_.size()); M = ts_.size(); 
    
    seg_state_set = vector<SegState>(M);

    
    // polycoeff segment / dimension 
    poly_coeff_set = new PolyCoeff<T>*[dim];
    for (int dd = 0;dd <dim ; dd++) 
        poly_coeff_set[dd] = new PolyCoeff<T>[M];

}


template<typename T,size_t dim> void PolyTrajGen<T,dim>::findSegInterval(float t,uint& m){
    auto it = lower_bound(this->ts.begin(),this->ts.end(),t);
    m = it - this->ts.begin();
}
/**
 *
 * @tparam T
 * @tparam dim
 * @param t : qeury time
 * @param m : the poly segment containing t. ts(m) <= t < ts(m+1)
 * @param tau : normalized time (0<= tau <=1)
 */
template <typename  T,size_t dim> void PolyTrajGen<T,dim>::findSegInterval(float t, uint &m, float &tau) {
    findSegInterval(t,m);
    tau = (t - this->ts[m]) / (this->ts[m+1] - this->ts[m]);
}

template<typename T,size_t dim> void PolyTrajGen<T,dim>::addPin(const Pin<T,dim>& pin){    
    
    TrajGen<T,dim>::addPin(pin);
    


}



