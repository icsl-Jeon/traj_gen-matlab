#include <Eigen/Core>
using namespace Eigen;

namespace trajgen
{
    template <typename Type, size_t Size> using Vector = Eigen::Matrix<Type, Size, 1>;
    
    /////////////////////////////////////////////////
    // PIN for equality or inequality constriants  //
    /////////////////////////////////////////////////
    
    enum PIN_TYPE{
        FIX_PIN = 0, // equality constraint  
        LOOSE_PIN = 1 // inequality constraint
    };

    template <typename T,size_t n>
    struct Pin{                
        float t; // imposed time 
        unsigned int d; // imposed order of derivative  
        Pin(float t_,unsigned int d_) : t(t_),d(d_){};      
        virtual PIN_TYPE getType() = 0;
    };
        
    template <typename T,size_t n>    
    struct FixPin : public Pin<T,n>{
        Vector<T,n> x; // waypoint 
        FixPin<T,n>(float t_,unsigned int d_,Vector<T,n> x_):Pin<T,n>(t_,d_),x(x_) {};
        PIN_TYPE getType() {return PIN_TYPE::FIX_PIN;}
    }; 

    template <typename T,size_t n>    
    struct LoosePin : public Pin<T,n>{
        Vector<T,n> xl; // lower bound
        Vector<T,n> xu; // upper bound
        LoosePin<T,n>(float t_,unsigned int d_,Vector<T,n> xl_,Vector<T,n> xu_):Pin<T,n>(t_,d_),xl(xl_),xu(xu_) {};
        PIN_TYPE getType() {return PIN_TYPE::LOOSE_PIN;}    
    };

    /////////////////////////////
    // TrajGen as a base class //
    /////////////////////////////
    


} // namespace trajgen
