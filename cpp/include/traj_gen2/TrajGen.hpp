#include <Eigen/Core>
#include <vector>
using namespace Eigen;
using namespace std;

namespace trajgen
{
    template <typename Type, size_t Size> using Vector = Eigen::Matrix<Type, Size, 1>;
    typedef unsigned int d_order; // derivative order 
    /////////////////////////////////////////////////
    // PIN for equality or inequality constriants  //
    /////////////////////////////////////////////////
    
    enum PIN_TYPE{
        FIX_PIN = 0, // equality constraint  
        LOOSE_PIN = 1 // inequality constraint
    };

    template <typename T,size_t dim>
    struct Pin{                
        float t; // imposed time 
        unsigned int d; // imposed order of derivative  
        Pin(float t_,d_order d_) : t(t_),d(d_){};      
        virtual PIN_TYPE getType() = 0;
    };
        
    template <typename T,size_t dim>    
    struct FixPin : public Pin<T,dim>{
        Vector<T,dim> x; // waypoint 
        FixPin<T,dim>(float t_,unsigned int d_,Vector<T,dim> x_):Pin<T,dim>(t_,d_),x(x_) {};
        PIN_TYPE getType() {return PIN_TYPE::FIX_PIN;}
    }; 

    template <typename T,size_t dim>    
    struct LoosePin : public Pin<T,dim>{
        Vector<T,dim> xl; // lower bound
        Vector<T,dim> xu; // upper bound
        LoosePin<T,dim>(float t_,unsigned int d_,Vector<T,dim> xl_,Vector<T,dim> xu_):Pin<T,dim>(t_,d_),xl(xl_),xu(xu_) {};
        PIN_TYPE getType() {return PIN_TYPE::LOOSE_PIN;}    
    };

    /////////////////////////////
    // TrajGen as a base class //
    /////////////////////////////
    template<typename T,size_t dim>
    class TrajGen{
        private:
            // vector<Pin<T,dim>> pinSet;
            bool isSolved = false;
            vector<float> ts;

            vector<FixPin<T,dim>> fixPinSet; 
            vector<LoosePin<T,dim>> loosePinSet;

            void appendPin(const Pin<T,dim>&); // this should be called when we add Pin.
            VectorXf weight_mask;

        public:
            virtual void addPin(const Pin<T,dim>& pin) = 0;
            void addPinSet(const vector<Pin<T,dim>>& pinSet);
            vritual bool solve() = 0;
            void setDerivativeObj(VectorXf weights);
            virtual Vector<T,dim> eval(float t_eval, d_order d) = 0 ;
    };


} // namespace trajgen
