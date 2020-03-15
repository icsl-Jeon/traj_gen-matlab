#include <Eigen/Core>
#include <vector>
using namespace Eigen;
using namespace std;

namespace trajgen
{
    typedef unsigned int uint;
    template <typename Type, size_t Size> using Vector = Eigen::Matrix<Type, Size, 1>; 
    typedef uint d_order; // derivative order 
    typedef uint p_order; // polynomial order 
    typedef vector<float> time_knots;     
    template <typename Type> using PolyCoeff = Eigen::Matrix<Type, -1, 1>;  

    /////////////////////////////////////////////////
    // PIN for equality or inequality constriants  //
    /////////////////////////////////////////////////
    
    // *NOTE* 
    // We use vector container for unknown size data stack 
    // while array type is used for a container whose size can be determined at constructor 


    enum PIN_TYPE{
        FIX_PIN = 0, // equality constraint
        LOOSE_PIN = 1 // inequality constraint
    };

    template <typename T,size_t dim>
    struct Pin{                
        float t; // imposed time 
        uint d; // imposed order of derivative          
        Pin(float t_,d_order d_) : t(t_),d(d_){};      
        virtual PIN_TYPE getType() = 0;
    };
        
    template <typename T,size_t dim>    
    struct FixPin : public Pin<T,dim>{
        Vector<T,dim> x; // waypoint 
        FixPin<T,dim>(float t_,uint d_,Vector<T,dim> x_):Pin<T,dim>(t_,d_),x(x_) {};
        PIN_TYPE getType() {return PIN_TYPE::FIX_PIN;}
    }; 

    template <typename T,size_t dim>    
    struct LoosePin : public Pin<T,dim>{
        Vector<T,dim> xl; // lower bound
        Vector<T,dim> xu; // upper bound
        LoosePin<T,dim>(float t_,uint d_,Vector<T,dim> xl_,Vector<T,dim> xu_):Pin<T,dim>(t_,d_),xl(xl_),xu(xu_) {};
        PIN_TYPE getType() {return PIN_TYPE::LOOSE_PIN;}    
    };

    /////////////////////////////
    // TrajGen as a base class //
    /////////////////////////////
    template<typename T,size_t dim>
    class TrajGen{
        
        protected:
            uint M; // number of segment
        // vector<Pin<T,dim>> pinSet;
            bool isSolved = false; // solve flag
            time_knots ts; // knots

            vector<FixPin<T,dim>>* fixPinSet;  // equality constraints set of M segment. (In case of optimTrajGen, M = 1)
            vector<LoosePin<T,dim>>* loosePinSet; // inequality constraints set of ()
            vector<d_order>* fixPinOrderSet;
            VectorXf weight_mask; // high order derivative penalty weights 

            // Subroutine functions
            void findSegInterval(float t,uint& m);
            void findSegInterval(float t,uint& m,float& tau);


        public:
            TrajGen(time_knots ts_);
            void addPinSet(const vector<Pin<T,dim>>& pinSet);
            void setDerivativeObj(VectorXf weigts);
            virtual Vector<T,dim> eval(float t_eval, d_order d) = 0;
            virtual void addPin(const Pin<T,dim>& pin);            
            virtual bool solve() = 0;
    };

    ///////////////////////////////////////////
    // PolyTrajGen for piecewise polynomials //
    ///////////////////////////////////////////

    enum ALGORITHM{
        POLY_COEFF = 0,
        END_DERIVATIVE = 1
    };         

    // Parameters to define the behavior of polynomial
    struct Param{
        p_order poly_order = 4;
        d_order max_conti_order = 2;
        ALGORITHM algo = ALGORITHM::POLY_COEFF;
        Param() { printf("No default parameters given. Default values are used\n");};     
        Param(p_order pp,d_order dd,ALGORITHM algo): poly_order(pp),max_conti_order(dd),algo(algo) {};
    };
    
    // State of the polynomial segment. 
    struct SegState{
        uint Nc = 0; // order of continuity             
        uint Nf = 0; // number of fixed pins  
    };

    template<typename T,size_t dim>
    class PolyTrajGen : public TrajGen<T,dim>{
        
        private:
            Param param;
            vector<SegState> seg_state_set;
            PolyCoeff<T>** poly_coeff_set; // [dim][M] 
            p_order N; // poly order


            
        public:
            PolyTrajGen(time_knots ts_,Param param_) ;
            void addPin(const Pin<T,dim>& pin);
            Vector<T,dim> eval(float t_eval, d_order d) ;
            bool solve();        

    };








} // namespace trajgen
