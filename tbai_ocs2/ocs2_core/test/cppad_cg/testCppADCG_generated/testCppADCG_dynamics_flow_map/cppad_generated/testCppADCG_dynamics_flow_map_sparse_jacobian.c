#include <math.h>
#include <stdio.h>

typedef struct Array {
    void* data;
    unsigned long size;
    int sparse;
    const unsigned long* idx;
    unsigned long nnz;
} Array;

struct LangCAtomicFun {
    void* libModel;
    int (*forward)(void* libModel,
                   int atomicIndex,
                   int q,
                   int p,
                   const Array tx[],
                   Array* ty);
    int (*reverse)(void* libModel,
                   int atomicIndex,
                   int p,
                   const Array tx[],
                   Array* px,
                   const Array py[]);
};

void testCppADCG_dynamics_flow_map_sparse_jacobian(double const *const * in,
                                                   double*const * out,
                                                   struct LangCAtomicFun atomicFun) {
   //independent variables
   const double* x = in[0];

   //dependent variables
   double* jac = out[0];

   // auxiliary variables

   // dependent variables without operations
   jac[0] = 0.680375434309419;
   jac[1] = 0.823294715873569;
   jac[2] = -0.444450578393624;
   jac[3] = -0.270431054416313;
   jac[4] = 0.271423455919802;
   jac[5] = -0.967398856751341;
   jac[6] = -0.211234146361814;
   jac[7] = -0.604897261413232;
   jac[8] = 0.107939911590861;
   jac[9] = 0.026801820391231;
   jac[10] = 0.434593858865366;
   jac[11] = -0.514226458740526;
   jac[12] = 0.566198447517212;
   jac[13] = -0.329554488570222;
   jac[14] = -0.0452058962756795;
   jac[15] = 0.904459450349426;
   jac[16] = -0.716794889288393;
   jac[17] = -0.725536846427963;
   jac[18] = 0.596880066952147;
   jac[19] = 0.536459189623808;
   jac[20] = 0.257741849523849;
   jac[21] = 0.832390136007401;
   jac[22] = 0.213937752514117;
   jac[23] = 0.608353508453981;
}

