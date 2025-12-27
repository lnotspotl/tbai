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

void testCppADCG_dynamics_jump_map_sparse_jacobian(double const *const * in,
                                                   double*const * out,
                                                   struct LangCAtomicFun atomicFun) {
   //independent variables
   const double* x = in[0];

   //dependent variables
   double* jac = out[0];

   // auxiliary variables

   // dependent variables without operations
   jac[0] = -0.686641821491831;
   jac[1] = 0.997849036007118;
   jac[2] = 0.225279665191322;
   jac[3] = -0.012834026018546;
   jac[4] = -0.198111211507633;
   jac[5] = -0.563486189378186;
   jac[6] = -0.407936764605314;
   jac[7] = 0.945550047767139;
   jac[8] = -0.740419106437089;
   jac[9] = 0.0258647888087968;
   jac[10] = 0.275104535406038;
   jac[11] = -0.414966431173946;
   jac[12] = -0.782382395948461;
   jac[13] = 0.678224469385214;
   jac[14] = 0.0485743801335685;
   jac[15] = 0.54271539558783;
}

