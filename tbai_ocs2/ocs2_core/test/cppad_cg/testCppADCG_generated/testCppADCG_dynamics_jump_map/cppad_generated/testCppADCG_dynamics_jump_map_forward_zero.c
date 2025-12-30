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

void testCppADCG_dynamics_jump_map_forward_zero(double const *const * in,
                                                double*const * out,
                                                struct LangCAtomicFun atomicFun) {
   //independent variables
   const double* x = in[0];

   //dependent variables
   double* y = out[0];

   // auxiliary variables

   y[0] = 0.997849036007118 * x[2] + -0.686641821491831 * x[1] + 0.225279665191322 * x[3] + -0.012834026018546 * x[4];
   y[1] = -0.563486189378186 * x[2] + -0.198111211507633 * x[1] + -0.407936764605314 * x[3] + 0.945550047767139 * x[4];
   y[2] = 0.0258647888087968 * x[2] + -0.740419106437089 * x[1] + 0.275104535406038 * x[3] + -0.414966431173946 * x[4];
   y[3] = 0.678224469385214 * x[2] + -0.782382395948461 * x[1] + 0.0485743801335685 * x[3] + 0.54271539558783 * x[4];
}

