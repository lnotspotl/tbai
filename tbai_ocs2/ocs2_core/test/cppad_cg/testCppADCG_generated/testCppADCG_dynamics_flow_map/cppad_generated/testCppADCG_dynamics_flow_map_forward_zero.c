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

void testCppADCG_dynamics_flow_map_forward_zero(double const *const * in,
                                                double*const * out,
                                                struct LangCAtomicFun atomicFun) {
   //independent variables
   const double* x = in[0];

   //dependent variables
   double* y = out[0];

   // auxiliary variables

   y[0] = 0.823294715873569 * x[2] + 0.680375434309419 * x[1] + -0.444450578393624 * x[3] + -0.270431054416313 * x[4] + 0.271423455919802 * x[5] + -0.967398856751341 * x[6];
   y[1] = -0.604897261413232 * x[2] + -0.211234146361814 * x[1] + 0.107939911590861 * x[3] + 0.026801820391231 * x[4] + 0.434593858865366 * x[5] + -0.514226458740526 * x[6];
   y[2] = -0.329554488570222 * x[2] + 0.566198447517212 * x[1] + -0.0452058962756795 * x[3] + 0.904459450349426 * x[4] + -0.716794889288393 * x[5] + -0.725536846427963 * x[6];
   y[3] = 0.536459189623808 * x[2] + 0.596880066952147 * x[1] + 0.257741849523849 * x[3] + 0.832390136007401 * x[4] + 0.213937752514117 * x[5] + 0.608353508453981 * x[6];
}

