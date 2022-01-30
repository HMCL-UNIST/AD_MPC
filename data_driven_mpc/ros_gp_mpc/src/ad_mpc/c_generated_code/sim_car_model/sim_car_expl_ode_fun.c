/* This file was automatically generated by CasADi.
   The CasADi copyright holders make no ownership claim of its contents. */
#ifdef __cplusplus
extern "C" {
#endif

/* How to prefix internal symbols */
#ifdef CASADI_CODEGEN_PREFIX
  #define CASADI_NAMESPACE_CONCAT(NS, ID) _CASADI_NAMESPACE_CONCAT(NS, ID)
  #define _CASADI_NAMESPACE_CONCAT(NS, ID) NS ## ID
  #define CASADI_PREFIX(ID) CASADI_NAMESPACE_CONCAT(CODEGEN_PREFIX, ID)
#else
  #define CASADI_PREFIX(ID) sim_car_expl_ode_fun_ ## ID
#endif

#include <math.h>

#ifndef casadi_real
#define casadi_real double
#endif

#ifndef casadi_int
#define casadi_int int
#endif

/* Add prefix to internal symbols */
#define casadi_c0 CASADI_PREFIX(c0)
#define casadi_c1 CASADI_PREFIX(c1)
#define casadi_clear CASADI_PREFIX(clear)
#define casadi_clear_casadi_int CASADI_PREFIX(clear_casadi_int)
#define casadi_de_boor CASADI_PREFIX(de_boor)
#define casadi_f0 CASADI_PREFIX(f0)
#define casadi_f1 CASADI_PREFIX(f1)
#define casadi_fill CASADI_PREFIX(fill)
#define casadi_fill_casadi_int CASADI_PREFIX(fill_casadi_int)
#define casadi_low CASADI_PREFIX(low)
#define casadi_nd_boor_eval CASADI_PREFIX(nd_boor_eval)
#define casadi_s0 CASADI_PREFIX(s0)
#define casadi_s1 CASADI_PREFIX(s1)
#define casadi_s2 CASADI_PREFIX(s2)
#define casadi_s3 CASADI_PREFIX(s3)
#define casadi_s4 CASADI_PREFIX(s4)
#define casadi_s5 CASADI_PREFIX(s5)
#define casadi_s6 CASADI_PREFIX(s6)

/* Symbol visibility in DLLs */
#ifndef CASADI_SYMBOL_EXPORT
  #if defined(_WIN32) || defined(__WIN32__) || defined(__CYGWIN__)
    #if defined(STATIC_LINKED)
      #define CASADI_SYMBOL_EXPORT
    #else
      #define CASADI_SYMBOL_EXPORT __declspec(dllexport)
    #endif
  #elif defined(__GNUC__) && defined(GCC_HASCLASSVISIBILITY)
    #define CASADI_SYMBOL_EXPORT __attribute__ ((visibility ("default")))
  #else
    #define CASADI_SYMBOL_EXPORT
  #endif
#endif

void casadi_de_boor(casadi_real x, const casadi_real* knots, casadi_int n_knots, casadi_int degree, casadi_real* boor) {
  casadi_int d, i;
  for (d=1;d<degree+1;++d) {
    for (i=0;i<n_knots-d-1;++i) {
      casadi_real b, bottom;
      b = 0;
      bottom = knots[i + d] - knots[i];
      if (bottom) b = (x - knots[i]) * boor[i] / bottom;
      bottom = knots[i + d + 1] - knots[i + 1];
      if (bottom) b += (knots[i + d + 1] - x) * boor[i + 1] / bottom;
      boor[i] = b;
    }
  }
}

void casadi_fill(casadi_real* x, casadi_int n, casadi_real alpha) {
  casadi_int i;
  if (x) {
    for (i=0; i<n; ++i) *x++ = alpha;
  }
}

void casadi_fill_casadi_int(casadi_int* x, casadi_int n, casadi_int alpha) {
  casadi_int i;
  if (x) {
    for (i=0; i<n; ++i) *x++ = alpha;
  }
}

void casadi_clear(casadi_real* x, casadi_int n) {
  casadi_int i;
  if (x) {
    for (i=0; i<n; ++i) *x++ = 0;
  }
}

void casadi_clear_casadi_int(casadi_int* x, casadi_int n) {
  casadi_int i;
  if (x) {
    for (i=0; i<n; ++i) *x++ = 0;
  }
}

casadi_int casadi_low(casadi_real x, const casadi_real* grid, casadi_int ng, casadi_int lookup_mode) {
  switch (lookup_mode) {
    case 1:
      {
        casadi_real g0, dg;
        casadi_int ret;
        g0 = grid[0];
        dg = grid[ng-1]-g0;
        ret = (casadi_int) ((x-g0)*(ng-1)/dg);
        if (ret<0) ret=0;
        if (ret>ng-2) ret=ng-2;
        return ret;
      }
    case 2:
      {
        casadi_int start, stop, pivot;
        if (ng<2 || x<grid[1]) return 0;
        if (x>grid[ng-1]) return ng-2;
        start = 0;
        stop  = ng-1;
        while (1) {
          pivot = (stop+start)/2;
          if (x < grid[pivot]) {
            if (pivot==stop) return pivot;
            stop = pivot;
          } else {
            if (pivot==start) return pivot;
            start = pivot;
          }
        }
      }
    default:
      {
        casadi_int i;
        for (i=0; i<ng-2; ++i) {
          if (x < grid[i+1]) break;
        }
        return i;
      }
  }
}

void casadi_nd_boor_eval(casadi_real* ret, casadi_int n_dims, const casadi_real* all_knots, const casadi_int* offset, const casadi_int* all_degree, const casadi_int* strides, const casadi_real* c, casadi_int m, const casadi_real* all_x, const casadi_int* lookup_mode, casadi_int* iw, casadi_real* w) {
  casadi_int n_iter, k, i, pivot;
  casadi_int *boor_offset, *starts, *index, *coeff_offset;
  casadi_real *cumprod, *all_boor;
  boor_offset = iw; iw+=n_dims+1;
  starts = iw; iw+=n_dims;
  index = iw; iw+=n_dims;
  coeff_offset = iw;
  cumprod = w; w+= n_dims+1;
  all_boor = w;
  boor_offset[0] = 0;
  cumprod[n_dims] = 1;
  coeff_offset[n_dims] = 0;
  n_iter = 1;
  for (k=0;k<n_dims;++k) {
    casadi_real *boor;
    const casadi_real* knots;
    casadi_real x;
    casadi_int degree, n_knots, n_b, L, start;
    boor = all_boor+boor_offset[k];
    degree = all_degree[k];
    knots = all_knots + offset[k];
    n_knots = offset[k+1]-offset[k];
    n_b = n_knots-degree-1;
    x = all_x[k];
    L = casadi_low(x, knots+degree, n_knots-2*degree, lookup_mode[k]);
    start = L;
    if (start>n_b-degree-1) start = n_b-degree-1;
    starts[k] = start;
    casadi_clear(boor, 2*degree+1);
    if (x>=knots[0] && x<=knots[n_knots-1]) {
      if (x==knots[1]) {
        casadi_fill(boor, degree+1, 1.0);
      } else if (x==knots[n_knots-1]) {
        boor[degree] = 1;
      } else if (knots[L+degree]==x) {
        boor[degree-1] = 1;
      } else {
        boor[degree] = 1;
      }
    }
    casadi_de_boor(x, knots+start, 2*degree+2, degree, boor);
    boor+= degree+1;
    n_iter*= degree+1;
    boor_offset[k+1] = boor_offset[k] + degree+1;
  }
  casadi_clear_casadi_int(index, n_dims);
  for (pivot=n_dims-1;pivot>=0;--pivot) {
    cumprod[pivot] = (*(all_boor+boor_offset[pivot]))*cumprod[pivot+1];
    coeff_offset[pivot] = starts[pivot]*strides[pivot]+coeff_offset[pivot+1];
  }
  for (k=0;k<n_iter;++k) {
    casadi_int pivot = 0;
    for (i=0;i<m;++i) ret[i] += c[coeff_offset[0]+i]*cumprod[0];
    index[0]++;
    {
      while (index[pivot]==boor_offset[pivot+1]-boor_offset[pivot]) {
        index[pivot] = 0;
        if (pivot==n_dims-1) break;
        index[++pivot]++;
      }
      while (pivot>0) {
        cumprod[pivot] = (*(all_boor+boor_offset[pivot]+index[pivot]))*cumprod[pivot+1];
        coeff_offset[pivot] = (starts[pivot]+index[pivot])*strides[pivot]+coeff_offset[pivot+1];
        pivot--;
      }
    }
    cumprod[0] = (*(all_boor+index[0]))*cumprod[1];
    coeff_offset[0] = (starts[0]+index[0])*m+coeff_offset[1];
  }
}

static const casadi_int casadi_s0[1] = {0};
static const casadi_int casadi_s1[1] = {1};
static const casadi_int casadi_s2[1] = {3};
static const casadi_int casadi_s3[2] = {0, 9};
static const casadi_int casadi_s4[8] = {4, 1, 0, 4, 0, 1, 2, 3};
static const casadi_int casadi_s5[6] = {2, 1, 0, 2, 0, 1};
static const casadi_int casadi_s6[3] = {0, 0, 0};

static const casadi_real casadi_c0[5] = {1.0000000000000019e-06, 9.9999999999999995e-07, 1.0000000000000002e-06, 1.0000000000000002e-06, 1.0000000000000019e-06};
static const casadi_real casadi_c1[9] = {0., 0., 0., 0., 2., 4., 4., 4., 4.};

/* kapparef_s:(x)->(f) */
static int casadi_f1(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real w0, w1;
  /* #0: @0 = input[0][0] */
  w0 = arg[0] ? arg[0][0] : 0;
  /* #1: @1 = BSpline(@0) */
  casadi_clear((&w1), 1);
  CASADI_PREFIX(nd_boor_eval)((&w1),1,casadi_c1,casadi_s3,casadi_s2,casadi_s1,casadi_c0,1,(&w0),casadi_s0, iw, w);
  /* #2: output[0][0] = @1 */
  if (res[0]) res[0][0] = w1;
  return 0;
}

/* sim_car_expl_ode_fun:(i0[4],i1[2],i2[])->(o0[4]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real **res1=res+1;
  const casadi_real **arg1=arg+3;
  casadi_real w0, w1, w2, w3, w4, w5, w6, w7;
  /* #0: @0 = input[0][3] */
  w0 = arg[0] ? arg[0][3] : 0;
  /* #1: @1 = input[0][2] */
  w1 = arg[0] ? arg[0][2] : 0;
  /* #2: @2 = 0.5 */
  w2 = 5.0000000000000000e-01;
  /* #3: @3 = input[1][1] */
  w3 = arg[1] ? arg[1][1] : 0;
  /* #4: @4 = tan(@3) */
  w4 = tan( w3 );
  /* #5: @2 = (@2*@4) */
  w2 *= w4;
  /* #6: @2 = atan(@2) */
  w2 = atan( w2 );
  /* #7: @2 = (@1+@2) */
  w2  = (w1+w2);
  /* #8: @2 = cos(@2) */
  w2 = cos( w2 );
  /* #9: @2 = (@0*@2) */
  w2  = (w0*w2);
  /* #10: @4 = 1 */
  w4 = 1.;
  /* #11: @5 = input[0][1] */
  w5 = arg[0] ? arg[0][1] : 0;
  /* #12: @6 = input[0][0] */
  w6 = arg[0] ? arg[0][0] : 0;
  /* #13: @7 = kapparef_s(@6) */
  arg1[0]=(&w6);
  res1[0]=(&w7);
  if (casadi_f1(arg1, res1, iw, w, 0)) return 1;
  /* #14: @7 = (@5*@7) */
  w7  = (w5*w7);
  /* #15: @4 = (@4-@7) */
  w4 -= w7;
  /* #16: @2 = (@2/@4) */
  w2 /= w4;
  /* #17: output[0][0] = @2 */
  if (res[0]) res[0][0] = w2;
  /* #18: @2 = 0.5 */
  w2 = 5.0000000000000000e-01;
  /* #19: @4 = tan(@3) */
  w4 = tan( w3 );
  /* #20: @2 = (@2*@4) */
  w2 *= w4;
  /* #21: @2 = atan(@2) */
  w2 = atan( w2 );
  /* #22: @2 = (@1+@2) */
  w2  = (w1+w2);
  /* #23: @2 = sin(@2) */
  w2 = sin( w2 );
  /* #24: @2 = (@0*@2) */
  w2  = (w0*w2);
  /* #25: output[0][1] = @2 */
  if (res[0]) res[0][1] = w2;
  /* #26: @2 = 1.4 */
  w2 = 1.3999999999999999e+00;
  /* #27: @2 = (@0/@2) */
  w2  = (w0/w2);
  /* #28: @4 = 0.5 */
  w4 = 5.0000000000000000e-01;
  /* #29: @3 = tan(@3) */
  w3 = tan( w3 );
  /* #30: @4 = (@4*@3) */
  w4 *= w3;
  /* #31: @4 = atan(@4) */
  w4 = atan( w4 );
  /* #32: @3 = sin(@4) */
  w3 = sin( w4 );
  /* #33: @2 = (@2*@3) */
  w2 *= w3;
  /* #34: @1 = (@1+@4) */
  w1 += w4;
  /* #35: @1 = cos(@1) */
  w1 = cos( w1 );
  /* #36: @0 = (@0*@1) */
  w0 *= w1;
  /* #37: @1 = 1 */
  w1 = 1.;
  /* #38: @4 = kapparef_s(@6) */
  arg1[0]=(&w6);
  res1[0]=(&w4);
  if (casadi_f1(arg1, res1, iw, w, 0)) return 1;
  /* #39: @5 = (@5*@4) */
  w5 *= w4;
  /* #40: @1 = (@1-@5) */
  w1 -= w5;
  /* #41: @0 = (@0/@1) */
  w0 /= w1;
  /* #42: @1 = kapparef_s(@6) */
  arg1[0]=(&w6);
  res1[0]=(&w1);
  if (casadi_f1(arg1, res1, iw, w, 0)) return 1;
  /* #43: @0 = (@0*@1) */
  w0 *= w1;
  /* #44: @2 = (@2-@0) */
  w2 -= w0;
  /* #45: output[0][2] = @2 */
  if (res[0]) res[0][2] = w2;
  /* #46: @2 = input[1][0] */
  w2 = arg[1] ? arg[1][0] : 0;
  /* #47: output[0][3] = @2 */
  if (res[0]) res[0][3] = w2;
  return 0;
}

CASADI_SYMBOL_EXPORT int sim_car_expl_ode_fun(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int sim_car_expl_ode_fun_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int sim_car_expl_ode_fun_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void sim_car_expl_ode_fun_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int sim_car_expl_ode_fun_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void sim_car_expl_ode_fun_release(int mem) {
}

CASADI_SYMBOL_EXPORT void sim_car_expl_ode_fun_incref(void) {
}

CASADI_SYMBOL_EXPORT void sim_car_expl_ode_fun_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int sim_car_expl_ode_fun_n_in(void) { return 3;}

CASADI_SYMBOL_EXPORT casadi_int sim_car_expl_ode_fun_n_out(void) { return 1;}

CASADI_SYMBOL_EXPORT casadi_real sim_car_expl_ode_fun_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* sim_car_expl_ode_fun_name_in(casadi_int i){
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* sim_car_expl_ode_fun_name_out(casadi_int i){
  switch (i) {
    case 0: return "o0";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* sim_car_expl_ode_fun_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s4;
    case 1: return casadi_s5;
    case 2: return casadi_s6;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* sim_car_expl_ode_fun_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s4;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int sim_car_expl_ode_fun_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 6;
  if (sz_res) *sz_res = 4;
  if (sz_iw) *sz_iw = 8;
  if (sz_w) *sz_w = 20;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
