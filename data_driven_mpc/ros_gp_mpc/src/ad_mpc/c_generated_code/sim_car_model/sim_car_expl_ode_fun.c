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
#define casadi_f0 CASADI_PREFIX(f0)
#define casadi_s0 CASADI_PREFIX(s0)
#define casadi_s1 CASADI_PREFIX(s1)
#define casadi_s2 CASADI_PREFIX(s2)

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

static const casadi_int casadi_s0[11] = {7, 1, 0, 7, 0, 1, 2, 3, 4, 5, 6};
static const casadi_int casadi_s1[6] = {2, 1, 0, 2, 0, 1};
static const casadi_int casadi_s2[5] = {1, 1, 0, 1, 0};

/* sim_car_expl_ode_fun:(i0[7],i1[2],i2)->(o0[7]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real w0, w1, w2, w3, w4, w5, w6, w7, w8, w9, w10, w11;
  /* #0: @0 = input[0][3] */
  w0 = arg[0] ? arg[0][3] : 0;
  /* #1: @1 = input[0][2] */
  w1 = arg[0] ? arg[0][2] : 0;
  /* #2: @2 = cos(@1) */
  w2 = cos( w1 );
  /* #3: @2 = (@0*@2) */
  w2  = (w0*w2);
  /* #4: @3 = input[0][4] */
  w3 = arg[0] ? arg[0][4] : 0;
  /* #5: @4 = sin(@1) */
  w4 = sin( w1 );
  /* #6: @4 = (@3*@4) */
  w4  = (w3*w4);
  /* #7: @2 = (@2-@4) */
  w2 -= w4;
  /* #8: output[0][0] = @2 */
  if (res[0]) res[0][0] = w2;
  /* #9: @2 = sin(@1) */
  w2 = sin( w1 );
  /* #10: @2 = (@0*@2) */
  w2  = (w0*w2);
  /* #11: @1 = cos(@1) */
  w1 = cos( w1 );
  /* #12: @1 = (@3*@1) */
  w1  = (w3*w1);
  /* #13: @2 = (@2+@1) */
  w2 += w1;
  /* #14: output[0][1] = @2 */
  if (res[0]) res[0][1] = w2;
  /* #15: @2 = input[0][5] */
  w2 = arg[0] ? arg[0][5] : 0;
  /* #16: output[0][2] = @2 */
  if (res[0]) res[0][2] = w2;
  /* #17: @1 = input[2][0] */
  w1 = arg[2] ? arg[2][0] : 0;
  /* #18: @4 = input[1][0] */
  w4 = arg[1] ? arg[1][0] : 0;
  /* #19: @5 = 0.000666667 */
  w5 = 6.6666666666666664e-04;
  /* #20: @6 = 83458.1 */
  w6 = 8.3458139053772335e+04;
  /* #21: @7 = input[0][6] */
  w7 = arg[0] ? arg[0][6] : 0;
  /* #22: @8 = 1.08 */
  w8 = 1.0800000000000001e+00;
  /* #23: @8 = (@8*@2) */
  w8 *= w2;
  /* #24: @8 = (@3+@8) */
  w8  = (w3+w8);
  /* #25: @9 = 1e-99 */
  w9 = 1.0000000000000000e-99;
  /* #26: @9 = (@9+@0) */
  w9 += w0;
  /* #27: @8 = (@8/@9) */
  w8 /= w9;
  /* #28: @8 = (@7-@8) */
  w8  = (w7-w8);
  /* #29: @6 = (@6*@8) */
  w6 *= w8;
  /* #30: @5 = (@5*@6) */
  w5 *= w6;
  /* #31: @6 = sin(@7) */
  w6 = sin( w7 );
  /* #32: @5 = (@5*@6) */
  w5 *= w6;
  /* #33: @5 = (@4-@5) */
  w5  = (w4-w5);
  /* #34: @6 = (@3*@2) */
  w6  = (w3*w2);
  /* #35: @5 = (@5+@6) */
  w5 += w6;
  /* #36: @5 = (@1*@5) */
  w5  = (w1*w5);
  /* #37: @6 = 1 */
  w6 = 1.;
  /* #38: @6 = (@6-@1) */
  w6 -= w1;
  /* #39: @6 = (@6*@4) */
  w6 *= w4;
  /* #40: @5 = (@5+@6) */
  w5 += w6;
  /* #41: output[0][3] = @5 */
  if (res[0]) res[0][3] = w5;
  /* #42: @5 = 0.000666667 */
  w5 = 6.6666666666666664e-04;
  /* #43: @6 = 55638.8 */
  w6 = 5.5638759369181564e+04;
  /* #44: @8 = 1.62 */
  w8 = 1.6200000000000001e+00;
  /* #45: @8 = (@8*@2) */
  w8 *= w2;
  /* #46: @8 = (@8-@3) */
  w8 -= w3;
  /* #47: @6 = (@6*@8) */
  w6 *= w8;
  /* #48: @8 = 1e-99 */
  w8 = 1.0000000000000000e-99;
  /* #49: @8 = (@8+@0) */
  w8 += w0;
  /* #50: @6 = (@6/@8) */
  w6 /= w8;
  /* #51: @8 = 83458.1 */
  w8 = 8.3458139053772335e+04;
  /* #52: @9 = 1.08 */
  w9 = 1.0800000000000001e+00;
  /* #53: @9 = (@9*@2) */
  w9 *= w2;
  /* #54: @9 = (@3+@9) */
  w9  = (w3+w9);
  /* #55: @10 = 1e-99 */
  w10 = 1.0000000000000000e-99;
  /* #56: @10 = (@10+@0) */
  w10 += w0;
  /* #57: @9 = (@9/@10) */
  w9 /= w10;
  /* #58: @9 = (@7-@9) */
  w9  = (w7-w9);
  /* #59: @8 = (@8*@9) */
  w8 *= w9;
  /* #60: @9 = cos(@7) */
  w9 = cos( w7 );
  /* #61: @8 = (@8*@9) */
  w8 *= w9;
  /* #62: @6 = (@6+@8) */
  w6 += w8;
  /* #63: @5 = (@5*@6) */
  w5 *= w6;
  /* #64: @6 = (@0*@2) */
  w6  = (w0*w2);
  /* #65: @5 = (@5-@6) */
  w5 -= w6;
  /* #66: @5 = (@1*@5) */
  w5  = (w1*w5);
  /* #67: @6 = 1 */
  w6 = 1.;
  /* #68: @6 = (@6-@1) */
  w6 -= w1;
  /* #69: @8 = 1.62 */
  w8 = 1.6200000000000001e+00;
  /* #70: @9 = input[1][1] */
  w9 = arg[1] ? arg[1][1] : 0;
  /* #71: @10 = (@9*@0) */
  w10  = (w9*w0);
  /* #72: @11 = (@7*@4) */
  w11  = (w7*w4);
  /* #73: @10 = (@10+@11) */
  w10 += w11;
  /* #74: @8 = (@8*@10) */
  w8 *= w10;
  /* #75: @10 = 2.7 */
  w10 = 2.7000000000000002e+00;
  /* #76: @8 = (@8/@10) */
  w8 /= w10;
  /* #77: @6 = (@6*@8) */
  w6 *= w8;
  /* #78: @5 = (@5+@6) */
  w5 += w6;
  /* #79: output[0][4] = @5 */
  if (res[0]) res[0][4] = w5;
  /* #80: @5 = 0.000381039 */
  w5 = 3.8103947568968139e-04;
  /* #81: @6 = 1.08 */
  w6 = 1.0800000000000001e+00;
  /* #82: @8 = 83458.1 */
  w8 = 8.3458139053772335e+04;
  /* #83: @10 = 1.08 */
  w10 = 1.0800000000000001e+00;
  /* #84: @10 = (@10*@2) */
  w10 *= w2;
  /* #85: @10 = (@3+@10) */
  w10  = (w3+w10);
  /* #86: @11 = 1e-99 */
  w11 = 1.0000000000000000e-99;
  /* #87: @11 = (@11+@0) */
  w11 += w0;
  /* #88: @10 = (@10/@11) */
  w10 /= w11;
  /* #89: @10 = (@7-@10) */
  w10  = (w7-w10);
  /* #90: @8 = (@8*@10) */
  w8 *= w10;
  /* #91: @6 = (@6*@8) */
  w6 *= w8;
  /* #92: @8 = cos(@7) */
  w8 = cos( w7 );
  /* #93: @6 = (@6*@8) */
  w6 *= w8;
  /* #94: @8 = 1.62 */
  w8 = 1.6200000000000001e+00;
  /* #95: @10 = 55638.8 */
  w10 = 5.5638759369181564e+04;
  /* #96: @11 = 1.62 */
  w11 = 1.6200000000000001e+00;
  /* #97: @11 = (@11*@2) */
  w11 *= w2;
  /* #98: @11 = (@11-@3) */
  w11 -= w3;
  /* #99: @10 = (@10*@11) */
  w10 *= w11;
  /* #100: @11 = 1e-99 */
  w11 = 1.0000000000000000e-99;
  /* #101: @11 = (@11+@0) */
  w11 += w0;
  /* #102: @10 = (@10/@11) */
  w10 /= w11;
  /* #103: @8 = (@8*@10) */
  w8 *= w10;
  /* #104: @6 = (@6-@8) */
  w6 -= w8;
  /* #105: @5 = (@5*@6) */
  w5 *= w6;
  /* #106: @5 = (@1*@5) */
  w5  = (w1*w5);
  /* #107: @6 = 1 */
  w6 = 1.;
  /* #108: @6 = (@6-@1) */
  w6 -= w1;
  /* #109: @0 = (@9*@0) */
  w0  = (w9*w0);
  /* #110: @7 = (@7*@4) */
  w7 *= w4;
  /* #111: @0 = (@0+@7) */
  w0 += w7;
  /* #112: @7 = 2.7 */
  w7 = 2.7000000000000002e+00;
  /* #113: @0 = (@0/@7) */
  w0 /= w7;
  /* #114: @6 = (@6*@0) */
  w6 *= w0;
  /* #115: @5 = (@5+@6) */
  w5 += w6;
  /* #116: output[0][5] = @5 */
  if (res[0]) res[0][5] = w5;
  /* #117: output[0][6] = @9 */
  if (res[0]) res[0][6] = w9;
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
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    case 2: return casadi_s2;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* sim_car_expl_ode_fun_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int sim_car_expl_ode_fun_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 5;
  if (sz_res) *sz_res = 2;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 12;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
