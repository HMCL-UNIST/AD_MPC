/*
 *    This file was auto-generated by ACADO Toolkit.
 *
 *    ACADO Toolkit -- A Toolkit for Automatic Control and Dynamic Optimization.
 *    Copyright (C) 2008-2011 by Boris Houska, Hans Joachim Ferreau et al., K.U.Leuven.
 *    Developed within the Optimization in Engineering Center (OPTEC) under
 *    supervision of Moritz Diehl. All rights reserved.
 *
 */


#include "acado.h"


#include "auxiliary_functions.c"


/* SOME CONVENIENT DEFINTIONS: */
/* --------------------------------------------------------------- */
   #define NX          6      /* number of differential states  */
   #define NU          2      /* number of control inputs       */
   #define NP          3      /* number of fixed parameters     */
   #define N          10      /* number of control intervals    */
   #define NUM_STEPS   5      /* number of real time iterations */
   #define VERBOSE     1      /* show iterations: 1, silent: 0  */
/* --------------------------------------------------------------- */


/* GLOBAL VARIABLES FOR THE ACADO REAL-TIME ALGORITHM: */
/* --------------------------------------------------- */
   ACADOvariables acadoVariables;
   ACADOworkspace acadoWorkspace;

/* GLOBAL VARIABLES FOR THE QP SOLVER: */
/* ----------------------------------- */
   Vars         vars;
   Params       params;


/* A TEMPLATE FOR TESTING THE REAL-TIME IMPLEMENTATION: */
/* ---------------------------------------------------- */
int main(){

   /* INTRODUCE AUXILIARY VAIRABLES: */
   /* ------------------------------ */
      int    i, iter        ;
      real_t measurement[NX];


   /* INITIALIZE THE STATES AND CONTROLS: */
   /* ---------------------------------------- */
      for( i = 0; i < N; ++i )
	  {
		  acadoVariables.x[i*NX+0] = 0.941e+000;
		  acadoVariables.x[i*NX+1] = 1.121e+000;
		  acadoVariables.x[i*NX+2] = 0.307;
		  acadoVariables.x[i*NX+3] = 1.191;
		  acadoVariables.x[i*NX+4] = 0.498e+000;
		  acadoVariables.x[i*NX+5] = 0.0;
	  }

      for( i = 0; i < N; ++i )
	  {
		  acadoVariables.u[i*NU+0] = 0.0;
		  acadoVariables.u[i*NU+1] = 0.0;
	  }

	  acadoVariables.p[0] = 35.0;
	  acadoVariables.p[1] =  1.0;
	  acadoVariables.p[2] =  0.7;
	  //acadoVariables.p[2] =  0.15;


   /* // INITIALIZE THE STATES AND CONTROL REFERENCE: */
   /* // -------------------------------------------- */
      for( i = 0; i < N; ++i )
	  {
		  acadoVariables.xRef[i*NX+0] = 0.941e+000;
		  acadoVariables.xRef[i*NX+1] = 1.121e+000;
		  acadoVariables.xRef[i*NX+2] = 0.307;
		  acadoVariables.xRef[i*NX+3] = 1.191;
		  acadoVariables.xRef[i*NX+4] = 0.498e-001;
		  acadoVariables.xRef[i*NX+5] = 0.0;
	  }

      for( i = 0; i < N; ++i )
	  {
		  acadoVariables.uRef[i*NU+0] =  0.0;
		  acadoVariables.uRef[i*NU+1] =  0.0;
	  }


   /* SETUP THE FIRST STATE MEASUREMENT: */
   /* ------------------------------------------------ */
      for( i = 0; i < NX; ++i )  measurement[i] = 1.0*acadoVariables.x[i];

      if( VERBOSE ) printHeader();


   /* PREPARE FIRST STEP: */
   /* ------------------- */
      preparationStep();


   /* GET THE TIME BEFORE START THE LOOP: */
   /* ---------------------------------------------- */
      real_t t1 = getTime();


   /* THE REAL-TIME ITERATION LOOP: */
   /* ---------------------------------------------- */
      for( iter = 0; iter < NUM_STEPS; ++iter ){

        /* TAKE A MEASUREMENT: */
        /* ----------------------------- */
           /// meausrement = ...

        /* PERFORM THE FEEDBACK STEP: */
        /* ----------------------------- */
           feedbackStep( measurement );

        /* APPLY THE NEW CONTROL IMMEDIATELY TO THE PROCESS: */
        /* ------------------------------------------------- */
           /// send first piece of acadoVariables.u to process;
           if( VERBOSE ) printf("=================================================================\n\n" );
           if( VERBOSE ) printf("      Real-Time Iteration %d:  KKT Tolerance = %.3e\n", iter, getKKT() );
           if( VERBOSE ) printf("\n=================================================================\n" );

        /* OPTIONAL: SHIFT THE INITIALIZATION: */
        /* ----------------------------------- */
           /// shiftControls( acadoVariables.uRef );
           /// shiftStates  ( acadoVariables.xRef );

        /* PREPARE NEXT STEP: */
        /* ------------------ */
           preparationStep();
      }
      if( VERBOSE ) printf("\n\n              END OF THE REAL-TIME LOOP. \n\n\n");


   /* GET THE TIME AT THE END OF THE LOOP: */
   /* ---------------------------------------------- */
      real_t t2 = getTime();


   /* PRINT DURATION AND RESULTS: */
   /* -------------------------------------------------------------------------------------------------- */
      if( !VERBOSE )
      printf("\n\n AVERAGE DURATION OF ONE REAL-TIME ITERATION:   %.3g μs\n\n", 1e6*(t2-t1)/NUM_STEPS );

      printStates();
      printControls();

    return 0;
}