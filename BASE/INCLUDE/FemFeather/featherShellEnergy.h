#ifndef	_FEM_LINEAR_SHELL_ENERGY_H_
#define	_FEM_LINEAR_SHELL_ENERGY_H_

#include <mathclass/mathclass.h>

void E_A(double a1,double a2,double a3,double b1,double b2,double b3, matrixN& result);
void E_L(double v1,double v2,double v3, matrixN& result);
void E_B_SIN(double vax,double vay,double vaz,double vbx,double vby,double vbz,
	   double vcx,double vcy,double vcz,matrixN& result);
void E_B_COS(double vax,double vay,double vaz,double vbx,double vby,double vbz,
	   double vcx,double vcy,double vcz,matrixN& result);

#endif	// _FEM_LINEAR_SHELL_ENERGY_H_
