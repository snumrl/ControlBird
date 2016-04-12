
#ifndef _SAVITZKY_GOLAY_H_
#define _SAVITZKY_GOLAY_H_

void	savgol(m_real c[], int np, int nl, int nr, int ld, int m);

vector3	sgolay_1st_derivative(int k, int f, vector3* data);
vector3	sgolay_2nd_derivative(int k, int f, vector3* data);

#endif
