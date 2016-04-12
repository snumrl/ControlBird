
#ifndef _CUBIC_H_
#define _CUBIC_H_

int		solveCubic4ThreeRealRoots(vector3& c);	// Solve cubic equations that has definitely 3 real solutions
void	solveCubic4ThreeRealRootsOld(vector3& c);

int		solveCubic4RealRoots(vector3& c);
int		solveCubic4RealRootsOld(vector3& c);
void	solveCubic4RealRootsOldVersion2(vector3& c);

int		solveCubicAll(vector3& c);

void	sortDecreasing3(vector3& x);
void	sortDecreasing2(vector3& x);

void	sortIncreasing3(vector3& x);
void	sortIncreasing2(vector3& x);

int		solveQuadratic4RealRoots(vector3& c);
int		solveQuadratic4RealRootsElementaryVersion(vector3& c);

m_real	evaluateCubicRootError(vector3& c, vector3& s, int n);

#endif
