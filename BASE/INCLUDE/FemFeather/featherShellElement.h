#ifndef	_FEATHER_SHELL_ELEMENT_H_
#define	_FEATHER_SHELL_ELEMENT_H_

#include <mathclass/mathclass.h>

class	FeatherShellMesh;

// FemShellElement
//
class	FeatherShellElement
{
public:
	FeatherShellMesh*	mesh;

	position*	x;		// material coordinate
	position*	p;		// world coordinate
	
	int			id[4];	// node id

	m_real		A;		// area of the element
	
	matrixN		M, K;	// element mass and stiffness matrix
	matrixN		W;		// element rotation tensor matrix

public:
	FeatherShellElement();

	void	setNodeId(int i0, int i1, int i2);
	int		nodeId(int i) const	{ return id[i]; }

	void	setGlobal(position* x, position* p, m_real* m, FeatherShellMesh*);
	void	precomputeMaterialCoord();
	
	void	precomputeElementMatrices();	// area-based stiffness matrix
	void	precomputeRotationMatrix();

	m_real	WA(m_real A0, vector3[3]);		// stretching energy term

	position	COM() const;
	vector3		areaNormal() const;

	m_real		mass() const;

	m_real		area() const;
	m_real		volume() const;
	m_real		static_area() const		{ return A; }
	m_real		static_volume() const;
};


// FemShellEdge
//
class	FeatherShellEdge
{
public:
	FeatherShellMesh*	mesh;

	m_real		mu;			// element-wise bending stiffness

	position*	x;			// material coordinate
	position*	p;			// world coordinate

	int			id[4];		// node id: two end vertices + 2 opposite vertices for dihedral angles

	m_real		l0;			// length of the edge
	m_real		theta0;		// dihedral angle
	m_real		h0;			// average height

	matrixN		K_edge;		// edge-based stiffness matrix

public:
	FeatherShellEdge();

	void	setNodeId(int i0, int i1, int i2, int i3);
	int		nodeId(int i) const			{ return id[i]; }

	void	setGlobal(position* u, position* p, m_real* m, FeatherShellMesh*);
	void	setBendingStiffness(m_real _mu)	{ mu = _mu; }
	void	precomputeMaterialCoord();

	void	precomputeElementMatrices();	// edge-based stiffness matrix

	position	COM() const	{ return interpolate(0.5, p[id[0]], p[id[1]]); }

	m_real	WE(m_real k_l, m_real k_b, vector3[4]);	// edge-based energy term = WL + WB
	m_real	WL(vector3[2]);		// shearing energy term
	m_real	WB(vector3[4]);		// bending energy term

	m_real	dihedral(vector3[4]) const;

};

#endif	// _FEATHER_SHELL_ELEMENT_H_
