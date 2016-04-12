//
// Sparse Matrix:	Aug 11, 1998
//

#ifndef	_SPARSE_MATRIX_H_
#define	_SPARSE_MATRIX_H_

#define	HEAD_ENTITY	-1
#define	MEANINGLESS	0

struct	entity
{
	int		id;
	m_real	value;
	entity*	next;
};

class	vectorN;

class	smatrixN
{
private:
	int		n;		// # of rows
	int		m;		// # of columns

	int		nnze;	// number of non-zero entity
	entity*	rows;

public:
	smatrixN();
	smatrixN(int n, int m);
	~smatrixN();

	int		nnz() const;
	int		countNNZ() const;

	int		row() const				{ return n; }
	int		column() const			{ return m; }
	void	setSize(int n, int m);

	void	clear()	{ deallocate(); }

	void	allocate(int n);
	void	deallocate();

	m_real	norm();

	m_real	setValue(int row, int col, m_real value, entity** e = NULL);
	m_real	getValue(int row, int col) const;

	entity*	firstEntity(int row) const	{ return rows[row].next; }
	entity*	nextEntity(entity* e) const	{ return e->next; }
	m_real	value(entity* e)			{ return e->value; }
	int		column(entity* e)			{ return e->id; }

	entity  getRows( int i ) const { return rows[i]; }

	smatrixN&	assign(smatrixN const&);
	m_real		add(int row, int col, m_real increment);
	m_real		sub(int row, int col, m_real decrement);
	smatrixN&	add(smatrixN& a, smatrixN& b);
	smatrixN&	add(m_real s, smatrixN& a);
	smatrixN&	negate();
	smatrixN&	operator*=(m_real);

	bool	diff(smatrixN&, m_real tol = EPS, bool diagnostic = false);

	friend ostream& operator<<(ostream& os, smatrixN& A);
	friend istream& operator>>(istream& is, smatrixN& A);
};

#endif	// _SPARSE_MATRIX_H_
