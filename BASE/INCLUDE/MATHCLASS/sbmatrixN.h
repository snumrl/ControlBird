//
// Sparse Matrix:	Aug 11, 1998
//

#ifndef	_SPARSE_BLOCK_MATRIX_H_
#define	_SPARSE_BLOCK_MATRIX_H_

#define	HEAD_ENTITY	-1

struct	bentity
{
	int			id;
	matrix		value;
	bentity*	next;
};

class	bvectorN;

class	sbmatrixN
{
private:
	int		n;		// # of rows
	int		m;		// # of columns

	int			nnze;	// number of non-zero entity
	bentity*	rows;

public:
	sbmatrixN();
	sbmatrixN(const sbmatrixN& a);
	sbmatrixN(int n, int m);
	~sbmatrixN();

	int		nnz() const;
	int		countNNZ() const;

	inline int	row() const				{ return n; }
	inline int	column() const			{ return m; }

	void	setSize(int n, int m);

	void	clear()	{ deallocate(); }
	void	setExistingValuesToZero();

	void	allocate(int n);
	void	deallocate();
	
	matrix		setValue(int row, int col, const matrix& value, bentity** e = NULL);
	matrix		getValue(int row, int col) const;

	bentity*	firstEntity(int row) const	{ return rows[row].next; }
	bentity*	nextEntity(bentity* e) const	{ return e->next; }
	matrix		value(bentity* e)			{ return e->value; }
	int			column(bentity* e)			{ return e->id; }

	bentity		 getRows( int i ) const { return rows[i]; }

	sbmatrixN&	assign(const m_real s, sbmatrixN const&);
	sbmatrixN&	assign(sbmatrixN const&);
	void		add(int row, int col, const matrix& increment);
	void		sub(int row, int col, const matrix& decrement);
	sbmatrixN&	add(sbmatrixN& a, sbmatrixN& b);
	sbmatrixN&	add(const m_real s, sbmatrixN& a);
	sbmatrixN&	negate();

	sbmatrixN&  operator=(sbmatrixN const&);
	sbmatrixN&	operator*=(const m_real);

	bool	diff(sbmatrixN&, m_real tol = EPS, bool diagnostic = false);
};

// Compressed sparse row format of sbmatrixN
class	sbmatrixCSR
{
private:
	int	n;	// # of rows
	int	m;	// # of columns

	int	nnze;	// number of non-zero entity

	matrix*	values;
	int*	columns;
	int*	rows;

public:
	sbmatrixCSR();
	sbmatrixCSR(const sbmatrixN& A);
	~sbmatrixCSR();

	inline int		row() const		{ return n; }
	inline int		column() const	{ return m; }

	inline int		rowFirstIndex(int i) const	{ return rows[i]; }
	inline int		rowLastIndex(int i)	const 	{ return rows[i+1]; }
	inline int		columnIndex(int i) const		{ return columns[i]; }
	inline matrix&	value(int i) const			{ return values[i]; }

	matrix	getValue(int row, int col) const;

	void	clear();

	sbmatrixCSR&	assign(sbmatrixN const&);
	sbmatrixCSR&	operator=(sbmatrixN const&);
};

struct	sbmatrixCSRARow
{
	int	nnze;
	matrix*	values;
	int*	columns;
};

// Compressed sparse row in array format of sbmatrixN
class	sbmatrixCSRA
{
private:
	int	n;	// # of rows
	int	m;	// # of columns

	sbmatrixCSRARow*	rows;

public:
	sbmatrixCSRA();
	sbmatrixCSRA(const sbmatrixN& A);
	~sbmatrixCSRA();

	inline int		row() const		{ return n; }
	inline int		column() const	{ return m; }

	inline int		nnzRow(int i) const				{ return rows[i].nnze; }
	inline int		columnIndex(int i, int j) const	{ return rows[i].columns[j]; }
	inline matrix&	value(int i, int j) const		{ return rows[i].values[j]; }

	matrix	getValue(int row, int col) const;

	void	clear();

	sbmatrixCSRA&	assign(sbmatrixN const&);
	sbmatrixCSRA&	operator=(sbmatrixN const&);
};

#endif	// _SPARSE_MATRIX_H_
