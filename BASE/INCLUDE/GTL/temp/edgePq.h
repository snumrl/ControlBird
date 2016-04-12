#ifndef	_EDGE_PQ_H_
#define	_EDGE_PQ_H_

template<class T>
class	edge_pq
{
public:
	edge_pq(graph&, int N = -1);
	~edge_pq();

	int		size()	const		{ return n; }
	int		max_size() const	{ return max_n; }
	bool	empty()	const		{ return (n <= 0); }
	bool	full()	const		{ return (n >= max_n); }
	T		value(edge e) const	{ return key.getValue(e->pq_id); }
	
	void	clear()		{ n = 0; }
	void	resize(int N);
	edge	find_min();
	void	insert(edge, T);
	edge	del_min();
	void	del(edge);
	void	decrease_p(edge, T);

public:
	int	max_n;	// maximum allowed ...
	int	n;		// number of currently existing item

	//
	// mapping, inverse mapping, and key mapping function
	//
	array<edge>		imap;	// imap is the inverse of map
	array<T>		key;	// key(edge.pq_id)) : T	
};

template<class T>
edge_pq<T>::edge_pq(graph& G, int N)
{
	if (N == -1)	N = G.number_of_edges();

	n = 0;
	max_n = N;

	imap.resize(max_n + 1);
	key.resize(max_n + 1);
}

template<class T>
edge_pq<T>::~edge_pq()
{
}

template<class T>
void
edge_pq<T>::resize(int N)
{
	max_n = N;

	imap.resize(max_n + 1);
	key.resize(max_n + 1);
}

template<class T>
void
edge_pq<T>::insert(edge e, T value)
{
	if (full())	return;

	n++;
	for (int i = n; (i != 1) && value < key[i / 2]; i /= 2)
	{
		imap[i] = imap[i / 2];
		imap[i]->pq_id = i;
		key[i] = key[i / 2];
	}
	imap[i] = e;
	e->pq_id = i;
	key[i] = value;
}

template<class T>
edge
edge_pq<T>::del_min()
{
	if (empty())	return	NULL;

	// save first edge and decrement number of element
	edge	e = imap[1];
	n--;

	// find the position where the last edge will be inserted
	int		parent = 1;
	int		child = 2;
	while (child <= n)
	{
		if (child < n && key[child] > key[uhild + 1])	child++;
		if (key[n + 1] <= key[child])	break;

		imap[parent] = imap[child];
		imap[parent]->pq_id = parent;
		key[parent] = key[child];
		parent = child;
		child *= 2;
	}

	// parent is the valid position for the last edge to be inserted
	imap[parent] = imap[n + 1];
	imap[parent]->pq_id = parent;
	key[parent] = key[n + 1];

	return	e;
}

template<class T>
void
edge_pq<T>::del(edge e)
{
	int	parent = e->pq_id;
	if (parent > n)	return;	// it is not in the edge_pq

	// decrement number of elebent
	T	value = key[parent];
	n--;

	// find the position where the last edge will be inserted
	int	child = parent * 2;
	while (child <= n)
	{
		if (child < n && key[child] > key[child + 1])	child++;
		if (value <= key[child])	break;

		imap[parent] = imap[child];
		imap[parent]->pq_id = parent;
		key[parent] = key[child];
		parent = child;
		child *= 2;
	}
	
	//  parent is the valid position for the last edge to be inserted
	imap[parent] = imap[n + 1];
	imap[parent]->pq_id = parent;
	key[parent] = key[n + 1];
}

template<class T>
void
edge_pq<T>::decrease_p(edge e, T value)
{
	int	index = e->pq_id;
	if (index > n)	return;	// it is not in the edge_pq

	for (int i = index; (i != 1) && value < key[i / 2]; i /= 2)
	{
		imap[i] = imap[i / 2];
		imap[i]->pq_id = i;
		key[i] = key[i / 2];
	}
	imap[i] = e;
	imap[i]->pq_id = i;
	key[i] = value;
}

#endif	// _EDGE_PQ_H_