#ifndef	_NODE_PQ_H_
#define	_NODE_PQ_H_

template<class T>
class	node_pq
{
public:
	node_pq(graph&, int N = -1);
	~node_pq();

	int		size() const		{ return n; }
	int		max_size() const	{ return max_n; }
	bool	empty()	const		{ return (n <= 0); }
	bool	full() const		{ return (n >= max_n); }
	T		value(node v) const	{ return key.getValue(v->pq_id); }

	void	clear()				{ n = 0; }
	void	resize(int N);
	node	find_min();
	void	insert(node, T);
	node	del_min();
	void	del(node);
	void	decrease_p(node, T);

public:
	int	max_n;	// maximum allowed ...
	int	n;		// number of currently existing item

	//
	// mapping, inverse mapping, and key mapping function
	//
	array<node>		imap;	// imap is the inverse of map
	array<T>		key;	// key(node.pq_id)) : T	
};

template<class T>
node_pq<T>::node_pq(graph& G, int N)
{
	if (N == -1)	N = G.number_of_nodes();

	n = 0;
	max_n = N;

	imap.resize(max_n + 1);
	key.resize(max_n + 1);
}

template<class T>
node_pq<T>::~node_pq()
{
}

template<class T>
void
node_pq<T>::resize(int N)
{
	max_n = N;

	imap.resize(max_n + 1);
	key.resize(max_n + 1);
}

template<class T>
void
node_pq<T>::insert(node v, T value)
{
	if (full())	return;

	n++;
	int	i;
	for (i = n; (i != 1) && value < key[i / 2]; i /= 2)
	{
		imap[i] = imap[i / 2];
		imap[i]->pq_id = i;
		key[i] = key[i / 2];
	}
	imap[i] = v;
	v->pq_id = i;
	key[i] = value;
}

template<class T>
node
node_pq<T>::del_min()
{
	if (empty())	return	NULL;

	// save first node and decrement number of element
	node	v = imap[1];
	n--;

	// find the position where the last node will be inserted
	int		parent = 1;
	int		child = 2;
	while (child <= n)
	{
		if (child < n && key[child] > key[child + 1])	child++;
		if (key[n + 1] <= key[child])	break;

		imap[parent] = imap[child];
		imap[parent]->pq_id = parent;
		key[parent] = key[child];
		parent = child;
		child *= 2;
	}

	// parent is the valid position for the last node to be inserted
	imap[parent] = imap[n + 1];
	imap[parent]->pq_id = parent;
	key[parent] = key[n + 1];

	return	v;
}

template<class T>
void
node_pq<T>::del(node v)
{
	int	parent = v->pq_id;
	if (parent > n)	return;	// it is not in the node_pq

	// decrement number of element
	T	value = key[parent];
	n--;

	// find the position where the last node will be inserted
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
	
	//  parent is the valid position for the last node to be inserted
	imap[parent] = imap[n + 1];
	imap[parent]->pq_id = parent;
	key[parent] = key[n + 1];
}

template<class T>
void
node_pq<T>::decrease_p(node v, T value)
{
	int	index = v->pq_id;
	if (index > n)	return;				// it is not in the node_pq
	
	int	i;
	for (i = index; (i != 1) && value < key[i / 2]; i /= 2)
	{
		imap[i] = imap[i / 2];
		imap[i]->pq_id = i;
		key[i] = key[i / 2];
	}
	imap[i] = v;
	imap[i]->pq_id = i;
	key[i] = value;
}

#endif	// _NODE_PQ_H_