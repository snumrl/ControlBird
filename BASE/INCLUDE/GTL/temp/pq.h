#ifndef	_PRIORITY_QUEUE_H_
#define	_PRIORITY_QUEUE_H_

template<class T>	class	priority_queue;

template<class T>
class	priority_queue
{
public:
	priority_queue(int N = -1);
	~priority_queue();

	void	clear()		{ n = 0; }
	int		size()		{ return n; }
	int		max_size()	{ return max_n; }
	bool	empty()		{ return (n <= 0); }
	bool	full()		{ return (n >= max_n); }

	T		value(int id) const	{ return key.getValue(map[id]); }

	void	resize(int N);
	void	insert(int id, T);
	int		del_min();
	void	del(int id);
	void	decrease_p(int id, T);

public:
	int	max_n;	// maximum allowed ...
	int	n;	// number of currently existing item

	//
	// mapping, inverse mapping, and key mapping function
	//

	array<int>		imap;	// imap is the inverse of map: pq position -> id
	array<int>		map;	// map: id -> pq position
	array<T>		key;	// key(map(node.id)) : T	
};

template<class T>
priority_queue<T>::priority_queue(int N)
{
	if (N == -1)	N = 1;

	n = 0;
	max_n = N;

	imap.setSize(max_n + 1);
	map.setSize(max_n + 1);
	key.setSize(max_n + 1);
}

template<class T>
priority_queue<T>::~priority_queue()
{
}

template<class T>
void
priority_queue<T>::resize(int N)
{
	max_n = N;

	imap.resize(max_n + 1);
	map.resize(max_n + 1);
	key.resize(max_n + 1);
}

template<class T>
void
priority_queue<T>::insert(int id, T value)
{
	if (full())	return;

	n++;
	int	i = n;
	for (; (i != 1) && value < key[i / 2]; i /= 2)
	{
		imap[i] = imap[i / 2];
		map[imap[i]] = i;
		key[i] = key[i / 2];
	}
	imap[i] = id;
	map[id] = i;
	key[i] = value;
}

template<class T>
int
priority_queue<T>::del_min()
{
	if (empty())	return	NULL;

	// save first node and decrement number of element
	int	id = imap[1];
	n--;

	// find the position where the last node will be inserted
	int		parent = 1;
	int		child = 2;
	while (child <= n)
	{
		if (child < n && key[child] > key[child + 1])	child++;
		if (key[n + 1] <= key[child])	break;

		imap[parent] = imap[child];
		map[imap[parent]] = parent;
		key[parent] = key[child];
		parent = child;
		child *= 2;
	}

	// parent is the valid position for the last node to be inserted
	imap[parent] = imap[n + 1];
	map[imap[parent]] = parent;
	key[parent] = key[n + 1];

	return	id;
}

template<class T>
void
priority_queue<T>::del(int id)
{
	int	parent = map[id];
	if (parent > n)	return;	// it is not in the priority_queue

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
		map[imap[parent]] = parent;
		key[parent] = key[child];
		parent = child;
		child *= 2;
	}
	
	//  parent is the valid position for the last node to be inserted
	imap[parent] = imap[n + 1];
	map[imap[parent]] = parent;
	key[parent] = key[n + 1];
}

template<class T>
void
priority_queue<T>::decrease_p(int id, T value)
{
	int	index = map[id];
	if (index > n)	return;	// it is not in the priority_queue

	for (int i = index; (i != 1) && value < key[i / 2]; i /= 2)
	{
		imap[i] = imap[i / 2];
		map[imap[i]] = i;
		key[i] = key[i / 2];
	}
	imap[i] = id;
	map[imap[i]] = i;
	key[i] = value;
}

#endif	// _PRIORITY_QUEUE_H_
