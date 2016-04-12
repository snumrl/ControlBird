#ifndef	_LINKED_LIST_H_
#define	_LINKED_LIST_H_

#include <assert.h>

class	list_node;
template<class T>	class	list;
template<class T>	class	LIST_NODE;

typedef	list_node*	list_item;

//
// list_node
//
class	list_node
{
public:
	list_node()	{ prev = NULL; next = NULL; }

public:
	list_node*	prev;
	list_node*	next;
};

template<class T>
class	LIST_NODE : public list_node
{
public:
	T	value;
};

//
// list
//
template<class T>
class	list
{
public:
	list()	{ n = 0; start = end = NULL; }
	list(list<T>&);
	~list()	{ clear(); }

	int		empty() const	{ return (n == 0); }
	int		size() const	{ return n; }
	int		length() const	{ return n; }

	LIST_NODE<T>*	insert(T);
	LIST_NODE<T>*	insert(T, list_item);
	LIST_NODE<T>*	push(T v)	{ return append(v); }
	LIST_NODE<T>*	append(T);
	LIST_NODE<T>*	append(list_item, T);

	T		pop();
	void	remove(list_item);
	void	clear();

	LIST_NODE<T>*	first()	const { return start; }
	LIST_NODE<T>*	last()	const { return end; }
	LIST_NODE<T>*	pred(list_item it) const
	{
		if (it == NULL)	return	NULL;

		return	(LIST_NODE<T>*)((LIST_NODE<T>*)it)->prev;
	}
	LIST_NODE<T>*	succ(list_item it) const
	{
		if (it == NULL)	return	NULL;
	
		return	(LIST_NODE<T>*)((LIST_NODE<T>*)it)->next;
	}
	T&	operator[](list_item it) const	{ return	((LIST_NODE<T>*)it)->value; }
	LIST_NODE<T>*	getItem(int k);

	void	makeList(T* t, int n);
	list<T>&	operator=(list<T>&);	// Assign

	template<class T> friend list<T>&	operator+=(list<T>&, const list<T>&);	// Concatenate

	template<class T> friend T*		list2array(const list<T>&);
	template<class T> friend void	reverse(list<T>&);

public:
	// number of LIST_NODE
	int	n;

private:
	// for succ operation
	LIST_NODE<T>*	start;
	LIST_NODE<T>*	end;
};

//
// Method Implementation
//

template<class T>
list<T>::list(list<T>& l)
{
	n = 0; start = end = NULL;

	for (LIST_NODE<T>* it = l.first(); it; it = l.succ(it))
		append(l[it]);
}

template<class T>
void
list<T>::makeList(T* t, int n)
{
	clear();
	for (int i = 0; i < n; i++)
		append(t[i]);
}

template<class T>
LIST_NODE<T>*
list<T>::insert(T v)
{
	n++;

	LIST_NODE<T>*	curr = new LIST_NODE<T>;
	curr->value = v;

	if (!end) end = curr;

	curr->next = start;
	if (curr->next)	curr->next->prev = curr;
	curr->prev = NULL;

	start = curr;

	return	curr;
}

template<class T>
LIST_NODE<T>*
list<T>::insert(T v, list_item it)
{
	n++;

	LIST_NODE<T>*	next = (LIST_NODE<T>*)it;
	LIST_NODE<T>*	curr = new LIST_NODE<T>;
	curr->value = v;

	curr->next = next;
	curr->prev = next->prev;
	if (curr->prev)	curr->prev->next = curr;
	else			start = curr;
	next->prev = curr;

	return	curr;
}

template<class T>
LIST_NODE<T>*
list<T>::append(T v)
{
	n++;

	LIST_NODE<T>*	curr = new LIST_NODE<T>;
	curr->value = v;

	if (!start)	start = curr;

	curr->prev = end;
	if (curr->prev)	curr->prev->next = curr;
	curr->next = NULL;
	end = curr;

	return	curr;
}

template<class T>
LIST_NODE<T>*
list<T>::append(list_item it, T v)
{
	n++;

	LIST_NODE<T>*	prev = (LIST_NODE<T>*)it;
	LIST_NODE<T>*	curr = new LIST_NODE<T>;
	curr->value = v;

	curr->prev = prev;
	curr->next = prev->next;
	if (curr->next)	curr->next->prev = curr;
	else			end = curr;
	prev->next = curr;

	return	curr;
}

template<class T>
T
list<T>::pop(void)
{
	assert(end);

	T	temp_value = end->value;

	list_item	prev_end = end->prev;
	remove(end);
	end = (LIST_NODE<T>*)prev_end;

	return	temp_value;
}

template<class T>
void
list<T>::remove(list_item it)
{
	if (it == NULL)	return;

	LIST_NODE<T>*	r = (LIST_NODE<T>*)it;

	n--;

	if (r->prev)	r->prev->next = r->next;
	else			start = (LIST_NODE<T>*)it->next;

	if (r->next)	r->next->prev = r->prev;
	else			end = (LIST_NODE<T>*)it->prev;

	delete	r;
}

template<class T>
void
list<T>::clear()
{
	for (LIST_NODE<T>* p = first(); p; p = succ(p))
	{
		if (p->prev)	delete	p->prev;
		if (!p->next)	{ delete p; break; }
	}

	n = 0;
	start = end = NULL;
}

template<class T>
list<T>&
list<T>::operator=(list<T>& l)
{
	clear();
	for (LIST_NODE<T>* it = l.first(); it; it = l.succ(it))
		append(l[it]);

	return	*this;
}

// Concatenate
template<class T>
list<T>&
operator+=(list<T>& a, const list<T>& b)
{
	for (LIST_NODE<T>* it = b.first(); it; it = b.succ(it))
		a.append(b[it]);

	return	a;
}

template<class T>
LIST_NODE<T>*
list<T>::getItem(int k)
{
	int	i = 0;
	LIST_NODE<T>*	it;
	for (i = 0, it = first(); it; it = succ(it), i++)
		if (i == k)	return	it;

	return	NULL;
}

template<class T>
T*
list2array(const list<T>& l)
{
	T*	t = new T[l.n];
	int	i = 0;
	LIST_NODE<T>*	it;
	for (i = 0, it = l.first(); it; it = l.succ(it), i++)
		t[i] = l[it];
	
	return	t;
}

#endif	// _LINKED_LIST_H_