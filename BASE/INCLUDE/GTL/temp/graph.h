#ifndef	_GRAPH_H_
#define	_GRAPH_H_

class	graph;
class	graph_edge;
class	graph_node;
template<class N, class E> class	GRAPH;
template<class N, class E> class	GRAPH_EDGE;
template<class N, class E> class	GRAPH_NODE;

typedef	graph_node*	node;
typedef	graph_edge*	edge;

//
// graph GRAPH_NODE
//
class	graph_node
{
public:
	graph_node()	{ it = NULL; id = -1; pq_id = -1; }
	~graph_node()	{}

public:
	list_item	it;

	int	id;
	int	pq_id;	// for node_pq mapping function
};

template <class N, class E>
class	GRAPH_NODE : public graph_node
{
public:
	GRAPH_NODE()	{}
	GRAPH_NODE(N VALUE)	{ value = VALUE; }
	~GRAPH_NODE()	{}

public:
	N	value;

	list<GRAPH_EDGE<N, E>*>	adj_list;
	list<GRAPH_EDGE<N, E>*>	in_list;
};

//
// graph GRAPH_EDGE
//
class	graph_edge
{
public:
	graph_edge()	{it = adj_it = in_it = NULL; }
	~graph_edge()	{}

public:
	list_item	it;
	list_item	adj_it;
	list_item	in_it;

	int	id;
	int	pq_id;	// for edge_pq mapping function
};

template <class N, class E>
class	GRAPH_EDGE : public graph_edge
{
public:
	GRAPH_EDGE()	{}
	GRAPH_EDGE(GRAPH_NODE<N, E>* S, GRAPH_NODE<N, E>* T, E VALUE)
	{ s = S; t = T; value = VALUE; }
	~GRAPH_EDGE()	{}

	GRAPH_NODE<N, E>*	source()	{ return s; }
	GRAPH_NODE<N, E>*	target()	{ return t; }

public:
	E	value;

	GRAPH_NODE<N, E>*	s;	// source
	GRAPH_NODE<N, E>*	t;	// target
};

//
// graph
//
class	graph
{
public:
	graph()		{}
	~graph()	{}

	int		number_of_nodes()	{ return n_nodes; }
	int		number_of_edges()	{ return n_edges; }

public:
	int	n_nodes;
	int	n_edges;
};

template <class N, class E>
class	GRAPH : public graph
{
public:
	GRAPH()		{ n_nodes = 0; n_edges = 0; }
	~GRAPH()	{ clear(); }

	void	clear();
	
	GRAPH_NODE<N, E>*	new_node(const N&);
	GRAPH_EDGE<N, E>*	new_edge(node, node, const E);

	// these two functions assume that the last node/edge are issued
	//
	void	delete_node(node);	// remove the specified node and its adjacent edges
	void	delete_node();		// remove the last node and its adjacent edges
	void	delete_edge(node);	// remove the edge connected with the node

	node	source(edge e)	{ return ((GRAPH_EDGE<N, E>*)e)->source(); }
	node	target(edge e)	{ return ((GRAPH_EDGE<N, E>*)e)->target(); }

	node	first_node();
	node	last_node();
	node	succ_node(node);

	edge	first_edge();
	edge	last_edge();
	edge	succ_edge(edge);

	edge	first_adj_edge(node v);
	edge	last_adj_edge(node v);
	edge	adj_succ(edge);

	edge	first_in_edge(node v);
	edge	last_in_edge(node v);
	edge	in_succ(edge);

	int	id(node v)			{ return v->id; }
	
	int	outdeg(node v)		{ return ((GRAPH_NODE<N, E>*)v)->adj_list.length(); }
	int	indeg(node v)		{ return ((GRAPH_NODE<N, E>*)v)->in_list.length(); }
	int	degree(node v)		{ return outdeg(v) + indeg(v); }
	N&	operator[](node v)	{ return ((GRAPH_NODE<N, E>*)v)->value; }
	E&	operator[](edge e)	{ return ((GRAPH_EDGE<N, E>*)e)->value; }

public:
	list<GRAPH_NODE<N, E>*>	node_list;
	list<GRAPH_EDGE<N, E>*>	edge_list;
};

template<class N, class E>
node
GRAPH<N, E>::first_node()
{
	if (node_list.first())	return node_list[node_list.first()];
	else					return NULL;
}

template<class N, class E>
node
GRAPH<N, E>::last_node()
{
	if(node_list.last())	return node_list[node_list.last()];
	else					return NULL;
}

template<class N, class E>
edge
GRAPH<N, E>::first_edge()
{
	if (edge_list.first())	return edge_list[edge_list.first()];
	else					return NULL;
}

template<class N, class E>
edge
GRAPH<N, E>::last_edge()
{
	if (edge_list.last())	return edge_list[edge_list.last()];
	else					return NULL;
}

template<class N, class E>
GRAPH_NODE<N, E>*
GRAPH<N, E>::new_node(const N& VALUE)
{
	GRAPH_NODE<N, E>*	v = new GRAPH_NODE<N, E>(VALUE);
	v->it = node_list.append(v);
	v->id = n_nodes++;

	return	v;
}

template<class N, class E>
GRAPH_EDGE<N, E>*
GRAPH<N, E>::new_edge(node n1, node n2, const E value)
{
	GRAPH_NODE<N, E>*	v1 = (GRAPH_NODE<N, E>*)n1;
	GRAPH_NODE<N, E>*	v2 = (GRAPH_NODE<N, E>*)n2;
	GRAPH_EDGE<N, E>*	e = new GRAPH_EDGE<N, E>(v1, v2, value);
	e->it = edge_list.append(e);
	e->adj_it = v1->adj_list.append(e);
	e->in_it = v2->in_list.append(e);
	e->id = n_edges++;

	return	e;
}

template<class N, class E>
void
GRAPH<N, E>::delete_node(node v)
{
	delete_edge(v);
	node_list.remove(v->it);
	n_nodes--;
}

template<class N, class E>
void
GRAPH<N, E>::delete_node()
{
	if (node_list.size() == 0)	return;
	
	list_item	it = node_list.last();
	delete_edge(node_list[it]);
	node_list.remove(it);
	n_nodes--;
}

template<class N, class E>
void
GRAPH<N, E>::delete_edge(node n1)
{
	while (1)
	{
		if (edge_list.size() == 0)	return;

		list_item	it = edge_list.last();
		edge	e = edge_list[it];

		if (source(e) != n1 && target(e) != n1)	return;

		edge_list.remove(it);
		n_edges--;
	}
}

template<class N, class E>
node
GRAPH<N, E>::succ_node(node v)
{
	GRAPH_NODE<N, E>*	n = (GRAPH_NODE<N, E>*)v;

	list_item	it = node_list.succ(n->it);
	if (it == NULL)	return	NULL;

	return	node_list[it];
}

template<class N, class E>
edge
GRAPH<N, E>::succ_edge(edge ed)
{
	GRAPH_EDGE<N, E>*	e = (GRAPH_EDGE<N, E>*)ed;

	list_item	it = node_list.succ(e->it);
	if (it == NULL)	return	NULL;

	return	edge_list[it];
}

template<class N, class E>
edge
GRAPH<N, E>::first_adj_edge(node v)
{
	GRAPH_NODE<N, E>*	n = (GRAPH_NODE<N, E>*)v;

	list_item	it = n->adj_list.first();
	if (it == NULL)	return	NULL;

	return	n->adj_list[it];
}

template<class N, class E>
edge
GRAPH<N, E>::last_adj_edge(node v)
{
	GRAPH_NODE<N, E>*	n = (GRAPH_NODE<N, E>*)v;

	list_item	it = n->adj_list.last()
	if (it == NULL)	return	NULL;

	return	n->adj_list[it];
}

template<class N, class E>
edge
GRAPH<N, E>::adj_succ(edge ed)
{
	GRAPH_EDGE<N, E>*	e = (GRAPH_EDGE<N, E>*)ed;
	GRAPH_NODE<N, E>*	n = e->source();

	list_item	it = n->adj_list.succ(e->adj_it);
	if (it == NULL)	return	NULL;

	return	n->adj_list[it];
}

template<class N, class E>
edge
GRAPH<N, E>::first_in_edge(node v)
{
	GRAPH_NODE<N, E>*	n = (GRAPH_NODE<N, E>*)v;

	list_item	it = n->in_list.first();
	if (it == NULL)	return	NULL;

	return	n->in_list[it];
}

template<class N, class E>
edge
GRAPH<N, E>::last_in_edge(node v)
{
	GRAPH_NODE<N, E>*	n = (GRAPH_NODE<N, E>*)v;

	list_item	it = n->in_list.last()
	if (it == NULL)	return	NULL;

	return	n->in_list[it];
}

template<class N, class E>
edge
GRAPH<N, E>::in_succ(edge ed)
{
	GRAPH_EDGE<N, E>*	e = (GRAPH_EDGE<N, E>*)ed;
	GRAPH_NODE<N, E>*	n = e->source();

	list_item	it = n->in_list.succ(e->in_it);
	if (it == NULL)	return	NULL;

	return	n->in_list[it];
}

template<class N, class E>
void
GRAPH<N, E>::clear()
{
	n_nodes = 0;
	n_edges = 0;

	node_list.clear();
	edge_list.clear();
}

#endif	// _GRAPH_H_
