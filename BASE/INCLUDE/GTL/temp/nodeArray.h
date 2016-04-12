#ifndef	_NODE_ARRAY_H_
#define	_NODE_ARRAY_H_

template<class T>
class	node_array : public array<T>
{
public:
	node_array() {}
	node_array(graph& G) { resize(G.number_of_nodes()); }

	T&	operator[](node v)	{ return array<T>::operator[](v->id); }
};

#endif	// _NODE_ARRAY_H_