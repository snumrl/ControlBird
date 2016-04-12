#ifndef	_EDGE_ARRAY_H_
#define	_EDGE_ARRAY_H_

template<class T>	class	edge_array;

template<class T>
class	edge_array : public array<T>
{
public:
	edge_array() {}
	edge_array(graph& G) { resize(G.number_of_edges()); }

	T&	operator[](edge e) const	{ return array<T>::operator[](e->id); }
};

#endif	// _EDGE_ARRAY_H_