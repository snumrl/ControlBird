#ifndef	_assert_H_
#define	_assert_H_

#define	myassert(exp) \
{ \
	if (!(exp)) \
	{ \
		cerr << #exp; \
		cerr << " in line " << __LINE__; \
		cerr << " at file " << __FILE__; \
		cerr << endl; \
	} \
}

#endif