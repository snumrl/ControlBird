
#ifndef __OPTIMIZE_H
#define __OPTIMIZE_H

void mnbrak(m_real&, m_real&, m_real&,
			m_real&, m_real&, m_real&, m_real (*func)(m_real)) ;

m_real brent(m_real, m_real, m_real,
			m_real (*f)(m_real), m_real, m_real&) ;

void lnsrch(int n, vectorN& xold, m_real flold, vectorN& g, vectorN& p, vectorN& x,
			m_real& f, m_real stpmax, int& check, m_real (*func)(const vectorN&));

void linmin(vectorN&, vectorN&, int, m_real&, m_real (*func)(const vectorN&));

void gradient_descent(	vectorN&, int, m_real, int&, m_real&,
            m_real (*func)(const vectorN&),
            m_real (*dfunc)(const vectorN&, vectorN&));
void frprmn(vectorN&, int, m_real, int&, m_real&,
            m_real (*func)(const vectorN&),
            m_real (*dfunc)(const vectorN&, vectorN&));
void dfpmin(vectorN& p, int n, m_real gtol, int& iter, m_real& fret,
			m_real (*func)(const vectorN&), m_real (*dfunc)(const vectorN&, vectorN&));
void powell(vectorN& p, matrixN& xi, int n, m_real ftol,
			int& iter, m_real& fret, m_real (*func)(const vectorN&));

void error( char* );

#endif
