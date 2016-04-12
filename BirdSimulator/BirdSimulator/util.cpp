#include "util.h"

using namespace util;

// knot = c + order
bool SPLINE::genCurve(const std::vector<double>& ctrpnts, bool endPointInterp)
{
	if(ctrpnts.size()%dim != 0)
	{
		std::cout << "number of control points is not multiples of 3" << std::endl;
		return false;
	}
	double* cpoint;
	int nbpnt	= (int)ctrpnts.size()/dim;

	double *knot;

	cpoint = new double[dim*nbpnt];

	if(endPointInterp)	
	{
		int knotSeq = 1;

		knot = new double[nbpnt+order];

		for(int i=0; i<order; i++) knot[i] = 0;
		for(int i=order; i<nbpnt; i++) knot[i] = knotSeq++;
		for(int i=nbpnt; i<nbpnt+order; i++) knot[i] = knotSeq;

		//for(int i=0; i
	}
	else
	{
		knot = new double[nbpnt+order];
		for(int i=0; i<nbpnt+order; i++) knot[i] = i;
	}
		
	for(int i=0; i<nbpnt; i++)
	{
		for(int j=0; j<dim; j++)
			cpoint[dim*i+j] = ctrpnts[dim*i+j];
	}

	if(pc!=NULL) freeCurve(pc);
	pc = newCurve(nbpnt, order, knot, cpoint, 1, dim, 1);

	delete[] cpoint;
	delete[] knot;

	return true;
}

bool SPLINE::genInterpCurve(const std::vector<double>& ctrpnts, bool isOrdinary)
{
	double* cpoint;
	int nbpnt		= ctrpnts.size()/dim;
	int *nptyp;
	int cnsta		= 0;
	int cnend		= 0;
	int open		= 1;
	int k			= 4;
	double astpar	= 0.f;
	double cendpar	= 0.f;
	double *gpar	= NULL;
	int nbpar		= 0;
	int stat;

	cpoint = new double[dim*nbpnt];
	nptyp = new int[nbpnt];

	if(isOrdinary)
	{
		for(int i=0; i<nbpnt; i++)
		{
			// ordinary point
			nptyp[i] = 1;

			// point
			for(int j=0; j<dim; j++)
				cpoint[dim*i+j] = ctrpnts[dim*i+j];
		}
	}
	else
	{
		for(int i=0; i<nbpnt; i++)
		{
			if(i%2==0)
			{
				// ordinary point
				nptyp[i] = 1;

				// point
				for(int j=0; j<dim; j++)
					cpoint[dim*i+j] = ctrpnts[dim*i+j];
			}
			else
			{
				// derivative
				nptyp[i] = 3;

				// next derivative
				for(int j=0; j<dim; j++)
					cpoint[dim*i+j] = ctrpnts[dim*i+j];
			}
		}
	}

	if(pc!=NULL) freeCurve(pc);
	s1356(cpoint, nbpnt, dim, nptyp, cnsta, cnend, open, 
		k, astpar, &cendpar, &pc, &gpar, &nbpar, &stat);

	if(stat<0)
	{
		std::cout << "Error : genInterpCurve" << std::endl;
		return false;
	}
	else if(stat>0)
	{
		std::cout << "Warning : genInterpCurve" << std::endl;
		return false;
	}

	delete[] cpoint;
	delete[] nptyp;

	return true;
}

bool SPLINE::genCatMullRommLikeCurve(const std::vector<double>& ctrpnts)
{
	vector<double> ctrPnts;
	int num_ctrpnts = (int)ctrpnts.size() / dim;

	//ctrpnts[dim*(num_ctrpnts-1)

	for(int i=0; i<num_ctrpnts; i++)
	{
		if(i==0)
		{
			// point
			for(int j=0; j<dim; j++) ctrPnts.push_back( ctrpnts[dim*i + j] );
		}
		else if(i==num_ctrpnts-1)
		{
			// tangent
			for(int j=0; j<dim; j++) ctrPnts.push_back( 0.1 );
			// point
			for(int j=0; j<dim; j++) ctrPnts.push_back( ctrpnts[dim*i + j] );
		}
		else
		{
			// tangent
			for(int j=0; j<dim; j++) ctrPnts.push_back( (ctrpnts[dim*(i+1) + j] - ctrpnts[dim*(i-1) + j]) / 2.0f );
			// point
			for(int j=0; j<dim; j++) ctrPnts.push_back( ctrpnts[dim*i + j] );
		}
	}

	return genInterpCurve(ctrPnts);
}

bool SPLINE::genBlendingCurve(SPLINE *curve1, float param1, SPLINE *curve2, float param2)
{
	double	epsge	= geomResolution;
	double	*point1 = new double[dim];
	double	*point2 = new double[dim];
	int		blendtype = 2;
	int		stat;

	const std::vector<double>& p1 = curve1->getPoint(param1);
	const std::vector<double>& p2 = curve2->getPoint(param2);

	for(int i=0; i<dim; i++) { point1[i]=p1[i]; point2[i]=p2[i]; }

	if(pc!=NULL) freeCurve(pc);
	s1606(curve1->pc, curve2->pc, epsge, 
		point1, point2, blendtype, dim, order, &pc, &stat);

	if(stat<0)
	{
		std::cout << "Error : genBlendingCurve" << std::endl;
		return false;
	}
	else if(stat>0)
	{
		std::cout << "Warning : genBlendingCurve" << std::endl;
		return false;
	}

	delete[] point1;
	delete[] point2;

	return true;
}

bool SPLINE::genFilletCurve(SPLINE *curve1, SPLINE *curve2)
{
	double epsge	= geomResolution;
	double end1		= 0.f;
	double fillpar1	= 1.f;
	double end2		= 0.0f;
	double fillpar2	= 1.0f;
	int filltype	= 1;
	int stat;

	if(pc!=NULL) freeCurve(pc);
	s1607(curve1->pc, curve2->pc, epsge, 
		end1, fillpar1, end2, fillpar2, filltype, dim, order, &pc, &stat);

	if(stat<0)
	{
		std::cout << "Error : genFilletCurve" << std::endl;
		return false;
	}
	else if(stat>0)
	{
		std::cout << "Warning : genFilletCurve" << std::endl;
		return false;
	}

	return true;
}

void SPLINE::getParameterRange(float& sp, float& ep)
{
	int stat;
	double s, e;

	s1363(pc, &s, &e, &stat);

	sp = s;
	ep = e;
}

float SPLINE::getParameter(const std::vector<double>& p)
{
	double *point = new double[dim];
	double epsco=0.0001;
	double epsge = geomResolution;
	int numintpt;
	double *intpar;
	int numintcu;
	SISLIntcurve **intcurve;
	int stat;
	double sp, ep;

	double gpar = 0;
	double dist = 0;

	for(int i=0; i<dim; i++) point[i] = p[i];

	s1953(pc, point, dim, epsco, epsge, &numintpt, &intpar, &numintcu, &intcurve, &stat);
	s1363(pc, &sp, &ep, &stat);

	delete[] point;

	return (intpar[0]-sp)/(ep-sp);
}

std::vector<double> SPLINE::getPoint(double u)
{
	double sp, ep;
	int stat;
	double param;
	double *point = new double[dim];
	int leftknot;

	std::vector<double> result;

	if(u<0.0f)
	{
		std::cout << "Warning: curve parameter out of range (0.0>)" << std::endl;
		u = 0.0f;
	}
	else if(u>1.0f)
	{
		std::cout << "Warning: curve parameter out of range (1.0<)" << std::endl;
		u = 1.0f;
	}

	// get parameter range value
	s1363(pc, &sp, &ep, &stat);

	//std::cout << "stat: " << stat << std::endl;

	// get points
	param = sp + u*(ep-sp);
	s1227(pc, 0, param, &leftknot, point, &stat);

	//std::cout << "u: " << u << " p: " << param << " sp: " << sp << " ep: " << ep << std::endl;

	if(stat<0)
	{
		std::cout << "Error : getPoint" << std::endl;
		cin.get();
	}
	else if(stat>0)
	{
		std::cout << "Warning : getPoint" << std::endl;
		cin.get();
	}

	for(int i=0; i<dim; i++)
		result.push_back(point[i]);

	delete[] point;

	return result;
}

std::vector<double> SPLINE::getClosestPoint(const std::vector<double>& p)
{
	double *point = new double[dim];
	double epsco=0.0001;
	double epsge = geomResolution;
	int numintpt;
	double *intpar;
	int numintcu;
	SISLIntcurve **intcurve;
	int stat;
	double sp, ep;

	double gpar = 0;
	double dist = 0;

	for(int i=0; i<dim; i++) point[i] = p[i];
	//s1957(pc, point, dim, epsco, epsge, &gpar, &dist, &jstat);
	s1953(pc, point, dim, epsco, epsge, &numintpt, &intpar, &numintcu, &intcurve, &stat);
	s1363(pc, &sp, &ep, &stat);

	delete[] point;

	if(stat<0)
	{
		std::cout << "Error : getClosestPoint" << std::endl;
		cin.get();
	}
	else if(stat>0)
	{
		std::cout << "Warning : getClosestPoint" << std::endl;
		cin.get();
	}

	return getPoint((intpar[0]-sp)/(ep-sp));
}

std::vector<double> SPLINE::getTangent(const std::vector<double>& p, int der)
{
	std::vector<double> result;

	double parvalue;
	int leftknot;
	double *derive = new double[3*dim];
	int stat=0;

	double *point = new double[dim];
	double epsco = 0.0001;
	double epsge = geomResolution;
	int numintpt;
	double *intpar=0;
	int numintcu;
	SISLIntcurve **intcurve;

	double gpar=0;
	double dist=0;

	//cout << "gt0" << " ";

	/* get closest pont */
	for(int i=0; i<dim; i++) point[i] = p[i];

	//cout << "gt1" << " " << point[0] << " " << point[1] << " " << point[2] << " ";

	s1953(pc, point, dim, epsco, epsge, &numintpt, &intpar, &numintcu, &intcurve, &stat);
	//s1957(pc, point, dim, epsco, epsge, &gpar, &dist, &stat);

	//cout << "gt2" << " " << numintpt << " " << intpar[0] << " ";

	if(stat<0)
	{
		std::cout << "Error : getTangent" << std::endl;
		cin.get();
	}
	else if(stat>0)
	{
		std::cout << "Warning : getTangent" << std::endl;
		cin.get();
	}

	
	if(numintpt==0)
	{
		std::cout << "Error : getTangent - no close point" << std::endl;
		cin.get();
	}

	/* get tangent */
	s1221(pc, der, intpar[0], &leftknot, derive, &stat); // right hand
	//s1221(pc, der, gpar, &leftknot, derive, &stat); // right hand
	//s1227(pc, der, intpar[0], &leftknot, derive, &stat); // left hand

	//cout << "gt3" << " ";

	switch(der)
	{
	case 0:
		for(int i=0; i<1*dim; i++) result.push_back(derive[i]);
		break;
	case 1:
		for(int i=0; i<2*dim; i++) result.push_back(derive[i]);
		break;
	case 2:
		for(int i=0; i<3*dim; i++) result.push_back(derive[i]);
		break;
	}

	//cout << "gt4" << " ";

	delete[] derive;
	delete[] point;

	if(stat<0)
	{
		std::cout << "Error : getTangent" << std::endl;
		cin.get();
	}
	else if(stat>0)
	{
		std::cout << "Warning : getTangent" << std::endl;
		cin.get();
	}

	return result;
}

std::vector<double> SPLINE::getFrentFrame(const std::vector<double>& p)
{
	std::vector<double> result;

	double *point = new double[dim];
	double epsco = 0.0001;
	double epsge = geomResolution;
	int numintpt;
	double *intpar;
	int numintcu;
	SISLIntcurve **intcurve;
	int stat;

	double pp[3];
	double t[3];
	double n[3];
	double b[3];

	/* get closest pont */
	for(int i=0; i<dim; i++) point[i] = p[i];
	s1953(pc, point, dim, epsco, epsge, &numintpt, &intpar, &numintcu, &intcurve, &stat);

	/* get frent frame */
	s2559(pc, intpar, 1, pp, t, n, b, &stat);

	for(int i=0; i<dim; i++) result.push_back(t[i]);
	for(int i=0; i<dim; i++) result.push_back(n[i]);
	for(int i=0; i<dim; i++) result.push_back(b[i]);

	delete[] point;

	return result;
}

float SPLINE::getCurveLength()
{
	if(pc==NULL) return 0.0f;

	double epsge = geomResolution;
	double length;
	int stat;

	s1240(pc, epsge, &length, &stat);

	if(stat<0)
	{
		std::cout << "Error : getCurveLength" << std::endl;
		return false;
	}
	else if(stat>0)
	{
		std::cout << "Warning : getCurveLength" << std::endl;
		return false;
	}

	return length;
}

float SPLINE::getCurvature(const std::vector<double>& p)
{
	double point[3];
	double epsco = 0.0001;
	double epsge = geomResolution;
	int numintpt;
	double *intpar;
	int numintcu;
	SISLIntcurve **intcurve;
	int stat;
	double sp, ep;

	double gpar = 0;
	double dist = 0;

	point[0] = p[0]; point[1] = p[1]; point[2] = p[2];
	s1953(pc, point, dim, epsco, epsge, &numintpt, &intpar, &numintcu, &intcurve, &stat);

	double curvature;
	s2550(pc, intpar, 1, &curvature, &stat);

	return curvature;
}

