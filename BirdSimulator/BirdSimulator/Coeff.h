#pragma once

#include <fstream>
#include "util.h"

using namespace std;

struct LinkCoeff
{
	bool loaded;

	double pdGain[8];
	double spring[4];

	LinkCoeff()
	{
		loaded = false;
	}

	void load(string fileName)
	{
		ifstream fin;
		util::fileOpenWrapper(fin, fileName);

		float value;

		for(int i=0; i<4; i++)
		{
			fin >> pdGain[2*i];
			fin >> pdGain[2*i+1];
		}

		for(int i=0; i<2; i++)
		{
			fin >> spring[2*i];
			fin >> spring[2*i+1];
		}

		util::fileCloseWrapper(fin);

		loaded = true;
	}

	void save(ofstream& fout)
	{

	}
};

struct FemMeshCoeff
{
	bool loaded;

	/* primary */

	// density of feather
	double density_primary;
	// height of feather
	double height_primary;
	// stretching stiffness of feather
	double stretch_stiff_primary;
	// number of modes for simulation
	double num_mode_primary;
	// bending stiffness of feather near to body
	double bend_stiff_near_primary;
	// bending stifnness of feather far to body
	double bend_stiff_far_primary;
	// bending damping for simulation
	double bend_damping_primary;
	// twisting stiffness of feather
	double twist_stiff_primary;
	// tiwsting damping for simulation
	double twist_damping_primary;

	/* secondary */

	double density_secondary;
	double height_secondary;
	double stretch_stiff_secondary;
	double num_mode_secondary;
	double bend_stiff_near_secondary;
	double bend_stiff_far_secondary;
	double bend_damping_secondary;
	double twist_stiff_secondary;
	double twist_damping_secondary;

	/* tail */
	double density_tail;
	double height_tail;
	double stretch_stiff_tail;
	double num_mode_tail;
	double bend_stiff_near_tail;
	double bend_stiff_far_tail;
	double bend_damping_tail;
	double twist_stiff_tail;
	double twist_damping_tail;

	FemMeshCoeff()
	{
		loaded = false;
	}

	void load(string fileName)
	{
		ifstream fin;
		util::fileOpenWrapper(fin, fileName);

		// primary
		fin >> density_primary;
		fin >> height_primary;
		fin >> stretch_stiff_primary;
		fin >> num_mode_primary;
		fin >> bend_stiff_near_primary;
		fin >> bend_stiff_far_primary;
		fin >> bend_damping_primary;
		fin >> twist_stiff_primary;
		fin >> twist_damping_primary;
		
		// secondary
		fin >> density_secondary;
		fin >> height_secondary;
		fin >> stretch_stiff_secondary;
		fin >> num_mode_secondary;
		fin >> bend_stiff_near_secondary;
		fin >> bend_stiff_far_secondary;
		fin >> bend_damping_secondary;
		fin >> twist_stiff_secondary;
		fin >> twist_damping_secondary;

		// tail
		fin >> density_tail;
		fin >> height_tail;
		fin >> stretch_stiff_tail;
		fin >> num_mode_tail;
		fin >> bend_stiff_near_tail;
		fin >> bend_stiff_far_tail;
		fin >> bend_damping_tail;
		fin >> twist_stiff_tail;
		fin >> twist_damping_tail;

		util::fileCloseWrapper(fin);

		loaded = true;
	}

	void save(ofstream& fout)
	{
		// primary
		fout << density_primary << endl;
		fout << height_primary << endl;
		fout << stretch_stiff_primary << endl;
		fout << num_mode_primary << endl;
		fout << bend_stiff_near_primary << endl;
		fout << bend_stiff_far_primary << endl;
		fout << bend_damping_primary << endl;
		fout << twist_stiff_primary << endl;
		fout << twist_damping_primary << endl;
		fout << endl;
		
		// secondary
		fout << density_secondary << endl;
		fout << height_secondary << endl;
		fout << stretch_stiff_secondary << endl;
		fout << num_mode_secondary << endl;
		fout << bend_stiff_near_secondary << endl;
		fout << bend_stiff_far_secondary << endl;
		fout << bend_damping_secondary << endl;
		fout << twist_stiff_secondary << endl;
		fout << twist_damping_secondary << endl;
		fout << endl;

		// tail
		fout << density_tail << endl;
		fout << height_tail << endl;
		fout << stretch_stiff_tail << endl;
		fout << num_mode_tail << endl;
		fout << bend_stiff_near_tail << endl;
		fout << bend_stiff_far_tail << endl;
		fout << bend_damping_tail << endl;
		fout << twist_stiff_tail << endl;
		fout << twist_damping_tail << endl;
		fout << endl;
	}
};

struct FeatherAngleCoeff
{
	bool loaded;

	// 0 : priamary
	// 1 : secondary
	// 2 : tail
	double feather_spread_from[3];
	double feather_spread_to[3];
	double feather_bend_from[3];
	double feather_bend_to[3];

	FeatherAngleCoeff()
	{
		loaded = false;
	}

	void load(string fileName)
	{
		ifstream fin;
		util::fileOpenWrapper(fin, fileName);

		for(int i=0; i<3; i++)
		{
			fin >> feather_spread_from[i];
			fin >> feather_spread_to[i];
			fin >> feather_bend_from[i];
			fin >> feather_bend_to[i];
		}

		util::fileCloseWrapper(fin);

		loaded = true;
	}

	void save(ofstream& fout)
	{
		for(int i=0; i<3; i++)
		{
			fout << feather_spread_from[i] << endl;
			fout << feather_spread_to[i] << endl;
			fout << feather_bend_from[i] << endl;
			fout << feather_bend_to[i] << endl;
		}
	}
};

class Coeff
{
public:
	LinkCoeff*			linkCoeff;
	FemMeshCoeff*		femMeshCoeff;
	FeatherAngleCoeff*	featherAngleCoeff;

	Coeff(string fileLinkCoeff, string fileFemMeshCoeff, string fileFeatherAngleCoeff)
	{
		linkCoeff			= new LinkCoeff();
		femMeshCoeff		= new FemMeshCoeff();
		featherAngleCoeff	= new FeatherAngleCoeff();

		linkCoeff->load(fileLinkCoeff);
		femMeshCoeff->load(fileFemMeshCoeff);
		featherAngleCoeff->load(fileFeatherAngleCoeff);
	}

	~Coeff()
	{
		delete linkCoeff;
		delete femMeshCoeff;
		delete featherAngleCoeff;
	}
};
