# pragma once


#include "MATHCLASS/mathclass.h"
#include "GL/GLUT.h"

struct DIB2D
{
	BITMAPINFOHEADER	*Info;
	RGBQUAD					*palette;
	BYTE							*bits;
};

struct GLTXTLOAD
{
	GLint				format;
	GLint				perpixel;
	GLint				Width;
	GLint				Height;
	BYTE*			bits;
};

class SetDisplay
{
public:
	bool		LoadDIB(char*file,DIB2D*dib);
	long		ScanBytes(int pixWidth, int bitsPixel);
	bool		ScaleImage(DIB2D&dib,GLTXTLOAD&p);
	bool		LoadTexture(char*filename);
};

class MeshModel
{
public:
	const char*					model_name;
	
	int								index_N;			
	int								vertex_N;			
	int								normal_N;	
	int								texture_N;

	float							scale;
	GLuint							texID;
	//GLuint							floorID;
	GLuint							modelList;

	bool							vertex_check;

	int**							index;
	float**							vertex;
	float**							normal;
	float**							texture;

	float							sizeBB[2];
	float							norm_term;

	MeshModel(const char* name, float s = 1.0f);
	
	void								genDrawList();
	void								drawModel();

	void								createModel(const char* name);
	void								createAndCopyModel(const char* name, MeshModel* mm);

	void								setColor(float r, float g, float b);
	void								setPosition(float x, float y, float z);
	void								setOrientation(float x_axis, float y_axis, float z_axis);			// degree
	void								setScaling(float x, float y, float z);

	void								calcBB();
	void								normalizeMesh();

	void								loadVertex();
	void								loadNormal();
	void								loadIndex();
	void								loadTexture();

	void								copyVertex(MeshModel* mm);
	void								copyNormal(MeshModel* mm);
	void								copyIndex(MeshModel* mm);
	void								copyTexture(MeshModel* mm);
	
	void								displayModel();
	void								displayModel2();
	void								displayModel3();

	void								setTexture();
};