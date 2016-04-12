
#include <iostream>
#include <fstream>
#include <windows.h>

#include <sys/timeb.h>
#include <time.h>

#include "definitions.h"
#include "util.h"
#include "MeshModel.h"

using namespace std;

bool
SetDisplay::LoadDIB(char*file,DIB2D*dib)
{
	bool result=FALSE;
	HANDLE hfile=CreateFile(file,GENERIC_READ,FILE_SHARE_READ,NULL,OPEN_EXISTING ,FILE_ATTRIBUTE_NORMAL,0);
	if(hfile!=INVALID_HANDLE_VALUE)
	{
		DWORD readed;
		int size=GetFileSize(hfile,0);
		if(size>sizeof(BITMAPFILEHEADER))
		{
			BITMAPFILEHEADER bmfh;
			ReadFile(hfile,&bmfh,sizeof(BITMAPFILEHEADER),&readed,0);
			if((readed==sizeof(BITMAPFILEHEADER)) && (bmfh.bfType==0x4d42))
			{
				dib->Info=(BITMAPINFOHEADER*)(new BYTE[size-sizeof(BITMAPFILEHEADER)]);
				ReadFile(hfile,dib->Info,size-sizeof(BITMAPFILEHEADER),&readed,0);
				dib->bits=(BYTE*)(dib->Info+1);

				if(dib->Info->biBitCount==8)
				{
					dib->palette=(RGBQUAD*)dib->bits;
					if(dib->Info->biClrUsed) dib->bits+=dib->Info->biClrUsed*4;
					else dib->bits+=1024;
				}
				else dib->palette=NULL;
				result=TRUE;
			}
		}
		CloseHandle(hfile);
	}
	return result;
}
long
SetDisplay::ScanBytes(int pixWidth, int bitsPixel)
{
  return (((long)pixWidth*bitsPixel+31) / 32) * 4;
}
bool
SetDisplay::ScaleImage(DIB2D&dib,GLTXTLOAD&p)
{
	GLint glMaxTexDim;     // OpenGL maximum texture dimension
	GLint XDMaxTexDim=512; // user maximum texture dimension
	GLint minsize =2;
	double xPow2, yPow2;
	int ixPow2, iyPow2;
	int xSize2, ySize2;
	GLint m_iWidth=dib.Info->biWidth;
	GLint m_iHeight=dib.Info->biHeight;
	glGetIntegerv(GL_MAX_TEXTURE_SIZE, &glMaxTexDim);

	glMaxTexDim = min(XDMaxTexDim, glMaxTexDim);

	if (m_iWidth <= glMaxTexDim) xPow2 = log((double)m_iWidth) / log(2.0);
	else xPow2 = log((double)glMaxTexDim) / log(2.0);
	if (m_iHeight <= glMaxTexDim) yPow2 = log((double)m_iHeight) / log(2.0);
	else yPow2 = log((double)glMaxTexDim) / log(2.0);

	ixPow2 = (int)xPow2;
	iyPow2 = (int)yPow2;

	if (xPow2 != (double)ixPow2) ixPow2++;
	if (yPow2 != (double)iyPow2) iyPow2++;

	xSize2 = 1 << ixPow2;
	ySize2 = 1 << iyPow2;

	if(xSize2<minsize) xSize2=minsize;
	if(ySize2<minsize) ySize2=minsize;

	if(((xSize2==m_iWidth) && (ySize2==m_iHeight)))
	{
		if(dib.Info->biBitCount==24)
		{
			p.format=GL_BGR_EXT;
			p.perpixel=3;
			return FALSE;
		}
		if(dib.Info->biBitCount==32)
		{
			p.format=GL_BGRA_EXT;
			p.perpixel=4;
			return FALSE;
		}
	}

	BYTE *bits=(BYTE *)dib.bits;
	if(dib.Info->biBitCount==8)
	{
		// convert to TRUECOLOR
		int _perline=ScanBytes(8,m_iWidth);
		int perline=ScanBytes(24,m_iWidth);
		bits= new BYTE[perline*m_iHeight * sizeof(BYTE)];
		for(int y=0;y<m_iHeight;y++)
		{
			BYTE *_b=((BYTE *)dib.bits)+y*_perline;
			BYTE *b=bits+y*perline;
			for(int x=0;x<m_iWidth;x++)
			{
				RGBQUAD _p=dib.palette[*_b];
				_b++;
				*b=_p.rgbBlue;b++;
				*b=_p.rgbGreen;b++;
				*b=_p.rgbRed;b++;
			}
		}
	}
	BOOL isAlpha=(dib.Info->biBitCount==32);
	int _mem_size=xSize2 * ySize2 *  sizeof(BYTE);
	if(isAlpha)
	{
		_mem_size*=4;
		p.perpixel=4;
		p.format=GL_BGRA_EXT;
	}
	else
	{
		_mem_size*=3;
		p.perpixel=3;
		p.format=GL_BGR_EXT;
	}
	BYTE *pData = (BYTE*)new BYTE[_mem_size];
	if (!pData) return FALSE;

	if(isAlpha) gluScaleImage(GL_BGRA_EXT, m_iWidth, m_iHeight, GL_UNSIGNED_BYTE, bits, xSize2, ySize2, GL_UNSIGNED_BYTE, pData);
	else gluScaleImage(GL_RGB, m_iWidth, m_iHeight, GL_UNSIGNED_BYTE, bits, xSize2, ySize2, GL_UNSIGNED_BYTE, pData);

	if(bits!=dib.bits)delete bits;
	m_iWidth = xSize2 ;
	m_iHeight = ySize2 ;
	p.Width=m_iWidth;
	p.Height=m_iHeight;
	p.bits=pData;

	return TRUE ;
}
bool 
SetDisplay::LoadTexture(char*filename)
{
	DIB2D dib;
	GLTXTLOAD load;

	if(LoadDIB(filename,&dib))
	{
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

		if(ScaleImage(dib,load))
		{
			glTexImage2D(GL_TEXTURE_2D,0,load.perpixel,
			load.Width,load.Height,0,
			load.format,GL_UNSIGNED_BYTE,
			load.bits);
			delete load.bits;
		}
		else
		{
			glTexImage2D(GL_TEXTURE_2D,0,load.perpixel,
			dib.Info->biWidth,dib.Info->biHeight,
			0,load.format,GL_UNSIGNED_BYTE,dib.bits);
		}
		delete dib.Info;

		return true;
	}

	return false;
}



MeshModel::MeshModel(const char* name, float s)
{
	scale = s;
	norm_term = INFINITE;

	createModel(name);

	vertex_check = false;

	modelList = -1;
	genDrawList();
}

void
MeshModel::genDrawList()
{
	// pearl
	float kd[4] = {0.25,		0.20725,	0.20725,	0.75};
	float ka[4] = {1.0,			0.829,		0.829,		0.75};
	float ks[4] = {0.296648,	0.296648,	0.296648,	0.75};
	float exponent = 11.264;

	if(modelList == -1)
	{
		modelList = glGenLists(1);

		cout << modelList << endl;

		glNewList(modelList, GL_COMPILE);

			displayModel();

		glEndList();
	}
}

void
MeshModel::drawModel()
{
	// pearl
	float kd[4] = {0.25,		0.20725,	0.20725,	0.75};
	float ka[4] = {1.0,			0.829,		0.829,		0.75};
	float ks[4] = {0.296648,	0.296648,	0.296648,	0.75};
	float exponent = 11.264;

	//if(modelList != -1)
	{
		setTexture();

		glDisable(GL_CULL_FACE);
		//glEnable(GL_BLEND);
		glDisable(GL_BLEND);
		glEnable(GL_TEXTURE_2D);

		glBindTexture(GL_TEXTURE_2D,texID);

#ifdef PEACOCK
		glSetMaterial(5/255.0,60/255.0,120/255.0, 0.7);
#else
		glSetMaterial(kd, ka, ks, exponent);
#endif
		
		//glCallList(modelList);
		displayModel();

		glDisable(GL_TEXTURE_2D);
		glDisable(GL_BLEND);
		glEnable(GL_BLEND);
		glEnable(GL_CULL_FACE);
	}
}

void
MeshModel::loadVertex()
{
	ifstream		fin;
	char	cName[100];
	char ch[100];
	int i, j;

	sprintf(cName,"model/songbird/%s_vertex.txt",model_name);
	fin.open(INFORM_DIR+string(cName));
	printf("name : %s %d\n",cName,fin.is_open());
	fin >> ch;	vertex_N = atoi(ch);	
	vertex		= new float*[vertex_N];

	float mean[3] = {0.0f, 0.0f, 0.0f};
	
	for(i=0;i<vertex_N;i++)
	{
		vertex[i] = new float[3];
		for(j=0;j<3;j++)				{	fin >> ch;	vertex[i][j] = (float)atof(ch);	mean[j] += vertex[i][j];}
	}

	for(i=0;i<vertex_N;i++)	
	{
		vertex[i][1] -= 0.05f;

		for(j=0;j<3;j++)	vertex[i][j] *= scale;
	}

	fin.close();		fin.clear();	
}
void
MeshModel::loadNormal()
{
	ifstream		fin;
	char	cName[100];
	char ch[100];
	int i, j;

	sprintf(cName,"model/songbird/%s_normal.txt",model_name);
	fin.open(INFORM_DIR+string(cName));
	printf("name : %s %d\n",cName, fin.is_open());
	fin >> ch;	normal_N = atoi(ch);	
	normal		= new float*[normal_N];
	for(i=0;i<normal_N;i++)
	{
		normal[i] = new float[3];
		for(j=0;j<3;j++)					{	fin >> ch;	normal[i][j] = (float)atof(ch);	}
	}
	fin.close();		fin.clear();
}
void
MeshModel::loadIndex()
{
	ifstream		fin;
	char	cName[100];
	char ch[100];
	int i, j;

	sprintf(cName,"model/songbird/%s_index.txt",model_name);
	fin.open(INFORM_DIR+string(cName));
	fin.open(cName);
	printf("name : %s %d\n",cName, fin.is_open());
	fin >> index_N;	
	//index_N = 7465;
	
	index		= new int*[index_N];
	for(i=0;i<index_N;i++)
	{
		index[i] = new int[9];
		for(j=0;j<9;j++)					{	fin >> ch;	index[i][j] = atoi(ch);	}

	}
	fin.close();		fin.clear();
}
void
MeshModel::loadTexture()
{
	ifstream		fin;
	char	cName[100];
	char ch[100];
	int i, j;

	sprintf(cName,"model/songbird/%s_texture.txt",model_name);
	fin.open(INFORM_DIR+string(cName));
	fin.open(cName);
	printf("name : %s %d\n",cName, fin.is_open());
	fin >> ch;	texture_N = atoi(ch);//	5000;//10000;//
	texture		= new float*[texture_N];
	for(i=0;i<texture_N;i++)
	{
		texture[i] = new float[2];
		for(j=0;j<2;j++)					{	fin >> ch;	texture[i][j] = atof(ch);	}//	cout << texture[i][j] << ",";	}
	}
	
	fin.close();		fin.clear();
}
void
MeshModel::copyVertex(MeshModel *mm)
{
	int i, j;

	vertex_N		= mm->vertex_N;
	vertex		= new float*[vertex_N];
	for(i=0;i<vertex_N;i++)
	{
		vertex[i] = new float[3];
		for(j=0;j<3;j++)				vertex[i][j] = mm->vertex[i][j];	
	}
}
void
MeshModel::copyNormal(MeshModel *mm)
{
	int i, j;

	normal_N = mm->normal_N;
	normal = mm->normal;
}
void
MeshModel::copyIndex(MeshModel *mm)
{
	int i, j;

	index_N = mm->index_N;
	index = mm->index;
}
void
MeshModel::copyTexture(MeshModel *mm)
{
	int i, j;

	texture_N = mm->texture_N;
	texture = mm->texture;
}
void
MeshModel::createModel(const char* name)
{
	model_name = name;

	loadVertex();
	loadNormal();
	loadIndex();
	loadTexture();
	
	//normalizeMesh();
	calcBB();
}
void
MeshModel::createAndCopyModel(const char* name, MeshModel* mm)
{
	model_name = name;

	loadVertex();
	copyNormal(mm);
	copyIndex(mm);
	copyTexture(mm);

	texID	= mm->texID;
	
	//norm_term = mm->norm_term;

	//normalizeMesh();
	calcBB();
}
void
MeshModel::calcBB()
{
	int i;
	float min, max;

	min =INFINITE;
	max = -1.0*INFINITE;

	for(i=0;i<vertex_N;i++)
	{
		if(vertex[i][0] < min) min = vertex[i][0];
		if(vertex[i][0] > max) max = vertex[i][0];
	}

	sizeBB[0] = max*4+2;

	min =INFINITE;
	max = -1.0*INFINITE;

	for(i=0;i<vertex_N;i++)
	{
		if(vertex[i][2] < min) min = vertex[i][2];
		if(vertex[i][2] > max) max = vertex[i][2];
	}

	sizeBB[1] = max*4+2;
}
void
MeshModel::normalizeMesh()
{
	int i;
	float min;

	if(norm_term == INFINITE)
	{
		norm_term = INFINITE;

		for(i=0;i<vertex_N;i++)
			if(vertex[i][1] < norm_term)		norm_term = vertex[i][1];

		norm_term *= -1.0f;
	}

	for(i=0;i<vertex_N;i++)
		vertex[i][1] = vertex[i][1] + norm_term;

}
void
MeshModel::displayModel()
{
	int i,j,vi,ni,ti;

	//int vv[40] = {0,5,2,6,1,28,32,36,37,33,29,25,138,124,53,354,355,98,3,4,
	//	392,388,384,357,362,358,361,356,446,669,668,407,468,480,381,385,389,393,359,360};
	//int nn[40];
	//int tt[40];

	glBegin (GL_TRIANGLES);
	for(i=0;i<index_N;i++)
	for(j=0;j<3;j++)
	{
		vi	=	index[i][j];
		ni	=	index[i][j+3];
		ti	=	index[i][j+6];

		////cout << normal_N << "," << ni << endl;

		//if(vi==0||vi==5||vi==2||vi==6||vi==1||vi==28||vi==32||vi==36||vi==37||vi==33||vi==29||vi==25||vi==138||vi==124||vi==53||vi==354||vi==355||vi==98)
		//	cout << vi << " " << ni << " " << ti << endl;
		
		/*for(int k=0;k<40;k++)
		{
			if(vv[k] == vi)
			{
				nn[k] = ni;
				tt[k] = ti;
			}
		}*/
		glTexCoord2f(texture[ti][0],texture[ti][1]);
		glNormal3f (normal[ni][0],normal[ni][1],normal[ni][2]);
		glVertex3f (vertex[vi][0],vertex[vi][1],vertex[vi][2]);
	}

    glEnd ();

	//print
	/*ofstream fout("songbird_temp.txt");

	for(i=0;i<36;i++)
		fout << vv[i] << " " << nn[i] << " " << tt[i] << endl;

	fout.close();
	fout.clear();*/

	/*int nnSet[3];
	int ttSet[3];
	int vvSet[3];

	ofstream fout("songbird_temp3.txt");
	ifstream fin("songbird_temp2.txt");

	for(j=0;j<40;j++)
	{
		for(i=0;i<3;i++)
		{
			fin >> vvSet[i];

			for(int k=0;k<40;k++)
			{
				if(vv[k] == vvSet[i])	
				{
					nnSet[i] = nn[k];
					ttSet[i] = tt[k];
					break;
				}
			}
		}
		
		fout << vvSet[0] << " " << vvSet[1] << " " << vvSet[2] << " ";
		fout << nnSet[0] << " " << nnSet[1] << " " << nnSet[2] << " ";
		fout << ttSet[0] << " " << ttSet[1] << " " << ttSet[2] << " ";

		fout << endl;
	}*/
	
	//fout.close();
	//fout.clear();
	//fin.close();
	//fin.clear();


	// test
	//for(i=0;i<8996;i++)
	//{
	//	//glSetMaterial(1,0,0);
	//	//if(i>=250 && i<500)	
	//	if(i<1000)
	//	for(j=0;j<3;j++)
	//	{
	//		if(i%6==0)	glSetMaterial(1,0,0);
	//		if(i%6==1)	glSetMaterial(0,1,0);
	//		if(i%6==2)	glSetMaterial(0,0,1);
	//		if(i%6==3)	glSetMaterial(1,1,0);
	//		if(i%6==4)	glSetMaterial(0,1,1);
	//		if(i%6==5)	glSetMaterial(1,0,1);

	//		//cout << normal_N << "," << ni << endl;

	//		//glTexCoord2f(texture[ti][0],texture[ti][1]);
	//		//glNormal3f (normal[ni][0],normal[ni][1],normal[ni][2]);
	//		glSphere(vertex[i][0],vertex[i][1],vertex[i][2],0.0005f);
	//	}
	//}


}
void
MeshModel::setTexture()
{
	char name[100];
	SetDisplay* setTexture = new SetDisplay;

	glGenTextures(1,&texID);

	sprintf(name,"%smodel/songbird/%s_gray4.bmp",INFORM_DIR,model_name);

	glBindTexture(GL_TEXTURE_2D,texID);

	setTexture->LoadTexture(name);

	glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
}