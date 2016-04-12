#include "RenderBird.h"
#include "MeshModel.h"
#include <fstream>

using namespace std;

extern Bird*				simBird;
extern MeshModel*			meshModel;

void
RenderBird::initializeFromCurrentBird()
{
	current.initialize();

	vector3 tempV;
	
	current.shoulder[0].init = simBird->birdBody->length[Body::Trunk]/2.0f;
	current.shoulder[1].init = simBird->birdBody->length[Body::Trunk]/2.0f;

	tempV = vector3(1.0f, simBird->birdBody->wingPosition[0], simBird->birdBody->wingPosition[1]);
	for(int i=0;i<3;i++)		current.shoulder[0].init[i] *= tempV[i];

	tempV = vector3(-1.0f, simBird->birdBody->wingPosition[0], simBird->birdBody->wingPosition[1]);
	for(int i=0;i<3;i++)		current.shoulder[1].init[i] *= tempV[i];

	current.elbow[0].init.set_x( simBird->birdBody->length[Body::Lwing1][axis_X] );
	current.elbow[1].init.set_x( -simBird->birdBody->length[Body::Rwing1][axis_X] );

	current.wrist[0].init.set_x( simBird->birdBody->length[Body::Lwing2][axis_X] );
	current.wrist[1].init.set_x( -simBird->birdBody->length[Body::Rwing2][axis_X] );

	int Left	= 0;
	int Right	= 1;

	for(int i=0;i<current.feathers.size();i++)
	for(int j=0;j<current.feathers[i].fjoint.size();j++)
	for(int k=0;k<current.feathers[i].fjoint[j].joints.size();k++)
	{
		vector3 v1(0,0,0);

		switch(i)
		{
			case Feathers::PrimL:	v1[0] = simBird->birdBody->length[Body::Lwing3][0]/2.0f;		break;
			case Feathers::PrimR:	v1[0] = -simBird->birdBody->length[Body::Lwing3][0]/2.0f;		break;
			case Feathers::SeconL:	v1[0] = simBird->birdBody->length[Body::Lwing2][0]/2.0f;		break;
			case Feathers::SeconR:	v1[0] = -simBird->birdBody->length[Body::Lwing2][0]/2.0f;		break;
			case Feathers::TertL:	v1[0] = simBird->birdBody->length[Body::Lwing1][0]/2.0f;		break;
			case Feathers::TertR:	v1[0] = -simBird->birdBody->length[Body::Lwing1][0]/2.0f;		break;
			case Feathers::Tail:	v1[2] = -simBird->birdBody->length[Body::Trunk][2]/2.0f;		break;
		}

		vector3 v2 = simBird->birdFeathers->featherGroup[i]->each[j]->linkToFeatherTransf.getTranslation();

		if(k==0)	v2[2] = simBird->birdFeathers->featherGroup[i]->each[j]->init[k*2]->pos[0][2];
		else if(k==current.feathers[i].fjoint[j].joints.size()-1)
					v2[2] = simBird->birdFeathers->featherGroup[i]->each[j]->init[k*2]->pos[1][2]-simBird->birdFeathers->featherGroup[i]->each[j]->init[(k-1)*2]->pos[0][2];
		else
					v2[2] = simBird->birdFeathers->featherGroup[i]->each[j]->init[k*2]->pos[0][2]-simBird->birdFeathers->featherGroup[i]->each[j]->init[(k-1)*2]->pos[0][2];
		
		if(k==0)
			current.feathers[i].fjoint[j].joints[k].init = v1 + v2;
		else 
			current.feathers[i].fjoint[j].joints[k].init = vector3(0,0,v2[axis_Z]);

		current.feathers[i].fjoint[j].joints[k].pos.clear();

		if(k==current.feathers[i].fjoint[j].joints.size()-1)
		{
			vector3 pos[3];

			float trans = simBird->birdFeathers->featherGroup[i]->each[j]->init[k*2]->pos[1][axis_Z];

			for(int s=0;s<3;s++)	
			{
				pos[s] = position2vector(simBird->birdFeathers->featherGroup[i]->each[j]->init[k*2]->pos[s]);
				pos[s][axis_Z] -= trans;
				current.feathers[i].fjoint[j].joints[k].pos.push_back(pos[s]);
			}
		}
		else
		{
			vector3 pos[4];

			float trans = simBird->birdFeathers->featherGroup[i]->each[j]->init[k*2]->pos[0][axis_Z];

			pos[0] = position2vector(simBird->birdFeathers->featherGroup[i]->each[j]->init[k*2]->pos[0]);
			pos[1] = position2vector(simBird->birdFeathers->featherGroup[i]->each[j]->init[k*2]->pos[1]);
			pos[2] = position2vector(simBird->birdFeathers->featherGroup[i]->each[j]->init[k*2+1]->pos[0]);
			pos[3] = position2vector(simBird->birdFeathers->featherGroup[i]->each[j]->init[k*2+1]->pos[1]);

			for(int s=0;s<4;s++)	
			{
				pos[s][axis_Z] -= trans;
				current.feathers[i].fjoint[j].joints[k].pos.push_back(pos[s]);
			}
		}
	}
	
	saveTemplate();
}
void
RenderBird::initializeFromTemplateFile()
{
	current.initialize();
	loadTemplate();
}
void
RenderBird::update()
{
	current.initializeOri();

	updateBody();
	updateFeather();
}
void
RenderBird::saveTemplate()
{
	ofstream fout;

	fout.open("bird_template.txt");

	WRITE_ARRAY3(fout, current.root.init);

	for(int i=0;i<2;i++)
	{
		WRITE_ARRAY3(fout, current.shoulder[i].init);
		WRITE_ARRAY3(fout, current.elbow[i].init);
		WRITE_ARRAY3(fout, current.wrist[i].init);
	}

	for(int i=0;i<current.feathers.size();i++)
	for(int j=0;j<current.feathers[i].fjoint.size();j++)
	for(int k=0;k<current.feathers[i].fjoint[j].joints.size();k++)
	{
		WRITE_ARRAY3(fout, current.feathers[i].fjoint[j].joints[k].init);

		fout << current.feathers[i].fjoint[j].joints[k].pos.size() << endl;
		for(int s=0;s<current.feathers[i].fjoint[j].joints[k].pos.size();s++)
			WRITE_ARRAY3(fout, current.feathers[i].fjoint[j].joints[k].pos[s]);
	}

	fout.close();
	fout.clear();
}
void
RenderBird::loadTemplate()
{
	ifstream fin;

	fin.open("bird_template.txt");

	READ_ARRAY3(fin, current.root.init);

	for(int i=0;i<2;i++)
	{
		READ_ARRAY3(fin, current.shoulder[i].init);
		READ_ARRAY3(fin, current.elbow[i].init);
		READ_ARRAY3(fin, current.wrist[i].init);
	}

	for(int i=0;i<current.feathers.size();i++)
	for(int j=0;j<current.feathers[i].fjoint.size();j++)
	for(int k=0;k<current.feathers[i].fjoint[j].joints.size();k++)
	{
		READ_ARRAY3(fin, current.feathers[i].fjoint[j].joints[k].init);

		int num;	fin >> num;

		current.feathers[i].fjoint[j].joints[k].pos.clear();

		for(int s=0;s<num;s++)
		{
			vector3 pp;
			READ_ARRAY3(fin, pp);

			current.feathers[i].fjoint[j].joints[k].pos.push_back(pp);
		}
	}

	fin.close();
	fin.clear();
}
void
RenderBird::updateBody()
{
	current.root.updateStateBack(position2vector(simBird->birdBody->currentS.trunkPosition), simBird->birdBody->currentS.trunkOrientation);

	for(int i=0;i<2;i++)
	{
		current.shoulder[i].updateStateBack(current.root.gPos, current.root.rot);
		current.shoulder[i].setShoulderRotation( simBird->birdBody->currentS.trunkOrientation.inverse() * simBird->birdBody->currentS.shoulder[i] );
	
		current.elbow[i].updateStateBack(current.shoulder[i].gPos, current.shoulder[i].rot);
		current.elbow[i].setElbowAngle(simBird->birdBody->currentS.elbowBend[i], simBird->birdBody->currentS.elbowTwist[i]);
	
		current.wrist[i].updateStateBack(current.elbow[i].gPos, current.elbow[i].rot);
		current.wrist[i].setWristAngle( simBird->birdBody->currentS.wristBend[i] );
	}
}
void
RenderBird::updateFeather()
{
	for(int i=0;i<current.feathers.size();i++)
	for(int j=0;j<current.feathers[i].fjoint.size();j++)
	for(int k=0;k<current.feathers[i].fjoint[j].joints.size();k++)
	{
		if(k==0)
		{
			if(i==Feathers::PrimL)	current.feathers[i].fjoint[j].joints[k].updateStateBack(current.wrist[0].gPos, current.wrist[0].rot);
			if(i==Feathers::PrimR)	current.feathers[i].fjoint[j].joints[k].updateStateBack(current.wrist[1].gPos, current.wrist[1].rot);
			if(i==Feathers::SeconL)	current.feathers[i].fjoint[j].joints[k].updateStateBack(current.elbow[0].gPos, current.elbow[0].rot);
			if(i==Feathers::SeconR)	current.feathers[i].fjoint[j].joints[k].updateStateBack(current.elbow[1].gPos, current.elbow[1].rot);
			if(i==Feathers::TertR)	current.feathers[i].fjoint[j].joints[k].updateStateBack(current.shoulder[0].gPos, current.shoulder[0].rot);
			if(i==Feathers::TertR)	current.feathers[i].fjoint[j].joints[k].updateStateBack(current.shoulder[1].gPos, current.shoulder[1].rot);
			if(i==Feathers::Tail)	current.feathers[i].fjoint[j].joints[k].updateStateBack(current.root.gPos, current.root.rot);
		}
		else
		{
			current.feathers[i].fjoint[j].joints[k].updateStateBack( current.feathers[i].fjoint[j].joints[k-1].gPos, current.feathers[i].fjoint[j].joints[k-1].rot );
		}

		current.feathers[i].fjoint[j].joints[k].setFeatherRotation( simBird->birdFeathers->featherGroup[i]->each[j]->current[k*2]->ori );
	}
}
void
RenderBird::saveCurrentState()
{
	saveState.push_back(current);
}
void
RenderBird::deleteSavedFrame()
{
	saveState.clear();
}
void
RenderBird::loadCurrentFrameState()
{
	current = saveState[currentFrame];
	updateFromImportedFramesLocal();
}
void
RenderBird::updateFromImportedFramesLocal()
{
	for(int i=0;i<2;i++)	
	{
		current.shoulder[i].updateStateFront(current.root.gPos, current.root.rot);
		current.elbow[i].updateStateFront(current.shoulder[i].gPos, current.shoulder[i].rot);
		current.wrist[i].updateStateFront(current.elbow[i].gPos, current.elbow[i].rot);
	}

	for(int i=0;i<current.feathers.size();i++)
	for(int j=0;j<current.feathers[i].fjoint.size();j++)
	for(int k=0;k<current.feathers[i].fjoint[j].joints.size();k++)
	{
		if(k==0)
		{
			if(i==Feathers::PrimL)	current.feathers[i].fjoint[j].joints[k].updateStateFront(current.wrist[0].gPos, current.wrist[0].rot);
			if(i==Feathers::PrimR)	current.feathers[i].fjoint[j].joints[k].updateStateFront(current.wrist[1].gPos, current.wrist[1].rot);
			if(i==Feathers::SeconL)	current.feathers[i].fjoint[j].joints[k].updateStateFront(current.elbow[0].gPos, current.elbow[0].rot);
			if(i==Feathers::SeconR)	current.feathers[i].fjoint[j].joints[k].updateStateFront(current.elbow[1].gPos, current.elbow[1].rot);
			if(i==Feathers::TertR)	current.feathers[i].fjoint[j].joints[k].updateStateFront(current.shoulder[0].gPos, current.shoulder[0].rot);
			if(i==Feathers::TertR)	current.feathers[i].fjoint[j].joints[k].updateStateFront(current.shoulder[1].gPos, current.shoulder[1].rot);
			if(i==Feathers::Tail)	current.feathers[i].fjoint[j].joints[k].updateStateFront(current.root.gPos, current.root.rot);
		}
		else
		{
			current.feathers[i].fjoint[j].joints[k].updateStateFront( current.feathers[i].fjoint[j].joints[k-1].gPos, current.feathers[i].fjoint[j].joints[k-1].rot );
		}
	}
}
void
RenderBird::drawBird()
{
	if(saveState.size() == 0)	return;

	glSetMaterial(1,0,0);

	for(int i=0;i<2;i++)
	{
		//if(i==0)	glSetMaterial(1,0,0);
		//if(i==1)	glSetMaterial(0,1,0);
		glSetMaterial(0.8,0.8,0.8);

		drawLine_start_end( vector2position(current.root.gPos), vector2position(current.shoulder[i].gPos) );
		drawLine_start_end( vector2position(current.shoulder[i].gPos), vector2position(current.elbow[i].gPos) );
		drawLine_start_end( vector2position(current.elbow[i].gPos), vector2position(current.wrist[i].gPos) );
	}

	glPushMatrix();
		glTransform(current.root.gPos, current.root.rot);
		//glScalef(0.85f,0.6f,1.f);
		glScalef(0.6f,0.5f,1.3f);
		glScalef(0.8f,0.8f,0.8f);
		glRotatef(-25, 1, 0, 0);
		glTranslatef(0, 0.015, 0);
		meshModel->drawModel();
	glPopMatrix();
}
void
RenderBird::drawJoint()
{
	glSetMaterial(0.8f,0.8f,0.8f);
	current.root.drawGpos();
	for(int i=0;i<2;i++)
	{
		if(i==0)	glSetMaterial(1,0,0);
		if(i==1)	glSetMaterial(0,1,0);

		current.shoulder[i].drawGpos();
		current.elbow[i].drawGpos();
		current.wrist[i].drawGpos();
	}

	return;

	for(int i=0;i<current.feathers.size();i++)
	for(int j=0;j<current.feathers[i].fjoint.size();j++)
	for(int k=0;k<current.feathers[i].fjoint[j].joints.size();k++)
	{
		if(i%2==0)	glSetMaterial(1.0f,1.0f-(float)k/current.feathers[i].fjoint[j].joints.size(),(float)j/current.feathers[i].fjoint.size());
		else		glSetMaterial(1.0f-(float)k/current.feathers[i].fjoint[j].joints.size(),1.0f,(float)j/current.feathers[i].fjoint.size());
		
		current.feathers[i].fjoint[j].joints[k].drawGpos();
	}
}
void
RenderBird::drawFeather()
{
	glEnable(GL_BLEND);
	glDisable(GL_LIGHTING);
	glDisable(GL_CULL_FACE);
	//glDisable(GL_DEPTH_TEST);

	vector3 color1, color2, c;
	float alpha = 0;

	cout << current.feathers.size() << endl;

	for(int i=0;i<current.feathers.size();i++)
	{
		switch(i)
		{
		case Feathers::PrimL:	case Feathers::PrimR:
			color1 = vector3(217/255.0,112/255.0,50/255.0);
			color2 = vector3(215/255.0,146/255.0,51/255.0);
			break;
		case Feathers::SeconL:	case Feathers::SeconR:
			color1 = vector3(20/255.0,35/255.0,90/255.0);
			color2 = vector3(10/255.0,10/255.0,7/255.0);
			break;
		case Feathers::TertL:	case Feathers::TertR:
			break;
		case Feathers::Tail:
			color1 = vector3(36/255.0,103/255.0,50/255.0);
			color2 = vector3(180/255.0,160/255.0,0/255.0);
			break;
		}

		for(int j=0;j<current.feathers[i].fjoint.size();j++)
		for(int k=0;k<current.feathers[i].fjoint[j].joints.size();k++)
		{
			/*if(i%2==0)	glSetMaterial(1.0f,1.0f-(float)k/current.feathers[i].fjoint[j].joints.size(),(float)j/current.feathers[i].fjoint.size());
			else		glSetMaterial(1.0f-(float)k/current.feathers[i].fjoint[j].joints.size(),1.0f,(float)j/current.feathers[i].fjoint.size());*/

			//glSetMaterial(featherColor[0], featherColor[1], featherColor[2], 0.5);

			alpha = k/(float)((int)current.feathers[i].fjoint[j].joints.size()-1);
			c = (1-alpha) * color1 + alpha * color2;

			glColor4f(c[0],c[1],c[2],0.6);
		
			current.feathers[i].fjoint[j].joints[k].drawSurface();
		}
	}

	glEnable(GL_LIGHTING);
	glEnable(GL_CULL_FACE);
	glEnable(GL_DEPTH_TEST);
}
void
RenderBird::drawPath()
{
	glDisable(GL_LIGHTING);
		
	glColor3f(pathColor[0], pathColor[1], pathColor[2]);

	glBegin(GL_LINE_STRIP);
		for(int i=0; i<(int)path.size(); i++) glVertex3f(path[i][0], path[i][1], path[i][2]);
	glEnd();

	glEnable(GL_LIGHTING);
}
void
RenderBird::drawTrajectory()
{
	// 240 frame play기준
	int s = 0, e = currentFrame;

	int length = 100;

	if(e > length) s = e - length;

	glDisable(GL_LIGHTING);
	glEnable(GL_BLEND);

	glBegin(GL_LINE_STRIP);
		for(int i=s; i<e; i++)
		{
			// 새에서 먼 부분은 흐리게 그리기
			float alpha = 0.7f*(1.0f-(e-i+1)/(float)length);
			//glSetMaterial(trajColor[0], trajColor[1], trajColor[2], alpha);
			glColor4f(trajColor[0], trajColor[1], trajColor[2], alpha);
			glVertex3f(trajectory[i][0], trajectory[i][1], trajectory[i][2]); 
		}
	glEnd();

	glEnable(GL_LIGHTING);
}
bool
RenderBird::importFrames()
{
	ifstream fin("firstTest.bm");

	if(fin.fail()) return false;

	int totalFrame;

	fin >> totalFrame;

	trajectory.clear();

	int cnt = 0;
	for(int f=0;f<totalFrame;f++)
	{
		JointState one = current;	

		READ_ARRAY3(fin, one.root.gPos);		trajectory.push_back(one.root.gPos);
		READ_ARRAY4(fin, one.root.rot);
		
		cnt += 2;

		for(int i=0;i<2;i++)
		{
			READ_ARRAY4(fin, one.shoulder[i].rot);
			READ_ARRAY4(fin, one.elbow[i].rot);
			READ_ARRAY4(fin, one.wrist[i].rot);

			cnt += 3;
		}

		for(int i=0;i<current.feathers.size();i++)
		for(int j=0;j<current.feathers[i].fjoint.size();j++)
		for(int k=0;k<current.feathers[i].fjoint[j].joints.size();k++)
		{
			READ_ARRAY4(fin, one.feathers[i].fjoint[j].joints[k].rot);
			cnt++;
		}

		saveState.push_back(one);

	}

	fin.close();
	fin.clear();

	cout << "complete import frame " << saveState.size() << endl;
	cout << "--------------------------------------------------------" << endl;

	return true;
}
bool
RenderBird::importFrames(const char* name)
{
	char buf[256];

	cout << "--------------------------------------------------------" << endl;
	sprintf(buf, "Recording/%s.bm", name);

	ifstream fin(buf);

	if(fin.fail())
	{
		cout << "Warning : no such a file!!! - " << buf << endl;
		cout << "--------------------------------------------------------" << endl;
		return false;
	}

	trajectory.clear();

	int totalFrame;

	fin >> totalFrame;

	int cnt = 0;
	for(int f=0;f<totalFrame;f++)
	{
		JointState one = current;	

		READ_ARRAY3(fin, one.root.gPos);		trajectory.push_back(one.root.gPos);
		READ_ARRAY4(fin, one.root.rot);
		
		cnt += 2;

		for(int i=0;i<2;i++)
		{
			READ_ARRAY4(fin, one.shoulder[i].rot);
			READ_ARRAY4(fin, one.elbow[i].rot);
			READ_ARRAY4(fin, one.wrist[i].rot);

			cnt += 3;
		}

		for(int i=0;i<current.feathers.size();i++)
		for(int j=0;j<current.feathers[i].fjoint.size();j++)
		for(int k=0;k<current.feathers[i].fjoint[j].joints.size();k++)
		{
			READ_ARRAY4(fin, one.feathers[i].fjoint[j].joints[k].rot);
			cnt++;
		}

		saveState.push_back(one);

	}

	fin.close();
	fin.clear();

	cout << "complete import frame " << saveState.size() << endl;
	cout << "--------------------------------------------------------" << endl;

	return true;
}
void
RenderBird::importPath(const char* name)
{
	path.clear();

	char buf[256];

	cout << "--------------------------------------------------------" << endl;
	sprintf(buf, "Recording/%s.path", name);

	ifstream fin(buf);

	if(fin.fail())
	{
		cout << "Warning : no such a file!!! - " << buf << endl;
		cout << "--------------------------------------------------------" << endl;
		return;
	}

	int totalPoint, temp;

	fin >> temp;
	fin >> totalPoint;

	for(int i=0; i<totalPoint; i++)
	{
		position p;
		fin >> p[0]; fin >> p[1]; fin >> p[2];

		path.push_back(p);
	}

	fin.clear();
	fin.close();

	cout << "complete import path : " << name << " / " << path.size() << endl;
	cout << "--------------------------------------------------------" << endl;
}
void
RenderBird::exportFramesLocalRightNow()
{
	quater q;

	util::fileOpenWrapper(foutLocal, "firstTest.local");

	dataSize += (int)saveState.size();

	for(int f=0;f<(int)saveState.size();f++)
	{
		WRITE_ARRAY3(foutLocal, saveState[f].root.gPos);
		WRITE_ARRAY4(foutLocal, saveState[f].root.rot);

		for(int i=0;i<2;i++)
		{
			q = saveState[f].root.rot.inverse() * saveState[f].shoulder[i].rot;			WRITE_ARRAY4(foutLocal, q);
			q = saveState[f].shoulder[i].rot.inverse() * saveState[f].elbow[i].rot;		WRITE_ARRAY4(foutLocal, q);
			q = saveState[f].elbow[i].rot.inverse() * saveState[f].wrist[i].rot;		WRITE_ARRAY4(foutLocal, q);
		}

		for(int i=0;i<saveState[f].feathers.size();i++)
		for(int j=0;j<saveState[f].feathers[i].fjoint.size();j++)
		for(int k=0;k<saveState[f].feathers[i].fjoint[j].joints.size();k++)
		{
			if(k==0)
			{
				if(i==Feathers::PrimL)	q = saveState[f].wrist[0].rot.inverse();
				if(i==Feathers::PrimR)	q = saveState[f].wrist[1].rot.inverse();
				if(i==Feathers::SeconL)	q = saveState[f].elbow[0].rot.inverse();
				if(i==Feathers::SeconR)	q = saveState[f].elbow[1].rot.inverse();
				if(i==Feathers::TertL)	q = saveState[f].shoulder[0].rot.inverse();
				if(i==Feathers::TertR)	q = saveState[f].shoulder[1].rot.inverse();
				if(i==Feathers::Tail)	q = saveState[f].root.rot.inverse();
			}
			else
			{
				q = saveState[f].feathers[i].fjoint[j].joints[k-1].rot.inverse() ;
			}

			q = q * saveState[f].feathers[i].fjoint[j].joints[k].rot;						WRITE_ARRAY4(foutLocal, q);
		}

		if( ((int)saveState.size()>10) && f%((int)saveState.size()/10) == 0 )
		{
			cout << '\r' << "export : " << 10 * f/((int)saveState.size()/10.0) << "% complete ";
		}
	}

	foutLocal.flush();
	deleteSavedFrame();
}
void
RenderBird::convertData()
{
	char buf[256], ch;

	foutLocal.close();

	/* save state */

	ifstream fin("firstTest.local");
	ofstream fout("firstTest.bm");

	fout << dataSize << endl;
	
	while(!fin.eof())
	{
		fin.get(ch);
		fout.put(ch);
	}

	fin.close();
	fin.clear();
	fout.close();
	fout.clear();
}
void
RenderBird::reset()
{
	deleteSavedFrame();
	currentFrame = saveState.size();

	//initializeFromCurrentBird();
	initializeFromTemplateFile();

	if(foutLocal.is_open())
	{
		foutLocal.close();
		foutLocal.clear();
	}
}