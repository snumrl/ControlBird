#include "definitions.h"
#include "util.h"
#include "CaptureMotion.h"
#include "GL/glut.h"

void
CaptureMotion::setCaptureDataInformation()
{
	ifstream fin;
	util::fileOpenWrapper(fin, INFORM_DIR+string("Capture_inform.txt"));

	whatBird			= (BirdName) atoi(getNext(fin, "//").c_str());
	captureFPS			= atoi(getNext(fin, "//").c_str());
								
	for(int i=0;i<maxBirdName;i++)		
	{
		if(i==whatBird)		str_birdName	= getNext(fin, "//").c_str();	
		else				getNext(fin, "//").c_str();
	}
	for(int i=0;i<maxBirdName;i++)		
	{
		if(i==whatBird)		numOfClips	= atoi(getNext(fin, "//").c_str());
		else				getNext(fin, "//").c_str();
	}
	
	util::fileCloseWrapper(fin);
}
void
CaptureMotion::loadCaptureData()
{
	ifstream cfin;

	for(int  s=0;s<numOfClips;s++)
	{
		char filename[256];
		char ch[1024];
		float numOfFrames;

		sprintf(filename,"CaptureData/%s%02d.trc",str_birdName.c_str(),s+1);

		util::fileOpenWrapper(cfin, INFORM_DIR+string(filename));

		for(int i=0;i<2;i++)	cfin.getline(ch,1024);
		for(int i=0;i<2;i++)	cfin >> ch;

		cfin >> ch;		numOfFrames		= atoi(ch);

		OneClip*	one = new OneClip(maxMarkerName, maxDOFname, numOfFrames);

		for(int i=0;i<4;i++)	cfin.getline(ch,1024);

		for(int i=0;i<one->numOfFrames;i++)
		{
			for(int j=0;j<2;j++)	cfin >> ch;

			for(int j=0;j<maxMarkerName;j++)
			for(int k=0;k<3;k++)
			{
				cfin >> ch;
				one->pos[i][j][k]	= atof(ch)/1000.0f;
			}
		}

		dataConverting(one);

		clips.push_back(one);

		util::fileCloseWrapper(cfin);
	}
}
void
CaptureMotion::dataConverting(OneClip* one)
{
	getRoot(one);
	getShoulder(one);
	getElbow(one);
	getWrist(one);
	getTail(one);
	getDelimiter(one);
}
void
CaptureMotion::getRoot(OneClip* one)
{
	int windowS = 10;
	int cnt = 0;

	for(int i=0;i<one->numOfFrames;i++)
	{
		vector3 vy, vz;
		cnt = 0;

		for(int j=i-windowS;j<=i+windowS;j++)
		{
			if(j<0 || j>=one->numOfFrames)	continue;

			vector3 v1 = one->pos[j][m_rshoulder] - one->pos[j][m_cback];	v1 = normalize(v1);
			vector3 v2 = one->pos[j][m_lshoulder] - one->pos[j][m_cback];	v2 = normalize(v2);
				
			vy += v1 * v2;
			vz += (v1+v2)/2.0f;
		
			cnt++;
		}

		one->root[i][axis_Z] = vz/(float)cnt;									one->root[i][axis_Z] = normalize(one->root[i][axis_Z]);
		one->root[i][axis_Y] = vy/(float)cnt;									one->root[i][axis_Y] = normalize(one->root[i][axis_Y]);
		one->root[i][axis_X] = one->root[i][axis_Y] * one->root[i][axis_Z];		one->root[i][axis_X] = normalize(one->root[i][axis_X]);
		
		// root의 orientation을 약간 회전시킨다. shoulder marker를 이용해 root를 approximation한 것에 대한 보정
		float angle = (-15.0f/180.0f)*M_PI;
		vector3 axis;	axis = sin(angle/2.0f) * one->root[i][axis_X];
		quater rot(cos(angle/2.0f), axis[0], axis[1], axis[2]);

		one->root[i][axis_Y] = rotate(rot, one->root[i][axis_Y]);				one->root[i][axis_Y] = normalize(one->root[i][axis_Y]);
		one->root[i][axis_Z] = rotate(rot, one->root[i][axis_Z]);				one->root[i][axis_Z] = normalize(one->root[i][axis_Z]);

		// calc rotation quaternion of new root
		one->rotQ[i] = Matrix2Quater( matrix(one->root[i][axis_X], one->root[i][axis_Y], one->root[i][axis_Z]));
	}
}
void
CaptureMotion::getShoulder(OneClip* one)
{
	quater frontNtopQ, twistQ;
	vector3 localE, localW, globalTE, globalTW, temp;
	int m_shoulder, m_elbow, m_wrist, d_shoulder;
	float sign;

	for(int j=0;j<2;j++)
	for(int i=0;i<one->numOfFrames;i++)
	{
		if(j==0)
		{
			m_shoulder	= m_lshoulder;
			m_elbow		= m_lelbow;
			m_wrist		= m_lwrist;

			d_shoulder	= d_lshoulder1;

			sign		= 1.0f;
		}
		else if(j==1)
		{
			m_shoulder	= m_rshoulder;
			m_elbow		= m_relbow;
			m_wrist		= m_rwrist;

			d_shoulder	= d_rshoulder1;

			sign		= -1.0f;
		}
		
		localE = rotate(one->rotQ[i].inverse(), one->pos[i][m_elbow] - one->pos[i][m_shoulder] );	localE = normalize(localE);
		localW = rotate(one->rotQ[i].inverse(), one->pos[i][m_wrist] - one->pos[i][m_shoulder] );	localW = normalize(localW);
		
		
		frontNtopQ = quaterFromVector(vector3(sign,0,0), localE);

		globalTW = rotate( (one->rotQ[i] * frontNtopQ.inverse()), localW );
		temp = one->root[i][axis_X] * (one->root[i][axis_X] - globalTW);	
		float ang	= acos(normalize(temp) % one->root[i][axis_Y]);
		if( (one->root[i][axis_Y] * normalize(temp)) % one->root[i][axis_X] < 0)	ang *= -1.0f;
		twistQ = quaterFromAxisAngle( vector3(1,0,0), ang );

		temp = ln(frontNtopQ * twistQ);
		
		temp[1] *= sign;
		temp[2] *= sign;

		for(int k=0;k<3;k++)		one->DOF[i][d_shoulder+k]	= temp[k];
	}
}
void
CaptureMotion::getElbow(OneClip* one)
{
	vector3 a, b, c, cross1, cross2;
	float acos_angle, sign;
	int m_shoulder, m_elbow, m_wrist, m_hand, d_elbow;
	
	for(int j=0;j<2;j++)
	for(int i=0;i<one->numOfFrames;i++)
	{
		if(j==0)
		{
			m_shoulder	= m_lshoulder;
			m_elbow		= m_lelbow;
			m_wrist		= m_lwrist;
			m_hand		= m_lhand;

			d_elbow		= d_lelbow_bend;

			sign		= 1.0f;
		}
		else if(j==1)
		{
			m_shoulder	= m_rshoulder;
			m_elbow		= m_relbow;
			m_wrist		= m_rwrist;
			m_hand		= m_rhand;

			d_elbow		= d_relbow_bend;

			sign		= -1.0f;
		}

		a = one->pos[i][m_shoulder] - one->pos[i][m_elbow];
		b = one->pos[i][m_wrist]	- one->pos[i][m_elbow];
		c = one->pos[i][m_wrist]	- one->pos[i][m_hand];

		acos_angle = (a%b)/(a.length() *  b.length());
		one->DOF[i][d_elbow] = acos(acos_angle);			
		
		cross1 = a*b;	cross2 = c*b;

		acos_angle = (cross1%cross2)/(cross1.length() *  cross2.length());
		one->DOF[i][d_elbow+1] = -acos(acos_angle) * sign;	

		a = cross2*cross1;	a = normalize(a);
		b = -b;				b = normalize(b);
		if(a%b < 0.0f)		one->DOF[i][d_elbow+1] *= -1.0f;
	}
}
void
CaptureMotion::getWrist(OneClip* one)
{
	vector3 a, b;
	float acos_angle;
	int m_elbow, m_wrist, m_hand, d_wrist;

	for(int j=0;j<2;j++)
	for(int i=0;i<one->numOfFrames;i++)
	{
		if(j==0)
		{
			m_elbow		= m_lelbow;
			m_wrist		= m_lwrist;
			m_hand		= m_lhand;

			d_wrist		= d_lwrist_bend;
		}
		else if(j==1)
		{
			m_elbow		= m_relbow;
			m_wrist		= m_rwrist;
			m_hand		= m_rhand;

			d_wrist		= d_rwrist_bend;
		}
	
		a = one->pos[i][m_elbow] - one->pos[i][m_wrist];
		b = one->pos[i][m_hand] - one->pos[i][m_wrist];
		acos_angle = (a%b)/(a.length() *  b.length());

		one->DOF[i][d_wrist]		= acos(acos_angle);
	}
}
void
CaptureMotion::getTail(OneClip* one)
{
	vector3 a, b, cross;
	float acos_angle;

	for(int i=0;i<one->numOfFrames;i++)
	{
		// spread
		a = one->pos[i][m_ltail] - one->pos[i][m_spine];		a = normalize(a);
		b = one->pos[i][m_rtail] - one->pos[i][m_spine];		b = normalize(b);
		acos_angle = (a%b);
		one->DOF[i][d_tail_spread] = acos(acos_angle);
		
		// bend : 꼬리 깃털면의 normal을 trunk local axis의 yz 평면으로 VECTOR_PROJECTION한 vector와 local axis Y축과 이루는 각도로 정의
		cross = a*b;		cross = normalize(cross);
		a = VECTOR_PROJECTION(cross, one->root[i][axis_X]) * one->root[i][axis_X];	// norm vector를 X축으로 VECTOR_PROJECTION
		b = cross - a;	b = normalize(b);
		acos_angle = ( one->root[i][axis_Y] % b);
		one->DOF[i][d_tail_bend] = acos(acos_angle);

		// twist : 꼬리 left, right 이은 vector가 trunk local axis의 xy 평면으로 VECTOR_PROJECTION한 vector와 trunk local axis Xr 이루는 각도로 정의
		a = one->pos[i][m_ltail] - one->pos[i][m_rtail];
		b = VECTOR_PROJECTION(a, one->root[i][axis_Z]) * one->root[i][axis_X];
		a = a - b;	a = normalize(a);
		acos_angle = (one->root[i][axis_X] % a);
		//tail_spread = acos(acos_angle);

	}
}
void
CaptureMotion::getDelimiter(OneClip* one)
{
	bool check;
	int margin = 8;
	int delimDOF = d_lelbow_bend;

	for(int i=margin;i<one->numOfFrames-margin;i++)
	{
		check = true;
		for(int k=1;k<=margin;k++)	
		{
			if((one->DOF[i-k][delimDOF] < one->DOF[i][delimDOF]) || (one->DOF[i+k][delimDOF] < one->DOF[i][delimDOF]))
			{	
				check = false; 
				break;	
			}
		}
		
		if(check)		one->delimFrame.push_back(i);
	}
}
void
CaptureMotion::getWingbeat()
{
	avgWingbeatDuration = 0.0f;

	for(int i=0;i<numOfClips;i++)
	for(int j=0;j<clips[i]->delimFrame.size()-1;j++)
	{
		CaptureWingbeat* oneW = new CaptureWingbeat(maxDOFname);

		int start	= clips[i]->delimFrame[j];
		int end		= clips[i]->delimFrame[j+1];

		oneW->duration = (end - start);

		oneW->k_clip = i;
		oneW->k_frame = start;

		for(int s=start;s<end;s++)
		for(int k=0;k<maxDOFname;k++)
			oneW->pos[k].push_back( clips[i]->DOF[s][k] );

		avgWingbeatDuration += oneW->duration;

		wingbeats.push_back(oneW);
	}

	avgWingbeatDuration /= (float)wingbeats.size();
}
void
CaptureMotion::reflectWingbeat()
{
	int currentWingbeatSize = wingbeats.size();

	for(int i=0;i<currentWingbeatSize;i++)
	{
		CaptureWingbeat* oneW = new CaptureWingbeat(maxDOFname);

		oneW->duration	= wingbeats[i]->duration;
		oneW->k_clip	= wingbeats[i]->k_clip;
		oneW->k_frame	= wingbeats[i]->k_frame;

		for(int s=0;s<oneW->duration;s++)
		{
			oneW->pos[d_lshoulder1].push_back( wingbeats[i]->pos[d_rshoulder1][s] );
			oneW->pos[d_lshoulder2].push_back( wingbeats[i]->pos[d_rshoulder2][s] );
			oneW->pos[d_lshoulder3].push_back( wingbeats[i]->pos[d_rshoulder3][s] );

			oneW->pos[d_rshoulder1].push_back( wingbeats[i]->pos[d_lshoulder1][s] );
			oneW->pos[d_rshoulder2].push_back( wingbeats[i]->pos[d_lshoulder2][s] );
			oneW->pos[d_rshoulder3].push_back( wingbeats[i]->pos[d_lshoulder3][s] );

			oneW->pos[d_lelbow_bend].push_back( wingbeats[i]->pos[d_relbow_bend][s] );
			oneW->pos[d_lelbow_twist].push_back( wingbeats[i]->pos[d_relbow_twist][s] );

			oneW->pos[d_relbow_bend].push_back( wingbeats[i]->pos[d_lelbow_bend][s] );
			oneW->pos[d_relbow_twist].push_back( wingbeats[i]->pos[d_lelbow_twist][s] );

			oneW->pos[d_lwrist_bend].push_back( wingbeats[i]->pos[d_rwrist_bend][s] );

			oneW->pos[d_rwrist_bend].push_back( wingbeats[i]->pos[d_lwrist_bend][s] );

			oneW->pos[d_tail_bend].push_back( wingbeats[i]->pos[d_tail_bend][s] );
			oneW->pos[d_tail_spread].push_back( wingbeats[i]->pos[d_tail_spread][s] );
		}

		wingbeats.push_back(oneW);
	}
}
void
CaptureMotion::normalizeWingbeat()
{
	refWingbeat = new CaptureWingbeat(maxDOFname);
	refWingbeat->duration = (int)avgWingbeatDuration;
	refWingbeat->timewarp = (avgWingbeatDuration-1.0f) / (avgWingbeatDuration-1.0f);

	for(int i=0;i<wingbeats.size();i++)
	{
		float scale = (float)(wingbeats[i]->duration-1) / (refWingbeat->duration-1);

		wingbeats[i]->timewarp = scale;

		for(int j=0;j<refWingbeat->duration;j++)
		{
			float value;

			int index = (int)(scale * j);
			float w = scale * j - index;	
			
			for(int k=0;k<maxDOFname;k++)
			{
				if(w > 0.0f)	value = wingbeats[i]->pos[k][index] * (1.0f - w) + wingbeats[i]->pos[k][index+1] * w;
				else			value = wingbeats[i]->pos[k][index];
				
				wingbeats[i]->normalizedPos[k].push_back( value );
			}
		}
	}
}
void
CaptureMotion::getReferenceWingbeat()
{
	for(int k=0;k<maxDOFname;k++)
	{
		float min = 2*M_PI;
		float max = -2*M_PI;

		for(int j=0;j<refWingbeat->duration;j++)
		{
			refWingbeat->pos[k].push_back(0.0f);

			for(int i=0;i<wingbeats.size();i++)
			{
				refWingbeat->pos[k][j] += wingbeats[i]->normalizedPos[k][j];
			}

			refWingbeat->pos[k][j] /= (float)wingbeats.size();
			
			if(min > refWingbeat->pos[k][j])	min = refWingbeat->pos[k][j];
			if(max < refWingbeat->pos[k][j])	max = refWingbeat->pos[k][j];
		}

		refWingbeat->maxPoint[k] = max;
		refWingbeat->minPoint[k] = min;
	}
}
void
CaptureMotion::parameterizeWingbeat()
{
	// min & max check
	for(int i=0;i<wingbeats.size();i++)
	for(int j=0;j<maxDOFname;j++)
	{
		float min = 2*M_PI;
		float max = -2*M_PI;

		for(int k=0;k<wingbeats[i]->duration;k++)
		{
			if(min > wingbeats[i]->pos[j][k])	min = wingbeats[i]->pos[j][k];
			if(max < wingbeats[i]->pos[j][k])	max = wingbeats[i]->pos[j][k];
		}

		wingbeats[i]->maxPoint[j] = max;
		wingbeats[i]->minPoint[j] = min;
	}

	
	for(int j=0;j<maxDOFname;j++)
	{
		float referMinPoint		= refWingbeat->minPoint[j];
		float referAmplitude	= refWingbeat->maxPoint[j] - refWingbeat->minPoint[j];

		refWingbeat->translation[j] = 0.0f;
		refWingbeat->scale[j] = 1.0f;

		// 임의로 꼬리 들어올리도록
		if(j==d_tail_bend)
		{
			refWingbeat->translation[j] = -0.3f;
			refWingbeat->scale[j] = 0.4f;
		}
#ifdef OLD
#else
		if(j==d_tail_bend)
		{
			refWingbeat->scale[j] *= 1.5f;
		}

		if(j==d_tail_spread)
		{
			refWingbeat->scale[j] *= 1.5f;
		}
#endif

		for(int i=0;i<wingbeats.size();i++)
		{
			float thisMinPoint	= wingbeats[i]->minPoint[j];
			float thisAmplitude	= wingbeats[i]->maxPoint[j] - wingbeats[i]->minPoint[j];

			wingbeats[i]->translation[j]	= thisMinPoint - referMinPoint;
			wingbeats[i]->scale[j]			= thisAmplitude / referAmplitude;
		}
	}	
}
void
CaptureMotion::reconstructWingbeat()
{
	for(int i=0;i<wingbeats.size();i++)
	{
		int ff =  (int)(wingbeats[i]->timewarp * (refWingbeat->duration-1)) + 1;
		
		for(int j=0;j<ff;j++)
		{
			float value;
			
			int index = (int)(j / wingbeats[i]->timewarp );
			float w = j / wingbeats[i]->timewarp - index;	

			for(int k=0;k<maxDOFname;k++)
			{
				if(w > 0.0f)	value = refWingbeat->pos[k][index] * (1.0f - w) + refWingbeat->pos[k][index+1] * w;	
				else			value = refWingbeat->pos[k][index];	

				value = (value - refWingbeat->minPoint[k]) * wingbeats[i]->scale[k] + refWingbeat->minPoint[k] + wingbeats[i]->translation[k];

				wingbeats[i]->reconstructPos[k].push_back( value );
			}

		}
	}
}
vector3
CaptureMotion::getCurrentMarker(MarkerName nn)
{
	return clips[currentClip]->pos[currentFrame][nn];
}
void
CaptureMotion::drawTitle(float x, float y, char* title)
{
	//glColor3f(0,0,0);
	//gl_font(FL_HELVETICA_BOLD, 13);
	//gl_draw(title, x, y);
}
void
CaptureMotion::drawEachAxis(float x, float y)
{
	float sx = 0.03f;
	float sy = 0.45f;

	glColor3f(0.96f, 0.96f, 0.96f);
	glBegin(GL_QUADS);
	glVertex2f(x, y);
	glVertex2f(x + clips[currentClip]->numOfFrames*sx, y);
	glVertex2f(x + clips[currentClip]->numOfFrames*sx, y + M_PI*sy);
	glVertex2f(x, y + M_PI*sy);
	glEnd();

	glLineWidth(1.0f);
	glColor3f(0.8f, 0.8f, 0.8f);	
	glBegin(GL_LINES);	
	glVertex2f(x, y);					glVertex2f(x + clips[currentClip]->numOfFrames*sx, y);		
	glVertex2f(x, y + M_PI/2.0f*sy);	glVertex2f(x + clips[currentClip]->numOfFrames*sx, y + M_PI/2.0f*sy);		
	glEnd();

	// delimiter frame
	glColor3f(0.8f, 0.8f, 0.8f);	
	glBegin(GL_LINES);	
	for(int i=0;i<clips[currentClip]->delimFrame.size();i++)
	{
		glVertex2f(x + clips[currentClip]->delimFrame[i]*sx, y);			
		glVertex2f(x + clips[currentClip]->delimFrame[i]*sx, y + M_PI*sy);		
	}
	glEnd();

	// current frame
	glLineWidth(1.5f);
	glColor3f(0.5f, 0.5f, 0.5f);	
	glBegin(GL_LINES);	
	glVertex2f(x + currentFrame*sx, y);					glVertex2f(x + currentFrame*sx, y + M_PI*sy);		
	glEnd();

	// reference wingbeat axis
	x += 13.0f;
	glColor3f(0.96f, 0.96f, 0.96f);
	glBegin(GL_QUADS);
	glVertex2f(x, y);
	glVertex2f(x + refWingbeat->duration*sx, y);
	glVertex2f(x + refWingbeat->duration*sx, y + M_PI*sy);
	glVertex2f(x, y + M_PI*sy);
	glEnd();

	glLineWidth(1.0f);
	glColor3f(0.8f, 0.8f, 0.8f);	
	glBegin(GL_LINES);	
	glVertex2f(x, y);					glVertex2f(x + refWingbeat->duration*sx, y);		
	glVertex2f(x, y + M_PI/2.0f*sy);	glVertex2f(x + refWingbeat->duration*sx, y + M_PI/2.0f*sy);		
	glEnd();
}
void
CaptureMotion::drawEachGraph(int idx, float x, float y, vector3 color)
{
	float sx = 0.03f;
	float sy = 0.45f;
	
	glLineWidth(1.0f);

	color *= 0.7f;
	glColor3f(color[0], color[1], color[2]);
	glBegin(GL_LINE_STRIP);
	for(int i=0;i<clips[currentClip]->numOfFrames;i++)
		glVertex2f(x + i*sx, y + clips[currentClip]->DOF[i][idx]*sy);
	glEnd();

	glLineWidth(2.0f);

	// reconstruct wingbeat
	if(false)
	{
		glBegin(GL_LINE_STRIP);
		for(int i=0;i<wingbeats.size()/2;i++)
		{
			if(currentClip != wingbeats[i]->k_clip)	continue;

			for(int j=0;j<wingbeats[i]->duration;j++)
				glVertex2f(x + (j+wingbeats[i]->k_frame)*sx, y + wingbeats[i]->reconstructPos[idx][j]*sy);
		}
		glEnd();
	}

	// reference wingbeat
	x += 13.0f;
	glBegin(GL_LINE_STRIP);
	for(int i=0;i<refWingbeat->duration+1;i++)
	{
		if(i==refWingbeat->duration)	glVertex2f(x + i*sx, y + refWingbeat->pos[idx][0] *sy);
		else							glVertex2f(x + i*sx, y + refWingbeat->pos[idx][i] *sy);
	}
	glEnd();
}
void
CaptureMotion::drawAllGraph()
{
	glDisable(GL_LIGHTING);
	
	float unit = 2.0f;
	char title[256];

	sprintf(title,"Shoulder first component");	drawTitle(0, unit*6.8f, title);	drawEachAxis(0.0f, unit*6.0f);
	drawEachGraph( d_lshoulder1, 0.0f, unit*6.0f, autoColor(0));
	drawEachGraph( d_rshoulder1, 0.0f, unit*6.0f, autoColor(1));

	sprintf(title,"Shoulder second component");	drawTitle(0, unit*5.8f, title);	drawEachAxis(0.0f, unit*5.0f);
	drawEachGraph( d_lshoulder2, 0.0f, unit*5.0f, autoColor(0));
	drawEachGraph( d_rshoulder2, 0.0f, unit*5.0f, autoColor(1));

	sprintf(title,"Shoulder third component");	drawTitle(0, unit*4.8f, title);	drawEachAxis(0.0f, unit*4.0f);
	drawEachGraph( d_lshoulder3, 0.0f, unit*4.0f, autoColor(0));
	drawEachGraph( d_rshoulder3, 0.0f, unit*4.0f, autoColor(1));

	sprintf(title,"Elbow bend");				drawTitle(0, unit*3.8f, title);	drawEachAxis(0.0f, unit*3.0f);
	drawEachGraph( d_lelbow_bend, 0.0f, unit*3.0f, autoColor(0));
	drawEachGraph( d_relbow_bend, 0.0f, unit*3.0f, autoColor(1));

	sprintf(title,"Elbow Twist");				drawTitle(0, unit*2.8f, title);	drawEachAxis(0.0f, unit*2.0f);
	drawEachGraph( d_lelbow_twist, 0.0f, unit*2.0f, autoColor(0));
	drawEachGraph( d_relbow_twist, 0.0f, unit*2.0f, autoColor(1));

	sprintf(title,"Wrist Bend");				drawTitle(0, unit*1.8f, title);	drawEachAxis(0.0f, unit*1.0f);
	drawEachGraph( d_lwrist_bend, 0.0f, unit*1.0f, autoColor(0));
	drawEachGraph( d_rwrist_bend, 0.0f, unit*1.0f, autoColor(1));

	sprintf(title,"Tail");						drawTitle(0, unit*0.8f, title);	drawEachAxis(0.0f, unit*0.0f);
	drawEachGraph( d_tail_bend, 0.0f, unit*0.0f, autoColor(2));
	drawEachGraph( d_tail_spread, 0.0f, unit*0.0f, autoColor(3));

	glEnable(GL_LIGHTING);
}
void
CaptureMotion::drawRootAxis()
{
	vector3 start	= getCurrentMarker(MarkerName(m_cback));
	vector3 end1	= getCurrentMarker(MarkerName(m_cback)) + clips[currentClip]->root[currentFrame][axis_X] * 0.1f;		
	vector3 end2	= getCurrentMarker(MarkerName(m_cback)) + clips[currentClip]->root[currentFrame][axis_Y] * 0.1f;		
	vector3 end3	= getCurrentMarker(MarkerName(m_cback)) + clips[currentClip]->root[currentFrame][axis_Z] * 0.1f;		

	glBegin(GL_LINES);
	glSetMaterial(1,0,0);	glVertex3f(start[0], start[1], start[2]);	glVertex3f(end1[0], end1[1], end1[2]);	
	glSetMaterial(0,1,0);	glVertex3f(start[0], start[1], start[2]);	glVertex3f(end2[0], end2[1], end2[2]);	
	glSetMaterial(0,0,1);	glVertex3f(start[0], start[1], start[2]);	glVertex3f(end3[0], end3[1], end3[2]);	
	glEnd();
}
void
CaptureMotion::drawCapturedBird()
{
	drawRootAxis();

	glDisable(GL_CULL_FACE);

	vector3 color;
	vector3 pos;
		
	// joint
	for(int i=0;i<maxMarkerName;i++)
	{
		pos = getCurrentMarker(MarkerName(i));	
		
		switch(i)
		{
			case m_head:		case m_cback:	case m_spine:	case m_ltail:	case m_rtail:	case m_ctail:	color = autoColor(1)*0.6f;	break;
			case m_lshoulder:	case m_lelbow:	case m_lwrist:	case m_lhand:									color = autoColor(0)*0.6f;	break;
			case m_rshoulder:	case m_relbow:	case m_rwrist:	case m_rhand:									color = autoColor(2)*0.6f;	break;
			default:																							color = autoColor(3)*0.6f;	break;
		}

		glSetMaterial(color[0], color[1], color[2]);
		drawSphere(pos[0], pos[1], pos[2], 0.005f);
	}

	// link
	color = autoColor(1)*0.9f;
	glSetMaterial(color[0], color[1], color[2], 0.8f);
	simpleEllipse(getCurrentMarker(MarkerName(m_head)), getCurrentMarker(MarkerName(m_cback)));
	simpleEllipse(getCurrentMarker(MarkerName(m_cback)), getCurrentMarker(MarkerName(m_spine)));
	simpleEllipse(getCurrentMarker(MarkerName(m_spine)), getCurrentMarker(MarkerName(m_ltail)));
	simpleEllipse(getCurrentMarker(MarkerName(m_spine)), getCurrentMarker(MarkerName(m_rtail)));
	drawTraiangle(getCurrentMarker(MarkerName(m_spine)),getCurrentMarker(MarkerName(m_ctail)),getCurrentMarker(MarkerName(m_ltail)));
	drawTraiangle(getCurrentMarker(MarkerName(m_spine)),getCurrentMarker(MarkerName(m_rtail)),getCurrentMarker(MarkerName(m_ctail)));

	color = autoColor(0)*0.9f;
	glSetMaterial(color[0], color[1], color[2], 0.8f);
	simpleEllipse(getCurrentMarker(MarkerName(m_cback)), getCurrentMarker(MarkerName(m_lshoulder)));
	simpleEllipse(getCurrentMarker(MarkerName(m_lshoulder)), getCurrentMarker(MarkerName(m_lelbow)));
	simpleEllipse(getCurrentMarker(MarkerName(m_lelbow)), getCurrentMarker(MarkerName(m_lwrist)));
	simpleEllipse(getCurrentMarker(MarkerName(m_lwrist)), getCurrentMarker(MarkerName(m_lhand)));

	color = autoColor(2)*0.9f;
	glSetMaterial(color[0], color[1], color[2], 0.8f);
	simpleEllipse(getCurrentMarker(MarkerName(m_cback)), getCurrentMarker(MarkerName(m_rshoulder)));
	simpleEllipse(getCurrentMarker(MarkerName(m_rshoulder)), getCurrentMarker(MarkerName(m_relbow)));
	simpleEllipse(getCurrentMarker(MarkerName(m_relbow)), getCurrentMarker(MarkerName(m_rwrist)));
	simpleEllipse(getCurrentMarker(MarkerName(m_rwrist)), getCurrentMarker(MarkerName(m_rhand)));

	glEnable(GL_CULL_FACE);

	//glDisable(GL_DEPTH_TEST);
	drawReconsBird();
	//glEnable(GL_DEPTH_TEST);
}
void
CaptureMotion::drawReconsBird()
{
	int m_shoulder, m_elbow, m_wrist, m_hand, d_shoulder, d_elbow, d_wrist;
	vector3 shoulder, elbow, wrist, hand, ss, ee, ww, temp;
	quater rotQ;
	float sign;
	
	vector3 color = autoColor(3)*0.9f;
	glSetMaterial(color[0], color[1], color[2], 0.8f);

	for(int i=0;i<2;i++)
	{
		if(i==0)
		{
			m_shoulder	= m_lshoulder;
			m_elbow		= m_lelbow;
			m_wrist		= m_lwrist;
			m_hand		= m_lhand;

			d_shoulder	= d_lshoulder1;
			d_elbow		= d_lelbow_bend;
			d_wrist		= d_lwrist_bend;

			sign		= 1.0f;
		}
		else if(i==1)
		{
			m_shoulder	= m_rshoulder;
			m_elbow		= m_relbow;
			m_wrist		= m_rwrist;
			m_hand		= m_rhand;

			d_shoulder	= d_rshoulder1;
			d_elbow		= d_relbow_bend;
			d_wrist		= d_rwrist_bend;

			sign		= -1.0f;
		}

		ss = vector3( sign*(getCurrentMarker(MarkerName(m_elbow)) - getCurrentMarker(MarkerName(m_shoulder))).length(),0,0 );
		ee = vector3( sign*(getCurrentMarker(MarkerName(m_wrist)) - getCurrentMarker(MarkerName(m_elbow))).length(),0,0 );
		ww = vector3( sign*(getCurrentMarker(MarkerName(m_hand)) - getCurrentMarker(MarkerName(m_wrist))).length(),0,0 );

		shoulder = getCurrentMarker(MarkerName(m_shoulder));

		for(int k=0;k<3;k++)	temp[k]	= clips[currentClip]->DOF[currentFrame][d_shoulder+k];	temp[1] *= sign;	temp[2] *= sign;
		rotQ = clips[currentClip]->rotQ[currentFrame] * exp(temp);
		elbow = rotate(rotQ, ss) + shoulder;

		temp = vector3( clips[currentClip]->DOF[currentFrame][d_elbow+1]*(-1.0f), sign*( clips[currentClip]->DOF[currentFrame][d_elbow] - M_PI) ,0);
		rotQ = rotQ * EulerAngle2Quater(temp);
		wrist = rotate(rotQ, ee) + elbow;

		temp = vector3(0, (M_PI-clips[currentClip]->DOF[currentFrame][d_wrist])*sign,0);
		rotQ = rotQ * EulerAngle2Quater(temp);
		hand = rotate(rotQ, ww) + wrist;

		simpleEllipse(shoulder, elbow);		drawSphere(elbow[0],	elbow[1],	elbow[2], 0.005f);
		simpleEllipse(elbow, wrist);		drawSphere(wrist[0],	wrist[1],	wrist[2], 0.005f);
		simpleEllipse(wrist, hand);			drawSphere(hand[0],		hand[1],	hand[2], 0.005f);
	}
}
