

#include "util.h"

#include "GUI.h"
#include "MeshModel.h"
#include "Bird.h"
#include "Controller.h"
#include "RenderBird.h"

extern MeshModel*	meshModel;
extern Bird*		simBird;
extern SystemState*	sysState;
extern PhyEngine*	phyEngine;
extern Controller*	controller;
extern RenderBird*	rendBird;

typedef enum { BODY, FEATHER, BODY_MAT, FEATHER_MAT } DisplayLists;


int winwidth = 1;
int winheight = 1;
Camera camera = Camera();
int mousePrevX=0;
int mousePrevY=0;
bool press_1=false;
bool press_2=false;
bool press_3=false;
bool wheel_up=false;
bool wheel_down=false;
position birdPos;

Camera::Camera()
{
}

void Camera::set(position& eye,position& lookAt,vector3& up)
{
	this->eye = eye;
	this->lookAt = lookAt;
	this->up = up;
}

void Camera::move(vector3& translation)
{
	this->eye += translation;
	this->lookAt += translation;
}

void Camera::transform()
{
	/*transf se3_1 = translate_transf(centerX, centerY, centerZ);
	transf se3_2 = rotate_transf(rotateY, vector3(0,1,0));
	transf se3_3 = rotate_transf(rotateX, vector3(1,0,0));
	transf se3_4 = translate_transf(0, 0, distance);
	transf se3 = se3_4 * se3_3 * se3_2 * se3_1;

	glTransform(se3);*/

	gluLookAt(
		eye[0],		eye[1],		eye[2],
		lookAt[0],	lookAt[1],	lookAt[2],
		up[0],		up[1],		up[2]);
}

static void draw_axis1()
{
	glLineWidth(2);
	glBegin(GL_LINES);
	glColor3f(1,0,0);
	glVertex3f(1,0,0);
	glVertex3f(0,0,0);
	glColor3f(0,1,0);
	glVertex3f(0,1,0);
	glVertex3f(0,0,0);
	glEnd();
	glLineWidth(1);
}

void callBackReshape(int w, int h)
{
	glViewport(0,0,w,h);

	winwidth  = w;
	winheight = h;
}

void callBackDisplay(void)
{
	glClearColor(0,0,0,1);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(60.0, (GLdouble)winwidth/winheight, 0.01, 100);

	camera.move( phyEngine->getPositionBody(Body::Trunk) - camera.lookAt );
	camera.transform();

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	glDisable(GL_LIGHTING);
	draw_axis();
	glEnable(GL_LIGHTING);


	glPushMatrix();
	glSetMaterial(1,1,1);
	simBird->drawBird();		// bird model
	glPopMatrix();

	glutSwapBuffers();
}

void callBackMouse(int button, int state, int x, int y)
{
	//cout << "callBackMouse: " << button << " " << state << " " << x << " " << y << endl;

	/*int mouseX = x;
	int mouseY = y;

	if (state == GLUT_DOWN)
	{
		if (button == GLUT_LEFT_BUTTON) {
			press_1 = true;
		}
		else if (button == GLUT_MIDDLE_BUTTON) {
			press_2 = true;
		}
		else if (button == GLUT_RIGHT_BUTTON){
			press_3 = true;
		}
	}
	else if( state == GLUT_UP )
	{
		if (button == GLUT_LEFT_BUTTON) {
			press_1 = false;
		}
		else if (button == GLUT_MIDDLE_BUTTON) {
			press_2 = false;
		}
		else if (button == GLUT_RIGHT_BUTTON){
			press_3 = false;
		}
		else if(button==3 || button==4)
		{
			int sign = (button==3) ? 1 : -1;
			camera.distance -= 25 * sign / 2.0;
			if (camera.distance < 0.0) 
				camera.distance = 0.0;
		}
	}

	mousePrevX = mouseX;
	mousePrevY = mouseY;
	*/

	glutPostRedisplay();
}

void callBackMotion(int mouseX, int mouseY)
{
	//cout << "callBackMotion: " << mouseX << " " << mouseY << endl;

	//double scale = 0.1;

	////if (event == FL_DRAG && (Fl::event_ctrl() == 0 || Fl::event_shift() == 0))
	//{	
	//	int mouseDeltaX = scale * (mouseX - mousePrevX);
	//	int mouseDeltaY = scale * (mouseY - mousePrevY);

	//	if (press_1 == true && press_3 == false) {
	//		camera.rotateY -= Deg2Rad(double(mouseDeltaX));
	//		camera.rotateX -= Deg2Rad(double(mouseDeltaY));
	//	}
	//	else if (press_2 == false && press_3 == true){
	//		camera.centerX -= cos(camera.rotateY) * double(mouseDeltaX) / 1;
	//		camera.centerZ -= -sin(camera.rotateY) * double(mouseDeltaX) / 1;

	//		camera.centerX -= sin(camera.rotateY) * double(mouseDeltaY) / 1;
	//		camera.centerZ -= cos(camera.rotateY) * double(mouseDeltaY) / 1;
	//	}
	//}

	//mousePrevX = mouseX;
	//mousePrevY = mouseY;

	glutPostRedisplay();
}

void callBackKeyboard(unsigned char key, int, int) 
{
	switch (key)
	{
	case 'q':
		sysState->set(sysState->PLAY);
		break;
	case 'w':
		sysState->set(sysState->PAUSE);
		break;
	case 'e':
		sysState->set(sysState->STOP);
		simBird->resetBird();
		controller->reset();
		rendBird->reset();
		break;
	case 27:             // ESCAPE key
		exit (0);
		break;
	}
}

void callBackTimer(int value)
{
	if( sysState->get(sysState->PLAY) || sysState->get(sysState->PLAY_ONESTEP) )
	{
		for(int i=0; i<sysState->PLAY_SPEED; i++)
		{		
			simBird->stateUpdata_beforeSim();

			phyEngine->advanceOneStep(sysState->TIME_STEP);

			simBird->stateUpdata_afterSim();

			if(sysState->get(sysState->PLAY_ONESTEP))		sysState->set(sysState->PAUSE);

			if( !(sysState->get(sysState->PLAY)||sysState->get(sysState->PLAY_ONESTEP)) ) break;
		}
			
	}
	
	glutTimerFunc((double)(1.0f/sysState->FPS)*1000,callBackTimer,0);

	glutPostRedisplay();
}

void initCallBack()
{
	glutMouseFunc(callBackMouse);
	glutMotionFunc(callBackMotion);
	glutKeyboardFunc(callBackKeyboard);

	glutReshapeFunc(callBackReshape);
	glutDisplayFunc(callBackDisplay);

	glutTimerFunc(33,callBackTimer,0);
}

void SetupMaterials(void)
{
	GLfloat mtn_ambuse[] =   { 0.426, 0.256, 0.108, 1.0 };
	GLfloat mtn_specular[] = { 0.394, 0.272, 0.167, 1.0 };
	GLfloat mtn_shininess[] = { 10 };

	GLfloat water_ambuse[] =   { 0.0, 0.1, 0.5, 1.0 };
	GLfloat water_specular[] = { 0.0, 0.1, 0.5, 1.0 };
	GLfloat water_shininess[] = { 10 };

	GLfloat tree_ambuse[] =   { 0.4, 0.25, 0.1, 1.0 };
	GLfloat tree_specular[] = { 0.0, 0.0, 0.0, 1.0 };
	GLfloat tree_shininess[] = { 0 };

	GLfloat leaf_ambuse[] =   { 0.0, 0.8, 0.0, 1.0 };
	GLfloat leaf_specular[] = { 0.0, 0.8, 0.0, 1.0 };
	GLfloat leaf_shininess[] = { 10 };

	/*glNewList(MOUNTAIN_MAT, GL_COMPILE);
		glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, mtn_ambuse);
		glMaterialfv(GL_FRONT, GL_SPECULAR, mtn_specular);
		glMaterialfv(GL_FRONT, GL_SHININESS, mtn_shininess);
	glEndList();

	glNewList(WATER_MAT, GL_COMPILE);
		glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, water_ambuse);
		glMaterialfv(GL_FRONT, GL_SPECULAR, water_specular);
		glMaterialfv(GL_FRONT, GL_SHININESS, water_shininess);
	glEndList();

	glNewList(TREE_MAT, GL_COMPILE);
		glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, tree_ambuse);
		glMaterialfv(GL_FRONT, GL_SPECULAR, tree_specular);
		glMaterialfv(GL_FRONT, GL_SHININESS, tree_shininess);
	glEndList();

	glNewList(LEAF_MAT, GL_COMPILE);
		glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, leaf_ambuse);
		glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, leaf_specular);
		glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, leaf_shininess);
	glEndList();*/
}

void initLight(void)
{
	GLfloat light_ambient[] = { 1.0, 1.0, 1.0, 1.0 };
	GLfloat light_diffuse[] = { 1.0, 1.0, 1.0, 1.0 };
	GLfloat light_specular[] = { 1.0, 1.0, 1.0, 1.0 };
	GLfloat light_position[] = { 0.0, 0.3, 0.3, 0.0 };

	GLfloat lmodel_ambient[] = { 0.4, 0.4, 0.4, 1.0 };

	glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);
	glLightfv(GL_LIGHT0, GL_SPECULAR, light_specular);
	glLightfv(GL_LIGHT0, GL_POSITION, light_position);
    
	glLightModelfv(GL_LIGHT_MODEL_AMBIENT, lmodel_ambient);

	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);

	glDepthFunc(GL_LEQUAL);
	glEnable(GL_DEPTH_TEST);

	glEnable(GL_NORMALIZE);

	glShadeModel(GL_SMOOTH);

	//SetupMaterials();
	//CreateTreeLists();

	//glFlush();
} 

GUI::GUI(int argc, char** argv)
{
	position p = phyEngine->getPositionBody(Body::Trunk);
	camera.set(
		p + vector3(0,0.1,0.5),
		p,
		vector3(0,1,0));
		

	glutInit(&argc, argv);

	glutInitWindowSize(800, 800);
	glutInitWindowPosition(10,90);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
	glutCreateWindow("Bird Simulator");

	initCallBack();

	initLight(); 

	glutMainLoop(); // you could use Fl::run() instead
}