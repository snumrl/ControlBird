#pragma once
#include "GL\glut.h"

class Camera
{
public:
	Camera();

	void set(position& eye,position& lookAt,vector3& up);
	void move(vector3& translation);
	void transform();

	position	lookAt;
	position	eye;
	vector3		up;
};

class GUI
{
public:
	GUI(int argc, char** argv);

private:

};