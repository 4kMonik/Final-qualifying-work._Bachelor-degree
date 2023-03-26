#define _USE_MATH_DEFINES
#include <gl/freeglut.h>
#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>
#include <string>
#include <tuple>

using namespace std;
int rx = 26, ry = 26, rz = 26;
const int fps = 60;

class Body
{
protected:
	double v, phi, theta;
	double delayTime;
	tuple<double, double, double> tau;
	double a = pow(v,2);
public:
	vector<tuple<double, double, double>> R;
	Body(tuple<double, double, double> &r0, double &v, double timer, double &phi, double &theta );
	virtual std::tuple<double, double, double> r(double t) = 0;
	double x(double), y(double), z(double);
	double getV() const;
	double getPhi() const;
	double getTheta() const;
	void setTau(double, double);
	virtual void updateT_N() = 0;
};

class Evader : public Body
{
private:
	std::tuple<double, double, double> r(double t);
public:
	Evader(tuple<double, double, double> &r0, double &v, double phi, double theta) : Body(r0, v, 0, phi, theta) {}
	void updateT_N();
};

class Pursuer : public Body
{
private:
	std::tuple<double, double, double> r(double t);
	Evader& currentTarget;
	double advanceCoef;
public:
	Pursuer(tuple < double, double, double> &r0, double &v, double timer, Evader& target, double phi = 0, double theta = 0, double AC = 0 ) : Body(r0, v, timer, phi, theta), currentTarget(target) { advanceCoef = AC; }
	void updateT_N();
};

bool isCollide = false;

// Время
double tColl = -1;
double dt = 1. / fps;
double t0 = 0;
double t = t0;

//Уколняющееся тело
double x2_0 = 0, y2_0 = 10, z2_0 = 0;
auto r2_0 = std::make_tuple(x2_0, y2_0, z2_0);
double v2 = 6;
double phi2 = 0;
double theta2 = M_PI_2;

Evader simpleEvader(r2_0, v2, phi2, theta2);

//Преследующее тело
double x1_0 = 10, y1_0 = 0, z1_0 = 0;
tuple<double, double, double> r1_0 = std::make_tuple(x1_0, y1_0, z1_0);
double v1 = 8;
double a = 0;
double delayTime1 = 0;
double phi1 = M_PI_2;
double theta1 = M_PI_2;
double R1;

Pursuer simplePursuer(r1_0, v1, delayTime1, simpleEvader, phi1, theta1, a);



void drawBody(Body &Body);
void drawCollisionPoint(double, double, double);
void drawCollisionPoint(tuple<double, double, double>);
void drawNet(double left, double bottom, double right, double top);
void drawNet3d(double left, double bottom, double back, double right, double top, double front);
void drawTrajectory(vector<double> &, vector<double> &);
void drawTrajectory(vector<tuple<double, double>> &);
void drawArrow(double, double, double, double, double, double);
void drawAxis(double, double, double, double, double, double);
void drawUI(double, double, double, double);

void display();
void reshape(int, int);
void reshape3d(int, int);
void timer(int);


int main(int argc, char** argv)
{
	setlocale(LC_CTYPE, "Russian");
	cout << "r1: ";
	cin >> x1_0 >> y1_0 >> z1_0;
	cout << "v1: ";
	cin >> v1;
	cout << "phi1: ";
	cin >> phi1;
	phi1 = phi1 * M_PI / 180.;
	cout << "theta1: ";
	cin >> theta1;
	theta1 = theta1 * M_PI / 180.;
	cout << "R: ";
	cin >> R1;
	simplePursuer.R.back() = std::make_tuple(x1_0, y1_0, z1_0);
	simplePursuer.setTau(phi1, theta1);

	cout << "r2: ";
	cin >> x2_0 >> y2_0 >> z2_0 ;
	cout << "v2: ";
	cin >> v2;
	cout << "phi2: ";
	cin >> phi2;
	phi2 = phi2 * M_PI / 180.;
	cout << "theta2: ";
	cin >> theta2;
	theta2 = theta2 * M_PI / 180.;
	simpleEvader.R.back() = std::make_tuple(x2_0, y2_0, z2_0);
	simpleEvader.setTau(phi2, theta2);




	glutInit(&argc, argv);
	glutInitWindowSize(1600, 900);
	glutInitWindowPosition(200, 100);
	glutCreateWindow("Визуальное представление результатов вычислений задачи 1");
	glClearColor(1.0, 1.0, 1.0, 0.0);

	glutDisplayFunc(display);
	glutReshapeFunc(reshape3d);
	glutTimerFunc(0, timer, 0);
	glutMainLoop();
	return 0;
}

Body::Body(tuple<double, double, double> &r0, double &v0, double timer, double &phi0, double &theta0) : R(1, r0), v(v0), delayTime(timer), phi(phi0), theta(theta0)
{
	this->setTau(phi, theta);
	//cout << std::get<0>(tau) << std::get<1>(tau) << std::get<2>(tau) << endl;
}

double Body::x(double t)
{
	return std::get<0>(r(t));
}

double Body::y(double t)
{
	return std::get<1>(r(t));
}

double Body::z(double t)
{
	return std::get<2>(r(t));
}

double Body::getV() const
{
	return v;
}

double Body::getPhi() const
{
	return phi;
}
double Body::getTheta() const
{
	return theta;
}
void Body::setTau(double newPhi, double newTheta = M_PI_2)
{
	phi = newPhi;
	theta = newTheta;
	std::get<0>(tau) = cos(phi) * sin(theta);
	std::get<1>(tau) = sin(phi) * sin(theta);
	std::get<2>(tau) = cos(theta);
}

void Evader::updateT_N()
{
	if (t < delayTime)
		return;
	v = v2;
	double R = 5;
	a = v*v/R;
	theta += v / R * dt;
	double dX = R * sin(theta);
	double dY = -0.2;
	double dZ = R * cos(theta);
	double alphaX = dX / sqrt(pow(dX, 2) + pow(dY, 2) + pow(dZ, 2));
	double alphaY = dY / sqrt(pow(dX, 2) + pow(dY, 2) + pow(dZ, 2));
	double alphaZ = dZ / sqrt(pow(dX, 2) + pow(dY, 2) + pow(dZ, 2));
	double alphaTau = alphaX * std::get<0>(tau) + alphaY * std::get<1>(tau) + alphaZ * std::get<2>(tau);
	double alphaN = sqrt(1 - pow(alphaTau, 2));
	double tauX = 0, tauY = 0, tauZ = 0;
	
	tauX = std::get<0>(tau) + a * dt * (alphaX - std::get<0>(tau)) / v ;
	tauY = std::get<1>(tau) + a * dt * (alphaY - std::get<1>(tau)) / v ;
	tauZ = std::get<2>(tau) + a * dt * (alphaZ - std::get<2>(tau)) / v ;
	double lTau = sqrt(tauX * tauX + tauY * tauY + tauZ * tauZ);
	tau = std::make_tuple(tauX/lTau, tauY/lTau, tauZ/lTau);
	
	return;
}

void Pursuer::updateT_N()
{
	if (t < delayTime)
	{
		//v = 0;
		return;
	}
	else
		v = v1;
	
	double dX = std::get<0>(currentTarget.R.back()) - std::get<0>(this->R.back()) + advanceCoef  * cos(currentTarget.getPhi()) * sin(currentTarget.getTheta()) * sqrt(pow(std::get<0>(currentTarget.R.back()) - std::get<0>(this->R.back()),2) + pow(std::get<1>(currentTarget.R.back()) - std::get<1>(this->R.back()), 2) + pow(std::get<2>(currentTarget.R.back()) - std::get<2>(this->R.back()), 2));
	double dY = std::get<1>(currentTarget.R.back()) - std::get<1>(this->R.back()) + advanceCoef  * sin(currentTarget.getPhi()) * sin(currentTarget.getTheta()) * sqrt(pow(std::get<0>(currentTarget.R.back()) - std::get<0>(this->R.back()), 2) + pow(std::get<1>(currentTarget.R.back()) - std::get<1>(this->R.back()), 2) + pow(std::get<2>(currentTarget.R.back()) - std::get<2>(this->R.back()), 2));
	double dZ = std::get<2>(currentTarget.R.back()) - std::get<2>(this->R.back()) + advanceCoef  * cos(currentTarget.getTheta()) * sqrt(pow(std::get<0>(currentTarget.R.back()) - std::get<0>(this->R.back()), 2) + pow(std::get<1>(currentTarget.R.back()) - std::get<1>(this->R.back()), 2) + pow(std::get<2>(currentTarget.R.back()) - std::get<2>(this->R.back()), 2));
	
	double alphaX = dX / sqrt(pow(dX, 2) + pow(dY, 2) + pow(dZ, 2));
	double alphaY = dY / sqrt(pow(dX, 2) + pow(dY, 2) + pow(dZ, 2));
	double alphaZ = dZ / sqrt(pow(dX, 2) + pow(dY, 2) + pow(dZ, 2));
	
	double alphaTau = alphaX * std::get<0>(tau) + alphaY * std::get<1>(tau) + alphaZ * std::get<2>(tau);
	double alphaN = sqrt(1 - pow(alphaTau,2));

	//double ro = 5./(sqrt(pow(dX, 2) + pow(dY, 2) + pow(dZ, 2)));
	double ro = 1./R1;
	//drawAxis(std::get<0>(this->R.back()), std::get<1>(this->R.back()), std::get<2>(this->R.back()) , alphaX + std::get<0>(this->R.back()), alphaY + std::get<1>(this->R.back()), alphaZ + std::get<2>(this->R.back()));
	//drawAxis(std::get<0>(this->R.back()), std::get<1>(this->R.back()), std::get<2>(this->R.back()), std::get<0>(tau) + std::get<0>(this->R.back()), std::get<1>(tau) + std::get<1>(this->R.back()), std::get<2>(tau) + std::get<2>(this->R.back()));

	if (alphaTau == -1)
	{
		std::get<0>(tau) += 0.1;
		std::get<1>(tau) += 0.1;
	}

	double tauX = std::get<0>(tau) + v * ro *  dt * (alphaX - std::get<0>(tau)  );
	double tauY = std::get<1>(tau) + v * ro *  dt * (alphaY - std::get<1>(tau)  );
	double tauZ = std::get<2>(tau) + v * ro *  dt * (alphaZ - std::get<2>(tau)  );
	double lTau = sqrt(tauX * tauX + tauY * tauY + tauZ * tauZ);
	tau = std::make_tuple(tauX / lTau, tauY / lTau, tauZ / lTau);
	//std::cout << " p: " << sqrt(tauX*tauX + tauY* tauY + tauZ* tauZ);
}

std::tuple<double, double, double> Evader::r(double t)
{
	double x = dt * this->v * std::get<0>(tau) + std::get<0>(R.back());
	double y = dt * this->v * std::get<1>(tau) + std::get<1>(R.back());
	double z = dt * this->v * std::get<2>(tau) + std::get<2>(R.back());

	//double x = dt * sqrt(this->v*this->v - v*v*cos(t)*cos(t))/2 + std::get<0>(R.back());
	//double y = dt * v*cos(t)/2 + std::get<1>(R.back());
	//double z = 0;
	return make_tuple(x, y, z);
}

std::tuple<double, double, double> Pursuer::r(double t)
{
	//cout << "PursuerXYZ" << endl;
	double x = dt * this->v * std::get<0>(tau) + std::get<0>(R.back());
	double y = dt * this->v * std::get<1>(tau) + std::get<1>(R.back());
	double z = dt * this->v * std::get<2>(tau) + std::get<2>(R.back());


	return make_tuple(x, y, z);
}

void drawBody(Body &Body)
{
	glBegin(GL_POINTS);
	tuple<double, double, double> bodyCoords = Body.r(t);
	glVertex3f(std::get<0>(bodyCoords), std::get<1>(bodyCoords), std::get<2>(bodyCoords));
	Body.R.push_back(std::make_tuple(std::get<0>(bodyCoords), std::get<1>(bodyCoords), std::get<2>(bodyCoords)));
	Body.updateT_N();
	//cout << t << "||" << Body.x(t) << " | " << Body.y(t) << " | " << Body.z(t) << endl;
	glEnd();
}

void drawCollisionPoint(double x, double y, double z)
{
	glColor3d(1, 0, 0);
	glPointSize(20.0);
	glBegin(GL_POINTS);
	glVertex3d(x, y, z);
	glEnd();
}
void drawCollisionPoint(tuple<double, double, double> r)
{
	double textX = rx - 10;
	double textY = ry - 5;
	double textZ = std::get<2>(r);
	glColor3d(0,0,0);
	glLineWidth(3);

	drawAxis(textX -0.5, textY - 0.5, textZ, std::get<0>(r), std::get<1>(r), std::get<2>(r));

	glRasterPos3f(textX, textY, std::get<2>(r));
	glutBitmapString(GLUT_BITMAP_HELVETICA_18, reinterpret_cast<const unsigned char *>("Point of Collision"));
}

void drawNet(double left, double bottom, double right, double top)
{
	glLineWidth(1);
	for (double x_i = left; x_i < right; x_i += 1)
	{
		glBegin(GL_LINES);
		glColor3d(0.75, 0.75, 0.75);
		glVertex3f(x_i, bottom, 0);
		glVertex3f(x_i, top, 0);
		glEnd();
		glColor3d(0, 0, 0);
		glRasterPos2f(x_i, bottom - 1);

		string num = std::to_string(static_cast<int>(x_i));
		glutBitmapString(GLUT_BITMAP_HELVETICA_18, reinterpret_cast<const unsigned char *>(num.c_str()));
	}


	for (double y_i = bottom; y_i < top; y_i += 1)
	{
		glBegin(GL_LINES);
		glColor3d(0.75, 0.75, 0.75);
		glVertex2f(left, y_i);
		glVertex2f(right, y_i);
		glEnd();
		glColor3d(0, 0, 0);
		if (y_i < 10)
			glRasterPos2f(left - 0.75, y_i);
		else
			glRasterPos2f(left - 1, y_i);
		string num = std::to_string(static_cast<int>(y_i));
		glutBitmapString(GLUT_BITMAP_HELVETICA_18, reinterpret_cast<const unsigned char *>(num.c_str()));
	}

	glLineWidth(3);
	glBegin(GL_LINES);
	glVertex3f(0, bottom, 0);
	glVertex3f(0, top-1, 0);
	glVertex3f(left, 0, 0);
	glVertex3f(right-1, 0, 0);
	glEnd();
	drawArrow(rx - 1,0,0,1,0, M_PI_2);
	drawArrow(0, ry - 1, 0, 1, M_PI_2, M_PI_2);
	glRasterPos3f(rx + 0.5, bottom, 0);
	glutBitmapString(GLUT_BITMAP_HELVETICA_18, reinterpret_cast<const unsigned char *>("x"));
	glRasterPos3f(left, ry + 0.5, 0);
	glutBitmapString(GLUT_BITMAP_HELVETICA_18, reinterpret_cast<const unsigned char *>("y"));
}

void drawNet3d(double left, double bottom, double back, double right, double top, double front)
{
	glBegin(GL_LINES);
	glColor3d(0.75, 0.75, 0.75);
	glVertex3f(left, bottom, back);
	glVertex3f(right, bottom, back);

	glVertex3f(left, bottom, front);
	glVertex3f(right, bottom, front);
	glEnd();

	for (double x_i = left; x_i < right; x_i += 1)
	{
		glColor3d(0, 0, 0);
		glRasterPos3f(x_i, bottom - 1, back);
		string num = std::to_string(static_cast<int>(x_i));
		glutBitmapString(GLUT_BITMAP_HELVETICA_18, reinterpret_cast<const unsigned char *>(num.c_str()));
	}

	glBegin(GL_LINES);
	glColor3d(0.75, 0.75, 0.75);
	glVertex3f(left, bottom, back);
	glVertex3f(left, top, back);
	glEnd();

	for (double y_i = bottom; y_i < top; y_i += 1)
	{
		glColor3d(0, 0, 0);
		if (y_i < 10)
			glRasterPos3f(left - 0.5, y_i, back);
		else
			glRasterPos2f(left - 0.8, y_i);
		string num = std::to_string(static_cast<int>(y_i));
		glutBitmapString(GLUT_BITMAP_HELVETICA_18, reinterpret_cast<const unsigned char *>(num.c_str()));
	}

	glBegin(GL_LINES);
	glColor3d(0.75, 0.75, 0.75);
	glVertex3f(left, bottom, back);
	glVertex3f(left, bottom, front);

	glVertex3f(right, bottom, back);
	glVertex3f(right, bottom, front);
	glEnd();

	for (double z_i = back + 1; z_i < front; z_i += 2)
	{
		glColor3d(0, 0, 0);
		glRasterPos3f(left - 0.5, bottom + 0.25, z_i);
		string num = std::to_string(static_cast<int>(z_i));
		glutBitmapString(GLUT_BITMAP_HELVETICA_18, reinterpret_cast<const unsigned char *>(num.c_str()));
	}

	drawAxis(0, 0, 0, rx, 0 ,0);

	drawAxis(0, 0, 0, 0, ry, 0);

	drawAxis(0, 0, 0, 0, 0, rz); 

	glRasterPos3f(rx + 0.5, bottom, back);
	glutBitmapString(GLUT_BITMAP_HELVETICA_18, reinterpret_cast<const unsigned char *>("x"));
	glRasterPos3f(left , ry + 0.5, back);
	glutBitmapString(GLUT_BITMAP_HELVETICA_18, reinterpret_cast<const unsigned char *>("y"));
	glRasterPos3f(left , bottom , rz + 0.5);
	glutBitmapString(GLUT_BITMAP_HELVETICA_18, reinterpret_cast<const unsigned char *>("z"));
}

void drawTrajectory(vector<double> &X, vector<double> &Y, vector<double> &Z)
{
	glLineWidth(1);
	glBegin(GL_LINES);

	vector<double>::iterator pX = X.begin();
	vector<double>::iterator pY = Y.begin();
	for (; pX != X.end(); pX++)
	{
		glVertex2f(*pX, *pY);
		pY++;
	}
	glEnd();
}
void drawTrajectory(vector<tuple<double, double, double>> &R)
{
	glLineWidth(3);
	vector<tuple<double, double, double>>::iterator pR = R.begin();
	glBegin(GL_LINE_STRIP);
	for (; pR != R.end(); pR++)
	{
		glVertex3d(std::get<0>(*pR), std::get<1>(*pR), std::get<2>(*pR));
	}
	glEnd();

	double phi;
	double theta;

	for (int i = 2; i < R.size() ; i+= 30)
	{
		phi = atan2((std::get<1>(R[i]) - std::get<1>(R[i-2])) , (std::get<0>(R[i]) - std::get<0>(R[i-2])));
		if (((std::get<1>(R[i]) - std::get<1>(R[i - 2]) == 0) && (std::get<0>(R[i]) - std::get<0>(R[i - 2]) == 0)))
			phi = 0;
		theta = atan2(sqrt(pow(std::get<0>(R[i]) - std::get<0>(R[i - 2]), 2) + pow(std::get<1>(R[i]) - std::get<1>(R[i - 2]), 2)) , (std::get<2>(R[i]) - std::get<2>(R[i - 2]) ));

		drawArrow(std::get<0>(R[i]), std::get<1>(R[i]), std::get<2>(R[i]), 0.75, phi, theta);
	}


}

void drawUI(double t, double x, double y, double z)
{
	glColor3d(0, 0, 0);
	glRasterPos3f(rx + 1, -1, 0);
	string currTime = "Time: " + std::to_string(t);
	glutBitmapString(GLUT_BITMAP_HELVETICA_18, reinterpret_cast<const unsigned char *>(currTime.c_str()));
	glRasterPos3f(rx + 1, -2, 0);
	string currX = "x: " + std::to_string(x);
	glutBitmapString(GLUT_BITMAP_HELVETICA_18, reinterpret_cast<const unsigned char *>(currX.c_str()));
	glRasterPos3f(rx + 1, -3, 0);
	string currY = "y: " + std::to_string(y);
	glutBitmapString(GLUT_BITMAP_HELVETICA_18, reinterpret_cast<const unsigned char *>(currY.c_str()));
	glRasterPos3f(rx + 1, -4, 0);
	string currZ = "z: " + std::to_string(z);
	glutBitmapString(GLUT_BITMAP_HELVETICA_18, reinterpret_cast<const unsigned char *>(currZ.c_str()));
}

void display()
{

	glClear(GL_COLOR_BUFFER_BIT);
	glLoadIdentity();

	//drawNet(0, 0, rx, ry);
	drawNet3d(0, 0, 0, rx, ry, rz);

	glColor3d(0, 1, 0);
	drawTrajectory(simpleEvader.R);
	glColor3d(0, 0, 1);
	drawTrajectory(simplePursuer.R);

	glLineWidth(1);
	glPointSize(5.0);
	if (!isCollide)
	{
		glColor3f(0.0, 1.0, 0.0);
		drawBody(simpleEvader);
		glColor3f(0.0, 0.0, 1.0);
		drawBody(simplePursuer);

	}
	else
	{
		drawCollisionPoint(simpleEvader.R.back());
		std::cout << "\nВремя встречи тел: " << round(t * 100) / 100 << " с.";
	}

	drawUI(t, std::get<0>(simplePursuer.R.back()), std::get<1>(simplePursuer.R.back()), std::get<2>(simplePursuer.R.back()));
	
	glFlush();
}

void reshape(int w, int h)
{
	//v
	glViewport(0, 0, w - 200, h);
	//p
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluOrtho2D(-2, rx + 2, -5, ry + 2);
	glMatrixMode(GL_MODELVIEW);
}

void reshape3d(int w, int h)
{
	//v
	glViewport(0, 0, w - 200, h);
	//p
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(-10, rx + 2, -10, ry + 2, -rz, rz);
	glRotatef(30, -0.5, 1, -0.1);
	glTranslated(-5, 0 , -5);
	glMatrixMode(GL_MODELVIEW);
}

void timer(int)
{
	glutPostRedisplay();

	if (sqrt(pow(simpleEvader.x(t) - simplePursuer.x(t), 2) + pow(simpleEvader.y(t) - simplePursuer.y(t), 2) + pow(simpleEvader.z(t) - simplePursuer.z(t), 2)) <  abs(v2 - v1) * dt)
	{
		isCollide = true;
		tColl = t;
	}
	if (!isCollide)
		glutTimerFunc(1000 / fps, timer, 0);
	t += dt;
}

void drawArrow(double x, double y, double z, double length, double phi, double theta)
{
	glPushMatrix();
	glTranslated(x, y, z);
	glRotated(phi * 180 / M_PI, 0, 0, 1);
	glRotated(theta * 180 / M_PI, 0, 1, 0);
	glutSolidCone(length / 4, length, 10, 6);
	glPopMatrix();
}

void drawAxis(double x1, double y1, double z1, double x2, double y2, double z2)
{
	double L = sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2) + pow(z2 - z1, 2));
	double arrowL = L / 30;
	double arrowW = arrowL / 2;
	double phi = atan2((y2 - y1) , (x2 - x1));

	double theta = atan2(sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2)) , (z2 - z1));

	double arrowLx = arrowL * cos(phi) * sin(theta);
	double arrowLy =arrowL * sin(phi) * sin(theta);
	double arrowLz = arrowL * cos(theta);

	glLineWidth(3);
	glBegin(GL_LINES);
	glVertex3d(x1, y1, z1);
	glVertex3d(x2 - arrowLx, y2 - arrowLy, z2 - arrowLz);
	glEnd();

	drawArrow(x2 - arrowLx, y2 - arrowLy, z2 - arrowLz, arrowL, phi, theta);

}