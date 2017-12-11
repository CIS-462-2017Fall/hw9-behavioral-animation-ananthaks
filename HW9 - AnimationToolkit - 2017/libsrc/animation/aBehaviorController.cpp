#include "aBehaviorController.h"

#include "aVector.h"
#include "aRotation.h"
#include <Windows.h>
#include <algorithm>

#include "GL/glew.h"
#include "GL/glut.h"
#include <iostream>


#define Truncate(a, b, c) (a = max<double>(min<double>(a,c),b))

double BehaviorController::gMaxSpeed = 1000.0; 
double BehaviorController::gMaxAngularSpeed = 200.0;  
double BehaviorController::gMaxForce = 2000.0;  
double BehaviorController::gMaxTorque = 2000.0;
double BehaviorController::gKNeighborhood = 500.0;   
double BehaviorController::gOriKv = 1.0;    
double BehaviorController::gOriKp = 1.0;  
double BehaviorController::gVelKv = 1.0;    
double BehaviorController::gAgentRadius = 80.0;  
double BehaviorController::gMass = 1;
double BehaviorController::gInertia = 1;
double BehaviorController::KArrival = 1.0; 
double BehaviorController::KDeparture = 12000.0;
double BehaviorController::KNoise = 15.0;
double BehaviorController::KWander = 80.0;   
double BehaviorController::KAvoid = 600.0;  
double BehaviorController::TAvoid = 1000.0;   
double BehaviorController::KSeparation = 12000.0; 
double BehaviorController::KAlignment = 1.0;  
double BehaviorController::KCohesion = 1.0;  

const double M2_PI = M_PI * 2.0;

BehaviorController::BehaviorController() 
{
	m_state.resize(m_stateDim);
	m_stateDot.resize(m_stateDim);
	m_controlInput.resize(m_controlDim);

	vec3 m_Pos0 = vec3(0, 0, 0);
	vec3 m_Vel0 = vec3(0, 0, 0);
	vec3 m_lastVel0 = vec3(0, 0, 0);
	vec3 m_Euler = vec3(0, 0, 0);
	vec3 m_VelB = vec3(0, 0, 0);
	vec3 m_AVelB = vec3(0, 0, 0);
	
	m_Vdesired = vec3(0, 0, 0);
	m_lastThetad = 0.0;

	m_Active = true; 
	mpActiveBehavior = nullptr;
	mLeader = false;

	reset();
}

AActor* BehaviorController::getActor()
{
	return m_pActor;
}

void BehaviorController::setActor(AActor* actor)

{
	m_pActor = actor;
	m_pSkeleton = m_pActor->getSkeleton();

}


void BehaviorController::createBehaviors(vector<AActor>& agentList, vector<Obstacle>& obstacleList)
{
	
	m_AgentList = &agentList;
	m_ObstacleList = &obstacleList;

	m_BehaviorList.clear();
	m_BehaviorList[SEEK] = new Seek(m_pBehaviorTarget);
	m_BehaviorList[FLEE] = new Flee(m_pBehaviorTarget);
	m_BehaviorList[ARRIVAL] = new Arrival(m_pBehaviorTarget);
	m_BehaviorList[DEPARTURE] = new Departure(m_pBehaviorTarget);
	m_BehaviorList[WANDER] = new Wander();
	m_BehaviorList[COHESION] = new Cohesion(m_AgentList);
	m_BehaviorList[ALIGNMENT] = new Alignment(m_pBehaviorTarget, m_AgentList);
	m_BehaviorList[SEPARATION] = new Separation(m_pBehaviorTarget, m_AgentList);
	m_BehaviorList[LEADER] = new Leader(m_pBehaviorTarget, m_AgentList);
	m_BehaviorList[FLOCKING] = new Flocking(m_pBehaviorTarget, m_AgentList);
	m_BehaviorList[AVOID] = new Avoid(m_pBehaviorTarget, m_ObstacleList);
}

BehaviorController::~BehaviorController()
{
	mpActiveBehavior = nullptr;
}

void BehaviorController::reset()
{
	vec3 startPos;
	startPos[0] = ((double)rand()) / RAND_MAX;
	startPos[1] =  ((double)rand()) / RAND_MAX, 
	startPos[2] = ((double)rand()) / RAND_MAX;
	startPos = startPos - vec3(0.5, 0.5, 0.5);

	startPos[1] = 0; // set equal to zero for 2D case (assume y is up)

	m_Guide.setLocalTranslation(startPos * 500.0);
	
	for (int i = 0; i < m_stateDim; i++)
	{
		m_state[i] = 0.0;
		m_stateDot[i] = 0.0;
	}

	m_force = 0.0;
	m_torque = 0.0;
	m_thetad = 0.0;
	m_vd = 0.0;
}

///////////////////////////////////////////////////

inline void ClampAngle(double& angle)
{
	while (angle > M_PI)
	{
		angle -= M2_PI;
	}
	while (angle < -M_PI)
	{
		angle += M2_PI;
	}
}

void BehaviorController::sense(double deltaT)
{
	if (mpActiveBehavior)
	{
		// find the agents in the neighborhood of the current character.
	}
	
}

void BehaviorController::control(double deltaT)
// Given the active behavior this function calculates a desired velocity vector (Vdesired).  
// The desired velocity vector is then used to compute the desired speed (vd) and direction (thetad) commands

{

	if (mpActiveBehavior != nullptr)
	{ 
		m_Vdesired = mpActiveBehavior->calcDesiredVel(this);
		m_Vdesired[1] = 0;

		//  force and torque inputs are computed from vd and thetad as follows:
		//              Velocity P controller : force = mass * Kv * (vd - v)
		//              Heading PD controller : torque = Inertia * (-Kv * thetaDot + Kp * (thetad - theta))
		//  where the values of the gains Kv and Kp are different for each controller

		// TODO: insert your code here to compute m_force and m_torque

		vec3 velVec = vec3(m_Vdesired);

		m_vd = m_Vdesired.Length();

		std::cout << "The Velocity  is " << m_vd << std::endl;

		//velVec.Normalize();

		//vec3 worldVel = m_Vel0.Normalize();

		//float dotProd = acos(Dot(velVec, worldVel));

		//vec3 thetaD = vec3(0, dotProd, 0);

		//vec3 angleInDegree = thetaD * (M_PI) / 180.0;

		double angle = atan2(m_Vdesired[2] , m_Vdesired[0]) - m_Euler[1];

		ClampAngle(angle);

		vec3 angleToRotate = vec3(0, angle, 0);

		gOriKv = 32;
		gOriKp = 256;

		m_force = gMass * gOriKv * (m_Vdesired - m_Vel0);

		m_torque = gInertia * ((-gOriKv * m_stateDot[1]) + (gOriKp * angleToRotate));

		std::cout << "The Torque is " << m_torque[0] << " " << m_torque[1] << " " << m_torque[2] << std::endl;

		// when agent desired agent velocity and actual velocity < 2.0 then stop moving
		if (m_vd < 2.0 &&  m_state[VEL][_Z] < 2.0)
		{
			m_force[2] = 0.0;
			m_torque[1] = 0.0;
		}
	}
	else
	{
		m_force[2] = 0.0;
		m_torque[1] = 0.0;
	}

	// set control inputs to current force and torque values
	m_controlInput[0] = m_force;
	m_controlInput[1] = m_torque;
}

void BehaviorController::act(double deltaT)
{
	computeDynamics(m_state, m_controlInput, m_stateDot, deltaT);
	
	int EULER = 0;
	int RK2 = 1;
	updateState(deltaT, EULER);
}
// m_state[0] = m_Pos0 = [x 0 z]T for the 2D planar case
// m_state[1] = m_Euler = [ 0 theta 0]T for the 2D planar case 
// m_state[2] = m_VelB = [ Vx 0 Vz]T for the 2D planar case
// m_state[3] = m_AVelB =  [ 0 thetaDot 0]T for the 2D planar case 



void BehaviorController::computeDynamics(vector<vec3>& state, vector<vec3>& controlInput, vector<vec3>& stateDot, double deltaT)
// Compute stateDot vector given the control input and state vectors
//  This function sets derive vector to appropriate values after being called
{
	vec3& force = controlInput[0];
	vec3& torque = controlInput[1];

	// Compute the stateDot vector given the values of the current state vector and control input vector
	// TODO: add your code here

	stateDot[0] = m_Vel0;
	stateDot[1] = m_AVelB;
	stateDot[2] = force / gMass;
	stateDot[3] = torque / gInertia;

	//std::cout << "The Torque is " << stateDot[1][0] << " " << stateDot[1][1] << " " << stateDot[1][2] << std::endl;

}

void BehaviorController::updateState(float deltaT, int integratorType)
{
	//  Update the state vector given the m_stateDot vector using Euler (integratorType = 0) or RK2 (integratorType = 1) integratio
	//  this should be similar to what you implemented in the particle system assignment

	// TODO: add your code here

	//std::cout << "The local force is " << m_controlInput[0][0] << " " << m_controlInput[0][1] << " " << m_controlInput[0][2] << std::endl;

	computeDynamics(m_state, m_controlInput, m_stateDot, deltaT);

	vec3 dottk;
	vec3 dottk_1;

	//TODO:  Add your code here to update the state using EULER and Runge Kutta2  integration
	switch (integratorType)
	{
	case 0: // EULER
		
		// Updating Position
		m_state[0] = m_state[0] + m_stateDot[0] * deltaT;

		// Updating Orientation

		//std::cout << "The B4 Orientation is " << m_state[1][0] << " " << m_state[1][1] << " " << m_state[1][2] << std::endl;

		m_state[1] = m_state[1] + m_stateDot[1] * deltaT;

		//std::cout << "The new Orientation is " << m_state[1][0] << " " << m_state[1][1] << " " << m_state[1][2] << std::endl;

		// Updating Velocity
		m_state[2] = m_state[2] + m_stateDot[2] * deltaT;

		// Updating Angular velocity
		m_state[3] = m_state[3] + m_stateDot[3] * deltaT;

		break;

	case 1: // 2nd Order Runge Kutta
	
		dottk = m_stateDot[0]; // Velocity at Tk
		dottk_1 = m_stateDot[0] + m_stateDot[0] * deltaT; // Velocity at Tk + 1
		m_state[0] = m_state[0] + 0.5 * (dottk + dottk_1) * deltaT;

		dottk = m_stateDot[1]; // Orientation at Tk
		dottk_1 = m_stateDot[1] + m_stateDot[1] * deltaT; // Orientation at Tk + 1
		m_state[1] = m_state[1] + 0.5 * (dottk + dottk_1) * deltaT;

		dottk = m_stateDot[2]; // Acceleration at Tk
		dottk_1 = m_stateDot[2] + ((m_force / gMass)) * deltaT; // Acceleration at Tk + 1
		m_state[2] = m_state[2] + 0.5 * (dottk + dottk_1) * deltaT;

		dottk = m_stateDot[3]; // Angular Acceleration at Tk
		dottk_1 = m_stateDot[3] + ((m_torque / gInertia)) * deltaT; // Angular Acceleration at Tk + 1
		m_state[3] = m_state[3] + 0.5 * (dottk + dottk_1) * deltaT;

		break;
	
	}


	//  given the new values in m_state, these are the new component state values 
	m_Pos0 = m_state[POS];
	m_Euler = m_state[ORI];
	m_VelB = m_state[VEL];
	m_AVelB = m_state[AVEL];  
	

	/*for (int i = 0; i < 3; ++i)
	{
		if (m_VelB[i] > gMaxSpeed)
		{
			m_VelB[i] = gMaxSpeed;
			m_state[VEL][i] = gMaxSpeed;
		}
	}

	for (int i = 0; i < 3; ++i)
	{
		if (m_AVelB[i] > gMaxAngularSpeed)
		{
			m_AVelB[i] = gMaxAngularSpeed;
			m_state[AVEL][i] = gMaxAngularSpeed;
		}
	}

	for (int i = 0; i < 3; ++i)
	{
		if (m_force[i] > gMaxForce)
		{
			m_force[i] = gMaxForce;
		}
	}

	for (int i = 0; i < 3; ++i)
	{
		if (m_torque[i] > gMaxTorque)
		{
			m_torque[i] = gMaxTorque;
		}
	}*/


	float speed = m_VelB.Length();

	/*m_Vel0[0] = speed * cos(m_state[ORI][1]);
	m_Vel0[1] = 0;
	m_Vel0[2] = speed * sin(m_state[ORI][1]);*/

	m_Vel0 = m_VelB;



	std::cout << "The Local Angle is " << m_state[ORI][1] << std::endl;

	//m_Vel0 =  m_Guide.getLocal2Global() * m_VelB + m_Guide.getLocalTranslation();

	//std::cout << "The Orientation is " << m_Euler[0] << " " << m_Euler[1] << " " << m_Euler[2] << std::endl;

	//std::cout << "The Global Velocity is " << m_Vel0[0] << " " << m_Vel0[1] << " " << m_Vel0[2] << std::endl;


	//  Perform validation check to make sure all values are within MAX values
	// TODO: add your code here



	// update the guide orientation
	// compute direction from nonzero velocity vector
	vec3 dir;
	if (m_Vel0.Length() < 1.0)
	{
		dir = m_lastVel0;
		dir.Normalize();;
		m_state[ORI] = atan2(dir[_Z], dir[_X]);
	}
	else
	{
		dir = m_Vel0;
		m_lastVel0 = m_Vel0;
	}

	dir.Normalize();
	vec3 up(0.0, 1.0, 0.0);
	vec3 right = up.Cross(dir);

	//std::cout << "Right Vector is " << right[0] << " " << right[1] << " " << right[2] << std::endl;

	right.Normalize();
	mat3 rot(right, up, dir);
	m_Guide.setLocalRotation(rot.Transpose());
	m_Guide.setLocalTranslation(m_Guide.getLocalTranslation() + m_Vel0*deltaT);

}


void BehaviorController::setTarget(AJoint& target)
{
	m_pBehaviorTarget = &target;
	for (unsigned int i = 0; i < m_BehaviorList.size(); i++)
	{
		BehaviorType index = (BehaviorType) i;
		m_BehaviorList[index]->setTarget(m_pBehaviorTarget);
	}



}

void BehaviorController::setActiveBehavior(Behavior* pBehavior)
{
	mpActiveBehavior = pBehavior;
}

void BehaviorController::setActiveBehaviorType(BehaviorType type)
{
	m_BehaviorType = type;
	Behavior* pActiveBehavior = m_BehaviorList[type];
	setActiveBehavior(pActiveBehavior);

}

void BehaviorController::display()
{ // helps with debugging behaviors.  red line is actual velocity vector, green line is desired velocity vector
	
	vec3 pos = getPosition();
	double scale = 1.0;
	vec3 vel = scale* getVelocity();
	double velMag = vel.Length();
	vec3 dvel = scale* getDesiredVelocity();
	vec3 angle = getOrientation() * (180.0 / 3.14159);

	glBegin(GL_LINES);
	glColor3f(1, 0, 0);
	glVertex3f(pos[0], pos[1], pos[2]);
	glVertex3f(pos[0] + vel[0], pos[1] + vel[1], pos[2] + vel[2]);
	glColor3f(0, 1, 0);
	glVertex3f(pos[0], pos[1], pos[2]);
	glVertex3f(pos[0] + dvel[0], pos[1] + dvel[1], pos[2] + dvel[2]);
	glEnd();

	if (this->isLeader())
		glColor3f(0, 0, 1);
	else glColor3f(0.5, 0, 0);

	glPushMatrix();
	glTranslatef(pos[0], pos[1], pos[2]);
	glRotatef(90 - angle[1], 0, 1, 0);
	glutSolidCone(40, 80, 10, 10);
	glutSolidSphere(35, 10, 10);
	glPopMatrix();

	BehaviorType active = getActiveBehaviorType();
	Behavior* pBehavior = m_BehaviorList[active];
	pBehavior->display(this);

}

