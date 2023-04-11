
#include "btBulletDynamicsCommon.h"
#include "BulletCollision/CollisionShapes/btHeightfieldTerrainShape.h"
#include "GLDebugFont.h"

#include "BulletDynamics/MLCPSolvers/btDantzigSolver.h"
#include "BulletDynamics/MLCPSolvers/btSolveProjectedGaussSeidel.h"
#include "BulletDynamics/MLCPSolvers/btMLCPSolver.h"


btScalar maxMotorImpulse = 1400.f;
btVector3 m_cameraTargetPositionpast;
//the sequential impulse solver has difficulties dealing with large mass ratios (differences), between loadMass and the fork parts
btScalar loadMass = 350.f;//350.f;//
//btScalar loadMass = 10.f;//this should work fine for the SI solver


#ifndef M_PI
#define M_PI       3.14159265358979323846
#endif

#ifndef M_PI_2
#define M_PI_2     1.57079632679489661923
#endif

#ifndef M_PI_4
#define M_PI_4     0.785398163397448309616
#endif

//#define LIFT_EPS 0.0000001f
//
// By default, Bullet Vehicle uses Y as up axis.
// You can override the up axis, for example Z-axis up. Enable this define to see how to:
//#define FORCE_ZAXIS_UP 1
//

#ifdef FORCE_ZAXIS_UP
int rightIndex = 0;
int upIndex = 2;
int forwardIndex = 1;
btVector3 wheelDirectionCS0(0, 0, -1);
btVector3 wheelAxleCS(1, 0, 0);
#else
int rightIndex = 0;
int upIndex = 1;
int forwardIndex = 2;
btVector3 wheelDirectionCS0(0, -1, 0);
btVector3 wheelAxleCS(-1, 0, 0);
#endif

bool useMCLPSolver = true;
bool helpToggle = true;
bool scoreExtended = false;

#include "GLDebugDrawer.h"
#include <stdio.h> //printf debugging

#include "GL_ShapeDrawer.h"

#include "GlutStuff.h"
#include "ForkLiftDemo.h"

//control variables ---------
int ARRAY_SIZE_X;
int ARRAY_SIZE_Y;
int ARRAY_SIZE_Z;

float START_POS_X;
float START_POS_Y;
float START_POS_Z;


float start_x;
float start_y;
float start_z;
const int maxProxies = 32766;
const int maxOverlap = 65535;
int first_time = 0;
float global_camdist;
///btRaycastVehicle is the interface for the constraint that implements the raycast vehicle
///notice that for higher-quality slow-moving vehicles, another approach might be better
///implementing explicit hinged-wheel constraints with cylinder collision, rather then raycasts
float	gEngineForce = 0.f;

float	defaultBreakingForce = 10.f;
float	gBreakingForce = 100.f;

float	maxEngineForce = 1000.f;//this should be engine/velocity dependent
float	maxBreakingForce = 100.f;

float	gVehicleSteering = 0.f;
float	steeringIncrement = 0.04f;
float	steeringClamp = 0.3f;
float	wheelRadius = 0.5f;
float	wheelWidth = 0.4f;
float	wheelFriction = 500;//BT_LARGE_FLOAT;
float	suspensionStiffness = 20.f;
float	suspensionDamping = 2.3f;
float	suspensionCompression = 4.4f;
float	rollInfluence = 0.1f;//1.0f;
float chassis_x = 1.f;// 1,2,0.5
float chassis_y = 0.5f;
float chassis_z = 2.f;
float cannon = 1;
float wings = 0;
btCollisionShape* chassisShape2;
btCollisionShape* camcar;
btTransform localTrans;
btTransform tr;
btTransform suppLocalTrans;
btTransform suppLocalTransShoot;
btTransform suppLocalTranscam;
btTransform liftTrans;
// For chassis and camera actor
btCollisionShape* chassisShape;
btCollisionShape* liftShape;
btHingeConstraint* liftHinge2;
btRigidBody* m_carChassis;
btRigidBody* m_liftBody;
btRigidBody* m_canon;
btHingeConstraint* m_liftHinge;

btScalar suspensionRestLength(0.6);

#define CUBE_HALF_EXTENTS 1

btCompoundShape* compound;// = new btCompoundShape();




bool wingsEnabled=false;
bool monsterTruckMode=false;
bool gharpoon=false;
////////////////////////////////////




ForkLiftDemo::ForkLiftDemo()
	:
	//m_carChassis(0),
	//m_canon(0),
	m_shootobject(0),
	//m_liftBody(0),
	m_forkBody(0),
	m_loadBody(0),
	m_indexVertexArrays(0),
	m_vertices(0),
	m_cameraHeight(4.f),
	m_minCameraDistance(3.f),
	m_offsetCameraDistance(10.f),
	m_maxCameraDistance(10.f)
{
	m_vehicle = 0;
	m_wheelShape = 0;
	m_cameraPosition = btVector3(0, 0, 0);//30,30,30
	m_cameraTargetPositionpast = btVector3(0, 0, 0);
	m_useDefaultCamera = false;
	setTexturing(true);
	setShadows(true);

}


void ForkLiftDemo::exitPhysics()
{
	//cleanup in the reverse order of creation/initialization

	//remove the rigidbodies from the dynamics world and delete them
	int i;
        //printf(" In deletion A \n");
	for (i = m_dynamicsWorld->getNumCollisionObjects() - 1; i >= 0; i--)
	{
		btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[i];
		btRigidBody* body = btRigidBody::upcast(obj);
		if (body && body->getMotionState())
		{

			while (body->getNumConstraintRefs())
			{
				btTypedConstraint* constraint = body->getConstraintRef(0);
				m_dynamicsWorld->removeConstraint(constraint);
				delete constraint;
			}
			delete body->getMotionState();
			m_dynamicsWorld->removeRigidBody(body);
		}
		else
		{
			m_dynamicsWorld->removeCollisionObject(obj);
		}
		delete obj;
	}
        //printf(" In deletion B \n");
	//delete collision shapes
	for (int j = 0; j<m_collisionShapes.size(); j++)
	{
		btCollisionShape* shape = m_collisionShapes[j];
		delete shape;
	}
	m_collisionShapes.clear();
 
         //delete m_canon->getMotionState();
	//m_dynamicsWorld->removeRigidBody(m_canon);
        //m_dynamicsWorld->removeConstraint(m_liftHinge);
	//delete m_liftHinge;
	delete m_indexVertexArrays;
	delete m_vertices;

	//delete dynamics world
	delete m_dynamicsWorld;

	delete m_vehicleRayCaster;

	delete m_vehicle;

	delete m_wheelShape;

	//delete solver
	delete m_constraintSolver;

	//delete broadphase;
	delete m_overlappingPairCache;

	//delete dispatcher
	delete m_dispatcher;

	delete m_collisionConfiguration;
        
        printf(" In deletion A \n");

}

ForkLiftDemo::~ForkLiftDemo()
{
	exitPhysics();
}


// *************** Code for Building Instancing *************

void buildingtype1(float START_POS_X, float START_POS_Y, float START_POS_Z, btScalar mass, btTransform startTransform, btVector3 localInertia, btBoxShape* colShape, btDynamicsWorld* m_dynamicsWorld)
{
	ARRAY_SIZE_X = 4;
	ARRAY_SIZE_Y = 8;
	ARRAY_SIZE_Z = 4;

	start_x = START_POS_X - ARRAY_SIZE_X / 2;
	start_y = START_POS_Y;
	start_z = START_POS_Z - ARRAY_SIZE_Z / 2;

	for (int k = 0; k<ARRAY_SIZE_Y; k++)
	{
		for (int i = 0; i<ARRAY_SIZE_X; i++)
		{
			for (int j = 0; j<ARRAY_SIZE_Z; j++)
			{
				startTransform.setOrigin(btVector3(
					btScalar(2.2*i + start_x),
					btScalar(2.2*k + start_y),
					btScalar(2.2*j + start_z)));


				//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
				btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
				btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, colShape, localInertia);
				btRigidBody* body = new btRigidBody(rbInfo);

				m_dynamicsWorld->addRigidBody(body);//,1,1+2);
			}
		}
	}

	START_POS_X = START_POS_X;
	START_POS_Y = 2 * ARRAY_SIZE_Y + 1;
	START_POS_Z = START_POS_Z + ARRAY_SIZE_Z / 2;

	ARRAY_SIZE_X = 2;
	ARRAY_SIZE_Y = 4;
	ARRAY_SIZE_Z = 2;

	start_x = START_POS_X - ARRAY_SIZE_X / 2;
	start_y = START_POS_Y;
	start_z = START_POS_Z - ARRAY_SIZE_Z / 2;

	for (int k = 0; k<ARRAY_SIZE_Y; k++)
	{
		for (int i = 0; i<ARRAY_SIZE_X; i++)
		{
			for (int j = 0; j<ARRAY_SIZE_Z; j++)
			{
				startTransform.setOrigin(btVector3(
					btScalar(2.2*i + start_x),
					btScalar(2.2*k + start_y),
					btScalar(2.2*j + start_z)));


				//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
				btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
				btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, colShape, localInertia);
				btRigidBody* body = new btRigidBody(rbInfo);

				m_dynamicsWorld->addRigidBody(body);//,1,1+2);
			}
		}
	}

}




void ForkLiftDemo::initPhysics()
{
	 m_total_time = 120;
#ifdef FORCE_ZAXIS_UP
	m_cameraUp = btVector3(0, 0, 1);
	m_forwardAxis = 1;
#endif
       
        printf(" In deletion B \n");
	btCollisionShape* groundShape = new btBoxShape(btVector3(100, 50, 100));// 60,50,60
	m_collisionShapes.push_back(groundShape);
	m_collisionConfiguration = new btDefaultCollisionConfiguration();
	m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);
	btDbvtBroadphase * m_broadphase = new btDbvtBroadphase();
	btVector3 worldMin(-10000, -10000, -10000);
	btVector3 worldMax(10000, 10000, 10000);
	//m_overlappingPairCache = new btAxisSweep3(worldMin,worldMax);



	/*if (useMCLPSolver)
	{
	btDantzigSolver* mlcp = new btDantzigSolver();
	//btSolveProjectedGaussSeidel* mlcp = new btSolveProjectedGaussSeidel;
	btMLCPSolver* sol = new btMLCPSolver(mlcp);
	m_constraintSolver = sol;
	} else
	{
	m_constraintSolver = new btSequentialImpulseConstraintSolver();
	}*/

        printf(" In deletion C \n");
	m_constraintSolver = new btSequentialImpulseConstraintSolver();
	//m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher,m_overlappingPairCache,m_constraintSolver,m_collisionConfiguration);
	m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher, m_broadphase, m_constraintSolver, m_collisionConfiguration);
	if (useMCLPSolver)
	{
		m_dynamicsWorld->getSolverInfo().m_minimumSolverBatchSize = 1;//for direct solver it is better to have a small A matrix
	}
	else
	{
		m_dynamicsWorld->getSolverInfo().m_minimumSolverBatchSize = 128;//for direct solver, it is better to solve multiple objects together, small batches have high overhead
	}



#ifdef FORCE_ZAXIS_UP
	//m_dynamicsWorld->setGravity(btVector3(0,0,-10));
#endif 
        printf(" In deletion D \n");
	m_dynamicsWorld->setGravity(btVector3(0, -10, 0));
	//btTransform tr;
	tr.setIdentity();
	tr.setOrigin(btVector3(0, -50, 0));

	//either use heightfield or triangle mesh


	//create ground object
	//localCreateRigidBody(0,tr,groundShape);//***** To create the ground

	/************* The building Code *****************/

	if (1)
	{
		//create a few dynamic rigidbodies
		// Re-using the same collision is better for memory usage and performance

		btBoxShape* colShape = new btBoxShape(btVector3(1, 1, 1));
		//colShape = new btBoxShape(btVector3(1, 1, 1));
                m_colShape = colShape;
		m_collisionShapes.push_back(colShape);

		/// Create Dynamic Objects
		btTransform startTransform;
		startTransform.setIdentity();

		btScalar	mass(350.f);//350
		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(0, 0, 0);
		if (isDynamic)
			colShape->calculateLocalInertia(mass, localInertia);
		//---------------------------

		// One building -----------------------------------------

		for (int j = 0; j < 2; j++){
			for (int i = 0; i < 3; i++){

				START_POS_X = -30 + (50 * i);
				START_POS_Y = 0;
				START_POS_Z = 20 + (50 * j);
				buildingtype1(START_POS_X, START_POS_Y, START_POS_Z, mass, startTransform, localInertia, colShape, m_dynamicsWorld);
			}
		}
	}
	if (1)//useGroundShape
	{
		btScalar mass(0.);

		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(0, 0, 0);
		if (isDynamic)
			groundShape->calculateLocalInertia(mass, localInertia);

		//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
		//btDefaultMotionState* myMotionState = new btDefaultMotionState(groundTransform);
		btDefaultMotionState* myMotionState = new btDefaultMotionState(tr);
		btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, groundShape, localInertia);
		btRigidBody* body = new btRigidBody(rbInfo);

		//add the body to the dynamics world
		m_dynamicsWorld->addRigidBody(body, 1, 1 + 2);//,1,1+2);
	}

	/******************* building code ends *************************/
        
	createcar();
	resetForklift();
	setCameraDistance(5.f);//26

}

void ForkLiftDemo::createcar()
{
#ifdef FORCE_ZAXIS_UP
	//   indexRightAxis = 0; 
	//   indexUpAxis = 2; 
	//   indexForwardAxis = 1; 
	//btCollisionShape* chassisShape = new btBoxShape(btVector3(1.f,2.f, 0.5f));
	/*if(wings == 1){
	btCollisionShape* chassisShape;
	btCollisionShape* liftShape;
	btHingeConstraint* liftHinge2;
	}*/
	//btCompoundShape* compound = new btCompoundShape();//*** to add things to the car
	compound = new btCompoundShape();
	btTransform localTrans;
	localTrans.setIdentity();
	//localTrans effectively shifts the center of mass with respect to the chassis
	localTrans.setOrigin(btVector3(0, 0, 1));//0 0 1
#else
	chassisShape = new btBoxShape(btVector3(chassis_x, chassis_y, chassis_z));// 1,2,0.5 // 2,0.5,2.5
	if (wings == 1){
		chassisShape2 = new btBoxShape(btVector3(3.5f, 0.2f, 0.2f));
		m_collisionShapes.push_back(chassisShape2);
	}
	m_collisionShapes.push_back(chassisShape);
	btCompoundShape* compound = new btCompoundShape();
	//btCompoundShape* compoundcanon = new btCompoundShape();
	m_collisionShapes.push_back(compound);
	//btTransform localTrans;
	localTrans.setIdentity();
	//localTrans effectively shifts the center of mass with respect to the chassis
	localTrans.setOrigin(btVector3(0, 1, 0));// 0 1 0
        if(wings == 1){
             localTrans.setOrigin(btVector3(0, .7, 0));//0.7 - glide number
             
        }
#endif
	if (wings == 1){
		compound->addChildShape(localTrans, chassisShape2);
	}
	compound->addChildShape(localTrans, chassisShape);
	tr.setOrigin(m_cameraPosition);
	m_carChassis = localCreateRigidBody(800, tr, compound);
        if (wings == 1){
            //m_carChassis->setGravity(btVector3(0,0,0));
        }

	//tr.setOrigin(btVector3(0,0.f,0));
	//tr.setOrigin(m_cameraPosition);
	

	{
		liftShape = new btBoxShape(btVector3(1.f, 1.f, 1.f));
		//liftShape = new btSphereShape(btScalar(1));
		m_collisionShapes.push_back(liftShape);
		//btTransform liftTrans;
		m_liftStartPos = btVector3(0.0f, 1.f, 5.f);
		if (wings == 1){
			m_liftStartPos = m_cameraPosition;
		}
		liftTrans.setIdentity();
		//liftTrans.setOrigin(m_liftStartPos);
		m_liftBody = localCreateRigidBody(1, liftTrans, liftShape);

		btTransform localA, localB;
		localA.setIdentity();
		localB.setIdentity();
		localA.getBasis().setEulerZYX(0, M_PI_2, 0);
		localA.setOrigin(btVector3(0.0, 1.5, -9.5));//0, 1.5, -9.5
		localB.getBasis().setEulerZYX(0, M_PI_2, 0);
		localB.setOrigin(btVector3(0.0, -1.5, -1.05));
		liftHinge2 = new btHingeConstraint(*m_carChassis, *m_liftBody, localA, localB);
		liftHinge2->setLimit(0.0f, 0.0f);
		m_dynamicsWorld->addConstraint(liftHinge2, true); //**** Creation of Camera Actor
	}


        {

                btVector3 cyl_coord = btVector3(0.5, 1, 1);
		btCollisionShape* suppShape = new btCylinderShapeZ(cyl_coord);
		suppLocalTrans.setIdentity();
		suppLocalTrans.setOrigin(m_cameraPosition);
		m_canon = localCreateRigidBody(1, suppLocalTrans, suppShape);


		btTransform localC, localD;
		localC.setIdentity();
		localD.setIdentity();
		localC.getBasis().setEulerZYX(0, M_PI_2, 0);
		localC.setOrigin(btVector3(0.0, 0.2, 1.05));//0, 1, 5
		localD.getBasis().setEulerZYX(0, M_PI_2, 0);
		localD.setOrigin(btVector3(0.0, -1.5, -1.05));
		//m_liftHinge2 = new btGeneric6DofConstraint( *m_carChassis,*m_canon, localC, localD, true ); 
		m_liftHinge = new btHingeConstraint(*m_carChassis, *m_canon, localC, localD, true);
		m_liftHinge->setLimit(0.0f, 0.0f, 0.9, 0.4);
		m_dynamicsWorld->addConstraint(m_liftHinge, true);

        }
	/// create vehicle
        
	//{

		m_vehicleRayCaster = new btDefaultVehicleRaycaster(m_dynamicsWorld);
		m_vehicle = new btRaycastVehicle(m_tuning, m_carChassis, m_vehicleRayCaster);
                
		///never deactivate the vehicle
		m_carChassis->setActivationState(DISABLE_DEACTIVATION);

		m_dynamicsWorld->addVehicle(m_vehicle);

                //if(wings != 1){
                m_wheelShape = new btCylinderShapeX(btVector3(wheelWidth, wheelRadius, wheelRadius));

                
		float connectionHeight = 1.2f;


		bool isFrontWheel = true;

		//choose coordinate system
		m_vehicle->setCoordinateSystem(rightIndex, upIndex, forwardIndex); // 0, 1, 2
                
       

                if (wings == 1){
                #ifdef FORCE_ZAXIS_UP
		btVector3 connectionPointCS0(CUBE_HALF_EXTENTS - (4.0*wheelWidth), 2 * CUBE_HALF_EXTENTS - wheelRadius, connectionHeight);
#else
		btVector3 connectionPointCS0(CUBE_HALF_EXTENTS - (4.0*wheelWidth), connectionHeight, 2 * CUBE_HALF_EXTENTS - wheelRadius);
#endif
                //if(wings != 1){
		m_vehicle->addWheel(connectionPointCS0, wheelDirectionCS0, wheelAxleCS, suspensionRestLength, wheelRadius, m_tuning, isFrontWheel);
                //}
#ifdef FORCE_ZAXIS_UP
		connectionPointCS0 = btVector3(-CUBE_HALF_EXTENTS + (4.0*wheelWidth), 2 * CUBE_HALF_EXTENTS - wheelRadius, connectionHeight);
#else
		connectionPointCS0 = btVector3(-CUBE_HALF_EXTENTS + (4.0*wheelWidth), connectionHeight, 2 * CUBE_HALF_EXTENTS - wheelRadius);
#endif
                //if(wings != 1){
		m_vehicle->addWheel(connectionPointCS0, wheelDirectionCS0, wheelAxleCS, suspensionRestLength, wheelRadius, m_tuning, isFrontWheel);
                //}
#ifdef FORCE_ZAXIS_UP
		connectionPointCS0 = btVector3(-CUBE_HALF_EXTENTS + (4.0*wheelWidth), -2 * CUBE_HALF_EXTENTS + wheelRadius, connectionHeight);
#else
		connectionPointCS0 = btVector3(-CUBE_HALF_EXTENTS + (4.0*wheelWidth), connectionHeight, -2 * CUBE_HALF_EXTENTS + wheelRadius);
#endif //FORCE_ZAXIS_UP
		isFrontWheel = false;
                //if(wings != 1){
		m_vehicle->addWheel(connectionPointCS0, wheelDirectionCS0, wheelAxleCS, suspensionRestLength, wheelRadius, m_tuning, isFrontWheel);
                //}
#ifdef FORCE_ZAXIS_UP
		connectionPointCS0 = btVector3(CUBE_HALF_EXTENTS - (4.0*wheelWidth), -2 * CUBE_HALF_EXTENTS + wheelRadius, connectionHeight);
#else
		connectionPointCS0 = btVector3(CUBE_HALF_EXTENTS - (4.0*wheelWidth), connectionHeight, -2 * CUBE_HALF_EXTENTS + wheelRadius);
#endif          
                //if(wings != 1){
		m_vehicle->addWheel(connectionPointCS0, wheelDirectionCS0, wheelAxleCS, suspensionRestLength, wheelRadius, m_tuning, isFrontWheel);
                //}
		for (int i = 0; i<m_vehicle->getNumWheels(); i++)
		{
			btWheelInfo& wheel = m_vehicle->getWheelInfo(i);
			wheel.m_suspensionStiffness = suspensionStiffness;
			wheel.m_wheelsDampingRelaxation = suspensionDamping;
			wheel.m_wheelsDampingCompression = suspensionCompression;
			wheel.m_frictionSlip = wheelFriction;
			wheel.m_rollInfluence = rollInfluence;
		}
                }

                else{

#ifdef FORCE_ZAXIS_UP
		btVector3 connectionPointCS0(CUBE_HALF_EXTENTS-(0.3*wheelWidth),2*CUBE_HALF_EXTENTS-wheelRadius, connectionHeight);
#else
		btVector3 connectionPointCS0(CUBE_HALF_EXTENTS-(0.3*wheelWidth),connectionHeight,2*CUBE_HALF_EXTENTS-wheelRadius);
#endif
                
		m_vehicle->addWheel(connectionPointCS0,wheelDirectionCS0,wheelAxleCS,suspensionRestLength,wheelRadius,m_tuning,isFrontWheel);
                
#ifdef FORCE_ZAXIS_UP
		connectionPointCS0 = btVector3(-CUBE_HALF_EXTENTS+(0.3*wheelWidth),2*CUBE_HALF_EXTENTS-wheelRadius, connectionHeight);
#else
		connectionPointCS0 = btVector3(-CUBE_HALF_EXTENTS+(0.3*wheelWidth),connectionHeight,2*CUBE_HALF_EXTENTS-wheelRadius);
#endif
                
		m_vehicle->addWheel(connectionPointCS0,wheelDirectionCS0,wheelAxleCS,suspensionRestLength,wheelRadius,m_tuning,isFrontWheel);
                
#ifdef FORCE_ZAXIS_UP
		connectionPointCS0 = btVector3(-CUBE_HALF_EXTENTS+(0.3*wheelWidth),-2*CUBE_HALF_EXTENTS+wheelRadius, connectionHeight);
#else
		connectionPointCS0 = btVector3(-CUBE_HALF_EXTENTS+(0.3*wheelWidth),connectionHeight,-2*CUBE_HALF_EXTENTS+wheelRadius);
#endif //FORCE_ZAXIS_UP
		isFrontWheel = false;
                
		m_vehicle->addWheel(connectionPointCS0,wheelDirectionCS0,wheelAxleCS,suspensionRestLength,wheelRadius,m_tuning,isFrontWheel);
                
#ifdef FORCE_ZAXIS_UP
		connectionPointCS0 = btVector3(CUBE_HALF_EXTENTS-(0.3*wheelWidth),-2*CUBE_HALF_EXTENTS+wheelRadius, connectionHeight);
#else
		connectionPointCS0 = btVector3(CUBE_HALF_EXTENTS-(0.3*wheelWidth),connectionHeight,-2*CUBE_HALF_EXTENTS+wheelRadius);
#endif
                
		m_vehicle->addWheel(connectionPointCS0,wheelDirectionCS0,wheelAxleCS,suspensionRestLength,wheelRadius,m_tuning,isFrontWheel);
                
		for (int i = 0; i<m_vehicle->getNumWheels(); i++)
		{
			btWheelInfo& wheel = m_vehicle->getWheelInfo(i);
			wheel.m_suspensionStiffness = suspensionStiffness;
			wheel.m_wheelsDampingRelaxation = suspensionDamping;
			wheel.m_wheelsDampingCompression = suspensionCompression;
			wheel.m_frictionSlip = wheelFriction;
			wheel.m_rollInfluence = rollInfluence;
		}
                
	        }
        
}


//to be implemented by the demo
void ForkLiftDemo::renderme()
{
int scoreValue = DemoApplication::getScore();
	if(scoreValue >=20 && !wingsEnabled)
	{
			wingsEnabled = true;
			wings = 1;
			maxEngineForce = 1000.0f;
			wheelFriction = 100.0f;
			//delete liftHinge2;
			delete m_carChassis->getMotionState();
			m_dynamicsWorld->removeRigidBody(m_carChassis);
			delete m_liftBody->getMotionState();
			m_dynamicsWorld->removeRigidBody(m_liftBody);
                        delete m_canon->getMotionState();
                        m_dynamicsWorld->removeRigidBody(m_canon);
			createcar();
			
	}
	if(scoreValue >=40 && !monsterTruckMode)
	{
			wings = 0;
			maxEngineForce = 1000.0f;
			wheelFriction = 400.0f;
			wingsEnabled = true;
			monsterTruckMode = true;
			delete m_carChassis->getMotionState();
			m_dynamicsWorld->removeRigidBody(m_carChassis);
			delete m_liftBody->getMotionState();
			m_dynamicsWorld->removeRigidBody(m_liftBody);
                        delete m_canon->getMotionState();
                        m_dynamicsWorld->removeRigidBody(m_canon);
			chassis_x = 2.0f;// 2,0.5,2.5
			chassis_y = 0.6;
			chassis_z = 0.5;
                        wheelWidth = 0.6f;
                        createcar();
	}
       if(scoreValue == 60 && !gharpoon)
	{
	  gharpoon = true;
          m_harpoon = 1;
          printf(" code in harpoon set \n");
         
        }


       
	updateCamera();

	ATTRIBUTE_ALIGNED16(btScalar) m[16];
	int i;

	btVector3 wheelColor(1, 0, 0);

	btVector3	worldBoundsMin, worldBoundsMax;
	getDynamicsWorld()->getBroadphase()->getBroadphaseAabb(worldBoundsMin, worldBoundsMax);


        //if(wings !=1){
	for (i = 0; i<m_vehicle->getNumWheels(); i++)
	{
		//synchronize the wheels with the (interpolated) chassis worldtransform
		m_vehicle->updateWheelTransform(i, true);
		//draw wheels (cylinders)
		m_vehicle->getWheelInfo(i).m_worldTransform.getOpenGLMatrix(m);
                
		m_shapeDrawer->drawOpenGL(m, m_wheelShape, wheelColor, getDebugMode(), worldBoundsMin, worldBoundsMax);
	}
        //}

	int lineWidth = 400;
	int xStart = m_glutScreenWidth - lineWidth;
	int yStart = 20;

	if ((getDebugMode() & btIDebugDraw::DBG_NoHelpText) == 0)// **********Commands
	{
		setOrthographicProjection();
		glDisable(GL_LIGHTING);
		glColor3f(0, 0, 0);
		char buf[124];
		if(helpToggle == true)
		{
			
			sprintf(buf, "Up Arrow : Accelerate ");
			GLDebugDrawString(xStart, yStart, buf);
			yStart += 20;
			sprintf(buf, "Down Arrow : Decelerate");
			GLDebugDrawString(xStart, yStart, buf);
			sprintf(buf, "Right Arrow : Right Turn");		
			yStart += 20;
			GLDebugDrawString(xStart, yStart, buf);
			yStart += 20;
			sprintf(buf, "Left Arrow : Left Turn");
			GLDebugDrawString(xStart, yStart, buf);
			yStart += 20;
			sprintf(buf, " . : Fire Cannon");
			GLDebugDrawString(xStart, yStart, buf);
			yStart += 20;
			sprintf(buf, "Space Bar : Reset Car");
			GLDebugDrawString(xStart, yStart, buf);
		}
			sprintf(buf, "F8 - Toggle Help Menu");
			yStart += 20;
			GLDebugDrawString(xStart, yStart, buf);
			resetPerspectiveProjection();
			glEnable(GL_LIGHTING);
	}
	DemoApplication::renderme();
}

void ForkLiftDemo::clientMoveAndDisplay()
{

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);


	{
		int wheelIndex = 2;
		m_vehicle->applyEngineForce(gEngineForce, wheelIndex);
		m_vehicle->setBrake(gBreakingForce, wheelIndex);
		wheelIndex = 3;
		m_vehicle->applyEngineForce(gEngineForce, wheelIndex);
		m_vehicle->setBrake(gBreakingForce, wheelIndex);


		wheelIndex = 0;
		m_vehicle->setSteeringValue(gVehicleSteering, wheelIndex);
		wheelIndex = 1;
		m_vehicle->setSteeringValue(gVehicleSteering, wheelIndex);
                m_speed = m_vehicle->getCurrentSpeedKmHour ();
                //printf(" Speed %f \n",m_vehicle->getCurrentSpeedKmHour ());// Get speed of car

                

	}


	float dt = getDeltaTimeMicroseconds() * 0.000001f;

	if (m_dynamicsWorld)
	{
		//during idle mode, just run 1 simulation step maximum
		int maxSimSubSteps = m_idle ? 1 : 2;
		if (m_idle)
			dt = 1.0 / 420.f;

		int numSimSteps;
		numSimSteps = m_dynamicsWorld->stepSimulation(dt, maxSimSubSteps);

		if (m_dynamicsWorld->getConstraintSolver()->getSolverType() == BT_MLCP_SOLVER)
		{
			btMLCPSolver* sol = (btMLCPSolver*)m_dynamicsWorld->getConstraintSolver();
			int numFallbacks = sol->getNumFallbacks();
			if (numFallbacks)
			{
				static int totalFailures = 0;
				totalFailures += numFallbacks;
				printf("MLCP solver failed %d times, falling back to btSequentialImpulseSolver (SI)\n", totalFailures);
			}
			sol->setNumFallbacks(0);
		}


		//#define VERBOSE_FEEDBACK
#ifdef VERBOSE_FEEDBACK
		if (!numSimSteps)
			printf("Interpolated transforms\n");
		else
		{
			if (numSimSteps > maxSimSubSteps)
			{
				//detect dropping frames
				printf("Dropped (%i) simulation steps out of %i\n", numSimSteps - maxSimSubSteps, numSimSteps);
			}
			else
			{
				printf("Simulated (%i) steps\n", numSimSteps);
			}
		}
#endif //VERBOSE_FEEDBACK

	}







#ifdef USE_QUICKPROF 
	btProfiler::beginBlock("render");
#endif //USE_QUICKPROF 


	renderme();

	//optional but useful: debug drawing
	if (m_dynamicsWorld)
		m_dynamicsWorld->debugDrawWorld();

#ifdef USE_QUICKPROF 
	btProfiler::endBlock("render");
#endif 


	glFlush();
	glutSwapBuffers();

}



void ForkLiftDemo::displayCallback(void)
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	renderme();

	//optional but useful: debug drawing
	if (m_dynamicsWorld)
		m_dynamicsWorld->debugDrawWorld();

	glFlush();
	glutSwapBuffers();
}


void ForkLiftDemo::clientResetScene()
{       
        printf(" In deletion RS \n");
	exitPhysics();
	initPhysics();
}

void ForkLiftDemo::resetForklift()
{
	gVehicleSteering = 0.f;
	gBreakingForce = defaultBreakingForce;
	gEngineForce = 0.f;

	m_carChassis->setCenterOfMassTransform(btTransform::getIdentity());
	m_carChassis->setLinearVelocity(btVector3(0, 0, 0));
	m_carChassis->setAngularVelocity(btVector3(0, 0, 0));
	m_dynamicsWorld->getBroadphase()->getOverlappingPairCache()->cleanProxyFromPairs(m_carChassis->getBroadphaseHandle(), getDynamicsWorld()->getDispatcher());
	if (m_vehicle)
	{
		m_vehicle->resetSuspension();
		for (int i = 0; i<m_vehicle->getNumWheels(); i++)
		{
			//synchronize the wheels with the (interpolated) chassis worldtransform
			m_vehicle->updateWheelTransform(i, true);
		}
	}
	btTransform liftTrans;
	liftTrans.setIdentity();
	liftTrans.setOrigin(m_liftStartPos);

	btTransform forkTrans;
	forkTrans.setIdentity();
	forkTrans.setOrigin(m_forkStartPos);

	btTransform loadTrans;
	loadTrans.setIdentity();
	loadTrans.setOrigin(m_loadStartPos);
        printf(" In deletion H \n");
	//m_loadBody->activate();
	//m_loadBody->setCenterOfMassTransform(loadTrans);
	//m_loadBody->setLinearVelocity(btVector3(0,0,0));
	//m_loadBody->setAngularVelocity(btVector3(0,0,0));

}



void ForkLiftDemo::specialKeyboardUp(int key, int x, int y)
{
	switch (key)
	{
		case GLUT_KEY_UP:
		{
			//lockForkSlider(); maine kiya hai
			gEngineForce = 0.f;
			gBreakingForce = defaultBreakingForce;
			break;
		}
		case GLUT_KEY_DOWN:
		{
			//lockForkSlider(); maine kiya hai
			gEngineForce = 0.f;
			gBreakingForce = defaultBreakingForce;
			break;
		}
		case GLUT_KEY_LEFT:
		case GLUT_KEY_RIGHT:
		{
			//lockLiftHinge();
			break;
		}	
		default:
			DemoApplication::specialKeyboardUp(key, x, y);
			break;
	}
}


void ForkLiftDemo::specialKeyboard(int key, int x, int y)
{

	if (key == GLUT_KEY_END)
		return;

	//	printf("key = %i x=%i y=%i\n",key,x,y);

	int state;
	state = glutGetModifiers();
	
		switch (key)
		{
		case GLUT_KEY_F8:
		{
			if(helpToggle == true)
			{
				helpToggle = false;
			}
			else
			{
				helpToggle = true;
			}
		}
		case GLUT_KEY_LEFT:
		{
			gVehicleSteering += steeringIncrement;
			if (gVehicleSteering > steeringClamp)
				gVehicleSteering = steeringClamp;
                        
			//updateCamera();
			break;
		}
		case GLUT_KEY_RIGHT:
		{
			gVehicleSteering -= steeringIncrement;
			if (gVehicleSteering < -steeringClamp)
				gVehicleSteering = -steeringClamp;
                        
			//updateCamera();
			break;
		}
		case GLUT_KEY_UP:
		{       // Put the code for increasing speed here dynamically acc to points
			gEngineForce = maxEngineForce;
			gBreakingForce = 0.f;//0
                        //if(wings == 1)
                           //gEngineForce = 2*maxEngineForce;
			//updateCamera();
			break;
		}
		case GLUT_KEY_DOWN:
		{
			gEngineForce = -maxEngineForce;
			gBreakingForce = 0.f;//0
                        //if(wings == 1)
                           //gEngineForce = -2*maxEngineForce;
                        
			//updateCamera();
			break;
		}
		case GLUT_KEY_F5:
			m_useDefaultCamera = !m_useDefaultCamera;
			break;
		case GLUT_KEY_F3:
		{
                        delete m_carChassis->getMotionState();
			m_dynamicsWorld->removeRigidBody(m_carChassis);
			delete m_liftBody->getMotionState();
			m_dynamicsWorld->removeRigidBody(m_liftBody);
			chassis_x = 2.0f;// 2,0.5,2.5
			chassis_y = 0.6;
			chassis_z = 0.5;
                        wheelWidth = 0.6f;
                        createcar();
			//initPhysics();
			break;
		}
		case GLUT_KEY_F2:
		{
			cannon = 1;
			//initPhysics();
			if (cannon == 1){
				btVector3 cyl_coord = btVector3(0.5, 1, 1);
				btCollisionShape* suppShape = new btCylinderShapeZ(cyl_coord);
				suppLocalTrans.setIdentity();
				suppLocalTrans.setOrigin(m_cameraPosition);
				m_canon = localCreateRigidBody(1, suppLocalTrans, suppShape);


				btTransform localC, localD;
				localC.setIdentity();
				localD.setIdentity();
				localC.getBasis().setEulerZYX(0, M_PI_2, 0);
				localC.setOrigin(btVector3(0.0, 0.2, 1.05));//0, 1, 5
				localD.getBasis().setEulerZYX(0, M_PI_2, 0);
				localD.setOrigin(btVector3(0.0, -1.5, -1.05));
				//m_liftHinge2 = new btGeneric6DofConstraint( *m_carChassis,*m_canon, localC, localD, true ); 
				m_liftHinge = new btHingeConstraint(*m_carChassis, *m_canon, localC, localD, true);
				m_liftHinge->setLimit(0.0f, 0.0f, 0.9, 0.4);
				//m_liftHinge->enableAngularMotor(false, 0, 0);
				//m_liftHinge->setParam( BT_CONSTRAINT_STOP_CFM, 0, -1);
				//m_liftHinge->setParam(BT_CONSTRAINT_STOP_ERP, 2, 10);
				//m_liftHinge->enableAngularMotor(false, 1,10);
				m_dynamicsWorld->addConstraint(m_liftHinge, true);
                                cannon = 0;
			}
			break;
		}
		default:
			DemoApplication::specialKeyboard(key, x, y);
			break;
		}
	
	glutPostRedisplay();


}



void	ForkLiftDemo::updateCamera()
{

	//#define DISABLE_CAMERA 1
	btVector3 m_cameraCanonPositiontemp;
	m_cameraCanonPosition[0] = 0;
	m_cameraCanonPosition[1] = 0;
	m_cameraCanonPosition[2] = 0;

	if (m_useDefaultCamera)
	{
		DemoApplication::updateCamera();
		return;
	}

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	btTransform chassisWorldTrans;
	btTransform carcampos;
	btTransform suppLocalTrans2;

	//look at the vehicle
	m_carChassis->getMotionState()->getWorldTransform(chassisWorldTrans);
	m_cameraTargetPosition = chassisWorldTrans.getOrigin();

	m_liftBody->getMotionState()->getWorldTransform(carcampos);
	m_cameraPosition = carcampos.getOrigin();

	// Canon Transforms
	if (cannon == 1){
		m_canon->getMotionState()->getWorldTransform(suppLocalTrans2);
		m_cameraCanonPositiontemp = suppLocalTrans2.getOrigin();
		m_cameraCanonPosition[0] = m_cameraCanonPositiontemp[0];//0,1.0,2.5
		m_cameraCanonPosition[1] = m_cameraCanonPositiontemp[1];//0.2, 0.5, 1
		m_cameraCanonPosition[2] = m_cameraCanonPositiontemp[2];
	}

	btVector3 camToObject = m_cameraCanonPosition - m_cameraPosition;
	float cameraDistance = camToObject.length();
	float correctionFactor = 0.f;
	correctionFactor = (m_offsetCameraDistance - cameraDistance) / cameraDistance;
	m_cameraCanonPosition -= correctionFactor*camToObject;

	//update OpenGL camera settings
	btScalar aspect = m_glutScreenWidth / (btScalar)m_glutScreenHeight;
	glFrustum(-aspect, aspect, -1.0, 1.0, 1.0, 10000.0);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	gluLookAt(m_cameraPosition[0], m_cameraPosition[1], m_cameraPosition[2],
		m_cameraTargetPosition[0], m_cameraTargetPosition[1], m_cameraTargetPosition[2],
		m_cameraUp.getX(), m_cameraUp.getY(), m_cameraUp.getZ());
}
