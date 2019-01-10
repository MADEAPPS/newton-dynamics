/* Copyright (c) <2003-2016> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#include "toolbox_stdafx.h"
#include "SkyBox.h"
#include "TargaToOpenGl.h"
#include "DemoMesh.h"
#include "DemoEntityManager.h"
#include "DemoCamera.h"
#include "PhysicsUtils.h"
#include "HeightFieldPrimitive.h"
#include "DebugDisplay.h"


#define HEAVY_VEHICLE_THIRD_PERSON_VIEW_HIGHT		2.0f
#define HEAVY_VEHICLE_THIRD_PERSON_VIEW_DIST		12.0f
#define HEAVY_VEHICLE_THIRD_PERSON_VIEW_FILTER		0.125f

#if 0
struct VehicleParameters
{
	dFloat m_vehicleMass;
	dFloat m_engineMass;
	dFloat m_tireMass;
	dFloat m_engineRotorRadio;
	dFloat m_frontSteeringAngle;
	dFloat m_rearSteeringAngle;
	dFloat m_vehicleWeightDistribution;
	dFloat m_cluthFrictionTorque;
	dFloat m_engineIdleTorque;
	dFloat m_engineRPMAtIdleTorque;
	dFloat m_enginePeakTorque;
	dFloat m_engineRPMAtPeakTorque;
	dFloat m_enginePeakHorsePower;
	dFloat m_egineRPMAtPeakHorsePower;
	dFloat m_engineRPMAtRedLine;
	dFloat m_vehicleTopSpeed;
	dFloat m_tireLaretalStiffeness;
	dFloat m_tireLongitudinalStiffness;
	dFloat m_tireAligningMomemtTrail;
	dFloat m_tireSuspensionSpringConstant;
	dFloat m_tireSuspensionDamperConstant;
	dFloat m_tireSuspensionLength;
	dSuspensionType m_TireSuspensionType;
	dFloat m_tireBrakesTorque;
	dFloat m_chassisYaxisComBias;
	dFloat m_transmissionGearRatio0;
	dFloat m_transmissionGearRatio1;
	dFloat m_transmissionGearRatio2;
	dFloat m_transmissionGearRatio3;
	dFloat m_transmissionGearRatio4;
	dFloat m_transmissionGearRatio6;
	dFloat m_transmissionRevereGearRatio;
	dFloat m_tirePivotOffset;
	dFloat m_aerodynamicsDownForceWeightCoeffecient0;
	dFloat m_aerodynamicsDownForceWeightCoeffecient1;
	dFloat m_aerodynamicsDownForceSpeedFactor;
	int m_wheelHasCollisionFenders;
	int m_differentialType; // 0 rear wheel drive, 1 front wheel drive, 3 four wheel drive
	dMatrix m_aligmentMatrix;
};


static VehicleParameters heavyTruck = 
{
	2000.0f,							// m_vehicleMass
	100.0f,								// m_engineMass
	 75.0f,								// m_tireMass
	0.125f,								// m_engineRotorRadio
	 
	  25.0f,							// m_frontSteeringAngle
	   0.0f,							// m_rearSteeringAngle
	   0.5f,							// m_vehicleWeightDistribution

    10000.0f,							// m_cluthFrictionTorque
	
	3500.0f,							// IDLE_TORQUE
	 700.0f,							// IDLE_TORQUE_RPM
	2800.0f,							// PEAK_TORQUE
	3000.0f,							// PEAK_TORQUE_RPM
	2000.0f,							// PEAK_HP
	4000.0f,							// PEAK_HP_RPM
	4500.0f,							// REDLINE_RPM

	60.0f,								// VEHICLE_TOP_SPEED_KMH

	1.0f,								// LATERAL_STIFFNESS
	1.0f,								// LONGITUDINAL_STIFFNESS
	1.5f,								// ALIGNING_MOMENT_TRAIL
	
	100.0f,								// SUSPENSION_SPRING
	10.0f,								// SUSPENSION_DAMPER
	0.7f,								// SUSPENSION_LENGTH
	m_offroad,							//TIRE_SUSPENSION_TYPE

	10000.0f,							// BRAKE_TORQUE
	-0.6f,								// COM_Y_OFFSET

	2.5f,								// GEAR_1
	2.0f,								// GEAR_2
	1.5f,								// GEAR_3
	1.5f,								// GEAR_4
	1.5f,								// GEAR_5
	1.5f,								// GEAR_5
	2.9f,								// REVERSE_GEAR
	0.15f,								// TIRE_PIVOT_OFFSET_Y

	1.0f,								// DOWNFORCE_WEIGHT_FACTOR_0
	2.0f,								// DOWNFORCE_WEIGHT_FACTOR_1
	80.0f, 								// DOWNFORCE_WEIGHT_FACTOR_SPEED
	0,									// WHEEL_HAS_FENDER
	2,									// DIFFERENTIAL_TYPE
	dRollMatrix(90.0f * dDegreeToRad),
};

static VehicleParameters lightTruck = 
{
	2000.0f,							// m_vehicleMass
	100.0f,								// m_engineMass
	75.0f,								// m_tireMass
	0.125f,								// m_engineRotorRadio

	25.0f,							// m_frontSteeringAngle
	0.0f,							// m_rearSteeringAngle
	0.5f,							// m_vehicleWeightDistribution

	10000.0f,							// m_cluthFrictionTorque

	3500.0f,							// IDLE_TORQUE
	700.0f,							// IDLE_TORQUE_RPM
	2800.0f,							// PEAK_TORQUE
	3000.0f,							// PEAK_TORQUE_RPM
	2000.0f,							// PEAK_HP
	4000.0f,							// PEAK_HP_RPM
	4500.0f,							// REDLINE_RPM

	60.0f,								// VEHICLE_TOP_SPEED_KMH

	1.0f,								// LATERAL_STIFFNESS
	1.0f,								// LONGITUDINAL_STIFFNESS
	1.5f,								// ALIGNING_MOMENT_TRAIL

	100.0f,								// SUSPENSION_SPRING
	10.0f,								// SUSPENSION_DAMPER
	0.7f,								// SUSPENSION_LENGTH
	m_offroad,						//TIRE_SUSPENSION_TYPE

	10000.0f,							// BRAKE_TORQUE
	-0.6f,								// COM_Y_OFFSET

	2.5f,								// GEAR_1
	2.0f,								// GEAR_2
	1.5f,								// GEAR_3
	1.5f,								// GEAR_4
	1.5f,								// GEAR_5
	1.5f,								// GEAR_5
	2.9f,								// REVERSE_GEAR

	0.15f,								// TIRE_PIVOT_OFFSET_Y

	1.0f,								// DOWNFORCE_WEIGHT_FACTOR_0
	2.0f,								// DOWNFORCE_WEIGHT_FACTOR_1
	80.0f, 								// DOWNFORCE_WEIGHT_FACTOR_SPEED
	0,									// WHEEL_HAS_FENDER
	2,									// DIFFERENTIAL_TYPE
	dRollMatrix(90.0f * dDegreeToRad),
};

static VehicleParameters m1a1Param = 
{
/*
	5000.0f,								// MASS
	100.0f,									// TIRE_MASS
	0.125f,									// ENGINE_RADIO
	100.0f,									// TIRE_MASS
	 0.0f,									// STEER_ANGLE
	20000.0f,								// BRAKE_TORQUE
	-0.6f,									// COM_Y_OFFSET
	60.0f,									// VEHICLE_TOP_SPEED_KMH
	10000.0f,								// CLUTCH_FRICTION_TORQUE
	3500.0f,								// IDLE_TORQUE
	500.0f,									// IDLE_TORQUE_RPM
	1200.0f,								// PEAK_TORQUE
	3000.0f,								// PEAK_TORQUE_RPM
	800.0f,									// PEAK_HP
	4000.0f,								// PEAK_HP_RPM
	4500.0f,								// REDLINE_RPM
	2.5f,									// GEAR_1
	2.0f,									// GEAR_2
	1.5f,									// GEAR_3
	2.9f,									// REVERSE_GEAR
	1.2f,									// SUSPENSION_LENGTH
	350.0f,									// SUSPENSION_SPRING
	20.0f,									// SUSPENSION_DAMPER
	0.10f,									// TIRE_PIVOT_OFFSET_Y
	dBodyPartTire::Info::m_offroad, //TIRE_SUSPENSION_TYPE
	(5000.0f * DEMO_GRAVITY * 10.0f),		// LATERAL_STIFFNESS
	(5000.0f * DEMO_GRAVITY *  2.0f),		// LONGITUDINAL_STIFFNESS
	1.5f,									// ALIGNING_MOMENT_TRAIL
	dRollMatrix(90.0f * dDegreeToRad),
	0,										// WHEEL_HAS_FENDER	
	0,										// DIFFERENTIAL_TYPE
*/

	2000.0f,							// m_vehicleMass
	100.0f,								// m_engineMass
	75.0f,								// m_tireMass
	0.125f,								// m_engineRotorRadio

	0.0f,								// m_frontSteeringAngle
	0.0f,								// m_rearSteeringAngle
	0.5f,								// m_vehicleWeightDistribution

	10000.0f,							// m_cluthFrictionTorque

	3500.0f,							// IDLE_TORQUE
	700.0f,							// IDLE_TORQUE_RPM
	2800.0f,							// PEAK_TORQUE
	3000.0f,							// PEAK_TORQUE_RPM
	2000.0f,							// PEAK_HP
	4000.0f,							// PEAK_HP_RPM
	4500.0f,							// REDLINE_RPM

	60.0f,								// VEHICLE_TOP_SPEED_KMH

	1.0f,								// LATERAL_STIFFNESS
	1.0f,								// LONGITUDINAL_STIFFNESS
	1.5f,								// ALIGNING_MOMENT_TRAIL

	2000.0f,							// SUSPENSION_SPRING
	100.0f,								// SUSPENSION_DAMPER
	0.7f,								// SUSPENSION_LENGTH
	m_offroad,							//TIRE_SUSPENSION_TYPE

	10000.0f,							// BRAKE_TORQUE
	-0.6f,								// COM_Y_OFFSET

	2.5f,								// GEAR_1
	2.0f,								// GEAR_2
	1.5f,								// GEAR_3
	1.5f,								// GEAR_4
	1.5f,								// GEAR_5
	1.5f,								// GEAR_6
	2.9f,								// REVERSE_GEAR

   -0.0f,								// TIRE_PIVOT_OFFSET_Y

	1.0f,								// DOWNFORCE_WEIGHT_FACTOR_0
	2.0f,								// DOWNFORCE_WEIGHT_FACTOR_1
	80.0f, 								// DOWNFORCE_WEIGHT_FACTOR_SPEED
	0,									// WHEEL_HAS_FENDER
	1,									// DIFFERENTIAL_TYPE
	dRollMatrix(90.0f * dDegreeToRad),
};


class HeavyVehicleEntity: public DemoEntity
{
	public: 

	enum DrivingState
	{
		m_engineOff,
		m_engineIdle,
		m_engineStop,
		m_driveForward,
		m_preDriveForward,
		m_driveReverse,
		m_preDriveReverse,
	};


	class TrackSystem
	{
		public:
		class ConstantSpeedKnotInterpolant
		{
			public:
			dFloat m_u;
			dFloat m_dist;
		};

		TrackSystem()
			:m_bezierEntity(NULL)
			,m_bezierMesh(NULL)
			,m_intepolants(NULL)
			,m_linksEntity(NULL)
			,m_controlPointsOffset(NULL)
			,m_controlPointBindIndex(NULL)
		{
		}

		~TrackSystem()
		{
			if (m_intepolants) {
				delete[] m_linksEntity;
				delete[] m_intepolants;
			}
			if (m_controlPointsOffset) {
				delete[] m_controlPointsOffset;
				delete[] m_controlPointBindIndex;
			}
		}

		dFloat GetTireRadius (DemoEntity* const tire) const
		{
			dAssert (0);
/*
			for (dList<dBodyPartTire>::dListNode* node = m_vehicle->m_controller->GetFirstTire(); node; node = m_vehicle->m_controller->GetNextTire(node)) {
				const dBodyPartTire* const part = &node->GetInfo();
				if (part->GetUserData() == tire) {
					return part->GetInfo().m_radio;
				}
			}
			dAssert (0);
*/
			return 0.0f;
		}

		void CalculaterBinding()
		{
			dAssert (0);
/*
			m_controlPointCount = m_bezierMesh->m_curve.GetControlPointCount();
			m_controlPointsOffset = new dVector[m_controlPointCount];
			m_controlPointBindIndex = new int[m_controlPointCount];

			DemoEntity* const tire = (DemoEntity*) m_tire->GetUserData();
			dString name (tire->GetName());
			name.Replace(name.Find ("0"), 1, "", 0);

			dFloat radius[8];  
			dMatrix tireMatrix[8];  
			for (int i = 0; i < 8; i ++) {
				dString name1 (name + i);
				dAssert (m_vehicle->Find(name1.GetStr()));
				DemoEntity* const tireEnt = m_vehicle->Find(name1.GetStr());
				m_bindingTires[i] = tireEnt;
				radius[i] = GetTireRadius(tireEnt);
				tireMatrix[i] = tireEnt->GetMeshMatrix() * tireEnt->GetCurrentMatrix();
			}

			for (int i = 0; i < m_controlPointCount; i ++) {
				m_controlPointBindIndex[i] = -1;
				dBigVector q (m_bezierMesh->m_curve.GetControlPoint(i));
				m_controlPointsOffset[i] = m_shapeMatrix.TransformVector(dVector (q.m_x, q.m_y, q.m_z, q.m_w));
			}

			for (int i = 0; i < m_controlPointCount; i ++) {
				int index = -1;
				dFloat dist2 = 1.0e10f;

				const dVector& p0 = m_controlPointsOffset[i];
				for (int j = 0; j < 8; j ++) {
					dVector dist (p0 - tireMatrix[j].m_posit);
					dFloat mag2 = dist.DotProduct3(dist);
					if (mag2 < dist2) {
						dist2 = mag2;
						index = j;
					}
				}
				dAssert (index != -1);
				dFloat r2 = (radius[index] + 0.2f) * (radius[index] + 0.2f);
				if (dist2 < r2) {
					m_controlPointBindIndex[i] = index;
					m_controlPointsOffset[i] -= tireMatrix[index].m_posit;
				}
			}
*/
		}

		void CalculaterUniformSpaceSamples()
		{
			DemoEntity* const threadMesh = m_vehicle->Find ("threadShoe");
			dAssert (threadMesh);
			dFloat threadSize = 0.29f;
			m_linksCount = int (m_length / threadSize) + 1;
			threadSize = m_length / m_linksCount;
			
			m_linksEntity = new DemoEntity*[m_linksCount];
			for (int i = 0; i < m_linksCount; i ++) {
				DemoEntity* const threadPart = new DemoEntity (dGetIdentityMatrix(), m_vehicle);
				threadPart->SetMesh(threadMesh->GetMesh(), dGetIdentityMatrix());
				m_linksEntity[i] = threadPart;
			}


			// create the uniform speed knot interpolation table
			ConstantSpeedKnotInterpolant steps[1024];
			steps[0].m_u = 0.0f;
			steps[0].m_dist = 0.0f;

			int count = 0;
			steps[count].m_u = 0.0f;
			steps[count].m_dist = 0.0f;
			count ++;
			int samplingRate = m_linksCount * 10;
			dFloat distAcc = 0.0f;

			threadSize *= 0.25f;
			dFloat stepAcc = threadSize;
			dBigVector q (m_bezierMesh->m_curve.CurvePoint(0.0f));
			dVector p0 (q);
			for (int i = 1; i < samplingRate + 15; i ++) {
				dFloat u = dFloat (i) / samplingRate;
				dBigVector q (m_bezierMesh->m_curve.CurvePoint(dMod (u, 1.0f)));
				dVector p1 (q);
				dVector err (p1 - p0);
				dFloat errMag = dSqrt (err.DotProduct3(err));
				distAcc += errMag;
				if (distAcc > stepAcc) {
					stepAcc += threadSize;
					steps[count].m_u = u;
					steps[count].m_dist = distAcc;
					count ++;
					dAssert (count < int (sizeof (steps) / sizeof (steps[0])));
				}
				p0 = p1;
			}

			m_interpolantsCount = count;
			m_intepolants = new ConstantSpeedKnotInterpolant[count];
			for (int i = 0; i < count; i ++) {
				m_intepolants[i] = steps[i];
			}

			m_aligmentMatrix = dRollMatrix(90.0f * dDegreeToRad) * dPitchMatrix(dPi * 180.0f / 180.0f);
			m_shapeMatrix = m_bezierEntity->GetMeshMatrix() * m_bezierEntity->GetCurrentMatrix();
		}

		void Init (HeavyVehicleEntity* const me, const char* const name, dWheelJoint* const tire)
		{
			dAssert (0);
/*
			m_vehicle = me;
			m_tire = tire;
			m_parameter = 0.0f;
			m_bezierEntity = me->Find (name);
			dAssert (m_bezierEntity);
			m_bezierMesh = (DemoBezierCurve*) m_bezierEntity->GetMesh();
			dAssert (m_bezierMesh->IsType(DemoBezierCurve::GetRttiType()));

			m_length = dFloat (m_bezierMesh->m_curve.CalculateLength (0.01f));

			m_bezierEntity = me->Find (name);
			CalculaterUniformSpaceSamples();
			CalculaterBinding();

			NewtonWorld* const world = NewtonBodyGetWorld (m_tire->GetController()->GetBody());
			AnimateThread (*((DemoEntityManager*)NewtonWorldGetUserData(world)), 0.0f);
			AnimateThread (*((DemoEntityManager*)NewtonWorldGetUserData(world)), 0.0f);
*/
		}

		dFloat CalculateKnotParam (dFloat t) const
		{
			int low = 0;
			int high = m_interpolantsCount - 1;

			while ((high - low) >= 4) {
				int mid = (low + high) >> 1;
				if (t > m_intepolants[mid].m_dist) {
					low = mid;
				} else {
					high = mid;
				}
			}

			dAssert (m_intepolants[low].m_dist <= t);
			for (int i = low; i < m_interpolantsCount; i ++) {
				if (m_intepolants[i + 1].m_dist >= t) {
					low = i;
					break;
				}
			}

			dFloat u0 = m_intepolants[low].m_u;
			dFloat h0 = m_intepolants[low].m_dist;
			dFloat du = m_intepolants[low + 1].m_u - u0;
			dFloat dh = m_intepolants[low + 1].m_dist - h0;
			return dMod (u0 + du * (t - h0) / dh, 1.0f);
		}

		void InterpolateMatrix (DemoEntityManager& world, dFloat param)
		{
			for (int i = 0; i < m_linksCount; i ++) {
				DemoEntity* const threadPart = m_linksEntity[i];
				threadPart->InterpolateMatrix (world, param);
			}
		}

		void AnimateThread (DemoEntityManager& world, dFloat timestep) 
		{
			dAssert (0);
/*
			dVector matrixPalette[8];
			for (int i = 0; i < 8; i ++) {
				matrixPalette[i] = (m_bindingTires[i]->GetMeshMatrix() * m_bindingTires[i]->GetCurrentMatrix()).m_posit;
			}

			for (int i = 0; i < m_controlPointCount; i ++) {
				if (m_controlPointBindIndex[i] != -1) {
					int index = m_controlPointBindIndex[i];
					dVector p (m_controlPointsOffset[i] + matrixPalette[index]);
					m_bezierMesh->m_curve.SetControlPoint (i, m_shapeMatrix.UntransformVector(p));
				}
			}

			dFloat radio = m_tire->GetInfo().m_radio;
			dFloat omega = m_tire->GetRPM() / 9.55f;

			dFloat step = omega * radio * timestep;

			m_parameter = dMod (m_parameter + step, m_length);
			if (m_parameter < 0.0f) {
				m_parameter += m_length;
			}
			dAssert (m_parameter >= 0.0f);
			dAssert (m_parameter <= m_length);

			dMatrix matrix (dGetIdentityMatrix());
			dFloat size = m_length / m_linksCount;
			dFloat u0 = CalculateKnotParam (dMod (m_parameter + (m_linksCount - 1) * size, m_length));
			dVector q0 (m_bezierMesh->m_curve.CurvePoint(u0));
			for (int i = 0; i < m_linksCount; i ++) {
				DemoEntity* const threadPart = m_linksEntity[i];
				dFloat s = dMod (m_parameter + i * size, m_length);
				dFloat u1 = CalculateKnotParam (s);
				dVector q1 (m_bezierMesh->m_curve.CurvePoint(u1));
				dVector dir (q1 - q0);

				dir.m_x = 0.0f;
				dir = dir.Scale (1.0f / dSqrt (dir.DotProduct3(dir)));
				matrix.m_front = dVector (dir.m_z, -dir.m_y, 0.0f, 0.0f);
				matrix.m_up = dVector (dir.m_y, dir.m_z, 0.0f, 0.0f);
				matrix.m_right = dVector (0.0f, 0.0f, 1.0f, 0.0f);
				matrix = m_aligmentMatrix * matrix;
				matrix.m_posit = m_shapeMatrix.TransformVector (q0);
				threadPart->SetMatrix (world, dQuaternion(matrix), matrix.m_posit);
				q0 = q1;
				u0 = u1;
			}
*/
		}

		dMatrix m_shapeMatrix;
		dMatrix m_aligmentMatrix;
		
		HeavyVehicleEntity* m_vehicle;
		DemoEntity* m_bezierEntity;
		DemoBezierCurve* m_bezierMesh;
		dWheelJoint* m_tire;
		ConstantSpeedKnotInterpolant* m_intepolants;
		DemoEntity* m_bindingTires[8];
		DemoEntity** m_linksEntity;
		dVector* m_controlPointsOffset;
		int* m_controlPointBindIndex;

		dFloat m_length;
		dFloat m_parameter;
		int m_linksCount;
		int m_controlPointCount;
		int m_interpolantsCount;
	};

	class TireAligmentTransform: public UserData
	{
		public: 
		TireAligmentTransform (const dMatrix& matrix)
			:UserData()
			,m_matrix (matrix)
		{
		}

		virtual void OnRender (dFloat timestep) const{};

		dMatrix m_matrix;
	};

	HeavyVehicleEntity (DemoEntityManager* const scene, dCustomVehicleControllerManager* const manager, const dMatrix& location, const char* const filename, const VehicleParameters& parameters)
		:DemoEntity (dGetIdentityMatrix(), NULL)
		,m_controller(NULL)
		,m_gearUpKey (false)
		,m_gearDownKey (false)
		,m_reverseGear (false)
		,m_engineKeySwitch(false)
		,m_engineDifferentialLock(false)
		,m_automaticTransmission(true)
		,m_engineRPMOn(false)
		,m_drivingState(m_engineOff)
	{
		// add this entity to the scene for rendering
		scene->Append(this);

		// load the visual mesh 
		LoadNGD_mesh (filename, scene->GetNewton());

		// place entity in the world
		ResetMatrix (*scene, location);

		NewtonWorld* const world = scene->GetNewton();

		// create the car rigid body
		//NewtonBody* const body = CreateChassisBody (carEntity, world, materialID, params);
		NewtonCollision* const chassisCollision = CreateChassisCollision (world);

		// create the vehicle controller
		dMatrix chassisMatrix;
		chassisMatrix.m_front = dVector (1.0f, 0.0f, 0.0f, 0.0f);			// this is the vehicle direction of travel
		chassisMatrix.m_up	  = dVector (0.0f, 1.0f, 0.0f, 0.0f);			// this is the downward vehicle direction
		chassisMatrix.m_right = chassisMatrix.m_front.CrossProduct(chassisMatrix.m_up);	// this is in the side vehicle direction (the plane of the wheels)
		chassisMatrix.m_posit = dVector (0.0f, 0.0f, 0.0f, 1.0f);

		// create a default vehicle 
		m_controller = manager->CreateVehicle (chassisCollision, chassisMatrix, parameters.m_vehicleMass, PhysicsApplyGravityForce, dAbs(DEMO_GRAVITY));
		NewtonBody* const chassis = m_controller->GetBody();
		NewtonBodySetUserData(chassis, this);


		// get body from player
		NewtonBody* const body = m_controller->GetBody();

		// set the user data
		NewtonBodySetUserData(body, this);

		// set the transform callback
		NewtonBodySetTransformCallback (body, DemoEntity::TransformCallback);

		// set the standard force and torque call back
		NewtonBodySetForceAndTorqueCallback(body, PhysicsApplyGravityForce);

		// set the player matrix 
		NewtonBodySetMatrix(body, &location[0][0]);

		// destroy the collision helper shape 
		NewtonDestroyCollision(chassisCollision);

		for (int i = 0; i < int ((sizeof (m_gearMap) / sizeof (m_gearMap[0]))); i ++) {
			m_gearMap[i] = i;
		}
		m_gearMap[0] = 1;
		m_gearMap[1] = 0;
	}

	~HeavyVehicleEntity ()
	{
		((dCustomVehicleControllerManager*)m_controller->GetManager())->DestroyController(m_controller);
	}


	NewtonCollision* CreateChassisCollision (NewtonWorld* const world) const
	{
		DemoEntity* const chassis = dHierarchy<DemoEntity>::Find("chassis");
		dAssert (chassis);

		DemoMesh* const mesh = (DemoMesh*)chassis->GetMesh();
		dAssert (mesh->IsType(DemoMesh::GetRttiType()));
		//dAssert (chassis->GetMeshMatrix().TestIdentity());
		const dMatrix& meshMatrix = chassis->GetMeshMatrix();

		dVector* const temp = new dVector[mesh->m_vertexCount];
		meshMatrix.TransformTriplex (&temp[0].m_x, sizeof (dVector), mesh->m_vertex, 3 * sizeof (dFloat), mesh->m_vertexCount);
		NewtonCollision* const shape = NewtonCreateConvexHull (world, mesh->m_vertexCount, &temp[0].m_x, sizeof (dVector), 0.001f, 0, NULL);
		delete[] temp;
		return shape;
	}

	void CalculateTireDimensions (const char* const tireName, dFloat& width, dFloat& radius) const
	{
		NewtonBody* const body = m_controller->GetBody();
		NewtonWorld* const world = NewtonBodyGetWorld(body);

		// find the the tire visual mesh 
		DemoEntity* const entity = (DemoEntity*) NewtonBodyGetUserData(body);
		DemoEntity* const tirePart = entity->Find (tireName);
		dAssert (tirePart);

		// make a convex hull collision shape to assist in calculation of the tire shape size
		DemoMesh* const tireMesh = (DemoMesh*)tirePart->GetMesh();
		dAssert (tireMesh->IsType(DemoMesh::GetRttiType()));
		dMatrix meshMatrix (tirePart->GetMeshMatrix());
		dVector* const temp = new dVector [tireMesh->m_vertexCount];
		meshMatrix.TransformTriplex (&temp[0].m_x, sizeof (dVector), tireMesh->m_vertex, 3 * sizeof (dFloat), tireMesh->m_vertexCount);
		NewtonCollision* const collision = NewtonCreateConvexHull(world, tireMesh->m_vertexCount, &temp[0].m_x, sizeof (dVector), 0, 0, NULL);
		delete[] temp;

		// get the location of this tire relative to the car chassis
		dMatrix tireMatrix (tirePart->CalculateGlobalMatrix(entity));

		// find the support points that can be used to define the with and high of the tire collision mesh
		dVector extremePoint(0.0f);
		dVector upDir (tireMatrix.UnrotateVector(dVector (0.0f, 1.0f, 0.0f, 0.0f)));
		NewtonCollisionSupportVertex (collision, &upDir[0], &extremePoint[0]);
		radius = dAbs (upDir.DotProduct3(extremePoint));

		dVector widthDir (tireMatrix.UnrotateVector(dVector (0.0f, 0.0f, 1.0f, 0.0f)));
		NewtonCollisionSupportVertex (collision, &widthDir[0], &extremePoint[0]);
		width = widthDir.DotProduct3(extremePoint);

		widthDir = widthDir.Scale (-1.0f);
		NewtonCollisionSupportVertex (collision, &widthDir[0], &extremePoint[0]);
		width += widthDir.DotProduct3(extremePoint);

		// destroy the auxiliary collision
		NewtonDestroyCollision (collision);	
	}


	dWheelJoint* AddTire(const char* const tireName, dFloat width, dFloat radius, dFloat maxSteerAngle, const VehicleParameters& definition) 
	{
		dAssert(0);
		return NULL;
/*
		NewtonBody* const chassis = m_controller->GetBody();
		DemoEntity* const chassisEntiry = (DemoEntity*) NewtonBodyGetUserData(chassis);
		DemoEntity* const tirePart = chassisEntiry->Find (tireName);

		// add this tire, get local position and rise it by the suspension length 
		dMatrix tireMatrix (tirePart->CalculateGlobalMatrix(chassisEntiry));

		// add the offset location
		tireMatrix.m_posit.m_y += definition.m_tirePivotOffset;

		// add and alignment matrix,to match visual mesh to physics collision shape
		dMatrix aligmentMatrix (dGetIdentityMatrix());
		if (tireMatrix[0][2] < 0.0f) {
			aligmentMatrix = dYawMatrix(dPi);
		}

		TireAligmentTransform* const aligmentTransform = new TireAligmentTransform (definition.m_aligmentMatrix * aligmentMatrix);
		tirePart->SetUserData(aligmentTransform);

		// add the tire to the vehicle
		dTireInfo tireInfo;

//		tireInfo.m_location = tireMatrix.m_posit;
		tireInfo.m_mass = definition.m_tireMass;
		tireInfo.m_radio = radius;
		tireInfo.m_width = width;
		tireInfo.m_maxSteeringAngle = maxSteerAngle * dDegreeToRad;
		tireInfo.m_dampingRatio = definition.m_tireSuspensionDamperConstant;
		tireInfo.m_springStrength = definition.m_tireSuspensionSpringConstant;
		tireInfo.m_suspensionLength = definition.m_tireSuspensionLength;
		tireInfo.m_lateralStiffness = definition.m_tireLaretalStiffeness;
		tireInfo.m_longitudialStiffness = definition.m_tireLongitudinalStiffness;
		tireInfo.m_aligningMomentTrail = definition.m_tireAligningMomemtTrail;
		tireInfo.m_suspentionType = definition.m_TireSuspensionType;
		tireInfo.m_hasFender = definition.m_wheelHasCollisionFenders;

		dWheelJoint* const tireJoint = m_controller->AddTire(tireInfo);
		NewtonBody* const tireBody = tireJoint->GetTireBody();
		NewtonBodySetUserData(tireBody, tirePart);
		return tireJoint;
*/
	}


#if 0
	void BuildLightTruckVehicle (const VehicleParameters& parameters)
	{
		// Muscle cars have the front engine, we need to shift the center of mass to the front to represent that
		m_controller->SetCenterOfGravity (dVector (0.0f, parameters.COM_Y_OFFSET, 0.0f, 0.0f)); 
		
		// a car may have different size front an rear tire, therefore we do this separate for front and rear tires
		dFloat width;
		dFloat radius;
		dVector offset (0.0f, 0.15f, 0.0f, 0.0f);
		CalculateTireDimensions ("fl_tire", width, radius);

		// add left tires
		CustomVehicleControllerBodyStateTire* leftTire[2]; 
		leftTire[0] = AddTire ("fl_tire", offset, width, radius, parameters.TIRE_MASS, parameters.SUSPENSION_LENGTH, parameters.SUSPENSION_SPRING, parameters.SUSPENSION_DAMPER, parameters.LATERAL_STIFFNESS, parameters.LONGITUDINAL_STIFFNESS, parameters.ALIGNING_MOMENT_TRAIL, parameters.m_tireAligment);
		leftTire[1] = AddTire ("rl_tire", offset, width, radius, parameters.TIRE_MASS, parameters.SUSPENSION_LENGTH, parameters.SUSPENSION_SPRING, parameters.SUSPENSION_DAMPER, parameters.LATERAL_STIFFNESS, parameters.LONGITUDINAL_STIFFNESS, parameters.ALIGNING_MOMENT_TRAIL, parameters.m_tireAligment);

		// add right tires
		CustomVehicleControllerBodyStateTire* rightTire[2];
		CalculateTireDimensions ("rl_tire", width, radius);
		rightTire[0] = AddTire ("fr_tire", offset, width, radius, parameters.TIRE_MASS, parameters.SUSPENSION_LENGTH, parameters.SUSPENSION_SPRING, parameters.SUSPENSION_DAMPER, parameters.LATERAL_STIFFNESS, parameters.LONGITUDINAL_STIFFNESS, parameters.ALIGNING_MOMENT_TRAIL, parameters.m_tireAligment);
		rightTire[1] = AddTire ("rr_tire", offset, width, radius, parameters.TIRE_MASS, parameters.SUSPENSION_LENGTH, parameters.SUSPENSION_SPRING, parameters.SUSPENSION_DAMPER, parameters.LATERAL_STIFFNESS, parameters.LONGITUDINAL_STIFFNESS, parameters.ALIGNING_MOMENT_TRAIL, parameters.m_tireAligment);

		// add a steering Wheel
		CustomVehicleControllerComponentSteering* const steering = new CustomVehicleControllerComponentSteering (m_controller, parameters.STEER_ANGLE * dDegreeToRad);
		steering->AddSteeringTire (leftTire[0]);
		steering->AddSteeringTire (rightTire[0]);
		m_controller->SetSteering(steering);

		// add vehicle brakes
		CustomVehicleControllerComponentBrake* const brakes = new CustomVehicleControllerComponentBrake (m_controller, parameters.BRAKE_TORQUE);
		for (int i = 0; i < 2; i ++) {
			brakes->AddBrakeTire (leftTire[i]);
			brakes->AddBrakeTire (rightTire[i]);
		}
		m_controller->SetBrakes(brakes);

		// add an engine
		// make the differential
		CustomVehicleControllerComponentEngine::dSingleAxelDifferential* axels[2];
		for (int i = 0; i < 2; i ++) {
			axels[i] = new CustomVehicleControllerComponentEngine::dSingleAxelDifferential (m_controller, leftTire[i], rightTire[i]);
		}
		CustomVehicleControllerComponentEngine::dMultiAxelDifferential* const differencial = new CustomVehicleControllerComponentEngine::dMultiAxelDifferential (m_controller, 2, axels);

		// make the gear Box
		dFloat fowardSpeedGearsBoxRatios[] = {parameters.GEAR_1, parameters.GEAR_1, parameters.GEAR_3};
		CustomVehicleControllerComponentEngine::dGearBox* const gearBox = new CustomVehicleControllerComponentEngine::dGearBox(m_controller, parameters.REVERSE_GEAR, sizeof (fowardSpeedGearsBoxRatios) / sizeof (fowardSpeedGearsBoxRatios[0]), fowardSpeedGearsBoxRatios); 
		CustomVehicleControllerComponentEngine* const engine = new CustomVehicleControllerComponentEngine (m_controller, gearBox, differencial);

		// the the default transmission type
		engine->SetTransmissionMode(m_automaticTransmission.GetPushButtonState());

		m_controller->SetEngine(engine);


		dFloat viperIdleRPM = parameters.IDLE_TORQUE_RPM;
		dFloat viperIdleTorquePoundPerFoot = parameters.IDLE_TORQUE;

		dFloat viperPeakTorqueRPM = parameters.PEAK_TORQUE_RPM;
		dFloat viperPeakTorquePoundPerFoot = parameters.PEAK_TORQUE;

		dFloat viperPeakHorsePowerRPM = parameters.PEAK_HP_RPM;
		dFloat viperPeakHorsePower = parameters.PEAK_HP;

		dFloat viperRedLineRPM = parameters.REDLINE_RPM;
		dFloat viperRedLineTorquePoundPerFoot = parameters.REDLINE_TORQUE;

		dFloat vehicleTopSpeedKPH = parameters.VEHICLE_TOP_SPEED_KMH;
		engine->InitEngineTorqueCurve (vehicleTopSpeedKPH, viperIdleTorquePoundPerFoot, viperIdleRPM, viperPeakTorquePoundPerFoot, viperPeakTorqueRPM, viperPeakHorsePower, viperPeakHorsePowerRPM, viperRedLineTorquePoundPerFoot, viperRedLineRPM);

		// do not forget to call finalize after all components are added or after any change is made to the vehicle
		m_controller->Finalize();
	}



	void ApplyNPCControl ()
	{
		// simple park NPC vehicle 
		CustomVehicleControllerComponentBrake* const brakes = m_controller->GetBrakes();
		if (brakes) {
			brakes->SetParam (0.5f);
		}
	}
	
	void Debug () const 
	{
		NewtonBody* const body = m_controller->GetBody();
		const CustomVehicleControllerBodyStateChassis& chassis = m_controller->GetChassisState ();

		dFloat scale = -4.0f / (chassis.GetMass() * DEMO_GRAVITY);
		dVector p0 (chassis.GetCenterOfMass());

		glDisable (GL_LIGHTING);
		glDisable(GL_TEXTURE_2D);

		glLineWidth(3.0f);
		glBegin(GL_LINES);


		// draw vehicle weight at the center of mass
		dFloat lenght = scale * chassis.GetMass() * DEMO_GRAVITY;
		glColor3f(0.0f, 0.0f, 1.0f);
		glVertex3f (p0.m_x, p0.m_y, p0.m_z);
		glVertex3f (p0.m_x, p0.m_y - lenght, p0.m_z);

		// draw vehicle front dir
		glColor3f(1.0f, 1.0f, 1.0f);
		dVector r0 (p0 + chassis.GetMatrix()[1].Scale (0.5f));
		dVector r1 (r0 + chassis.GetMatrix()[0].Scale (1.0f));
		glVertex3f (r0.m_x, r0.m_y, r0.m_z);
		glVertex3f (r1.m_x, r1.m_y, r1.m_z);


		// draw the velocity vector, a little higher so that is not hidden by the vehicle mesh 
		dVector veloc;
		NewtonBodyGetVelocity(body, &veloc[0]);
		dVector q0 (p0 + chassis.GetMatrix()[1].Scale (1.0f));
		dVector q1 (q0 + veloc.Scale (0.25f));
		glColor3f(1.0f, 1.0f, 0.0f);
		glVertex3f (q0.m_x, q0.m_y, q0.m_z);
		glVertex3f (q1.m_x, q1.m_y, q1.m_z);
		
		for (CustomVehicleControllerBodyStateTire* node = m_controller->GetFirstTire(); node; node = m_controller->GetNextTire(node)) {
			const CustomVehicleControllerBodyStateTire& tire = *node;
			dVector p0 (tire.GetCenterOfMass());

			// offset the origin so that it is visible 
			const dMatrix& tireMatrix = tire.GetLocalMatrix ();
			p0 += chassis.GetMatrix()[2].Scale ((tireMatrix.m_posit.m_z > 0.0f ? 1.0f : -1.0f) * 0.25f);

			// draw the tire load 
			dVector p1 (p0 + tire.GetTireLoad().Scale (scale));
			glColor3f (0.0f, 0.0f, 1.0f);
			glVertex3f (p0.m_x, p0.m_y, p0.m_z);
			glVertex3f (p1.m_x, p1.m_y, p1.m_z);

			// show tire lateral force
			dVector p2 (p0 - tire.GetLateralForce().Scale (scale));
			glColor3f(1.0f, 0.0f, 0.0f);
			glVertex3f (p0.m_x, p0.m_y, p0.m_z);
			glVertex3f (p2.m_x, p2.m_y, p2.m_z);

			// show tire longitudinal force
			dVector p3 (p0 - tire.GetLongitudinalForce().Scale (scale));
			glColor3f (0.0f, 1.0f, 0.0f);
			glVertex3f (p0.m_x, p0.m_y, p0.m_z);
			glVertex3f (p3.m_x, p3.m_y, p3.m_z);
		}

		glEnd();

		glLineWidth(1.0f);

	}
#endif

	void ApplyPlayerControl ()
	{
		dAssert(0);
		/*

		NewtonBody* const body = m_controller->GetBody();
		NewtonWorld* const world = NewtonBodyGetWorld(body);
		DemoEntityManager* const scene = (DemoEntityManager*) NewtonWorldGetUserData(world);

		dEngineController* const engine = m_controller->GetEngine();
		dSteeringController* const steering = m_controller->GetSteering();
		dBrakeController* const brakes = m_controller->GetBrakes();
		//BrakeController* const handBrakes = m_controller->GetHandBrakes();

		//int gear = engine->GetGear();
		int engineIgnitionKey = m_engineKeySwitch.UpdatePushButton(scene, 'I');
		int engineDifferentialLock = m_engineDifferentialLock.UpdatePushButton(scene, 'L');
		//int automaticTransmission = engine->GetTransmissionMode();

		//dFloat brakePedal = 0.0f;
		dFloat handBrakePedal = 0.0f;
		dFloat cluthPedal = 1.0f;
		dFloat reverseGasPedal = 0.0f;
		dFloat forwardGasPedal = 0.0f;

		// get the steering input
		dFloat steeringVal = (dFloat(scene->GetKeyState('D')) - dFloat(scene->GetKeyState('A')));
	
		// get keyboard controls
		if (scene->GetKeyState('W')) {
			forwardGasPedal = 1.0f;
		}

		if (scene->GetKeyState('S')) {
			reverseGasPedal = 1.0f;
		}

		if (scene->GetKeyState(' ')) {
			handBrakePedal = 1.0f;
		}

#if 0
	#if 1
			static FILE* file = fopen ("log.bin", "wb");                                         
			if (file) {
				fwrite (&engineIgnitionKey, sizeof (int), 1, file);
				//fwrite (&automaticTransmission, sizeof (int), 1, file);
				//fwrite (&gear, sizeof (int), 1, file);
				fwrite (&steeringVal, sizeof (dFloat), 1, file);
				fwrite (&cluthPedal, sizeof (dFloat), 1, file);
				fwrite (&forwardGasPedal, sizeof (dFloat), 1, file);
				fwrite (&reverseGasPedal, sizeof (dFloat), 1, file);
				fwrite (&handBrakePedal, sizeof (dFloat), 1, file);
				fflush(file);
			}
	#else 
			static FILE* file = fopen ("log.bin", "rb");
			if (file) {		
				fread (&engineIgnitionKey, sizeof (int), 1, file);
				//fread (&automaticTransmission, sizeof (int), 1, file);
				//fread (&gear, sizeof (int), 1, file);
				fread (&steeringVal, sizeof (dFloat), 1, file);
				fread (&cluthPedal, sizeof (dFloat), 1, file);
				fread (&forwardGasPedal, sizeof (dFloat), 1, file);
				fread (&reverseGasPedal, sizeof (dFloat), 1, file);
				fread (&handBrakePedal, sizeof (dFloat), 1, file);
			}
	#endif
#endif

		if (steering) {
			steering->SetParam(steeringVal);
		}
		
		if (engine) {
			engine->SetDifferentialLock(engineDifferentialLock ? true : false);
			switch (m_drivingState)
			{
				case m_engineOff:
				{
					if (engineIgnitionKey) {
						m_drivingState = m_engineIdle;
						engine->SetIgnition(true);
						brakes->SetParam(0.0f);
						engine->SetGear(engine->GetNeutralGear());
					}
					else {
						engine->SetIgnition(false);
						engine->SetGear(engine->GetFirstGear());
						brakes->SetParam(1.0f);
					}
					break;
				}

				case m_engineIdle:
				{
					//brakes->SetParam(0.0f);
					brakes->SetParam(handBrakePedal);
					if (!engineIgnitionKey) {
						m_drivingState = m_engineOff;
					}
					else {
						if (forwardGasPedal) {
							m_drivingState = m_preDriveForward;
						}
						else if (reverseGasPedal) {
							m_drivingState = m_preDriveReverse;
						}
					}
					break;
				}

				case m_engineStop:
				{
					if (forwardGasPedal || reverseGasPedal) {
						brakes->SetParam(1.0f);
					}
					else {
						m_drivingState = m_engineIdle;
					}
					break;
				}

				case m_preDriveForward:
				{
					if (engine->GetSpeed() < -5.0f) {
						brakes->SetParam(0.5f);
						engine->SetClutchParam(0.0f);
						engine->SetGear(engine->GetNeutralGear());
					}
					else {
						m_drivingState = m_driveForward;
						engine->SetGear(engine->GetFirstGear());
					}
					break;
				}

				case m_driveForward:
				{
					engine->SetParam(forwardGasPedal);
					engine->SetClutchParam(cluthPedal);
					brakes->SetParam(handBrakePedal);
					if (reverseGasPedal) {
						brakes->SetParam(reverseGasPedal * 0.25f);
						if (engine->GetSpeed() < 5.0f) {
							engine->SetGear(engine->GetNeutralGear());
							m_drivingState = m_engineStop;
						}
					}
					else {
						brakes->SetParam(0.0f);
					}

					if (!engineIgnitionKey) {
						m_drivingState = m_engineStop;
					}
					break;
				}

				case m_preDriveReverse:
				{
					if (engine->GetSpeed() > 5.0f) {
						brakes->SetParam(0.5f);
						engine->SetClutchParam(0.0f);
						engine->SetGear(engine->GetNeutralGear());
					}
					else {
						m_drivingState = m_driveReverse;
						engine->SetGear(engine->GetReverseGear());
					}
					break;
				}

				case m_driveReverse:
				{
					engine->SetParam(reverseGasPedal);
					engine->SetClutchParam(cluthPedal);
					brakes->SetParam(handBrakePedal);
					if (forwardGasPedal) {
						brakes->SetParam(forwardGasPedal * 0.25f);
						if (engine->GetSpeed() > -5.0f) {
							engine->SetGear(engine->GetNeutralGear());
							m_drivingState = m_engineStop;
						}
					}
					else {
						brakes->SetParam(0.0f);
					}

					if (!engineIgnitionKey) {
						m_drivingState = m_engineStop;
					}
					break;
				}
			}
		} else if (brakes) {
 			brakes->SetParam(handBrakePedal);
		}
*/
	}


	// this function is an example of how to make a high performance super car
	void BuildAllWheelDriveVehicle(const VehicleParameters& definition)
	{
		dAssert(0);
/*
		// Muscle cars have the front engine, we need to shift the center of mass to the front to represent that
		m_controller->SetCenterOfGravity(dVector(0.0f, definition.COM_Y_OFFSET, 0.0f, 0.0f));

		// a car may have different size front an rear tire, therefore we do this separate for front and rear tires
		dFloat width;
		dFloat radius;
		CalculateTireDimensions("ltire_0", width, radius);

		// add left tires
		dCustomVehicleController::dBodyPartTire* leftTire[4];
		leftTire[0] = AddTire("ltire_0", width, radius, definition.STEER_ANGLE, definition);
		leftTire[1] = AddTire("ltire_1", width, radius, definition.STEER_ANGLE, definition);
		leftTire[2] = AddTire("ltire_2", width, radius, 				  0.0f, definition);
		leftTire[3] = AddTire("ltire_3", width, radius, 				  0.0f, definition);

		// add right tires
		dCustomVehicleController::dBodyPartTire* rightTire[4];
		rightTire[0] = AddTire("rtire_0", width, radius, definition.STEER_ANGLE, definition);
		rightTire[1] = AddTire("rtire_1", width, radius, definition.STEER_ANGLE, definition);
		rightTire[2] = AddTire("rtire_2", width, radius, 				   0.0f, definition);
		rightTire[3] = AddTire("rtire_3", width, radius, 				   0.0f, definition);

		// add a steering Wheel
		dCustomVehicleController::dSteeringController* const steering = new dCustomVehicleController::dSteeringController(m_controller);
		for (int i = 0; i < 2; i++) {
			steering->AddTire(leftTire[i]);
			steering->AddTire(rightTire[i]);
		}
		m_controller->SetSteering(steering);

		// add vehicle brakes
		dCustomVehicleController::dBrakeController* const brakes = new dCustomVehicleController::dBrakeController(m_controller, definition.BRAKE_TORQUE);
		for (int i = 0; i < 4; i++) {
			brakes->AddTire(leftTire[i]);
			brakes->AddTire(rightTire[i]);
		}
		m_controller->SetBrakes(brakes);

		// add the engine, differential and transmission 
		dCustomVehicleController::dEngineController::dInfo engineInfo;
		engineInfo.m_mass = definition.ENGINE_MASS;
		engineInfo.m_radio = definition.ENGINE_RADIO;
		engineInfo.m_vehicleTopSpeed = definition.VEHICLE_TOP_SPEED_KMH;
		engineInfo.m_clutchFrictionTorque = definition.CLUTCH_FRICTION_TORQUE;

		engineInfo.m_idleTorque = definition.IDLE_TORQUE;
		engineInfo.m_rpmAtIdleTorque = definition.IDLE_TORQUE_RPM;

		engineInfo.m_peakTorque = definition.PEAK_TORQUE;
		engineInfo.m_rpmAtPeakTorque = definition.PEAK_TORQUE_RPM;

		engineInfo.m_peakHorsePower = definition.PEAK_HP;
		engineInfo.m_rpmAtPeakHorsePower = definition.PEAK_HP_RPM;

		engineInfo.m_redLineRpm = definition.REDLINE_RPM;

		engineInfo.m_gearsCount = 3;
		engineInfo.m_gearRatios[0] = definition.GEAR_1;
		engineInfo.m_gearRatios[1] = definition.GEAR_2;
		engineInfo.m_gearRatios[2] = definition.GEAR_3;
		engineInfo.m_reverseGearRatio = definition.REVERSE_GEAR;

		dCustomVehicleController::dEngineController::dDifferential8wd differential;
		differential.m_type = dCustomVehicleController::dEngineController::dDifferential::m_8wd;
		differential.m_axel.m_leftTire = leftTire[0];
		differential.m_axel.m_rightTire = rightTire[0];
		differential.m_secondAxel.m_axel.m_leftTire = leftTire[1];
		differential.m_secondAxel.m_axel.m_rightTire = rightTire[1];
		differential.m_second4Wd.m_axel.m_leftTire = leftTire[2];
		differential.m_second4Wd.m_axel.m_rightTire = rightTire[2];
		differential.m_second4Wd.m_secondAxel.m_axel.m_leftTire = leftTire[3];
		differential.m_second4Wd.m_secondAxel.m_axel.m_rightTire = rightTire[3];
	
		engineInfo.m_differentialLock = 0;
		engineInfo.m_userData = this;
		dCustomVehicleController::dEngineController* const engineControl = new dCustomVehicleController::dEngineController(m_controller, engineInfo, differential);

		// the the default transmission type
		engineControl->SetTransmissionMode(m_automaticTransmission.GetPushButtonState());
		engineControl->SetIgnition(true);

		m_controller->SetEngine(engineControl);

		// do not forget to call finalize after all components are added or after any change is made to the vehicle
		m_controller->Finalize();
*/
	}


	void BuildTrackedVehicle(const VehicleParameters& definition)
	{
		dAssert (0);
#if 0
		// Muscle cars have the front engine, we need to shift the center of mass to the front to represent that
		m_controller->SetCenterOfGravity(dVector(0.0f, definition.m_chassisYaxisComBias, 0.0f, 0.0f));

		// add all the tank tires
		dCustomVehicleController::dBodyPartTire* leftTire[8];
		dCustomVehicleController::dBodyPartTire* rightTire[8];
		for (int i = 0; i < 8; i++) {
			char name[32];
			dFloat width;
			dFloat radius;
			sprintf(name, "l_tire%d", i);
			CalculateTireDimensions(name, width, radius);
			// add the tire thread thickness
			radius += 0.1f;
			width *= 2.0f;

			leftTire[i] = AddTire(name, width, radius, definition.m_frontSteeringAngle, definition);
			sprintf(name, "r_tire%d", i);
			rightTire[i] = AddTire(name, width, radius, definition.m_frontSteeringAngle, definition);
		}
		for (int i = 1; i < 8; i++) {
			m_controller->LinkTiresKinematically(leftTire[0], leftTire[i]);
			m_controller->LinkTiresKinematically(rightTire[0], rightTire[i]);
		}

		dCustomVehicleController::dEngineController::dInfo engineInfo;

		engineInfo.m_gearRatiosSign = 1.0f;
		dCustomVehicleController::dBodyPartDifferential* const differential = m_controller->AddDifferential(leftTire[0], rightTire[0]);
		differential->SetTrackMode(true, 5.0f);

		// add vehicle brakes
		dCustomVehicleController::dBrakeController* const brakes = new dCustomVehicleController::dBrakeController(m_controller, definition.m_tireBrakesTorque);
		brakes->AddTire(leftTire[0]);
		brakes->AddTire(rightTire[0]);
		m_controller->SetBrakes(brakes);

		// add a tank skid steering engine
		dCustomVehicleController::dSteeringController* const steering = new dCustomVehicleController::dSteeringController(m_controller, differential);
		brakes->AddTire(leftTire[0]);
		brakes->AddTire(rightTire[0]);
		m_controller->SetSteering(steering);

/*
		dCustomVehicleController::dEngineController::dInfo engineInfo;
		engineInfo.m_mass = definition.ENGINE_MASS;
		engineInfo.m_radio = definition.ENGINE_RADIO;
		engineInfo.m_vehicleTopSpeed = definition.VEHICLE_TOP_SPEED_KMH;
		engineInfo.m_clutchFrictionTorque = definition.CLUTCH_FRICTION_TORQUE;

		engineInfo.m_idleTorque = definition.IDLE_TORQUE;
		engineInfo.m_rpmAtIdleTorque = definition.IDLE_TORQUE_RPM;

		engineInfo.m_peakTorque = definition.PEAK_TORQUE;
		engineInfo.m_rpmAtPeakTorque = definition.PEAK_TORQUE_RPM;

		engineInfo.m_peakHorsePower = definition.PEAK_HP;
		engineInfo.m_rpmAtPeakHorsePower = definition.PEAK_HP_RPM;

		engineInfo.m_redLineRpm = definition.REDLINE_RPM;

		engineInfo.m_gearsCount = 3;
		engineInfo.m_gearRatios[0] = definition.GEAR_1;
		engineInfo.m_gearRatios[1] = definition.GEAR_2;
		engineInfo.m_gearRatios[2] = definition.GEAR_3;
		engineInfo.m_reverseGearRatio = definition.REVERSE_GEAR;

		dCustomVehicleController::dEngineController::dDifferentialTracked differential (8, leftTire, rightTire);
		//CustomVehicleController::EngineController::DifferentialTracked differential (2, leftTire, rightTire);

		engineInfo.m_differentialLock = 0;
		engineInfo.m_userData = this;
		dCustomVehicleController::dEngineController* const engineControl = new dCustomVehicleController::dEngineController(m_controller, engineInfo, differential);

		// the the default transmission type
		engineControl->SetTransmissionMode(m_automaticTransmission.GetPushButtonState());
		engineControl->SetIgnition(true);

		m_controller->SetEngine(engineControl);
*/
		// set up the tank Track
		SetTracks(leftTire[0], rightTire[0]);

		// do not forget to call finalize after all components are added or after any change is made to the vehicle
		m_controller->Finalize();
#endif
	}

	void SetTracks(dWheelJoint* const leftTire, dWheelJoint* const rightTire)
	{
		m_leftTrack.Init(this, "leftTrackPath", leftTire);
		m_rightTrack.Init(this, "rightTrackPath", rightTire);
	}


	// interpolate all skeleton transform 
	virtual void InterpolateMatrix(DemoEntityManager& world, dFloat param)
	{
		DemoEntity::InterpolateMatrix(world, param);
		if (m_controller) {
			for (dList<NewtonBody*>::dListNode* node = m_controller->GetFirstBodyPart()->GetNext(); node; node = m_controller->GetNextBodyPart(node)) {
				NewtonBody* const body = node->GetInfo();
				DemoEntity* const entPart = (DemoEntity*)NewtonBodyGetUserData (body);
				if (entPart) {
					entPart->InterpolateMatrix(world, param);
				}
			}
		}

		if (m_leftTrack.m_bezierEntity) {
			m_leftTrack.InterpolateMatrix(world, param);
			m_rightTrack.InterpolateMatrix(world, param);
		}
	}


	void UpdateTireTransforms(dFloat timestep)
	{
		dAssert (0);
/*
		NewtonBody* const body = m_controller->GetBody();
		DemoEntityManager* const scene = (DemoEntityManager*)NewtonWorldGetUserData(NewtonBodyGetWorld(body));

		for (dList<dCustomVehicleController::dBodyPartTire>::dListNode* node = m_controller->GetFirstTire(); node; node = m_controller->GetNextTire(node)) {
			const dCustomVehicleController::dBodyPartTire* const part = &node->GetInfo();
			dCustomVehicleController::dBodyPart* const parent = part->GetParent();

			NewtonBody* const body = part->GetBody();
			NewtonBody* const parentBody = parent->GetBody();

			dMatrix matrix;
			dMatrix parentMatrix;
			NewtonBodyGetMatrix(body, &matrix[0][0]);
			NewtonBodyGetMatrix(parentBody, &parentMatrix[0][0]);

			DemoEntity* const tirePart = (DemoEntity*)part->GetUserData();
			TireAligmentTransform* const aligmentMatrix = (TireAligmentTransform*)tirePart->GetUserData();
			matrix = aligmentMatrix->m_matrix * matrix * parentMatrix.Inverse();

			dQuaternion rot(matrix);
			tirePart->SetMatrix(*scene, rot, matrix.m_posit);
		}

		// update the thread location
		if (m_leftTrack.m_bezierEntity) {
			m_leftTrack.AnimateThread(*scene, timestep);
			m_rightTrack.AnimateThread(*scene, timestep);
		}
*/
	}

	TrackSystem m_leftTrack;
	TrackSystem m_rightTrack;
	dCustomVehicleController* m_controller;
	DemoEntityManager::ButtonKey m_gearUpKey;
	DemoEntityManager::ButtonKey m_gearDownKey;
	DemoEntityManager::ButtonKey m_reverseGear;
	DemoEntityManager::ButtonKey m_engineKeySwitch;
	DemoEntityManager::ButtonKey m_engineDifferentialLock;
	DemoEntityManager::ButtonKey m_automaticTransmission;
	int m_gearMap[10];
	bool m_engineRPMOn;
	DrivingState m_drivingState;
};


class HeavyVehicleControllerManager: public dCustomVehicleControllerManager
{
	public:
	HeavyVehicleControllerManager (NewtonWorld* const world, int materialsCount, int* const materialList)
		:dCustomVehicleControllerManager (world, materialsCount, materialList)
		,m_externalView(true)
		,m_changeVehicle(false)
		,m_player (NULL) 
	{
		// hook a callback for 2d help display
		DemoEntityManager* const scene = (DemoEntityManager*) NewtonWorldGetUserData(world);
		scene->Set2DDisplayRenderFunction (RenderVehicleHud, NULL, this);
	}

	~HeavyVehicleControllerManager ()
	{
	}


	static void RenderVehicleHud (DemoEntityManager* const scene, void* const context)
	{
		HeavyVehicleControllerManager* const me = (HeavyVehicleControllerManager*) context;
		//me->RenderVehicleHud (scene, me);
		me->DrawHelp(scene);
	}
/*
	void RenderVehicleHud (DemoEntityManager* const scene, HeavyVehicleControllerManager* const manager, int lineNumber) const
	{
		if (m_player) {
			// set to transparent color
			glEnable (GL_BLEND);
			glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

			//print controllers help 
			manager->DrawHelp(scene, lineNumber);

			// restore color and blend mode
			glDisable (GL_BLEND);
		}
		//TestSpline();
	}
*/
	void DrawHelp(DemoEntityManager* const scene)
	{
		dVector color(1.0f, 1.0f, 0.0f, 0.0f);
		scene->Print (color, "Vehicle driving keyboard control:               Joystick control");
		scene->Print (color, "engine switch                   : 'I'           start engine");
		scene->Print (color, "accelerator                     : 'W'           stick forward");
		scene->Print (color, "brakes                          : 'S'           stick back");
		scene->Print (color, "turn left                       : 'A'           stick left");
		scene->Print (color, "turn right                      : 'D'           stick right");
		scene->Print (color, "engage clutch                   : 'K'           button 5");
		scene->Print (color, "engage differential lock        : 'L'           button 5");
		scene->Print (color, "gear up                         : '>'           button 2");
		scene->Print (color, "gear down                       : '<'           button 3");
		scene->Print (color, "manual transmission             : enter         button 4");
		scene->Print (color, "hand brakes                     : space         button 1");
		scene->Print (color, "next vehicle                    : 'V'");
		scene->Print (color, "hide help                       : 'H'");
	}

#if 0
	void TestSpline () const
	{
		glDisable (GL_LIGHTING);
		glDisable(GL_TEXTURE_2D);

		dFloat64 knots[] = {0.0f, 1.0f / 3.0f, 2.0f / 3.0f, 1.0f};
//dFloat64 knots[] = {0.0f, 1.0f};
		dBigVector control[] = 
		{
			dBigVector (200.0f, 200.0f, 0.0f, 1.0f),
			dBigVector (150.0f, 250.0f, 0.0f, 1.0f),
			dBigVector (250.0f, 300.0f, 0.0f, 1.0f),
			dBigVector (350.0f, 250.0f, 0.0f, 1.0f),
			dBigVector (250.0f, 150.0f, 0.0f, 1.0f),
			dBigVector (200.0f, 200.0f, 0.0f, 1.0f),
		};
		static dBezierSpline spline;
		if (!spline.GetControlPointArray()) {
			spline.CreateFromKnotVectorAndControlPoints(3, sizeof (knots) / sizeof (knots[0]), knots, control);
			dFloat64 u = (knots[1] + knots[2]) * 0.5f;
			spline.InsertKnot (u);
			spline.InsertKnot (u);
			spline.InsertKnot (u);
			spline.InsertKnot (u);
			spline.InsertKnot (u);
			spline.RemoveKnot (u, 1.0e-3f);
			//spline.RemoveKnot (u, 1.0e-3f);
		}

		const int segments = 20;
		glColor3f(1.0f, 1.0f, 1.0f);
		glBegin(GL_LINES);
		dBigVector p0 (spline.CurvePoint(0.0f)) ;
		for (int i = 1; i <= segments; i ++) {
			dBigVector p1 (spline.CurvePoint(dFloat (i) / segments)) ;
			glVertex3f (p0.m_x, p0.m_y, p0.m_z);
			glVertex3f (p1.m_x, p1.m_y, p1.m_z);
			p0 = p1;
		}
		glEnd();

		glColor3f(0.0f, 1.0f, 1.0f);
		glPointSize(4.0f);
		glBegin(GL_POINTS);
		for (int i = 0; i < spline.GetControlPointCount(); i ++) {
			dBigVector p (spline.GetControlPoint(i));
			glVertex3f (p.m_x, p.m_y, p.m_z);
		}
		glEnd();

		// recreate the spline from sample points at equally spaced distance 
		int pointCount = 4;
		dBigVector points[5];
		points[0] = spline.CurvePoint(0.0f / 3.0f) + dBigVector (300.0f, 0.0f, 0.0f, 0.0f);
		points[1] = spline.CurvePoint(1.0f / 3.0f) + dBigVector (300.0f, 0.0f, 0.0f, 0.0f);
		points[2] = spline.CurvePoint(2.0f / 3.0f) + dBigVector (300.0f, 0.0f, 0.0f, 0.0f);
		points[3] = spline.CurvePoint(3.0f / 3.0f) + dBigVector (300.0f, 0.0f, 0.0f, 0.0f);

		dBigVector derivP0 (spline.CurveDerivative(0.0f));
		dBigVector derivP1 (spline.CurveDerivative(1.0f));

		static dBezierSpline spline1;
		spline1.GlobalCubicInterpolation (pointCount, points, derivP0, derivP1);

		glColor3f(1.0f, 1.0f, 1.0f);
		glBegin(GL_LINES);
		p0 = spline1.CurvePoint(0.0f);
		for (int i = 1; i <= segments; i ++) {
			dFloat64 u = dFloat (i) / segments;
			dBigVector p1 (spline1.CurvePoint(u));
			glVertex3f (p0.m_x, p0.m_y, p0.m_z);
			glVertex3f (p1.m_x, p1.m_y, p1.m_z);
			p0 = p1;
		}
		glEnd();


		glPointSize(4.0f);
		glBegin(GL_POINTS);
		glColor3f(0.0f, 1.0f, 1.0f);
		for (int i = 0; i < spline.GetControlPointCount(); i ++) {
			dBigVector p (spline1.GetControlPoint(i));
			glVertex3f (p.m_x, p.m_y, p.m_z);
		}

		glColor3f(1.0f, 0.0f, 0.0f);
		for (int i = 0; i < pointCount; i ++) {
			glVertex3f (points[i].m_x, points[i].m_y, points[i].m_z);
		}
		glEnd();
	}



	// use this to display debug information about vehicle 
	void Debug () const
	{
		for (dListNode* ptr = GetFirst(); ptr; ptr = ptr->GetNext()) {
			CustomVehicleController* const controller = &ptr->GetInfo();
			HeavyVehicleEntity* const vehicleEntity = (HeavyVehicleEntity*)NewtonBodyGetUserData (controller->GetBody());
			vehicleEntity->Debug();
		}
	}
#endif

	void UpdateCamera (dFloat timestep)
	{
		if (m_player) {
			DemoEntityManager* const scene = (DemoEntityManager*) NewtonWorldGetUserData(GetWorld());
			DemoCamera* const camera = scene->GetCamera();
			dMatrix camMatrix (camera->GetNextMatrix ());
			dMatrix playerMatrix (m_player->GetNextMatrix());

			dVector frontDir (camMatrix[0]);
			dVector camOrigin(0.0f); 
			if (m_externalView) {
				camOrigin = playerMatrix.m_posit + dVector(0.0f, HEAVY_VEHICLE_THIRD_PERSON_VIEW_HIGHT, 0.0f, 0.0f);
				camOrigin -= frontDir.Scale (HEAVY_VEHICLE_THIRD_PERSON_VIEW_DIST);
			} else {
				dAssert (0);
				//            camMatrix = camMatrix * playerMatrix;
				//            camOrigin = playerMatrix.TransformVector(dVector(-0.8f, ARTICULATED_VEHICLE_CAMERA_EYEPOINT, 0.0f, 0.0f));
			}

			camera->SetNextMatrix (*scene, camMatrix, camOrigin);
		}
	}


	void SetAsPlayer(HeavyVehicleEntity* const player)
	{
		m_player = player;
	}

	virtual void PreUpdate(dFloat timestep)
	{
		dAssert(0);
		/*
		// apply the vehicle controls, and all simulation time effect
		NewtonWorld* const world = GetWorld();
		DemoEntityManager* const scene = (DemoEntityManager*)NewtonWorldGetUserData(world);

		m_helpKey.UpdatePushButton (scene, 'H');

		int changeVehicle = m_changeVehicle.UpdateTriggerButton(scene, 'V') ? 1 : 0;
		if (changeVehicle) {
			dListNode* nextPlater = GetLast();
			for (dListNode* ptr = GetFirst(); ptr; ptr = ptr->GetNext()) {
				dCustomVehicleController* const controller = &ptr->GetInfo();
				NewtonBody* const body = controller->GetBody();
				HeavyVehicleEntity* const vehicleEntity = (HeavyVehicleEntity*)NewtonBodyGetUserData(body);
				if (vehicleEntity == m_player) {
					dCustomVehicleController* const controller1 = &nextPlater->GetInfo();
					NewtonBody* const body1 = controller1->GetBody();
					SetAsPlayer((HeavyVehicleEntity*)NewtonBodyGetUserData(body1));
					break;
				}
				nextPlater = ptr;
			}
		}


		for (dListNode* ptr = GetFirst(); ptr; ptr = ptr->GetNext()) {
			dCustomVehicleController* const controller = &ptr->GetInfo();

			NewtonBody* const body = controller->GetBody();
			HeavyVehicleEntity* const vehicleEntity = (HeavyVehicleEntity*)NewtonBodyGetUserData(body);

			if (vehicleEntity == m_player) {
				// do player control
				vehicleEntity->ApplyPlayerControl();
			} else {
				// do no player control
				//vehicleEntity->ApplyNPCControl();
			}
		}

		// do the base class post update
		dCustomVehicleControllerManager::PreUpdate(timestep);
*/
	}


	virtual void PostUpdate(dFloat timestep)
	{
		// do the base class post update
		dCustomVehicleControllerManager::PostUpdate(timestep);

		// update the visual transformation matrices for all vehicle tires
		for (dListNode* node = GetFirst(); node; node = node->GetNext()) {
			dCustomVehicleController* const controller = &node->GetInfo();
			HeavyVehicleEntity* const vehicleEntity = (HeavyVehicleEntity*)NewtonBodyGetUserData(controller->GetBody());
			vehicleEntity->UpdateTireTransforms(timestep);
		}

		UpdateCamera(timestep);
	}

	bool m_externalView;
	DemoEntityManager::ButtonKey m_changeVehicle;
	HeavyVehicleEntity* m_player;
};
#endif

void MilitaryTransport (DemoEntityManager* const scene)
{
	// load the sky box
	scene->CreateSkyBox();

#if 0
//	CreateLevelMesh (scene, "flatPlane.ngd", 1);
	CreateHeightFieldTerrain(scene, HEIGHTFIELD_DEFAULT_SIZE, HEIGHTFIELD_DEFAULT_CELLSIZE, 4.0f, 0.1f, 200.0f, -30.0f);

//	dMatrix camMatrix (dRollMatrix(-20.0f * dDegreeToRad) * dYawMatrix(-45.0f * dDegreeToRad));
	dMatrix location (dGetIdentityMatrix());
	location.m_posit = dVector (1000.0f, 100.0f, 1000.0f, 1.0f);
location.m_posit.m_x = 126.0f;
location.m_posit.m_y = 50.0f;
location.m_posit.m_z = 50.0f;
//location.m_posit.m_x = 0.0f;
//location.m_posit.m_z = 0.0f;

	location.m_posit = FindFloor (scene->GetNewton(), location.m_posit, 200.0f);
	location.m_posit.m_y += 2.0f;

	NewtonWorld* const world = scene->GetNewton();

	// create a vehicle controller manager
	int defaulMaterial = NewtonMaterialGetDefaultGroupID(scene->GetNewton());
	NewtonMaterialSetDefaultFriction(world, defaulMaterial, defaulMaterial, 0.6f, 0.5f);
	int materialList[] = { defaulMaterial };
	HeavyVehicleControllerManager* const manager = new HeavyVehicleControllerManager (world, 1, materialList);


/*	
	HeavyVehicleEntity* const heavyVehicle = new HeavyVehicleEntity (scene, manager, location, "lav-25.ngd", heavyTruck);
	heavyVehicle->BuildAllWheelDriveVehicle (heavyTruck);
*/
/*
	// add a second Vehicle
	location.m_posit.m_z += 4.0f;
	location.m_posit = FindFloor (scene->GetNewton(), location.m_posit, 100.0f);
	location.m_posit.m_y += 2.0f;
	HeavyVehicleEntity* const lightVehicle = new HeavyVehicleEntity (scene, manager, location, "buggy_.ngd", lightTruck);
	lightVehicle->BuildLightTruckVehicle (lightTruck);
*/
	location.m_posit.m_z -= 12.0f;
	location.m_posit = FindFloor (scene->GetNewton(), location.m_posit, 100.0f);
	location.m_posit.m_y += 3.0f;
	HeavyVehicleEntity* const m1a1Tank = new HeavyVehicleEntity (scene, manager, location, "m1a1_.ngd", m1a1Param);
	m1a1Tank->BuildTrackedVehicle (m1a1Param);

	// set this vehicle as the player
	manager->SetAsPlayer(m1a1Tank);
//	manager->SetAsPlayer(heavyVehicle);

//NewtonDestroyBody (m1a1Tank->m_controller->GetBody());
//manager->DestroyController(m1a1Tank->m_controller);


/*
location.m_posit.m_z += 20.0f;
for (int i = 0; i < 5; i ++){
	location.m_posit.m_z += 12.0f;
	location.m_posit = FindFloor (scene->GetNewton(), location.m_posit, 100.0f);
	location.m_posit.m_y += 3.0f;
	HeavyVehicleEntity* const m1a1Tank1 = new HeavyVehicleEntity (scene, manager, location, "m1a1_.ngd", m1a1Param);
	m1a1Tank1->BuildTrackedVehicle (m1a1Param);
}
*/
	
	dMatrix camMatrix (manager->m_player->GetNextMatrix());
	//camMatrix = dYawMatrix (-0.5f * dPi) * camMatrix;
	//scene->SetCameraMouseLock (true);
	scene->SetCameraMatrix(camMatrix, camMatrix.m_posit);

/*
	location.m_posit.m_x += 20.0f;
	int defaultMaterialID = NewtonMaterialGetDefaultGroupID (scene->GetNewton());
	int count = 5;
	dMatrix shapeOffsetMatrix (dGetIdentityMatrix());
	dVector size (3.0f, 0.125f, 3.0f, 0.0f);
	AddPrimitiveArray(scene, 100.0f, location.m_posit, size, count, count, 5.0f, _BOX_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);

	size = dVector(1.0f, 0.5f, 1.0f, 0.0f);
	AddPrimitiveArray(scene, 10.0f, location.m_posit, size, count, count, 5.0f, _SPHERE_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
	AddPrimitiveArray(scene, 10.0f, location.m_posit, size, count, count, 5.0f, _BOX_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
	AddPrimitiveArray(scene, 10.0f, location.m_posit, size, count, count, 5.0f, _CAPSULE_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
	AddPrimitiveArray(scene, 10.0f, location.m_posit, size, count, count, 5.0f, _CYLINDER_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
	AddPrimitiveArray(scene, 10.0f, location.m_posit, size, count, count, 5.0f, _CHAMFER_CYLINDER_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
	AddPrimitiveArray(scene, 10.0f, location.m_posit, size, count, count, 5.0f, _CONE_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
	AddPrimitiveArray(scene, 10.0f, location.m_posit, size, count, count, 5.0f, _REGULAR_CONVEX_HULL_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
	AddPrimitiveArray(scene, 10.0f, location.m_posit, size, count, count, 5.0f, _RANDOM_CONVEX_HULL_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
*/
#endif
}

