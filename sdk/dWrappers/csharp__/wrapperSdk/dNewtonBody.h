/* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 
* 3. This notice may not be removed or altered from any source distribution.
*/

#ifndef _D_NEWTON_BODY_H_
#define _D_NEWTON_BODY_H_

#include "stdafx.h"
#include "dAlloc.h"

class dMatrix;
class NewtonBody;
class dNewtonBody;
class dNewtonWorld;
class dNewtonCollision;

class dNewtonBody: public dAlloc
{
	public:

	class ScopeLock
	{
		public:
		ScopeLock(unsigned int* const lock);
		~ScopeLock();
		unsigned* m_atomicLock;
	};

	dNewtonBody(const dMatrix& matrix);
	virtual void Destroy();

	void* GetBody() const;
	void* GetPosition();
	void* GetRotation();
	void SetPosition(dFloat x, dFloat y, dFloat z);
	void SetRotation(dFloat x, dFloat y, dFloat z, dFloat w);
	void* GetInterpolatedPosition();
	void* GetInterpolatedRotation();
	void SetUserData(void* userData);
	void* GetUserData();

	void* GetVelocity();
	void* GetOmega();
	void SetVelocity(dFloat x, dFloat y, dFloat z);
	void SetOmega(dFloat x, dFloat y, dFloat z);

	float GetLinearDamping();
	void SetLinearDamping(dFloat x);

	void* GetAngularDamping();
	void SetAngularDamping(dFloat x, dFloat y, dFloat z);

	void* GetCenterOfMass();
	void SetCenterOfMass(float com_x, float com_y, float com_z);
	void CalculateBuoyancyForces(const void* plane, void* force, void* torque, float bodyDensity);

	virtual void AddForce(dFloat x, dFloat y, dFloat z);
	virtual void AddTorque(dFloat x, dFloat y, dFloat z);

	bool GetSleepState() const;
	void SetSleepState(bool state) const;

	protected:
	virtual ~dNewtonBody();

	// called from newton update
	virtual void OnForceAndTorque(dFloat timestep, int threadIndex);

	// call each time the physics update the body transformation 
	virtual void OnBodyTransform(const dFloat* const matrix, int threadIndex);

	virtual void InitForceAccumulators();

	protected:
	static void OnBodyDestroy (const NewtonBody* const body);
	static void OnForceAndTorqueCallback (const NewtonBody* body, dFloat timestep, int threadIndex);
	static void OnBodyTransformCallback(const NewtonBody* const body, const dFloat* const matrix, int threadIndex);

	NewtonBody* m_body;
	void* m_userData;
	dVector m_posit0;
	dVector m_posit1;
	dVector m_interpolatedPosit;
	dQuaternion m_rotation0;
	dQuaternion m_rotation1;
	dQuaternion m_interpolatedRotation;
	dVector m_velocity;
	dVector m_omega;
	dVector m_com;
	dVector m_angulardamping;
	unsigned m_lock;

	friend class dNewtonWorld;
	friend class dNewtonBallAndSocket;
};


class dNewtonKinematicBody : public dNewtonBody
{
	public:
	dNewtonKinematicBody(dNewtonWorld* const world, dNewtonCollision* const collision, dMatrix matrix, dFloat mass);
};

class dNewtonDynamicBody: public dNewtonBody
{
	public:
	dNewtonDynamicBody(dNewtonWorld* const world, dNewtonCollision* const collision, dMatrix matrix, dFloat mass);

	private:
	virtual void OnForceAndTorque(dFloat timestep, int threadIndex);
	virtual void InitForceAccumulators();

	virtual void AddForce(dFloat x, dFloat y, dFloat z);
	virtual void AddTorque(dFloat x, dFloat y, dFloat z);

	dVector m_externalForce;
	dVector m_externalTorque;
};

#endif
