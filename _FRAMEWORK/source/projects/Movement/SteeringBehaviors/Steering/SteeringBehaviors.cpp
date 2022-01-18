//Precompiled Header [ALWAYS ON TOP IN CPP]
#include "stdafx.h"

//Includes
#include "SteeringBehaviors.h"
#include "../SteeringAgent.h"
#include "../Obstacle.h"
#include "framework\EliteMath\EMatrix2x3.h"

using namespace Elite;

//SEEK
//****
SteeringOutput Seek::CalculateSteering(float deltaT, SteeringAgent* pAgent)
{
	SteeringOutput steering = {};
	steering.LinearVelocity = m_Target.Position - pAgent->GetPosition();
	steering.LinearVelocity.Normalize();
	steering.LinearVelocity *= pAgent->GetMaxLinearSpeed();

	//Debug rendering
	if (pAgent->CanRenderBehavior())
	{
		DEBUGRENDERER2D->DrawDirection(pAgent->GetPosition(), steering.LinearVelocity, steering.LinearVelocity.Magnitude(), {1, 1, 0, 0.5f}, 0.4f);
	}

	return steering;
}

//FLEE
//****
SteeringOutput Flee::CalculateSteering(float deltaT, SteeringAgent* pAgent)
{
	SteeringOutput steering = {};
	
	float distanceToTarget = Distance(pAgent->GetPosition(), m_Target.Position);
	if (distanceToTarget > m_FleeRadius)
	{
		return SteeringOutput(ZeroVector2, 0.f, false);
	}

	steering.LinearVelocity = Seek::CalculateSteering(deltaT, pAgent).LinearVelocity * -1.f;

	return steering;
}

//ARRIVE
//****
SteeringOutput Arrive::CalculateSteering(float deltaT, SteeringAgent* pAgent)
{
	SteeringOutput steering = {};
	Elite::Vector2 toTarget = m_Target.Position - pAgent->GetPosition();
	const float distance = toTarget.Magnitude();

	Elite::Vector2 velocity = toTarget;
	velocity.Normalize();

	if (distance < m_SlowRadius)
	{
		velocity *= pAgent->GetMaxLinearSpeed() * distance / m_SlowRadius;
	}
	else
	{
		velocity *= pAgent->GetMaxLinearSpeed();
	}
	steering.LinearVelocity = velocity;

	//Debug rendering
	if (pAgent->CanRenderBehavior())
	{
		DEBUGRENDERER2D->DrawDirection(pAgent->GetPosition(), steering.LinearVelocity, steering.LinearVelocity.Magnitude(), { 0, 1, 0, 0.5f }, 0.4f);
	}

	return steering;
}

//FACE
//****
SteeringOutput Face::CalculateSteering(float deltaT, SteeringAgent* pAgent)
{
	pAgent->SetAutoOrient(false);
	SteeringOutput steering = {};
	
	Vector2 toTarget = m_Target.Position - pAgent->GetPosition();
	Vector2 direction = Elite::OrientationToVector(pAgent->GetOrientation());
	steering.AngularVelocity = Elite::AngleBetween(direction, toTarget);

	//Debug rendering
	if (pAgent->CanRenderBehavior())
	{
		DEBUGRENDERER2D->DrawDirection(pAgent->GetPosition(), direction, 5.f, { 0, 1, 0, 0.5f }, 0.4f);
	}

	return steering;
}

//WANDER
//****
SteeringOutput Wander::CalculateSteering(float deltaT, SteeringAgent* pAgent)
{
	SteeringOutput steering = {};

	Vector2 agentPosition = pAgent->GetPosition();
	Vector2 circleCenter = pAgent->GetPosition() + (GetNormalized(pAgent->GetDirection()) * m_OffsetDistance);
	
	// TODO SARI: add timer so AI only updates every second instead of every frame
	float angleOffset = randomFloat(m_MaxAngleChange * 2.f) - m_MaxAngleChange;
	m_WanderAngle += angleOffset;

	Vector2 newTarget = { m_Radius * sin(m_WanderAngle) + circleCenter.x, m_Radius * cos(m_WanderAngle) + circleCenter.y };

	steering.LinearVelocity = newTarget - pAgent->GetPosition();
	steering.LinearVelocity.Normalize();
	steering.LinearVelocity *= pAgent->GetMaxLinearSpeed();

	//Debug rendering
	if (pAgent->CanRenderBehavior())
	{
		DEBUGRENDERER2D->DrawCircle(circleCenter, m_Radius, { 0, 0, 1, 0.5f }, 0.4f);
		DEBUGRENDERER2D->DrawPoint(newTarget, 5.f, { 0, 0, 1, 0.5f }, 0.41f);
		DEBUGRENDERER2D->DrawDirection(agentPosition, steering.LinearVelocity, steering.LinearVelocity.Magnitude(), { 0, 0, 1, 0.5f }, 0.4f);
	}

	return steering;
}

//PERSUIT
//****
SteeringOutput Persuit::CalculateSteering(float deltaT, SteeringAgent* pAgent)
{
	//Elite::Vector2 newTarget{ m_Target.Position + m_Target.LinearVelocity };
	//m_Target.Position = newTarget;

	//SteeringOutput steering{ Seek::CalculateSteering(deltaT, pAgent) };

	//if (pAgent->CanRenderBehavior())
	//{
	//	DEBUGRENDERER2D->DrawDirection(pAgent->GetPosition(), steering.LinearVelocity, steering.LinearVelocity.Magnitude(), { 0, 1, 0, 0.5f }, 0.4f);
	//	DEBUGRENDERER2D->DrawPoint(newTarget, 5.f, { 0, 0, 1, 0.5f }, 0.41f);
	//}

	SteeringOutput steering = {};

	float targetVelMultiplier = 1.f;
	Vector2 predictedLocation = m_Target.Position + m_Target.LinearVelocity * targetVelMultiplier;

	// calculate the difference in speed between the target and persusuor and adjust the targeded location based on this
	float speedDif = pAgent->GetMaxLinearSpeed() - m_Target.LinearVelocity.Magnitude();
	auto offset = m_Target.LinearVelocity;
	offset.Normalize();
	offset *= speedDif;

	// make sure targeted location doesnt go behind target due to offset
	if (offset.Magnitude() <= (m_Target.LinearVelocity * targetVelMultiplier).Magnitude())
	{
		predictedLocation -= offset;
	}
	else
	{
		predictedLocation = m_Target.Position;
	}

	steering.LinearVelocity = predictedLocation - pAgent->GetPosition();
	steering.LinearVelocity.Normalize();
	steering.LinearVelocity *= pAgent->GetMaxLinearSpeed();

	//Debug rendering
	if (pAgent->CanRenderBehavior())
	{
		DEBUGRENDERER2D->DrawPoint(predictedLocation, 5.f, { 0, 0, 1, 0.5f }, 0.41f);
		DEBUGRENDERER2D->DrawDirection(pAgent->GetPosition(), steering.LinearVelocity, steering.LinearVelocity.Magnitude(), { 0, 1, 0, 0.5f }, 0.4f);
	}

	return steering;
}

//EVADE
//****
SteeringOutput Evade::CalculateSteering(float deltaT, SteeringAgent* pAgent)
{
	SteeringOutput steering = {};
	float distanceToTarget = Distance(pAgent->GetPosition(), m_Target.Position);
	if (distanceToTarget > m_EvadeRadius)
	{
		return SteeringOutput(ZeroVector2, 0.f, false);
	}
	steering.LinearVelocity = Persuit::CalculateSteering(deltaT, pAgent).LinearVelocity * -1;

	//Debug rendering
	if (pAgent->CanRenderBehavior())
	{
		DEBUGRENDERER2D->DrawCircle(pAgent->GetPosition(), m_EvadeRadius, { 1, 0, 0, 0.5f }, 0.4f);
		DEBUGRENDERER2D->DrawDirection(pAgent->GetPosition(), steering.LinearVelocity, steering.LinearVelocity.Magnitude(), { 1, 0, 0, 0.5f }, 0.4f);
	}

	return steering;
}
