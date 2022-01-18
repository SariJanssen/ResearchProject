#include "stdafx.h"
#include "FlockingSteeringBehaviors.h"
#include "TheFlock.h"
#include "../SteeringAgent.h"
#include "../SteeringHelpers.h"

//*******************
//COHESION (FLOCKING)
SteeringOutput Cohesion::CalculateSteering(float deltaT, SteeringAgent* pAgent)
{
	SteeringOutput steering {};

	if (m_pFlock->GetNrOfNeighbors() != 0)
	{
		Elite::Vector2 agentPos = pAgent->GetPosition();
		Elite::Vector2 average = m_pFlock->GetAverageNeighborPos();

		steering.LinearVelocity = average - agentPos;

		//Debug rendering
		if (pAgent->CanRenderBehavior())
		{
			DEBUGRENDERER2D->DrawPoint(average, 5.f, { 1, 0.5, 0, 0.5f }, 0.4f);
			DEBUGRENDERER2D->DrawDirection(agentPos, steering.LinearVelocity, steering.LinearVelocity.Magnitude(), { 1, 0.5, 0, 0.5f }, 0.4f);
		}
	}

	return steering;
}

//*********************
//SEPARATION (FLOCKING)
SteeringOutput Separation::CalculateSteering(float deltaT, SteeringAgent* pAgent)
{
	SteeringOutput steering{};
	
	const std::vector<SteeringAgent*>& neighbors = m_pFlock->GetNeighbors();
	const size_t nrOfNeighbors = m_pFlock->GetNrOfNeighbors();

	Elite::Vector2 separation{};

	for (size_t i{}; i < nrOfNeighbors; ++i)
	{
		Elite::Vector2 toTarget{ neighbors[i]->GetPosition() - pAgent->GetPosition() };
		Elite::Vector2 awayTarget{ toTarget * -1.f };
		Elite::Vector2 iverseProportional{ awayTarget.GetNormalized() * 30.f / toTarget.Magnitude() };

		separation += iverseProportional;

		//if (pAgent->CanRenderBehavior())
		//{
		//	DEBUGRENDERER2D->DrawDirection(pAgent->GetPosition(), iverseProportional, iverseProportional.Magnitude(), { 0,0,0 }, 0.4f);
		//}
	}

	steering.LinearVelocity = separation.GetNormalized() * pAgent->GetMaxLinearSpeed();

	//Debug rendering
	if (pAgent->CanRenderBehavior())
	{
		DEBUGRENDERER2D->DrawDirection(pAgent->GetPosition(), steering.LinearVelocity, steering.LinearVelocity.Magnitude(), { 1, 0, 1, 0.5f }, 0.4f);
	}

	return steering;
}

//*************************
//VELOCITY MATCH (FLOCKING)
SteeringOutput VelocityMatch::CalculateSteering(float deltaT, SteeringAgent* pAgent)
{
	SteeringOutput steering = {};
	steering.LinearVelocity = m_pFlock->GetAverageNeighborVelocity();

	//Debug rendering
	if (pAgent->CanRenderBehavior())
	{
		DEBUGRENDERER2D->DrawDirection(pAgent->GetPosition(), steering.LinearVelocity, 5.f, { 0, 1, 0, 0.5f }, 0.4f);
	}

	return steering;
}
