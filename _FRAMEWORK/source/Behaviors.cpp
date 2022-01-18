#include "stdafx.h"
#include "projects\DecisionMaking\BehaviorTrees\Behaviors.h"

#include "framework/EliteMath/EMath.h"
#include "framework/EliteAI/EliteDecisionMaking/EliteBehaviorTree/EBehaviorTree.h"
#include "projects/Shared/Agario/AgarioAgent.h"
#include "projects/Shared/Agario/AgarioFood.h"
#include "projects/Movement/SteeringBehaviors/Steering/SteeringBehaviors.h"

//-----------------------------------------------------------------
// Behaviors
//-----------------------------------------------------------------


// ACTION
Elite::BehaviorState ChangeToWander(Elite::Blackboard* pBlackboard)
{
	AgarioAgent* pAgent{ nullptr };
	pBlackboard->GetData("Agent", pAgent);

	if (!pAgent)
	{
		return Elite::BehaviorState::Failure;
	}

	pAgent->SetToWander();
	return Elite::BehaviorState::Success;
}

Elite::BehaviorState ChangeToSeek(Elite::Blackboard* pBlackboard)
{
	AgarioAgent* pAgent{ nullptr };
	Elite::Vector2 food{};
	pBlackboard->GetData("Agent", pAgent);
	bool success = pBlackboard->GetData("Target", food);

	if (!pAgent || !success)
	{
		return Elite::BehaviorState::Failure;
	}

	pAgent->SetToSeek(food);
	return Elite::BehaviorState::Success;
}

Elite::BehaviorState ChangeToFlee(Elite::Blackboard* pBlackboard)
{
	AgarioAgent* pAgent{ nullptr };
	AgarioAgent* pAgentFleeTarget{ nullptr };
	pBlackboard->GetData("Agent", pAgent);
	pBlackboard->GetData("AgentFleeTarget", pAgentFleeTarget);

	if (!pAgent || !pAgentFleeTarget)
	{
		return Elite::BehaviorState::Failure;
	}

	pAgent->SetToFlee(pAgentFleeTarget->GetPosition());
	return Elite::BehaviorState::Success;
}

Elite::BehaviorState ChangeToPersuit(Elite::Blackboard* pBlackboard)
{
	AgarioAgent* pAgent{ nullptr };
	AgarioAgent* pAgentPersuit{ nullptr };
	pBlackboard->GetData("Agent", pAgent);
	pBlackboard->GetData("AgentToPersuit", pAgentPersuit);

	if (!pAgent || !pAgentPersuit)
	{
		return Elite::BehaviorState::Failure;
	}

	pAgent->SetToPersuit(pAgentPersuit->GetPosition());
	return Elite::BehaviorState::Success;
}

// CONDITIONAL	
bool IsCloseToFood(Elite::Blackboard* pBlackboard)
{
	AgarioAgent* pAgent{ nullptr };
	std::vector<AgarioFood*>* foodVec;
	pBlackboard->GetData("Agent", pAgent);
	bool success = pBlackboard->GetData("FoodVec", foodVec);

	if (!pAgent || !success || (*foodVec).size() < 1)
	{
		return false;
	}

	// lamda die distance gaat vergelijken tss agent pos en pos food en distance teruggeven/true of false geven

	// Look for closest food to agent
	AgarioFood* closestFood = (*foodVec)[0];
	float closestDist = Elite::DistanceSquared(pAgent->GetPosition(), (*foodVec)[0]->GetPosition());
	for (size_t i = 1; i < (*foodVec).size(); ++i)
	{
		Elite::Vector2 agentToFood = (*foodVec)[i]->GetPosition() - pAgent->GetPosition();
		float currentDist = Elite::DistanceSquared(pAgent->GetPosition(), (*foodVec)[i]->GetPosition());
		if (currentDist < closestDist)
		{
			closestFood = (*foodVec)[i];
			closestDist = currentDist;
		}
	}

	// Check if closest food is in range
	const float closeToFoodRange{ 20.f };
	if (closestDist < closeToFoodRange * closeToFoodRange) // squared because faster
	{
		pBlackboard->ChangeData("Target", closestFood->GetPosition());
		return true;
	}

	return false;
}

bool IsCloseToBiggerAgent(Elite::Blackboard* pBlackboard)
{
	AgarioAgent* pAgent{ nullptr };
	std::vector<AgarioAgent*>* agentVec{};
	pBlackboard->GetData("Agent", pAgent);
	bool success = pBlackboard->GetData("AgentsVec", agentVec);

	if (!pAgent || !success)
	{
		return false;
	}

	// Look for big agent in range
	float fleeRadius = pAgent->GetFleeRadius();
	float closestDist{ FLT_MAX };
	AgarioAgent* pClosestBigAgent{ nullptr };
	for (auto pOtherAgent : *agentVec)
	{
		float currentDist{ DistanceSquared(pOtherAgent->GetPosition(), pAgent->GetPosition()) };
		if (currentDist < (fleeRadius * fleeRadius) && currentDist < closestDist)
		{
			if (pOtherAgent->GetRadius() > pAgent->GetRadius())
			{
				closestDist = currentDist;
				pClosestBigAgent = pOtherAgent;
			}
		}
	}

	if (pClosestBigAgent != nullptr)
	{
		pBlackboard->ChangeData("AgentFleeTarget", pClosestBigAgent);
		return true;
	}

	return false;
}

bool IsCloseToSmallerAgent(Elite::Blackboard* pBlackboard)
{
	AgarioAgent* pAgent{ nullptr };
	std::vector<AgarioAgent*>* agentVec{};
	pBlackboard->GetData("Agent", pAgent);
	bool success = pBlackboard->GetData("AgentsVec", agentVec);

	if (!pAgent || !success)
	{
		return false;
	}

	// Look for smaller agent in range
	float range{ 20.f };
	float closestDist{ range * range };
	AgarioAgent* pClosestSmallerAgent{ nullptr };
	for (auto pOtherAgent : *agentVec)
	{
		float currentDist{ DistanceSquared(pOtherAgent->GetPosition(), pAgent->GetPosition()) };
		if (currentDist < closestDist)
		{
			if (pOtherAgent->GetRadius() < (pAgent->GetRadius() - 1.f))
			{
				closestDist = currentDist;
				pClosestSmallerAgent = pOtherAgent;
			}
		}
	}

	if (pClosestSmallerAgent != nullptr)
	{
		pBlackboard->ChangeData("AgentToPersuit", pClosestSmallerAgent);
		return true;
	}

	return false;
}
