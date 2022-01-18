/*=============================================================================*/
// Copyright 2020-2021 Elite Engine
/*=============================================================================*/
// StatesAndTransitions.h: Implementation of the state/transition classes
/*=============================================================================*/
#ifndef ELITE_APPLICATION_FSM_STATES_TRANSITIONS
#define ELITE_APPLICATION_FSM_STATES_TRANSITIONS

#include "projects/Shared/Agario/AgarioAgent.h"
#include "projects/Shared/Agario/AgarioFood.h"
#include "projects/Movement/SteeringBehaviors/Steering/SteeringBehaviors.h"
#include "framework/EliteAI/EliteData/EBlackboard.h"

//------------
//---STATES---
//------------
class WanderState : public Elite::FSMState
{
public:
	virtual void OnEnter(Elite::Blackboard* pBlackboard) override
	{
		AgarioAgent* pAgent{ nullptr };
		bool success = pBlackboard->GetData("Agent", pAgent);

		if (!success)
		{
			return;
		}

		pAgent->SetToWander();
	}
};

class SeekState : public Elite::FSMState
{
public: 
	virtual void OnEnter(Elite::Blackboard* pBlackboard) override
	{
		AgarioAgent* pAgent{ nullptr };
		std::vector<AgarioFood*>* pFoodVec{};
		bool agentSuccess = pBlackboard->GetData("Agent", pAgent);
		bool foodSucces = pBlackboard->GetData("FoodVec", pFoodVec);

		if (!agentSuccess || !foodSucces)
		{
			return;
		}

		AgarioFood* closestFood{ nullptr };
		float distance{ FLT_MAX };
		for (auto food : *pFoodVec)
		{
			float tempDist = Distance(food->GetPosition(), pAgent->GetPosition());
			if (tempDist < distance)
			{
				closestFood = food;
				distance = tempDist;
			}
		}

		pBlackboard->ChangeData("FoodTarget", closestFood);
		pAgent->SetToSeek(closestFood->GetPosition());
	}

	virtual void OnExit(Elite::Blackboard* pBlackboard) override
	{
		pBlackboard->ChangeData("FoodTarget", static_cast<AgarioFood*>(nullptr));
	}

};

class FleeState : public Elite::FSMState
{
public: 
	virtual void OnEnter(Elite::Blackboard* pBlackboard) override
	{
		AgarioAgent* pAgent{ nullptr };
		AgarioAgent* pAgentToFlee{ nullptr };
		bool success = pBlackboard->GetData("Agent", pAgent);
		bool fleeSuccess = pBlackboard->GetData("AgentFleeTarget", pAgentToFlee);

		if (!success || !fleeSuccess || pAgentToFlee == nullptr)
		{
			return;
		}

		pAgent->SetToFlee(pAgentToFlee->GetPosition());
	}

	virtual void OnExit(Elite::Blackboard* pBlackboard) override
	{
		pBlackboard->ChangeData("AgentFleeTarget", static_cast<AgarioAgent*>(nullptr));
	}
};

//-----------------
//---TRANSITIONS---
//-----------------
class WanderToFlee : public Elite::FSMTransition
{
public: 
	virtual bool ToTransition(Elite::Blackboard* pBlackboard) const override
	{
		AgarioAgent* pAgent{ nullptr };
		AgarioAgent* pAgentToFlee{ nullptr };
		bool success = pBlackboard->GetData("Agent", pAgent);
		bool fleeSuccess = pBlackboard->GetData("AgentFleeTarget", pAgentToFlee);

		if (!success || !fleeSuccess)
		{
			return false;
		}

		float fleeRadius = static_cast<Flee*>(pAgent->GetSteeringBehavior())->GetFleeRadius();
		if (pAgentToFlee == nullptr)
		{
			std::vector<AgarioAgent*>* agentsVec{ nullptr };
			bool foundAgentVec = pBlackboard->GetData("AgentsVec", agentsVec);
			if (!foundAgentVec)
			{
				return false;
			}

			for (auto agent : *agentsVec)
			{
				Elite::Vector2 agentCircle = pAgent->GetPosition() + pAgent->GetRadius() * pAgent->GetDirection();
				Elite::Vector2 otherAgentCircle = agent->GetPosition() + agent->GetRadius() * agent->GetDirection();
				if (Distance(agentCircle, otherAgentCircle) < fleeRadius 
					&& agent->GetRadius() > pAgent->GetRadius())
				{
					pBlackboard->ChangeData("AgentFleeTarget", agent);
					return true;
				}
			}
		}
		else
		{
			Elite::Vector2 agentCircle = pAgent->GetPosition() + pAgent->GetRadius() * pAgent->GetDirection();
			Elite::Vector2 otherAgentCircle = pAgentToFlee->GetPosition() + pAgentToFlee->GetRadius() * pAgentToFlee->GetDirection();
			if (Distance(agentCircle, otherAgentCircle) < fleeRadius)
				return true;
		}

		return false;
	}
};

class SeekToFlee : public WanderToFlee
{
	virtual bool ToTransition(Elite::Blackboard* pBlackboard) const override
	{
		bool toTrans = WanderToFlee::ToTransition(pBlackboard);

		if (toTrans == false)
			return false;

		return true;
	}
};

class WanderToSeek : public Elite::FSMTransition
{
public:
	virtual bool ToTransition(Elite::Blackboard* pBlackboard) const override
	{
		AgarioAgent* pAgent{ nullptr };
		AgarioFood* foodTarget{ nullptr };
		std::vector<AgarioFood*>* foodVec{};
		bool agentSuccess = pBlackboard->GetData("Agent", pAgent);
		bool foodSucces = pBlackboard->GetData("FoodVec", foodVec);
		bool foodTargetSuccess = pBlackboard->GetData("FoodTarget", foodTarget);

		if (!agentSuccess || !foodSucces || !foodTargetSuccess)
		{
			return false;
		}

		if (foodVec->size() > 0)
			return true;

		if (foodTarget != nullptr)
		{
			return true;
		}

		std::vector<AgarioAgent*>* agentsVec{ nullptr };
		bool foundAgentVec = pBlackboard->GetData("AgentsVec", agentsVec);
		if (!foundAgentVec)
		{
			return false;
		}

		for (auto otherAgent : *agentsVec)
		{
			if (otherAgent->GetRadius() < pAgent->GetRadius())
			{
				return true;
			}
		}

		return false;
	}
};

class FleeToSeek : public WanderToSeek
{
public:
	virtual bool ToTransition(Elite::Blackboard* pBlackboard) const override
	{
		// Check if we're done Fleeing
		AgarioAgent* pAgent{ nullptr };
		AgarioAgent* pAgentToFlee{ nullptr };
		bool success = pBlackboard->GetData("Agent", pAgent);
		bool fleeSuccess = pBlackboard->GetData("AgentFleeTarget", pAgentToFlee);

		if (!success || !fleeSuccess)
		{
			return false;
		}

		float fleeRadius = static_cast<Flee*>(pAgent->GetSteeringBehavior())->GetFleeRadius();
		if (Distance(pAgent->GetPosition(), pAgentToFlee->GetPosition()) < fleeRadius)
		{
			return false;
		}

		// Check if we can seek
		return WanderToSeek::ToTransition(pBlackboard);
	}
};

class FleeToWander : public Elite::FSMTransition
{
public:
	virtual bool ToTransition(Elite::Blackboard* pBlackboard) const override
	{
		AgarioAgent* pAgent{ nullptr };
		AgarioAgent* pAgentToFlee{ nullptr };
		bool success = pBlackboard->GetData("Agent", pAgent);
		bool fleeSuccess = pBlackboard->GetData("AgentFleeTarget", pAgentToFlee);

		if (!success || !fleeSuccess || pAgentToFlee == nullptr)
		{
			return false;
		}

		float fleeRadius = static_cast<Flee*>(pAgent->GetSteeringBehavior())->GetFleeRadius();
		if (Distance(pAgent->GetPosition(), pAgentToFlee->GetPosition()) > fleeRadius)
		{
			return true;
		}
		return false;
	}
};

class SeekToWander : public FleeToWander
{
	virtual bool ToTransition(Elite::Blackboard* pBlackboard) const override
	{
		AgarioAgent* pAgent{ nullptr };
		std::vector<AgarioFood*>* foodVec{};
		bool success = pBlackboard->GetData("Agent", pAgent);
		bool foodSucces = pBlackboard->GetData("FoodVec", foodVec);

		if (!success || !foodSucces)
		{
			return false;
		}

		TargetData seekTarget = static_cast<Seek*>(pAgent->GetSteeringBehavior())->GetTarget();
		Elite::Vector2 agentCircle = pAgent->GetPosition() + pAgent->GetRadius() * pAgent->GetDirection();
		if (foodVec->size() > 0 && Distance(seekTarget.Position, agentCircle) > 0.5f)
		{
			return false;
		}

		return true;
	}
};

#endif