/*=============================================================================*/
// Copyright 2020-2021 Elite Engine
/*=============================================================================*/
// Behaviors.h: Implementation of certain reusable behaviors for the BT version of the Agario Game
/*=============================================================================*/
#ifndef ELITE_APPLICATION_BEHAVIOR_TREE_BEHAVIORS
#define ELITE_APPLICATION_BEHAVIOR_TREE_BEHAVIORS
//-----------------------------------------------------------------
// Includes & Forward Declarations
//-----------------------------------------------------------------
#include "framework/EliteMath/EMath.h"
#include "framework/EliteAI/EliteDecisionMaking/EliteBehaviorTree/EBehaviorTree.h"
#include "projects/Shared/Agario/AgarioAgent.h"
#include "projects/Shared/Agario/AgarioFood.h"
#include "projects/Movement/SteeringBehaviors/Steering/SteeringBehaviors.h"

//-----------------------------------------------------------------
// Behaviors
//-----------------------------------------------------------------


// ACTION
Elite::BehaviorState ChangeToWander(Elite::Blackboard* pBlackboard);

Elite::BehaviorState ChangeToSeek(Elite::Blackboard* pBlackboard);

Elite::BehaviorState ChangeToFlee(Elite::Blackboard* pBlackboard);

Elite::BehaviorState ChangeToPersuit(Elite::Blackboard* pBlackboard);

// CONDITIONAL	
bool IsCloseToFood(Elite::Blackboard* pBlackboard);

bool IsCloseToBiggerAgent(Elite::Blackboard* pBlackboard);

bool IsCloseToSmallerAgent(Elite::Blackboard* pBlackboard);

#endif