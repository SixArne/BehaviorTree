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

namespace BT_Actions
{
	Elite::BehaviorState ChangeToWander(Elite::Blackboard* pBlackboard)
	{
		AgarioAgent* agent{};
		if (!pBlackboard->GetData("Agent", agent) || agent == nullptr)
		{
			return Elite::BehaviorState::Failure;
		}

		agent->SetToWander();
		return Elite::BehaviorState::Success;
	}

	Elite::BehaviorState ChangeToSeek(Elite::Blackboard* pBlackboard)
	{
		AgarioAgent* agent{};
		Elite::Vector2 targetPos{};
		if (!pBlackboard->GetData("Agent", agent) || agent == nullptr)
		{
			return Elite::BehaviorState::Failure;
		}

		if (!pBlackboard->GetData("Target", targetPos))
		{
			return Elite::BehaviorState::Failure;
		}

		agent->SetToSeek(targetPos);
		return Elite::BehaviorState::Success;
	}
}

namespace BT_Conditions
{
	bool IsFoodNearby(Elite::Blackboard* pBlackboard)
	{
		AgarioAgent* agent{};
		std::vector<AgarioFood*>* foodVec{};

		if (!pBlackboard->GetData("Agent", agent) || agent == nullptr)
		{
			return false;
		}

		if (!pBlackboard->GetData("FoodVec", foodVec) || agent == nullptr)
		{
			return false;
		}

		const float searchRadius{ 50.f + agent->GetRadius()};


		AgarioFood* pClosestFood{};
		float closestDistanceSq{searchRadius * searchRadius};

		Elite::Vector2 agentPos = agent->GetPosition();

		// TODO: debug rendering
		for (auto& pFood : *foodVec)
		{
			float distSq = pFood->GetPosition().DistanceSquared(agentPos);
			if (distSq < closestDistanceSq)
			{
				closestDistanceSq = distSq;
				pClosestFood = pFood;
			}
		}

		if (pClosestFood != nullptr)
		{
			pBlackboard->ChangeData("Target", pClosestFood->GetPosition());
			return true;
		}

		return false;
	}
}











#endif