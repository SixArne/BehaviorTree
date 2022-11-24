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
namespace FSMStates 
{
	class WanderState : public Elite::FSMState
	{
	public:
		WanderState() : FSMState() {};
		virtual void OnEnter(Elite::Blackboard* pBlackBoard) override;
		virtual void OnExit(Elite::Blackboard* pBlackBoard) override;
		virtual void Update(Elite::Blackboard* pBlackBoard, float deltaTime) override;
	};

	class SeekFoodState: public Elite::FSMState
	{
	public:
		SeekFoodState() : Elite::FSMState() {};
		virtual void OnEnter(Elite::Blackboard* pBlackBoard) override;
		virtual void Update(Elite::Blackboard* pBlackBoard, float deltaTime) override;
	};
}

//-----------------
//---TRANSITIONS---
//-----------------

namespace FSMConditions
{
	class FoodNearByCondition : public Elite::FSMCondition 
	{
	public:
		FoodNearByCondition() : Elite::FSMCondition() {};

		// Inherited via FSMCondition
		virtual bool Evaluate(Elite::Blackboard* pBlackboard) const override;
	};

	class FoodGoneCondition : public Elite::FSMCondition
	{
	public:
		FoodGoneCondition() : Elite::FSMCondition() {};

		// Inherited via FSMCondition
		virtual bool Evaluate(Elite::Blackboard* pBlackboard) const override;
	};

	class FoodSpawnNearByCondition : public Elite::FSMCondition
	{
	public:
		FoodSpawnNearByCondition() : Elite::FSMCondition() {};

		// Inherited via FSMCondition
		virtual bool Evaluate(Elite::Blackboard* pBlackboard) const override;
	};
}

#endif