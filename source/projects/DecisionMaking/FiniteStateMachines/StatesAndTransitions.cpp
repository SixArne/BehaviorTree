#include "stdafx.h"
#include "StatesAndTransitions.h"

void FSMStates::WanderState::OnEnter(Elite::Blackboard* pBlackBoard)
{
	AgarioAgent* pAgent;

	bool isValid = pBlackBoard->GetData("Agent", pAgent);
	if (!isValid || !pAgent)
	{
		return;
	}

	pAgent->SetToWander();
}

void FSMStates::WanderState::OnExit(Elite::Blackboard* pBlackBoard)
{
}

void FSMStates::WanderState::Update(Elite::Blackboard* pBlackBoard, float deltaTime)
{
}

bool FSMConditions::FoodNearByCondition::Evaluate(Elite::Blackboard* pBlackboard) const
{
	AgarioAgent* pAgent;
	std::vector<AgarioFood*>* pFoods;
	const float foodRadius = 25.f;

	bool isValid = pBlackboard->GetData("Agent", pAgent);
	if (!isValid || !pAgent)
	{
		return false;
	}

	isValid = pBlackboard->GetData("FoodVec", pFoods);
	if (!isValid || !pAgent)
	{
		return false;
	}

	Elite::Vector2 agentPosition = pAgent->GetPosition();

	DEBUGRENDERER2D->DrawCircle(agentPosition, foodRadius, Elite::Color{ 1,1,1 }, DEBUGRENDERER2D->NextDepthSlice());

	auto elementDis = [agentPosition](AgarioFood* pFood, AgarioFood* pFood2) {
		float dist1 = agentPosition.DistanceSquared(pFood->GetPosition());
		float dist2 = agentPosition.DistanceSquared(pFood2->GetPosition());
		
		return dist1 < dist2; 
	};
	auto closestFoodIt = std::min_element(pFoods->begin(), pFoods->end(), elementDis);

	if (closestFoodIt != pFoods->end())
	{
		AgarioFood* pFood = *closestFoodIt;

		if (agentPosition.DistanceSquared(pFood->GetPosition()) < foodRadius * foodRadius)
		{
			pBlackboard->ChangeData("FoodNearBy", pFood);
			return true;
		}
	}

	return false;
}

void FSMStates::SeekFoodState::OnEnter(Elite::Blackboard* pBlackBoard)
{
	AgarioAgent* currentAgent{ nullptr };
	AgarioFood* nearbyFood{ nullptr };

	// Get Agent
	auto hasFoundData = pBlackBoard->GetData("Agent", currentAgent);
	if (!hasFoundData)
	{
		std::cout << "No agent data" << std::endl;
		throw std::runtime_error("Agent data not found");
	}

	// Get closest food
	hasFoundData = pBlackBoard->GetData("FoodNearBy", nearbyFood);
	if (!hasFoundData)
	{
		std::cout << "No food data" << std::endl;
		throw std::runtime_error("Food data not found");
	}

	currentAgent->SetToSeek(nearbyFood->GetPosition());
}

void FSMStates::SeekFoodState::Update(Elite::Blackboard* pBlackBoard, float deltaTime)
{
	
}

bool FSMConditions::FoodGoneCondition::Evaluate(Elite::Blackboard* pBlackboard) const
{
	AgarioFood* nearbyFood{};

	auto hasFoundData = pBlackboard->GetData("FoodNearBy", nearbyFood);

	if (nearbyFood->CanBeDestroyed())
	{
		return true;
	}

	return false;
}

bool FSMConditions::FoodSpawnNearByCondition::Evaluate(Elite::Blackboard* pBlackboard) const
{
	AgarioFood* nearbyFood{};
	std::vector<AgarioFood*>* foods{};
	AgarioAgent* agent{};

	const float foodDetectionRadius{ 25.f };

	// Early exit check
	auto hasFoundData = pBlackboard->GetData("FoodNearBy", nearbyFood);
	if (!hasFoundData)
	{
		std::cout << "No nearby food found" << std::endl;
		return false;
	}

	hasFoundData = pBlackboard->GetData("Agent", agent);
	if (!hasFoundData) 
	{
		throw std::runtime_error("Unable to find agent");
	}

	hasFoundData = pBlackboard->GetData("FoodVec", foods);
	if (!hasFoundData || foods->empty())
	{
		std::cout << "No food found" << std::endl;
		return false;
	}

	Elite::Vector2 agentPosition{agent->GetPosition()};
	auto distCompare = [agentPosition](AgarioFood* food1, AgarioFood* food2) {
		const float dist1 = agentPosition.DistanceSquared(food1->GetPosition());
		const float dist2 = agentPosition.DistanceSquared(food1->GetPosition());

		return dist1 < dist2;
	};

	auto minDistIt = std::min_element(foods->begin(), foods->end(), distCompare);
	if (minDistIt != foods->end()) // Check if its found
	{
		AgarioFood* food = *minDistIt;

		if (agentPosition.DistanceSquared(food->GetPosition()) < foodDetectionRadius * foodDetectionRadius)
		{
			pBlackboard->ChangeData("FoodNearBy", food);
			return true;
		}
	}


	return false;
}
