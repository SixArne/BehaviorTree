//=== General Includes ===
#include "stdafx.h"
#include "EFiniteStateMachine.h"
using namespace Elite;

FiniteStateMachine::FiniteStateMachine(FSMState* startState, Blackboard* pBlackboard)
    : m_pCurrentState(nullptr),
    m_pBlackboard(pBlackboard)
{
    ChangeState(startState);
}

FiniteStateMachine::~FiniteStateMachine()
{
    SAFE_DELETE(m_pBlackboard);
}

void FiniteStateMachine::AddTransition(FSMState* startState, FSMState* toState, FSMCondition* condition)
{
    auto it = m_Transitions.find(startState);
    if (it == m_Transitions.end())
    {
        m_Transitions[startState] = Transitions();
    }
   
    m_Transitions[startState].push_back(std::make_pair(condition, toState));
}

void FiniteStateMachine::Update(float deltaTime)
{
    // Find transitions for current state
    auto transitionIt = m_Transitions.find(m_pCurrentState);
    if (transitionIt != m_Transitions.end()) // If transitions found
    {
        // Loop over and get conditions
        for (auto transition : transitionIt->second)
        {
            auto condition = transition.first;
            auto state = transition.second;

            // See of condition gets evaluated to true (if we can switch)
            if (condition->Evaluate(m_pBlackboard))
            {
                ChangeState(transition.second);
            }
        }
    }

    // Update current State
    m_pCurrentState->Update(m_pBlackboard, deltaTime);
}

Blackboard* FiniteStateMachine::GetBlackboard() const
{
    return m_pBlackboard;
}

void FiniteStateMachine::ChangeState(FSMState* newState)
{
    if (m_pCurrentState != nullptr)
    {
        m_pCurrentState->OnExit(m_pBlackboard);
    }
   
    m_pCurrentState = newState;

    if (m_pCurrentState == nullptr)
    {
        throw std::runtime_error("Currentstate is null");
    }
   
    m_pCurrentState->OnEnter(m_pBlackboard); 
}
