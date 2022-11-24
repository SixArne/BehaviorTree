#include "stdafx.h"
#include "SandboxAgent.h"

using namespace Elite;

SandboxAgent::SandboxAgent(): BaseAgent()
{
	m_Target = GetPosition();
}

void SandboxAgent::Update(float dt)
{
	const float maxSpeed{ 50.f };
	const float arrivalRadius{ 1.0f };
	const float slowRadius{ 15.f };

	Elite::Vector2 toTarget{m_Target - GetPosition()};
	const float distance = toTarget.MagnitudeSquared();

	if (distance < arrivalRadius * arrivalRadius)
	{
		SetLinearVelocity({ 0,0 });
		return;
	}

	auto velocity = toTarget.GetNormalized();

	if (distance < slowRadius * slowRadius)
	{
		velocity *= maxSpeed * distance / (slowRadius * slowRadius);
	}
	else
	{
		velocity *= maxSpeed;
	}

	SetLinearVelocity(velocity);

	//Orientation
	AutoOrient();
}

void SandboxAgent::Render(float dt)
{
	BaseAgent::Render(dt); //Default Agent Rendering
}

void SandboxAgent::AutoOrient()
{
	//Determine angle based on direction
	Vector2 velocity = GetLinearVelocity();
	if (velocity.Magnitude() > 0)
	{
		velocity.Normalize();
		SetRotation(atan2(velocity.y, velocity.x) + E_PI_2);
	}

	SetRotation(GetRotation() + E_PI_2);
}