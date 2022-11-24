#include "stdafx.h"
#include "SpacePartitioning.h"
#include "projects\Movement\SteeringBehaviors\SteeringAgent.h"


// --- Cell ---
// ------------
Cell::Cell(float left, float bottom, float width, float height)
{
	boundingBox.bottomLeft = Elite::Vector2{ left, bottom };
	boundingBox.width = width;
	boundingBox.height = height;
}

std::vector<Elite::Vector2> Cell::GetRectPoints() const
{
	auto left = boundingBox.bottomLeft.x;
	auto bottom = boundingBox.bottomLeft.y;
	auto width = boundingBox.width;
	auto height = boundingBox.height;

	std::vector<Elite::Vector2> rectPoints =
	{
		{ left , bottom  },
		{ left , bottom + height  },
		{ left + width , bottom + height },
		{ left + width , bottom  },
	};

	return rectPoints;
}

// --- Partitioned Space ---
// -------------------------
CellSpace::CellSpace(float width, float height, int rows, int cols, int maxEntities)
	: m_SpaceWidth(width)
	, m_SpaceHeight(height)
	, m_NrOfRows(rows)
	, m_NrOfCols(cols)
	, m_Neighbors(maxEntities)
	, m_NrOfNeighbors(0)
{
	m_CellWidth = width / rows;
	m_CellHeight = height / cols;

	for (int rowIndex = 0; rowIndex < rows; ++rowIndex)
	{
		for (int colIndex = 0; colIndex < cols; ++colIndex)
		{
			m_Cells.push_back(Cell(colIndex * m_CellWidth, rowIndex * m_CellHeight, m_CellWidth, m_CellHeight));
		}
	}
}

void CellSpace::AddAgent(SteeringAgent* agent)
{
	int position = PositionToIndex(agent->GetPosition());
	m_Cells[position].agents.push_back(agent);
}

void CellSpace::UpdateAgentCell(SteeringAgent* agent, Elite::Vector2 oldPos)
{
	const Elite::Vector2 position{ agent->GetPosition() };
	int currentPosition = PositionToIndex(position);
	int oldPosition = PositionToIndex(oldPos);

	// Update position
	if (currentPosition != oldPosition)
	{
		m_Cells[oldPosition].agents.remove(agent);
		m_Cells[currentPosition].agents.push_back(agent);
	}
}

void CellSpace::RegisterNeighbors(SteeringAgent* agent, float queryRadius)
{
	// memory pool tracking
	m_NrOfNeighbors = 0;

	Elite::Vector2 position{ agent->GetPosition() };

	// Get grid index by position
	int agentCellIndex = PositionToIndex(position);
	int amountOfCells = queryRadius * 2 / m_CellWidth;

	// Get the current row and column
	int agentRow = agentCellIndex % m_NrOfCols;
	int agentCol = agentCellIndex / m_NrOfCols;

	// We determine the start of Row and column bounds
	int beginCol = Elite::Clamp(agentCol - amountOfCells, 0, m_NrOfCols - 1);
	int endCol = Elite::Clamp(agentCol + amountOfCells, 0, m_NrOfCols - 1);

	// We determine the end of Row and column bounds
	int beginRow = Elite::Clamp(agentRow - amountOfCells, 0, m_NrOfCols -1);
	int endRow = Elite::Clamp(agentRow + amountOfCells, 0, m_NrOfCols - 1);

	// This will only double loop over the actual neighborhood.
	// So for 1 cell radius this would be a 3x3 or 9 iteration loop.
	for (int c{ beginCol }; c <= endCol; c++)
	{
		for (int r{ beginRow }; r <= endRow; r++)
		{
			int cellNumber = r + c * m_NrOfCols;
			const Cell currentCell = m_Cells[cellNumber];

 			for (auto& cellAgent : currentCell.agents)
			{
				if (cellAgent == agent)
					continue;

				const float distanceSqr = cellAgent->GetPosition().DistanceSquared(position);

				if (distanceSqr <= queryRadius * queryRadius)
				{
					m_Neighbors[m_NrOfNeighbors++] = cellAgent;

					if (agent->CanRenderBehavior())
					{
						DEBUGRENDERER2D->DrawSolidCircle(cellAgent->GetPosition(), 2.f, { 0,1 }, { 1.f, 1.f, 0.f, 1.f });
					}
				}
			}

			if (agent->CanRenderBehavior())
			{
				Elite::Polygon polygonCenter = Elite::Polygon{ currentCell.GetRectPoints() };
				DEBUGRENDERER2D->DrawSolidPolygon(&polygonCenter, { 0,1,0 }, 0);
			}
		}
	}
}

void CellSpace::EmptyCells()
{
	for (Cell& c : m_Cells)
		c.agents.clear();
}

void CellSpace::RenderCells() const
{
	int count{};

	for (const Cell& c : m_Cells)
	{
		Elite::Polygon polygon = Elite::Polygon{ c.GetRectPoints() };
		DEBUGRENDERER2D->DrawPolygon(&polygon, { 1,0,0 });
		DEBUGRENDERER2D->DrawString(polygon.GetCenterPoint(), std::to_string(c.agents.size()).c_str());
	}
}

int CellSpace::PositionToIndex(const Elite::Vector2 pos) const
{
	// The x is the row, the y needs to be multiplied by nrOfCols to account for transforming it from
	// 2D to 1D.
	const int x = static_cast<int>(pos.x / m_CellWidth) % m_NrOfCols;
	const int y = static_cast<int>(pos.y / m_CellHeight) % m_NrOfRows * m_NrOfCols;

	return x + y;
}