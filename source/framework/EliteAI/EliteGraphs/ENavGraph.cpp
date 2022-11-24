#include "stdafx.h"
#include "ENavGraph.h"
#include "framework\EliteAI\EliteGraphs\EliteGraphAlgorithms\EAStar.h"

using namespace Elite;

Elite::NavGraph::NavGraph(const Polygon& contourMesh, float playerRadius = 1.0f) :
	Graph2D(false),
	m_pNavMeshPolygon(nullptr)
{
	//Create the navigation mesh (polygon of navigatable area= Contour - Static Shapes)
	m_pNavMeshPolygon = new Polygon(contourMesh); // Create copy on heap

	//Get all shapes from all static rigidbodies with NavigationCollider flag
	auto vShapes = PHYSICSWORLD->GetAllStaticShapesInWorld(PhysicsFlags::NavigationCollider);

	//Store all children
	for (auto shape : vShapes)
	{
		shape.ExpandShape(playerRadius);
		m_pNavMeshPolygon->AddChild(shape);
	}

	//Triangulate
	m_pNavMeshPolygon->Triangulate();

	//Create the actual graph (nodes & connections) from the navigation mesh
	CreateNavigationGraph();
}

Elite::NavGraph::~NavGraph()
{
	delete m_pNavMeshPolygon; 
	m_pNavMeshPolygon = nullptr;
}

int Elite::NavGraph::GetNodeIdxFromLineIdx(int lineIdx) const
{
	auto nodeIt = std::find_if(m_Nodes.begin(), m_Nodes.end(), [lineIdx](const NavGraphNode* n) { return n->GetLineIndex() == lineIdx; });
	if (nodeIt != m_Nodes.end())
	{
		return (*nodeIt)->GetIndex();
	}

	return invalid_node_index;
}

Elite::Polygon* Elite::NavGraph::GetNavMeshPolygon() const
{
	return m_pNavMeshPolygon;
}


void Elite::NavGraph::CreateNavigationGraph()
{
	this->Clear();

	//1. Go over all the edges of the navigation mesh and create nodes
	const std::vector<Line*> edges = m_pNavMeshPolygon->GetLines();
	int nodeId{};

	for (const Line* edge : edges)
	{
		const auto triangles = m_pNavMeshPolygon->GetTrianglesFromLineIndex(edge->index);
		if (triangles.size() > 1) // We only want lines that neighbor 2 or more triangles
		{
			const auto middle = Lerp(edge->p1, edge->p2, .5f);
			const auto node = new NavGraphNode{ nodeId++, edge->index, middle };
			AddNode(node);
		}
	}

	//2. Create connections now that every node is created
	const auto triangles = m_pNavMeshPolygon->GetTriangles();
	for (const auto& triangle : triangles)
	{
		const auto triangleLineIndices = triangle->metaData.IndexLines;

		std::vector<int> validNodeConnections{};
		for (const auto triangleLineIndex : triangleLineIndices)
		{
			int nodeIdx = GetNodeIdxFromLineIdx(triangleLineIndex);

			if (nodeIdx >= 0)
			{
				validNodeConnections.emplace_back(nodeIdx);
			}
		}

		if (validNodeConnections.size() == 2)
		{
			const auto conn1 = new GraphConnection2D{ validNodeConnections[0] , validNodeConnections[1] };
			AddConnection(conn1);
		}
		else if (validNodeConnections.size() == 3)
		{
			const auto conn1 = new GraphConnection2D{ validNodeConnections[0], validNodeConnections[1] };
			const auto conn2 = new GraphConnection2D{ validNodeConnections[1], validNodeConnections[2] };
			const auto conn3 = new GraphConnection2D{ validNodeConnections[2], validNodeConnections[0] };

			AddConnection(conn1);
			AddConnection(conn2);
			AddConnection(conn3);
		}
	}
	
	//3. Set the connections cost to the actual distance
	SetConnectionCostsToDistance();
}

