#pragma once
#include <vector>
#include <iostream>
#include "framework/EliteMath/EMath.h"
#include "framework\EliteAI\EliteGraphs\ENavGraph.h"
#include "framework\EliteAI\EliteGraphs\EliteGraphAlgorithms\EAStar.h"

namespace Elite
{
	class NavMeshPathfinding
	{
	public:
		static std::vector<Vector2> FindPath(Vector2 startPos, Vector2 endPos, NavGraph* pNavGraph, std::vector<Vector2>& debugNodePositions, std::vector<Portal>& debugPortals)
		{
			//Create the path to return
			std::vector<Vector2> finalPath{};

			//Get the start and endTriangle
			const auto startTriangle = pNavGraph->GetNavMeshPolygon()->GetTriangleFromPosition(startPos);
			const auto endTriangle = pNavGraph->GetNavMeshPolygon()->GetTriangleFromPosition(endPos);

			if (!startTriangle || !endTriangle)
				return finalPath;

			if (startTriangle == endTriangle)
			{
				finalPath.emplace_back(endPos);
				return finalPath;
			}

			//We have valid start/end triangles and they are not the same
			//=> Start looking for a path
			//Copy the graph
			auto graphCopy = pNavGraph->Clone();
			
			//Create extra node for the Start Node (Agent's position
			const int startNodeIndex = graphCopy->GetNextFreeNodeIndex();
			graphCopy->AddNode(new NavGraphNode{startNodeIndex, -1, startPos});

			for (const auto index : startTriangle->metaData.IndexLines)
			{
				const int nodeIndex = pNavGraph->GetNodeIdxFromLineIdx(index);

				if (nodeIndex == invalid_node_index)
					continue;

				const auto nodePos = graphCopy->GetNode(nodeIndex)->GetPosition();
				const auto connection = new GraphConnection2D{startNodeIndex, nodeIndex, Distance(startPos, nodePos)};
				graphCopy->AddConnection(connection);
			}
			
			//Create extra node for the endNode
			const int endNodeIndex = graphCopy->GetNextFreeNodeIndex() ;
			graphCopy->AddNode(new NavGraphNode{ endNodeIndex, -1, endPos });

			for (const auto index : endTriangle->metaData.IndexLines)
			{
				const int nodeIndex = pNavGraph->GetNodeIdxFromLineIdx(index);

				if (nodeIndex == invalid_node_index)
					continue;

				const auto nodePos = graphCopy->GetNode(nodeIndex)->GetPosition();
				const auto connection = new GraphConnection2D{ endNodeIndex, nodeIndex, Distance(endPos, nodePos) };
				graphCopy->AddConnection(connection);
			}

			//Run A star on new graph
			auto pathfinder = AStar<NavGraphNode, GraphConnection2D>(graphCopy.get(), Elite::HeuristicFunctions::Euclidean);
			const auto startNode = graphCopy->GetNode(startNodeIndex);
			const auto endNode = graphCopy->GetNode(endNodeIndex);

			const auto result = pathfinder.FindPath(startNode, endNode);


			debugNodePositions.clear();
			for (const auto node : result)
			{
				debugNodePositions.push_back(node->GetPosition());
			}

			//OPTIONAL BUT ADVICED: Debug Visualisation

			//Run optimiser on new graph, MAKE SURE the A star path is working properly before starting this section and uncommenting this!!!
			auto portals = SSFA::FindPortals(result, pNavGraph->GetNavMeshPolygon());
			debugPortals = portals;

			finalPath = SSFA::OptimizePortals(portals);

			return finalPath;
		}
	};
}
