#pragma once
#include <vector>
#include <iostream>
#include "framework/EliteMath/EMath.h"
#include "framework\EliteAI\EliteGraphs\ENavGraph.h"
#include "framework\EliteAI\EliteGraphs\EliteGraphAlgorithms\EAStar.h"

#define OPTIMIZED_PATH

namespace Elite
{
	class NavMeshPathfinding
	{
	public:
		static std::vector<Elite::Vector2> FindPath(Elite::Vector2 startPos, Elite::Vector2 endPos, Elite::NavGraph* pNavGraph, std::vector<Elite::Vector2>& debugNodePositions, std::vector<Elite::Portal>& debugPortals)
		{
			//Create the path to return
			std::vector<Elite::Vector2> tempPath{};
			std::vector<Elite::Vector2> finalPath{};

			//Get the start and endTriangle
			auto navMesh = pNavGraph->GetNavMeshPolygon();
			Triangle* startTriangle{ nullptr };
			Triangle* endTriangle{ nullptr };
			for (auto triangle : navMesh->GetTriangles())
			{
				if (Elite::PointInTriangle(startPos, triangle->p1, triangle->p2, triangle->p3, true))
				{
					startTriangle = triangle;
				}
				if (Elite::PointInTriangle(endPos, triangle->p1, triangle->p2, triangle->p3, true))
				{
					endTriangle = triangle;
				}
				if (endTriangle != nullptr && startTriangle != nullptr)
				{
					break;
				}
			}

			if (startTriangle == nullptr || endTriangle == nullptr)
			{
				return finalPath;
			}
			else if (startTriangle == endTriangle)
			{
				finalPath.push_back(endPos);
				return finalPath;
			}

			//We have valid start/end triangles and they are not the same
			//=> Start looking for a path
			//Copy the graph
			auto pGraphCopy = pNavGraph->Clone();

			//Create extra node for the Start Node (Agent's position)
			NavGraphNode* startNode = new NavGraphNode(pGraphCopy->GetNextFreeNodeIndex(), -1, startPos);
			pGraphCopy->AddNode(startNode);
			for (auto lineIdx : startTriangle->metaData.IndexLines)
			{
				for (auto node : pGraphCopy->GetAllNodes())
				{
					if (node->GetLineIndex() == lineIdx)
					{
						GraphConnection2D* pConnection = new GraphConnection2D{ startNode->GetIndex(), node->GetIndex() };
						pConnection->SetCost(Elite::Distance(startNode->GetPosition(), node->GetPosition()));
						pGraphCopy->AddConnection(pConnection);
						break;
					}
				}
			}
			
			//Create extra node for the endNode
			NavGraphNode* endNode = new NavGraphNode(pGraphCopy->GetNextFreeNodeIndex(), -1, endPos);
			pGraphCopy->AddNode(endNode);
			for (auto lineIdx : endTriangle->metaData.IndexLines)
			{
				for (auto node : pGraphCopy->GetAllNodes())
				{
					if (node->GetLineIndex() == lineIdx)
					{
						GraphConnection2D* pConnection = new GraphConnection2D{ node->GetIndex(), endNode->GetIndex()};
						pConnection->SetCost(Elite::Distance(endNode->GetPosition(), node->GetPosition()));
						pGraphCopy->AddConnection(pConnection);
						break;
					}
				}
			}

			//Run A star on new graph
			Elite::Heuristic heuristicFunction = Elite::HeuristicFunctions::Euclidean;
			auto pathfinder = AStar<NavGraphNode, GraphConnection2D>(pGraphCopy.get(), heuristicFunction);
			auto path = pathfinder.FindPath(startNode, endNode);

			for (auto node : path)
			{

#ifndef OPTIMIZED_PATH
				finalPath.push_back(node->GetPosition());
#endif // !OPTIMIZED_PATH

				debugNodePositions.push_back(node->GetPosition());
			}


			//OPTIONAL BUT ADVICED: Debug Visualisation

			//Run optimiser on new graph, MAKE SURE the A star path is working properly before starting this section and uncommenting this!!!
			//m_Portals = SSFA::FindPortals(nodes, m_pNavGraph->GetNavMeshPolygon());
			//finalPath = SSFA::OptimizePortals(m_Portals);

#ifdef OPTIMIZED_PATH
			auto& portals = SSFA::FindPortals(path, pNavGraph->GetNavMeshPolygon());
			debugPortals = portals;

			finalPath = SSFA::OptimizePortals(portals);
#endif // OPTIMIZED_PATH

			return finalPath;
		}
	};
}
