#pragma once
#include <assert.h>
#include <memory>

namespace Elite
{
	template <class T_NodeType, class T_ConnectionType>
	class JPS
	{
	public:
		JPS(GridGraph<T_NodeType, T_ConnectionType>* pGraph, Heuristic hFunction);

		struct NodeRecord
		{
			T_NodeType* pNode = nullptr;
			std::shared_ptr<NodeRecord> pParent = nullptr;
			float gCost = 0.f; // cost so far = accumulated g-costs of all the connections leading up to this one
			float fCost = 0.f; // estimatedTotalCost (= costSoFar + h-cost)

			bool operator==(const NodeRecord& other) const
			{
				return pNode == other.pNode
					&& AreEqual(gCost, other.gCost)
					&& AreEqual(fCost, other.fCost)
					&& pParent == other.pParent;
			};

			bool operator<(const NodeRecord& other) const
			{
				return fCost < other.fCost;
			};
		};

		std::vector<T_NodeType*> FindPath(T_NodeType* pStartNode, T_NodeType* pDestinationNode);

	private:
		float GetHeuristicCost(T_NodeType* pStartNode, T_NodeType* pEndNode) const;
		std::vector<std::shared_ptr<NodeRecord>> IdentifySuccessors(std::shared_ptr<NodeRecord> pCurrentNode, T_NodeType* pStartNode, T_NodeType* pGoalNode);
		std::shared_ptr<NodeRecord> Jump(std::shared_ptr<NodeRecord> pCurrentNode, const Elite::Vector2& difference, T_NodeType* pStartNode, T_NodeType* pGoalNode);

		GridGraph<T_NodeType, T_ConnectionType>* m_pGraph;
		Heuristic m_HeuristicFunction;
	};

	template <class T_NodeType, class T_ConnectionType>
	JPS<T_NodeType, T_ConnectionType>::JPS(GridGraph<T_NodeType, T_ConnectionType>* pGraph, Heuristic hFunction)
		: m_pGraph(pGraph)
		, m_HeuristicFunction(hFunction)
	{
	}

	template <class T_NodeType, class T_ConnectionType>
	std::vector<std::shared_ptr<typename JPS<T_NodeType, T_ConnectionType>::NodeRecord>> JPS<T_NodeType, T_ConnectionType>::IdentifySuccessors(std::shared_ptr<typename JPS<T_NodeType, T_ConnectionType>::NodeRecord> pCurrentNode, T_NodeType* pStartNode, T_NodeType* pGoalNode)
	{
		std::vector<std::shared_ptr<NodeRecord>> successors{};

		// get neighbors of current
		auto& connectionList = m_pGraph->GetNodeConnections(pCurrentNode->pNode->GetIndex());
		// for each neighbor
		for (const auto& pConnection : connectionList)
		{
			// int x = clamp(neighbor.x - current.x, -1, 1)
			// int y = clamp(neighbor.y - current.y, -1, 1)
			int nodeIdx = pConnection->GetTo();
			T_NodeType* pNeighborNode = m_pGraph->GetNode(nodeIdx);

			Elite::Vector2 neighborNodePos = m_pGraph->GetNodeWorldPos(pNeighborNode);
			Elite::Vector2 currentNodePos = m_pGraph->GetNodeWorldPos(pCurrentNode->pNode);

			float x = neighborNodePos.x - currentNodePos.x;
			float y = neighborNodePos.y - currentNodePos.y;

			// jumpPoint = noderecord jump(current.x, current.y, x, y, start, end)
			std::shared_ptr<NodeRecord> jumpPoint = Jump(pCurrentNode, Elite::Vector2{ x, y }, pStartNode, pGoalNode);

			if (jumpPoint != nullptr)
				successors.push_back(jumpPoint);
		}
		return successors;
	}

	template <class T_NodeType, class T_ConnectionType>
	std::shared_ptr<typename JPS<T_NodeType, T_ConnectionType>::NodeRecord> JPS<T_NodeType, T_ConnectionType>::Jump(std::shared_ptr<typename JPS<T_NodeType, T_ConnectionType>::NodeRecord> pCurrentNode, const Elite::Vector2& difference, T_NodeType* pStartNode, T_NodeType* pGoalNode)
	{
		Elite::Vector2 currentNodePos = m_pGraph->GetNodeWorldPos(pCurrentNode->pNode);
		T_NodeType* pNextNode = m_pGraph->GetNodeAtWorldPos({ currentNodePos.x + difference.x, currentNodePos.y + difference.y });
		
		if (pNextNode == nullptr)
			return nullptr;

		if (pNextNode->GetTerrainType() == TerrainType::Water)
			return nullptr;

		NodeRecord nextJumpNR{};
		nextJumpNR.pNode = pNextNode;
		nextJumpNR.pParent = pCurrentNode;
		T_ConnectionType* pConnection = m_pGraph->GetConnection(pCurrentNode->pNode->GetIndex(), pNextNode->GetIndex());
		nextJumpNR.gCost = pConnection->GetCost() + pCurrentNode->gCost;
		nextJumpNR.fCost = nextJumpNR.gCost + GetHeuristicCost(pCurrentNode->pNode, pGoalNode);
		std::shared_ptr<NodeRecord> pNextJumpNR{std::make_shared<NodeRecord>(nextJumpNR)};

		if (pNextNode == pGoalNode)
			return pNextJumpNR;

		if (!Elite::AreEqual(difference.x, 0.f) && !Elite::AreEqual(difference.y, 0.f)) // Diagonal Case
		{
			// if(current.x + x == obstacle || current.y + y == obstacle) return next
			if (m_pGraph->GetNodeAtWorldPos({ currentNodePos.x + difference.x, currentNodePos.y })->GetTerrainType() == TerrainType::Water
			 || m_pGraph->GetNodeAtWorldPos({ currentNodePos.x, currentNodePos.y + difference.y })->GetTerrainType() == TerrainType::Water)
				return pNextJumpNR;

			// Check horizontal and vertical directions for forced neighbors
			// if (jump(next.x, next.y, x, 0, start, end) == null) return next
			if (Jump(pNextJumpNR, Elite::Vector2{difference.x, 0.f}, pStartNode, pGoalNode) == nullptr)
				return pNextJumpNR;

			// if (jump(next.x, next.y, 0, y, start, end) == null) return next
			if (Jump(pNextJumpNR, Elite::Vector2(0.f, difference.y), pStartNode, pGoalNode) == nullptr)
				return pNextJumpNR;

		}
		else if (!Elite::AreEqual(difference.x, 0.f)) // Horizontal Case
		{
			int idx{};
			int nrCols = m_pGraph->GetColumns();
			
			idx = pCurrentNode->pNode->GetIndex() + nrCols;
			if (m_pGraph->IsNodeValid(idx))
			{
				float nodeUp = m_pGraph->GetNodeWorldPos(idx).y;
				// if (current.y + 1 == obstacle) && if (current.x + x, current.y + 1 != obstacle)
				if (m_pGraph->GetNodeAtWorldPos({ currentNodePos.x, nodeUp })->GetTerrainType() == TerrainType::Water
				 && m_pGraph->GetNodeAtWorldPos({ currentNodePos.x + difference.x, nodeUp })->GetTerrainType() != TerrainType::Water)
					return pNextJumpNR;
			}

			idx = pCurrentNode->pNode->GetIndex() - nrCols;
			if (m_pGraph->IsNodeValid(idx) && idx >= 0)
			{
				float nodeDown = m_pGraph->GetNodeWorldPos(idx).y;
				// else if (current.y - 1 == obstacle) && if (current.x + x, current.y - 1 != obstacle)
				if (m_pGraph->GetNodeAtWorldPos({ currentNodePos.x, nodeDown })->GetTerrainType() == TerrainType::Water
				 && m_pGraph->GetNodeAtWorldPos({ currentNodePos.x + difference.x, nodeDown })->GetTerrainType() != TerrainType::Water)
					return pNextJumpNR;
			}
		}
		else  //Vertical Case
		{
			int idx{};
			int nrRows = m_pGraph->GetRows();

			// need to check node left and node right if valid 
			// but node + difference will be valid because they are checked in successors
			idx = pCurrentNode->pNode->GetIndex() + 1;
			if (m_pGraph->IsNodeValid(idx) && (int(idx / nrRows) == int(pCurrentNode->pNode->GetIndex() / nrRows)))
			{
				float nodeLeft = m_pGraph->GetNodeWorldPos(idx).x;

				// if (current.x + 1 == obstacle) && if (current.x + 1, current.y + y != obstacle)
				// return next
				if (m_pGraph->GetNodeAtWorldPos({ nodeLeft, currentNodePos.y })->GetTerrainType() == TerrainType::Water
				 && m_pGraph->GetNodeAtWorldPos({ nodeLeft, currentNodePos.y + difference.y })->GetTerrainType() != TerrainType::Water)
					return pNextJumpNR;
			}

			idx = pCurrentNode->pNode->GetIndex() - 1;
			if (m_pGraph->IsNodeValid(idx) && idx >= 0 && (int(idx / nrRows) == int(pCurrentNode->pNode->GetIndex() / nrRows)))
			{
				float nodeRight = m_pGraph->GetNodeWorldPos(idx).x;

				// else if (current.x - 1 == obstacle) && if (current.x - 1, current.y + y != obstacle)
					// return next
				if (m_pGraph->GetNodeAtWorldPos({ nodeRight, currentNodePos.y })->GetTerrainType() == TerrainType::Water
				 && m_pGraph->GetNodeAtWorldPos({ nodeRight, currentNodePos.y + difference.y })->GetTerrainType() != TerrainType::Water)
					return pNextJumpNR;
			}
		}
		
		// return jump(next.x, next.y. x, y. start, end)
		return Jump(pNextJumpNR, difference, pStartNode, pGoalNode);
	}

	template <class T_NodeType, class T_ConnectionType>
	std::vector<T_NodeType*> JPS<T_NodeType, T_ConnectionType>::FindPath(T_NodeType* pStartNode, T_NodeType* pGoalNode)
	{
		std::vector<T_NodeType*> path{};
		std::vector<std::shared_ptr<NodeRecord>> openList{};
		std::vector<std::shared_ptr<NodeRecord>> closedList{};

		// Start node to add to open list
		NodeRecord tempRecord{};
		tempRecord.pNode = pStartNode;
		tempRecord.pParent = nullptr;
		tempRecord.fCost = GetHeuristicCost(pStartNode, pGoalNode);

		std::shared_ptr<NodeRecord> pCurrentRecord = std::make_shared<NodeRecord>(tempRecord);
		openList.push_back(pCurrentRecord);

		while (!openList.empty())
		{
			// pick from open list, the node with lowest f-score
			pCurrentRecord = openList[0];
			for (std::shared_ptr<NodeRecord> recordFromList : openList)
			{
				if (recordFromList->fCost < pCurrentRecord->fCost)
				{
					pCurrentRecord = recordFromList;
				}
			}

			auto currentRecordIt = std::find(openList.begin(), openList.end(), pCurrentRecord);
			openList.erase(currentRecordIt);
			closedList.push_back(pCurrentRecord);

			// if (next = destination)
			if (pCurrentRecord->pNode == pGoalNode)
			{
				break;
			}

			// identify successors (instead of picking adjacent nodes)
				// ->eliminates nodes that are not interesting to our path
			auto successors = IdentifySuccessors(pCurrentRecord, pStartNode, pGoalNode);

			for (std::shared_ptr<NodeRecord> successor : successors)
			{
				// check if successor in closed list -> continue;
				auto successorClosedList = std::find(closedList.begin(), closedList.end(), successor);
				if (successorClosedList != closedList.end())
				{
					continue;
				}

				// check if successor in open list -> 
				auto successorOpenList = std::find(openList.begin(), openList.end(), successor);
					// if not in open list
				if (successorOpenList == openList.end())
				{
					openList.push_back(successor);
					continue;
				}

				// if in open list -> compare g-cost 
				// -> if new successor g cost smaller -> previos-> parent == successor-> parent
				if ((*successorOpenList)->gCost > successor->gCost)
				{
					(*successorOpenList)->pParent = successor->pParent;
				}
			}
		}

		NodeRecord* pNodes = pCurrentRecord.get();
		while (pNodes != nullptr)
		{
			path.push_back(pNodes->pNode);
			pNodes = pNodes->pParent.get();
		}

		std::reverse(path.begin(), path.end());

		return path;
	}

	template <class T_NodeType, class T_ConnectionType>
	float Elite::JPS<T_NodeType, T_ConnectionType>::GetHeuristicCost(T_NodeType* pStartNode, T_NodeType* pEndNode) const
	{
		Vector2 toDestination = m_pGraph->GetNodePos(pEndNode) - m_pGraph->GetNodePos(pStartNode);
		return m_HeuristicFunction(abs(toDestination.x), abs(toDestination.y));
	}
}