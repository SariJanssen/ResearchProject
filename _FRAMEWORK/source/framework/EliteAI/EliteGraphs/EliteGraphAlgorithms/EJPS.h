#pragma once
#include <assert.h>

namespace Elite
{
	template <class T_NodeType, class T_ConnectionType>
	class JPS
	{
	public:
		JPS(IGraph<T_NodeType, T_ConnectionType>* pGraph, Heuristic hFunction);

		struct NodeRecord
		{
			T_NodeType* pNode = nullptr;
			T_ConnectionType* pConnection = nullptr; // connection from previous node to current pNode
			float gCost = 0.f; // cost so far = accumulated g-costs of all the connections leading up to this one
			float fCost = 0.f; // estimatedTotalCost (= costSoFar + h-cost)

			bool operator==(const NodeRecord& other) const
			{
				return pNode == other.pNode
					&& pConnection == other.pConnection
					&& AreEqual(gCost, other.gCost)
					&& AreEqual(fCost, other.fCost);
			};

			bool operator<(const NodeRecord& other) const
			{
				return fCost < other.fCost;
			};
		};

		std::vector<T_NodeType*> FindPath(T_NodeType* pStartNode, T_NodeType* pDestinationNode);

	private:
		float GetHeuristicCost(T_NodeType* pStartNode, T_NodeType* pEndNode) const;
		std::vector<NodeRecord> IdentifySuccessors(const NodeRecord& pCurrentNode, T_NodeType* pStartNode, T_NodeType* pGoalNode);
		NodeRecord Jump(const NodeRecord& pCurrentNode, const Elite::Vector2& difference, T_NodeType* pStartNode, T_NodeType* pGoalNode);

		IGraph<T_NodeType, T_ConnectionType>* m_pGraph;
		Heuristic m_HeuristicFunction;
	};

	template <class T_NodeType, class T_ConnectionType>
	JPS<T_NodeType, T_ConnectionType>::JPS(IGraph<T_NodeType, T_ConnectionType>* pGraph, Heuristic hFunction)
		: m_pGraph(pGraph)
		, m_HeuristicFunction(hFunction)
	{
	}

	template <class T_NodeType, class T_ConnectionType>
	std::vector<typename JPS<T_NodeType, T_ConnectionType>::NodeRecord> JPS<T_NodeType, T_ConnectionType>::IdentifySuccessors(const typename JPS<T_NodeType, T_ConnectionType>::NodeRecord& pCurrentNode, T_NodeType* pStartNode, T_NodeType* pGoalNode)
	{
		// vector IDENTIFY SUCCESSORS(current, start, end) 
		std::vector<NodeRecord> successors{};

		// get neighbors of current
		auto& connectionList = m_pGraph->GetNodeConnections(pCurrentNode.pNode->GetIndex());
		// for each neighbor
		for (const auto& pConnection : connectionList)
		{
			// int x = clamp(neighbor.x - current.x, -1, 1)
			// int y = clamp(neighbor.y - current.y, -1, 1)
			int nodeIdx = pConnection->GetTo();
			T_NodeType* pNeighborNode = m_pGraph->GetNode(nodeIdx);
			Elite::Vector2 neighborNodePos = m_pGraph->GetNodeWorldPos(pNeighborNode);
			Elite::Vector2 currentNodePos = m_pGraph->GetNodeWorldPos(pCurrentNode.pNode);
			float x = clamp(neighborNodePos.x - currentNodePos.x, -1.f, 1.f);
			float y = clamp(neighborNodePos.y - currentNodePos.y, -1.f, 1.f);

			// jumpPoint = noderecord jump(current.x, current.y, x, y, start, end)
			NodeRecord jumpPoint = Jump(pCurrentNode, Elite::Vector2{ x, y }, pStartNode, pGoalNode);

			// if (jumpPoint) successorvec.pusback(jumpoint)
			if (jumpPoint.pNode != nullptr)
				successors.push_back(jumpPoint);
		}
		// return successorvec
		return successors;
	}

	template <class T_NodeType, class T_ConnectionType>
	typename JPS<T_NodeType, T_ConnectionType>::NodeRecord JPS<T_NodeType, T_ConnectionType>::Jump(const typename JPS<T_NodeType, T_ConnectionType>::NodeRecord& pCurrentNode, const Elite::Vector2& difference, T_NodeType* pStartNode, T_NodeType* pGoalNode)
	{
		// noderecord JUMP(current, Vec2{x,y}, start, end)
		Elite::Vector2 currentNodePos = m_pGraph->GetNodeWorldPos(pCurrentNode.pNode);
		// next = current + vec2
		T_NodeType* pNextNode = m_pGraph->GetNodeAtWorldPos({ currentNodePos.x + difference.x, currentNodePos.y + difference.y });

		NodeRecord nextJumpNR{};
		nextJumpNR.pNode = pNextNode;
		nextJumpNR.pConnection = m_pGraph->GetConnection(pCurrentNode.pNode->GetIndex(), pNextNode->GetIndex());
		nextJumpNR.gCost = nextJumpNR.pConnection->GetCost() + pCurrentNode.gCost;
		nextJumpNR.fCost = nextJumpNR.gCost + GetHeuristicCost(pCurrentNode.pNode, pGoalNode);

		// if(next is terrain) return null
		if (pNextNode->GetTerrainType() == TerrainType::Water)
			return NodeRecord{};
		
		// if(next == end) return next
		if (pNextNode == pGoalNode)
			return nextJumpNR;

		// diagonal case
		// if(x != 0 && y != 0)
		if (!Elite::AreEqual(difference.x, 0.f) && !Elite::AreEqual(difference.y, 0.f)) // Diagonal Case
		{
			// if(current.x + x == obstacle || current.y + y == obstacle) return next
			if (m_pGraph->GetNodeAtWorldPos({ currentNodePos.x + difference.x, currentNodePos.y })->GetTerrainType() == TerrainType::Water
			 || m_pGraph->GetNodeAtWorldPos({ currentNodePos.x, currentNodePos.y + difference.y })->GetTerrainType() == TerrainType::Water)
				return nextJumpNR;

			// Check horizontal and vertical directions for forced neighbors
			// if (jump(next.x, next.y, x, 0, start, end) == null) return next
			if (Jump(nextJumpNR, Elite::Vector2{difference.x, 0.f}, pStartNode, pGoalNode).pNode == nullptr)
				return nextJumpNR;

			// if (jump(next.x, next.y, 0, y, start, end) == null) return next
			if (Jump(nextJumpNR, Elite::Vector2(0.f, difference.y), pStartNode, pGoalNode).pNode == nullptr)
				return nextJumpNR;

		}

		// horizontal case
		// else if( x != 0)
		else if (!Elite::AreEqual(difference.x, 0.f)) // Horizontal Case
		{
			// if (current.y + 1 == obstacle) && if (current.x + x, current.y + 1 != obstacle)
				// return next
			if (m_pGraph->GetNodeAtWorldPos({ currentNodePos.x, currentNodePos.y + 1.f })->GetTerrainType() == TerrainType::Water
			 && m_pGraph->GetNodeAtWorldPos({ currentNodePos.x + difference.x, currentNodePos.y + 1.f })->GetTerrainType() != TerrainType::Water)
				return nextJumpNR;
			// else if (current.y - 1 == obstacle) && if (current.x + x, current.y - 1 != obstacle)
				// return next
			else if (m_pGraph->GetNodeAtWorldPos({ currentNodePos.x, currentNodePos.y - 1.f })->GetTerrainType() == TerrainType::Water
				  && m_pGraph->GetNodeAtWorldPos({ currentNodePos.x + difference.x, currentNodePos.y + 1.f })->GetTerrainType() != TerrainType::Water)
				return nextJumpNR;
		}

		// vertical case
		else  //Vertical Case
		{
			// if (current.x + 1 == obstacle) && if (current.x + 1, current.y + y != obstacle)
				// return next
			if (m_pGraph->GetNodeAtWorldPos({ currentNodePos.x + 1.f, currentNodePos.y })->GetTerrainType() == TerrainType::Water
				&& m_pGraph->GetNodeAtWorldPos({ currentNodePos.x + 1.f, currentNodePos.y + difference.y })->GetTerrainType() != TerrainType::Water)
				return nextJumpNR;

			// else if (current.x - 1 == obstacle) && if (current.x - 1, current.y + y != obstacle)
				// return next
			if (m_pGraph->GetNodeAtWorldPos({ currentNodePos.x - 1.f, currentNodePos.y })->GetTerrainType() == TerrainType::Water
				&& m_pGraph->GetNodeAtWorldPos({ currentNodePos.x - 1.f, currentNodePos.y + difference.y })->GetTerrainType() != TerrainType::Water)
				return nextJumpNR;
		}
		
		// return jump(next.x, next.y. x, y. start, end)
		return Jump(nextJumpNR, difference, pStartNode, pGoalNode);
	}

	template <class T_NodeType, class T_ConnectionType>
	std::vector<T_NodeType*> JPS<T_NodeType, T_ConnectionType>::FindPath(T_NodeType* pStartNode, T_NodeType* pGoalNode)
	{
		std::vector<T_NodeType*> path{};
		std::vector<NodeRecord> openList{};
		std::vector<NodeRecord> closedList{};

		// Start node to add to open list
		NodeRecord currentRecord{};
		currentRecord.pNode = pStartNode;
		currentRecord.pConnection = nullptr;
		currentRecord.fCost = GetHeuristicCost(pStartNode, pGoalNode);
		openList.push_back(currentRecord);

		while (!openList.empty())
		{
			// pick from open list, the node with lowest f-score
			currentRecord = openList[0];
			for (const NodeRecord& recordFromList : openList)
			{
				if (recordFromList.fCost < currentRecord.fCost)
				{
					currentRecord = recordFromList;
				}
			}

			auto currentRecordIt = std::find(openList.begin(), openList.end(), currentRecord);
			openList.erase(currentRecordIt);
			closedList.push_back(currentRecord);

			// if (next = destination)
			if (currentRecord.pNode == pGoalNode)
			{
				break;
			}

			// identify successors (instead of picking adjacent nodes)
				// ->eliminates nodes that are not interesting to our path
			auto successors = IdentifySuccessors(currentRecord, pStartNode, pGoalNode);

			for (auto successor : successors)
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
				// -> if new successor g cost + 1 smaller -> previos-> parent == successor-> parent
				if (successorOpenList->gCost > successor.gCost) //TODO: current record.gcost 
				{
					successorOpenList->pConnection = successor.pConnection;
				}
			}
		}

		//current = next
			// while (current)
				// path.pushback(current)
				// current = current.connection.getfrom
			// reverse path
		path.push_back(pGoalNode);
		while (currentRecord.pNode != pStartNode)
		{
			for (size_t i{}; i < closedList.size(); ++i)
			{
				int prevNodeIdx = currentRecord.pConnection->GetFrom();
				if (closedList[i].pNode == m_pGraph->GetNode(prevNodeIdx))
				{
					path.push_back(closedList[i].pNode);
					currentRecord = closedList[i];
					break;
				}
			}
		}

		std::reverse(path.begin(), path.end());
		return path;
	}
	
	template <class T_NodeType, class T_ConnectionType>
	void PreviousPath(T_NodeType* pStartNode, T_NodeType* pGoalNode)
	{
		// pick from open list, the node with lowest f-score
			// if (next = destination)
				//current = next
				// while (current)
					// path.pushback(current)
					// current = current.connection.start
				// reverse path
				//end

		// identify successors (instead of picking adjacent nodes)
			// ->eliminates nodes that are not interesting to our path
		
			// vector IDENTIFY SUCCESSORS(current, start, end) 
			// get neighbors of current
			// for each neighbor
				// int x = clamp(neighbor.x - current.x, -1, 1)
				// int y = clamp(neighbor.y - current.y, -1, 1)

				// jumpPoint = noderecord jump(current.x, current.y, x, y, start, end)
				// if (jumpPoint) successorvec.pusback(jumpoint)

			// return successorvec
		
		// noderecord JUMP(current, Vec2{x,y}, start, end)
			// next = current + vec2
			// if(next is terrain) return null
			// if(next == end) return next

			// diagonal case
			// if(x != 0 && y != 0)
				// if(current.x + x == obstacle || current.y + y == obstacle) return next
				// if (jump(next.x, next.y, x, 0, start, end) == null) return next
				// if (jump(next.x, next.y, 0, y, start, end) == null) return next
			
			// horizontal case
			// else if( x != 0)
				// if (current.y + 1 == obstacle) && if (current.x + x, current.y + 1 != obstacle)
					// return next
				// else if (current.y - 1 == obstacle) && if (current.x + x, current.y - 1 != obstacle)
					// return next

			// vertical case
			// else
				// if (current.x + 1 == obstacle) && if (current.x + 1, current.y + y != obstacle)
					// return next
				// else if (current.x - 1 == obstacle) && if (current.x - 1, current.y + y != obstacle)
					// return next

			// return jump(next.x, next.y. x, y. start, end)

		// successors obtained
			// check if successor in closed list -> continue;
			// check if successor in open list -> 
				// if not in open list -> calculate f cost and add
				// if in open list -> compare g-cost 
				// -> if new successor g cost + 1 smaller -> previos-> parent == successor-> parent
				// calculate fcost
			

		std::vector<T_NodeType*> path{};
		std::vector<NodeRecord> openList{};
		std::vector<NodeRecord> closedList{};

		// 1. Create startRecord and add to open List to start while loop
		NodeRecord currentRecord{};
		currentRecord.pNode = pStartNode;
		currentRecord.pConnection = nullptr;
		currentRecord.fCost = GetHeuristicCost(pStartNode, pGoalNode);

		openList.push_back(currentRecord);

		// 2. Continue searching for a connection that leads to the end node
		while (!openList.empty())
		{
			// 2.a Get the connection with lowest F score
			currentRecord = openList[0];
			for (const NodeRecord& recordFromList : openList)
			{
				if (recordFromList.fCost < currentRecord.fCost)
				{
					currentRecord = recordFromList;
				}
			}

			// 2.b Check if that connection leads to the end node
			if (currentRecord.pNode == pGoalNode)
			{
				break;
			}

			// 2.c Get all the connections of the NodeRecord's node
			auto connectionList = m_pGraph->GetNodeConnections(currentRecord.pNode->GetIndex());
			for (const auto& pConnection : connectionList)
			{
				// Calculate the total cost so far
				float gCost = pConnection->GetCost() + currentRecord.gCost;

				// 2.d Check if any of them lead to a node already on the closed list
				int getToIdx = pConnection->GetTo();
				auto pGetToNode = m_pGraph->GetNode(getToIdx);

				bool isInClosed{ false };
				for (size_t i{}; i < closedList.size(); ++i)
				{
					if (closedList[i].pNode == pGetToNode)
					{
						isInClosed = true;
						if (closedList[i].gCost > gCost)
						{
							const vector<NodeRecord>::iterator& it = closedList.begin() + i;
							closedList.erase(it);
							break;
						}
					}
				}

				if (isInClosed == true)
				{
					continue;
				}

				// 2.e If 2.d check failed, check if any of those connections lead to a node already on the open list
				bool isInOpen{ false };
				for (size_t i{}; i < openList.size(); ++i)
				{
					if (openList[i].pNode == pGetToNode)
					{
						isInOpen = true;
						if (openList[i].gCost > gCost)
						{
							vector<NodeRecord>::iterator it = openList.begin() + i;
							openList.erase(it);
						}
						break;
					}
				}

				if (isInOpen == true)
				{
					continue;
				}

				// 2.f At this point any expensive connection should be removed. We create a new nodeRecord and add it to the openList
				NodeRecord newRecord{};
				newRecord.pNode = pGetToNode;
				newRecord.pConnection = pConnection;
				newRecord.gCost = gCost;
				newRecord.fCost = gCost + GetHeuristicCost(pGetToNode, pGoalNode);
				openList.push_back(newRecord);
			}

			// 2.g Remove NodeRecord from the openList and add it to the closedList
			auto it = std::find(openList.begin(), openList.end(), currentRecord);
			assert(it != openList.end());
			openList.erase(it);
			closedList.push_back(currentRecord);
		}

		// 3. Reconstruct path from last connection to start node
		path.push_back(pGoalNode);

		while (currentRecord.pNode != pStartNode)
		{
			for (size_t i{}; i < closedList.size(); ++i)
			{
				int prevNodeIdx = currentRecord.pConnection->GetFrom();
				if (closedList[i].pNode == m_pGraph->GetNode(prevNodeIdx))
				{
					path.push_back(closedList[i].pNode);
					currentRecord = closedList[i];
					break;
				}
			}
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