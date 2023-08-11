#include <cassert>
#include <algorithm>
#include <cmath>

#include "Macros.h"
#include "PathFinder.h"


using namespace AI;

#pragma region Static Defines

std::atomic<EPathBuilderType> PathFinder::s_pathBuilderType = EPathBuilderType::AStar;
std::atomic<EHeuristicType> PathFinder::s_heuristicType = EHeuristicType::Manhattan;

#pragma endregion


#pragma region Path Finding

//Set a different path builder when making a path (only AAStar exists for now) and you can also change heuristic to Manhattan & Euclidean
void PathFinder::SetOptions(const EPathBuilderType pathType, const EHeuristicType heuristicType)
{
	PathFinder::s_pathBuilderType = pathType;
	PathFinder::s_heuristicType = heuristicType;
}

//Breadth First and Dijkstras Algorithm is not included in this solution
bool PathFinder::FindPath(std::pair<int, int> Start, std::pair<int, int> Target, const std::vector<int>& Map, std::pair<int, int> MapDimensions, std::vector<int>& OutPath)
{
	bool foundPath = false;

	switch (s_pathBuilderType)
	{
	case EPathBuilderType::AStar:
		foundPath = UseAStar(Start, Target, Map, MapDimensions, OutPath, s_heuristicType);
		break;
	case EPathBuilderType::BreadthFirst:
		foundPath = false;
		break;
	case EPathBuilderType::DijkstraAlgorithm:
		foundPath = false;
		break;
	}

	if (!foundPath)
	{
		LOG(ELogType::Info, std::string("Could not reach destination."));
	}

	return foundPath;
}

bool PathFinder::UseAStar(const std::pair<int, int>& Start, const std::pair<int, int>& Target, const std::vector<int>& Map, const std::pair<int, int>& MapDimensions, std::vector<int>& OutPath, const EHeuristicType heuristicType)
{
	if (CheckValidPath(Start, Target, Map, MapDimensions))
	{
		if (Start == Target)
		{
			LOG(ELogType::Info, std::string("You are already on the target destination."));
			return true;
		}

		std::vector<Node> nodeMap = { static_cast<uint64_t>(MapDimensions.second) * static_cast<uint64_t>(MapDimensions.first), Node() };

		SetNodeMapValues(nodeMap, Map, MapDimensions);

		std::vector<Node*> neighbors;
		neighbors.reserve(4);

		Node& targetNode = nodeMap[GetNodeIndex(Target.first, Target.second, MapDimensions)];

		Node& startNode = nodeMap[GetNodeIndex(Start.first, Start.second, MapDimensions)];
		startNode.SetVisited();
		startNode.SetGCost(0);
		startNode.SetHCost(CalculateHeuristic(&startNode, &targetNode, heuristicType));
		startNode.CalculateTotalCost();

		std::set<Node*, Node::NodeComparator> nodeSet;
		nodeSet.insert(&startNode);

		while (!nodeSet.empty())
		{
			assert(*nodeSet.begin() != nullptr && "Top node in nodeSet was null, did we add invalid neighbors?");

			Node* current = *nodeSet.begin();
			nodeSet.erase(nodeSet.begin());

			current->SetInsideNodeSet(false);
			current->SetVisited();

			if (FoundTargetDestination(current, &targetNode))
			{
				BuildEndPath(current, startNode, OutPath);
				return true;
			}

			SearchAdjacentNeigbors(current, &targetNode, MapDimensions, nodeSet, nodeMap, neighbors, s_heuristicType);
		}
	}

	return false;
}

void PathFinder::SetNodeMapValues(std::vector<Node>& outNodeMap, const std::vector<int>& Map, const std::pair<int, int>& MapDimensions)
{
	int mapIndex = 0;

	for (int y = 0; y < MapDimensions.second; y++)
	{
		for (int x = 0; x < MapDimensions.first; x++)
		{
			Node& node = outNodeMap[GetNodeIndex(x, y, MapDimensions)];
			node.SetColumn(x);
			node.SetRow(y);
			node.SetRealMapIndex(mapIndex);
			node.SetPathType(Map[mapIndex]);
			node.SetAreaCost(CalculateAreaCost(&node));
			mapIndex++;
		}
	}
}

void PathFinder::SearchAdjacentNeigbors(Node* const currentNode, const Node* const targetNode, const std::pair<int, int>& MapDimensions, std::set<Node*, Node::NodeComparator>& nodeSet,
	std::vector<Node>& nodeMap, std::vector<Node*>& neighbors, const EHeuristicType heuristicType)
{
	if (TryGetNeighbors(currentNode, MapDimensions, nodeMap, neighbors))
	{
		const unsigned int newCost = currentNode->GetGCost() + 1u;

		for (unsigned int i = 0; i < neighbors.size(); i++)
		{
			if (!neighbors[i]->GetIsVisited() && !neighbors[i]->GetIsObstacle())
			{
				if (newCost < neighbors[i]->GetGCost() || !neighbors[i]->GetIsInsideNodeSet())
				{
					if (neighbors[i]->GetIsInsideNodeSet())
					{
						nodeSet.erase(neighbors[i]);
					}

					neighbors[i]->SetGCost(newCost);
					neighbors[i]->SetHCost(CalculateHeuristic(neighbors[i], targetNode, heuristicType));
					neighbors[i]->CalculateTotalCost();
					neighbors[i]->SetParent(currentNode);

					neighbors[i]->SetInsideNodeSet(true);
					nodeSet.insert(neighbors[i]);
				}
			}
		}
	}

	neighbors.clear();
}

void PathFinder::BuildEndPath(const Node* const currentNode, const Node& startNode, std::vector<int>& OutPath)
{
	const Node* curTempNode = currentNode;

	while (!(curTempNode->GetRow() == startNode.GetRow() && curTempNode->GetColumn() == startNode.GetColumn()))
	{
		OutPath.push_back(curTempNode->GetMapIndex());
		curTempNode = curTempNode->GetParent();
	}

	std::reverse(OutPath.begin(), OutPath.end());
}

#pragma region Valid Functions

bool PathFinder::TryGetNeighbors(const Node* const currentNode, const std::pair<int, int>& MapDimensions, std::vector<Node>& nodeMap, std::vector<Node*>& outNeighbors)
{
	const int currentRow = currentNode->GetRow();
	const int currentColumn = currentNode->GetColumn();

	const std::pair<int, int> right = { currentColumn + 1, currentRow };
	const std::pair<int, int> down = { currentColumn, currentRow + 1 };
	const std::pair<int, int> left = { currentColumn - 1, currentRow };
	const std::pair<int, int> up = { currentColumn, currentRow - 1 };

	if (IsValidCell(right.second, right.first, MapDimensions)) { outNeighbors.push_back(&nodeMap[GetNodeIndex(right.first, right.second, MapDimensions)]); }
	if (IsValidCell(down.second, down.first, MapDimensions)) { outNeighbors.push_back(&nodeMap[GetNodeIndex(down.first, down.second, MapDimensions)]); }
	if (IsValidCell(left.second, left.first, MapDimensions)) { outNeighbors.push_back(&nodeMap[GetNodeIndex(left.first, left.second, MapDimensions)]); }
	if (IsValidCell(up.second, up.first, MapDimensions)) { outNeighbors.push_back(&nodeMap[GetNodeIndex(up.first, up.second, MapDimensions)]); }

	const bool foundNeighbors = outNeighbors.size() > 0;

	return foundNeighbors;
}

bool PathFinder::CheckValidPath(const std::pair<int, int>& Start, const std::pair<int, int>& Target, const std::vector<int>& Map, const std::pair<int, int>& MapDimensions)
{
	const int mapDimensionColSize = MapDimensions.first;
	const int mapDimensionRowSize = MapDimensions.second;

	const int startColumn = Start.first;
	const int startRow = Start.second;

	const int targetColumn = Target.first;
	const int targetRow = Target.second;

	const int mapSize = static_cast<int>(Map.size());

	if (mapDimensionColSize * mapDimensionRowSize != mapSize)
	{
		LOG(ELogType::Warning, std::string("Map dimensions has to have the same size as the map."));
		return false;
	}

	if (mapSize > 0)
	{
		bool hasTraversableTerrain = false;

		for (const int mapIndex : Map)
		{
			if (mapIndex != 0 && mapIndex != 1)
			{
				LOG(ELogType::Warning, std::string("Map has values other than 0 & 1, remove them from the map."));
				return false;
			}

			if (!hasTraversableTerrain && mapIndex == 1) { hasTraversableTerrain = true; }
		}

		if (!hasTraversableTerrain)
		{
			LOG(ELogType::Warning, std::string("Map has no traversable terrain, set values of 1 to it."));
			return false;
		}
	}

	if (mapSize <= 0)
	{
		LOG(ELogType::Warning, std::string("Map has no values to set, add values to it."));
		return false;
	}

	if (startColumn < 0 || startRow < 0 || startColumn >= mapDimensionColSize || startRow >= mapDimensionRowSize)
	{
		LOG(ELogType::Warning, std::string("Invalid Start position."));
		return false;
	}

	if (targetColumn < 0 || targetRow < 0 || targetColumn >= mapDimensionColSize || targetRow >= mapDimensionRowSize)
	{
		LOG(ELogType::Warning, std::string("Invalid Target position."));
		return false;
	}

	if (Map[GetNodeIndex(targetColumn, targetRow, MapDimensions)] == static_cast<int>(EPathType::Obstacle))
	{
		LOG(ELogType::Warning, std::string("Target is an obstacle."));
		return false;
	}

	if (Map[GetNodeIndex(startColumn, startRow, MapDimensions)] == static_cast<int>(EPathType::Obstacle))
	{
		LOG(ELogType::Warning, std::string("Start is an obstacle."));
		return false;
	}

	return true;
}

bool PathFinder::FoundTargetDestination(const Node* const currentNode, const Node* const targetNode)
{
	return currentNode->GetRow() == targetNode->GetRow() && currentNode->GetColumn() == targetNode->GetColumn();
}

bool PathFinder::IsValidCell(const int row, const int col, const std::pair<int, int>& MapDimensions)
{
	return (row >= 0 && row < MapDimensions.second && col >= 0 && col < MapDimensions.first);
}

#pragma endregion

#pragma region Utility Functions

inline unsigned int AI::PathFinder::GetNodeIndex(const int column, const int row, const std::pair<int, int>& MapDimensions)
{
	return static_cast<unsigned int>(row * MapDimensions.first + column);
}

unsigned int AI::PathFinder::CalculateHeuristic(const Node* const fromNode, const Node* const targetNode, const EHeuristicType heuristicType)
{
	if (fromNode->GetIsHCalculated()) { return fromNode->GetHCost(); }

	unsigned int returnHeuristic = 0;

	switch (heuristicType)
	{
	case EHeuristicType::Manhattan:
		returnHeuristic = static_cast<unsigned int>(std::abs(fromNode->GetColumn() - targetNode->GetColumn()) + std::abs(fromNode->GetRow() - targetNode->GetRow()));
		break;
	case EHeuristicType::Euclidean:
		returnHeuristic = static_cast<unsigned int>(std::pow(std::abs(targetNode->GetColumn() - fromNode->GetColumn()), 2) + std::pow(std::abs(targetNode->GetRow() - fromNode->GetRow()), 2));
		break;
	default:
		returnHeuristic = std::numeric_limits<unsigned int>().max();
		break;
	}

	return returnHeuristic;
}

unsigned int AI::PathFinder::CalculateAreaCost(const Node* const currentNode)
{
	unsigned int returnAreaCost = 0u;

	switch (currentNode->GetPathType())
	{
	case EPathType::Path:
		returnAreaCost = static_cast<unsigned int>(EAreaCosts::Path);
		break;
	case EPathType::Water:
		returnAreaCost = static_cast<unsigned int>(EAreaCosts::Water);
		break;
	default:
		break;
	}

	return returnAreaCost;
}

#pragma endregion

#pragma endregion
