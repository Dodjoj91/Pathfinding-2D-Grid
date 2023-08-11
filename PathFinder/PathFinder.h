#pragma once
#include <vector>
#include <set>
#include <atomic>

#include "Node.h"


namespace AI
{
	class PathFinder
	{
	public:
		static void SetOptions(const EPathBuilderType pathType, const EHeuristicType heuristicType);
		static bool FindPath(std::pair<int, int> Start, std::pair<int, int> Target, const std::vector<int>& Map, std::pair<int, int> MapDimensions, std::vector<int>& OutPath);

	private:
		static bool UseAStar(const std::pair<int, int>& Start, const std::pair<int, int>& Target, const std::vector<int>& Map, const std::pair<int, int>& MapDimensions, std::vector<int>& OutPath, const EHeuristicType heuristicType);

		static void SetNodeMapValues(std::vector<Node>& outNodeMap, const std::vector<int>& Map, const std::pair<int, int>& MapDimensions);
		static void SearchAdjacentNeigbors(Node* const currentNode, const Node* const targetNode, const std::pair<int, int>& MapDimensions, std::set<Node*, Node::NodeComparator>& nodeSet,
			std::vector<Node>& nodeMap, std::vector<Node*>& neighbors, const EHeuristicType heuristicType);
		static void BuildEndPath(const Node* const currentNode, const Node& startNode, std::vector<int>& OutPath);

		static bool TryGetNeighbors(const Node* const currentNode, const std::pair<int, int>& MapDimensions, std::vector<Node>& nodeMap, std::vector<Node*>& outNeighbors);
		static bool CheckValidPath(const std::pair<int, int>& Start, const std::pair<int, int>& Target, const std::vector<int>& Map, const std::pair<int, int>& MapDimensions);

		static inline bool FoundTargetDestination(const Node* const currentNode, const Node* const targetNode);
		static inline bool IsValidCell(const int row, const int col, const std::pair<int, int>& MapDimensions);

		static inline unsigned int GetNodeIndex(const int column, const int row, const std::pair<int, int>& MapDimensions);

		static unsigned int CalculateHeuristic(const Node* const fromNode, const Node* const targetNode, const EHeuristicType heuristicType);
		static unsigned int CalculateAreaCost(const Node* const currentNode);

	private:
		static std::atomic<EPathBuilderType> s_pathBuilderType;
		static std::atomic<EHeuristicType> s_heuristicType;

	};
}
