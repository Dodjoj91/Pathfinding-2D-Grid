#pragma once

#pragma region Path Finding Enums

enum class EPathType : uint8_t
{
	Obstacle,
	Path,
	Water,
	Count
};

enum class EPathBuilderType : uint8_t
{
	AStar,
	BreadthFirst,
	DijkstraAlgorithm
};

enum class EHeuristicType : uint8_t
{
	Euclidean,
	Manhattan
};

//Using this to set area costs for different types, setting Path as a 0 cost for now (Water is not used since it is not included in the map)
enum class EAreaCosts : uint8_t
{
	Path = 0,
	Water = 10,
};

#pragma endregion

#pragma region General Enums

enum class ELogType : uint8_t
{
	Info,
	Warning,
	Error
};

#pragma endregion
