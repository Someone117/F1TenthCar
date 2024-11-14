#include <arpa/inet.h>
#include <cstddef>
#include <netinet/in.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <vector>

#include "stlastar.h"
#include <climits>
#include <cmath>
#include <utility>
#include <vector>

// adapted from https://github.com/justinhj/astar-algorithm-cpp

typedef uint16_t map_size;

const map_size MAP_WIDTH = 200;
const map_size MAP_HEIGHT = 200;
const map_size MULTIPLIER = 60;

class MapSearchNode {
public:
  std::pair<map_size, map_size> pos;

  MapSearchNode() { pos = {0, 0}; }
  MapSearchNode(map_size px, map_size py) {
    pos.first = px;
    pos.second = py;
  }

  float GoalDistanceEstimate(MapSearchNode &nodeGoal);
  bool IsGoal(MapSearchNode &nodeGoal);
  bool GetSuccessors(AStarSearch<MapSearchNode> *astarsearch,
                     MapSearchNode *parent_node);
  float GetCost(MapSearchNode &successor);
  bool IsSameState(MapSearchNode &rhs);
  size_t Hash();

  void PrintNodeInfo();
};

extern std::vector<std::vector<unsigned char>>
    occupancy_grid;

int GetMap(map_size x, map_size y);

std::vector<std::pair<map_size, map_size>>
astar(std::pair<map_size, map_size> start, std::pair<map_size, map_size> end);