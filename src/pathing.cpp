#include <arpa/inet.h>
#include <iostream>
#include <netinet/in.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>

#include <climits>
#include <cmath>
#include <iostream>
#include <ostream>
#include <utility>
#include <vector>
#include "pathing.h"
#include "stlastar.h"

// Adapted example from https://github.com/justinhj/astar-algorithm-cpp

std::vector<std::vector<unsigned char>>
    occupancy_grid(MAP_WIDTH, std::vector<unsigned char>(MAP_HEIGHT, 0));

int GetMap(map_size x, map_size y) {
  if (x < 0 || x >= MAP_WIDTH || y < 0 || y >= MAP_HEIGHT) {
    return 9;
  }
  return occupancy_grid[x][y];
}

bool MapSearchNode::IsSameState(MapSearchNode &rhs) {
  // same state in a maze search is simply when (x,y) are the same
  if ((pos.first == rhs.pos.first) && (pos.second == rhs.pos.second)) {
    return true;
  } else {
    return false;
  }
}

size_t MapSearchNode::Hash() {
  size_t h1 = std::hash<float>{}(pos.first);
  size_t h2 = std::hash<float>{}(pos.second);
  return h1 ^ (h2 << 1);
}

void MapSearchNode::PrintNodeInfo() {
  const int strSize = 100;
  char str[strSize];
  snprintf(str, strSize, "Node position : (%d,%d)\n", (int)pos.first,
           (int)pos.second);

  std::cout << str;
}

// Here's the heuristic function that estimates the distance from a Node
// to the Goal.

float MapSearchNode::GoalDistanceEstimate(MapSearchNode &nodeGoal) {
  return std::abs(pos.first - nodeGoal.pos.first) +
         std::abs(pos.second - nodeGoal.pos.second);
}

bool MapSearchNode::IsGoal(MapSearchNode &nodeGoal) {

  if ((pos.first == nodeGoal.pos.first) &&
      (pos.second == nodeGoal.pos.second)) {
    return true;
  }

  return false;
}

// This generates the successors to the given Node. It uses a helper function
// called AddSuccessor to give the successors to the AStar class. The A*
// specific initialisation is done for each node internally, so here you just
// set the state information that is specific to the application
bool MapSearchNode::GetSuccessors(AStarSearch<MapSearchNode> *astarsearch,
                                  MapSearchNode *parent_node) {

  map_size parent_x = -1;
  map_size parent_y = -1;

  if (parent_node) {
    parent_x = parent_node->pos.first;
    parent_y = parent_node->pos.second;
  }

  MapSearchNode NewNode;

  // push each possible move except allowing the search to go backwards
  float search = GetMap(pos.first - 1, pos.second);
  float current = GetMap(pos.first, pos.second);
  if ((search < 9) &&
      !((parent_x == pos.first - 1) && (parent_y == pos.second))) {
    NewNode = MapSearchNode(pos.first - 1, pos.second);
    astarsearch->AddSuccessor(NewNode);
  }

  // search = GetMap(pos.first, pos.second - 1);
  // if ((search < 9) && (current == 0 && search != 0) &&
  //     !((parent_x == pos.first) && (parent_y == pos.second - 1))) {
  //   NewNode = MapSearchNode(pos.first, pos.second - 1);
  //   astarsearch->AddSuccessor(NewNode);
  // }

  search = GetMap(pos.first + 1, pos.second);
  if ((search < 9) &&
      !((parent_x == pos.first + 1) && (parent_y == pos.second))) {
    NewNode = MapSearchNode(pos.first + 1, pos.second);
    astarsearch->AddSuccessor(NewNode);
  }

  search = GetMap(pos.first, pos.second + 1);
  if ((search < 9) &&
      !((parent_x == pos.first) && (parent_y == pos.second + 1))) {
    NewNode = MapSearchNode(pos.first, pos.second + 1);
    astarsearch->AddSuccessor(NewNode);
  }

  search = GetMap(pos.first + 1, pos.second + 1);
  if ((search < 9) &&
      !((parent_x == pos.first) && (parent_y == pos.second + 1))) {
    NewNode = MapSearchNode(pos.first, pos.second + 1);
    astarsearch->AddSuccessor(NewNode);
  }

  search = GetMap(pos.first - 1, pos.second + 1);
  if ((search < 9) &&
      !((parent_x == pos.first) && (parent_y == pos.second + 1))) {
    NewNode = MapSearchNode(pos.first, pos.second + 1);
    astarsearch->AddSuccessor(NewNode);
  }

  // search = GetMap(pos.first - 1, pos.second - 1);
  // if ((search < 9) && !((parent_x == pos.first) && (parent_y == pos.second +
  // 1))) {
  //   NewNode = MapSearchNode(pos.first, pos.second + 1);
  //   astarsearch->AddSuccessor(NewNode);
  // }

  // search = GetMap(pos.first + 1, pos.second - 1);
  // if ((search < 9) && !((parent_x == pos.first) && (parent_y == pos.second +
  // 1))) {
  //   NewNode = MapSearchNode(pos.first, pos.second + 1);
  //   astarsearch->AddSuccessor(NewNode);
  // }

  return true;
}

// given this node, what does it cost to move to successor. In the case
// of our map the answer is the map terrain value at this node since that is
// conceptually where we're moving

float MapSearchNode::GetCost(MapSearchNode &successor) {
  return (float)GetMap(pos.first, pos.second);
}

std::vector<std::pair<map_size, map_size>>
astar(std::pair<map_size, map_size> start, std::pair<map_size, map_size> end) {
  std::vector<std::pair<map_size, map_size>> solution = {};

  // Our sample problem defines the world as a 2d array representing a terrain
  // Each element contains an integer from 0 to 5 which indicates the cost
  // of travel across the terrain. Zero means the least possible difficulty
  // in travelling (think ice rink if you can skate) whilst 5 represents the
  // most difficult. 9 indicates that we cannot pass.

  // Create an instance of the search class...

  AStarSearch<MapSearchNode> astarsearch;

  unsigned int SearchCount = 0;

  const unsigned int NumSearches = 1;

  while (SearchCount < NumSearches) {

    // Create a start state
    MapSearchNode nodeStart;
    nodeStart.pos = start;

    // Define the goal state
    MapSearchNode nodeEnd;
    nodeEnd.pos = end;
    // Set Start and goal states

    astarsearch.SetStartAndGoalStates(nodeStart, nodeEnd);

    unsigned int SearchState;
    unsigned int SearchSteps = 0;

    do {
      SearchState = astarsearch.SearchStep();

      SearchSteps++;

#if DEBUG_LISTS

      cout << "Steps:" << SearchSteps << "\n";

      int len = 0;

      cout << "Open:\n";
      MapSearchNode *p = astarsearch.GetOpenListStart();
      while (p) {
        len++;
#if !DEBUG_LIST_LENGTHS_ONLY
        ((MapSearchNode *)p)->PrintNodeInfo();
#endif
        p = astarsearch.GetOpenListNext();
      }

      cout << "Open list has " << len << " nodes\n";

      len = 0;

      cout << "Closed:\n";
      p = astarsearch.GetClosedListStart();
      while (p) {
        len++;
#if !DEBUG_LIST_LENGTHS_ONLY
        p->PrintNodeInfo();
#endif
        p = astarsearch.GetClosedListNext();
      }

      cout << "Closed list has " << len << " nodes\n";
#endif

    } while (SearchState == AStarSearch<MapSearchNode>::SEARCH_STATE_SEARCHING);

    if (SearchState == AStarSearch<MapSearchNode>::SEARCH_STATE_SUCCEEDED) {
      // std::cout << "Search found goal state: " << SearchState << std::endl;

      MapSearchNode *node = astarsearch.GetSolutionStart();

#if DISPLAY_SOLUTION
      cout << "Displaying solution\n";
#endif
      int steps = 0;

      // node->PrintNodeInfo();
      solution.push_back(std::move(node->pos));
      for (;;) {
        node = astarsearch.GetSolutionNext();

        if (!node) {
          break;
        }
        solution.push_back(std::move(node->pos));
        // node->PrintNodeInfo();
        steps++;
      };

      // std::cout << "Solution steps " << steps << std::endl;

      // Once you're done with the solution you can free the nodes up
      astarsearch.FreeSolutionNodes();

    } else if (SearchState == AStarSearch<MapSearchNode>::SEARCH_STATE_FAILED) {
      std::cout << "Search terminated. Did not find goal state: " << SearchState
                << std::endl;
    }

    // Display the number of loops the search went through
    // std::cout << "SearchSteps : " << SearchSteps << "\n";

    SearchCount++;

    astarsearch.EnsureMemoryFreed();
  }

  return solution;
}