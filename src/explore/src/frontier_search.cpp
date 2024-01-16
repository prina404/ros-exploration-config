#include <explore/frontier_search.h>

#include <mutex>

#include <costmap_2d/cost_values.h>
#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/Point.h>

#include <explore/costmap_tools.h>

namespace frontier_exploration
{
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;
using costmap_2d::FREE_SPACE;

FrontierSearch::FrontierSearch(costmap_2d::Costmap2D* costmap,
                               double potential_scale, double gain_scale,
                               double min_frontier_size)
  : costmap_(costmap)
  , potential_scale_(potential_scale)
  , gain_scale_(gain_scale)
  , min_frontier_size_(min_frontier_size)
{
}

std::vector<Frontier> FrontierSearch::searchFrom(geometry_msgs::Point position)
{
  std::vector<Frontier> frontier_list;

  // Sanity check that robot is inside costmap bounds before searching
  unsigned int mx, my;
  if (!costmap_->worldToMap(position.x, position.y, mx, my)) {
    ROS_ERROR("Robot out of costmap bounds, cannot search for frontiers");
    return frontier_list;
  }

  // make sure map is consistent and locked for duration of search
  std::lock_guard<costmap_2d::Costmap2D::mutex_t> lock(*(costmap_->getMutex()));

  map_ = costmap_->getCharMap();
  size_x_ = costmap_->getSizeInCellsX();
  size_y_ = costmap_->getSizeInCellsY();

  // initialize flag arrays to keep track of visited and frontier cells
  std::vector<bool> frontier_flag(size_x_ * size_y_, false);
  std::vector<bool> visited_flag(size_x_ * size_y_, false);

  // initialize breadth first search
  std::queue<unsigned int> bfs;

  // find closest clear cell to start search
  unsigned int clear, pos = costmap_->getIndex(mx, my);
  if (nearestCell(clear, pos, FREE_SPACE, *costmap_)) {
    bfs.push(clear);
  } else {
    bfs.push(pos);
    ROS_WARN("Could not find nearby clear cell to start search");
  }
  visited_flag[bfs.front()] = true;

  while (!bfs.empty()) {
    unsigned int idx = bfs.front();
    bfs.pop();

    // iterate over 4-connected neighbourhood
    for (unsigned nbr : nhood4(idx, *costmap_)) {
      // add to queue all free, unvisited cells, use descending search in case
      // initialized on non-free cell
      if (map_[nbr] <= map_[idx] && !visited_flag[nbr]) {
        visited_flag[nbr] = true;
        bfs.push(nbr);
        // check if cell is new frontier cell (unvisited, NO_INFORMATION, free
        // neighbour)
      } else if (isNewFrontierCell(nbr, frontier_flag)) {
        frontier_flag[nbr] = true;
        Frontier new_frontier = buildNewFrontier(nbr, pos, frontier_flag);
        if (new_frontier.size * costmap_->getResolution() >=
            min_frontier_size_) {
          frontier_list.push_back(new_frontier);
        }
      }
    }
  }

  // set costs of frontiers
  for (auto& frontier : frontier_list) {
    frontier.cost = frontierCost(frontier);
  }
  std::sort(
      frontier_list.begin(), frontier_list.end(),
      [](const Frontier& f1, const Frontier& f2) { return f1.cost < f2.cost; });
  
  std::vector<Frontier> frontier_list_backup {frontier_list};
  int cnt = 0;
  for (auto& frontier : frontier_list) {
    if (cnt++ < 15) {
      uint goalx, goaly;
      costmap_->worldToMap(frontier.centroid.x, frontier.centroid.y, goalx, goaly);
      int astarDist = astarCost(mx, my, goalx, goaly);
      frontier.min_distance = double(astarDist);
      if (astarDist < 0)
        frontier.min_distance = 10000;
    } 
    else 
      frontier.min_distance = 10000;
    
    frontier.cost = frontierCost(frontier);
  }
  std::sort(
      frontier_list.begin(), frontier_list.end(),
      [](const Frontier& f1, const Frontier& f2) { return f1.cost < f2.cost; });

  // if aStar couldn't find the closest frontier, fall back to euclidean distance
  if (frontier_list[0].min_distance == 10000) return frontier_list_backup;

  return frontier_list;
}

Frontier FrontierSearch::buildNewFrontier(unsigned int initial_cell,
                                          unsigned int reference,
                                          std::vector<bool>& frontier_flag)
{
  // initialize frontier structure
  Frontier output;
  output.centroid.x = 0;
  output.centroid.y = 0;
  output.size = 0;
  output.min_distance = std::numeric_limits<double>::infinity();

  // record initial contact point for frontier
  unsigned int ix, iy;
  costmap_->indexToCells(initial_cell, ix, iy);
  costmap_->mapToWorld(ix, iy, output.initial.x, output.initial.y);

  // push initial gridcell onto queue
  std::queue<unsigned int> bfs;
  bfs.push(initial_cell);

  // cache reference position in world coords
  unsigned int rx, ry;
  double reference_x, reference_y;
  costmap_->indexToCells(reference, rx, ry);
  costmap_->mapToWorld(rx, ry, reference_x, reference_y);

  while (!bfs.empty()) {
    unsigned int idx = bfs.front();
    bfs.pop();

    // try adding cells in 8-connected neighborhood to frontier
    for (unsigned int nbr : nhood8(idx, *costmap_)) {
      // check if neighbour is a potential frontier cell
      if (isNewFrontierCell(nbr, frontier_flag)) {
        // mark cell as frontier
        frontier_flag[nbr] = true;
        unsigned int mx, my;
        double wx, wy;
        costmap_->indexToCells(nbr, mx, my);
        costmap_->mapToWorld(mx, my, wx, wy);

        geometry_msgs::Point point;
        point.x = wx;
        point.y = wy;
        output.points.push_back(point);

        // update frontier size
        output.size++;

        // update centroid of frontier
        output.centroid.x += wx;
        output.centroid.y += wy;

        // determine frontier's distance from robot, going by closest gridcell
        // to robot
        double distance = sqrt(pow((double(reference_x) - double(wx)), 2.0) +
                               pow((double(reference_y) - double(wy)), 2.0));
        if (distance < output.min_distance) {
          output.min_distance = distance;
          output.middle.x = wx;
          output.middle.y = wy;
        }

        // add to queue for breadth first search
        bfs.push(nbr);
      }
    }
  }

  // average out frontier centroid
  output.centroid.x /= output.size;
  output.centroid.y /= output.size;
  return output;
}

bool FrontierSearch::isNewFrontierCell(unsigned int idx,
                                       const std::vector<bool>& frontier_flag)
{
  // check that cell is unknown and not already marked as frontier
  if (map_[idx] != NO_INFORMATION || frontier_flag[idx]) {
    return false;
  }

  // frontier cells should have at least one cell in 4-connected neighbourhood
  // that is free
  for (unsigned int nbr : nhood4(idx, *costmap_)) {
    if (map_[nbr] == FREE_SPACE) {
      return true;
    }
  }

  return false;
}

double FrontierSearch::frontierCost(const Frontier& frontier)
{
  return (potential_scale_ * frontier.min_distance *
          costmap_->getResolution()) -
         (gain_scale_ * frontier.size * costmap_->getResolution());
}
double euclideanDist(uint x1, uint y1, uint x2, uint y2) {
  return sqrt(pow((double(x1) - double(x2)), 2.0) + pow((double(y1) - double(y2)), 2.0));
}


// The following code is going to be really really bad, someday I'll learn C++, maybe.
int FrontierSearch::astarCost(uint startx, uint starty, uint goalx, uint goaly) {
  struct Node {
      uint x;
      uint y;
      uint gx;
      uint gy;
      int cost;
      bool operator<(const Node& rhs) const {
        double d1 = double(cost) + euclideanDist(x,y,gx,gy);
        double d2 = double(cost) + euclideanDist(rhs.x,rhs.y,gx,gy);
        return d1 < d2;
      }

      bool operator==(const Node& rhs) const {
        return x==rhs.x && y==rhs.y;
      }
  };

  std::set<Node> s; 
  std::set<Node> visited;
  Node start = {startx, starty, goalx, goaly, 0};
  s.insert(start);
  while (!s.empty()) {
    Node curr = *s.begin();
    visited.insert(curr);
    s.erase(s.begin());

    if (visited.size() > 20000) // limit number of visited nodes
      return 10000;

    if (curr.x == goalx && curr.y == goaly)
      return curr.cost;
    

    uint idx = costmap_->getIndex(curr.x, curr.y);
    for (uint nbr: nhood4(idx, *costmap_)) {
      if (map_[nbr] != LETHAL_OBSTACLE) {
        uint mx, my;
        costmap_->indexToCells(nbr, mx, my);
        Node nbrCell = {mx, my, goalx, goaly, curr.cost+1};
        if (visited.count(nbrCell) == 0)
          s.insert(nbrCell);
      }
    }
  }
  return -1;
}
}
