#include "knnheuristic.h"
#include "expansion.h"
#include "geometry.h"
#include "searchnode.h"
#include "successor.h"
#include "vertex.h"
#include "mesh.h"
#include "point.h"
#include "consts.h"
#include <queue>
#include <vector>
#include <cassert>
#include <iostream>
#include <algorithm>
#include <ctime>

namespace polyanya {

int KnnHeuristic::search() {
  init_search();
  timer.start();
  if (mesh == nullptr) {
    timer.stop();
    return 0;
  }

  while (!open_list.empty()) {
    SearchNodePtr node = open_list.top(); open_list.pop();

    nodes_popped++;
    if (node->reached) {
      deal_final_node(node);
      if ((int)final_nodes.size() == K) break;
      continue;
    }

    //if (reached.find(node->heuristic_gid) != reached.end()) {
    if (fabs(reached[node->heuristic_gid] - INF) > EPSILON) {
      // reset heuristic goal
      const Point& root = node->root == -1? start: mesh->mesh_vertices[node->root].p;
      std::pair<int, double> nexth = get_min_hueristic(root, node->left, node->right);
      node->heuristic_gid = nexth.first;
      node->f = nexth.second + node->g;
      nodes_reevaluate++;
      if (node->heuristic_gid == -1) {
        break;
      };
      open_list.push(node);
      nodes_pushed++;
      continue;
    }

    const int root = node->root;
    if (root != -1) {
      assert(root < (int) root_g_values.size());
      if (root_search_ids[root] == search_id) {
        // We've been here before!
        // Check whether we've done better.
        if (root_g_values[root] + EPSILON < node->g) {
          nodes_pruned_post_pop++;
          continue;
        }
      }
    }
    int num_nodes = 1;
    search_nodes_to_push[0] = *node;
    for (int i = 0; i < num_nodes; i++) {
      // update h value before we push
      const SearchNodePtr nxt = new (node_pool->allocate()) SearchNode(search_nodes_to_push[i]);
      const Point& nxt_root = (nxt->root == -1 ? start: mesh->mesh_vertices[nxt->root].p);
      assert(node->heuristic_gid != -1);
      double geth = get_h_value(nxt_root, goals[node->heuristic_gid], nxt->left, nxt->right);
      if (fabs(geth - (node->f - nxt->g)) <= EPSILON) { // heuristic not change
        nxt->heuristic_gid = node->heuristic_gid;
        nxt->f = node->f;
      }
      else {
        std::pair<int, double> nxth = {-1, INF};
        if (nxth.first == -1 || fabs(reached[nxth.first] - INF) > EPSILON) {
          nxth = get_min_hueristic(nxt_root, nxt->left, nxt->right, geth, node->heuristic_gid);
        }
        nxt->heuristic_gid = nxth.first;
        nxt->f = nxt->g + nxth.second;
      }
      nxt->parent = node;
      open_list.push(nxt);
      nodes_pushed++;
      nodes_generated++;

      // when nxt can be final_node
      int nxt_poly = nxt->next_polygon;
      if (!end_polygons[nxt_poly].empty()) {
        gen_final_nodes(nxt, nxt_root);
      }
    }
  }
  timer.stop();
  return (int)final_nodes.size();
}

void KnnHeuristic::deal_final_node(const SearchNodePtr node) {

  const Point& goal = goals[node->goal_id];
  const int final_root = [&]() {
      const Point& root = root_to_point(node->root);
      const Point root_goal = goal - root;
      // If root-left-goal is not CW, use left.
      if (root_goal * (node->left - root) < -EPSILON) {
          return node->left_vertex;
      }
      // If root-right-goal is not CCW, use right.
      if ((node->right - root) * root_goal < -EPSILON)
      {
          return node->right_vertex;
      }
      // Use the normal root.
      return node->root;
  }();

  //if (reached.find(node->goal_id) == reached.end()) {
  if (fabs(reached[node->goal_id]-INF) < EPSILON) {
    int end_polygon = node->next_polygon;
    const SearchNodePtr true_final =
      new (node_pool->allocate()) SearchNode
      {node, final_root, goal, goal, -1, -1, end_polygon, node->f, node->g};
    true_final->set_reached();
    true_final->set_goal_id(node->goal_id);
    reached[node->goal_id] = node->f;
    final_nodes.push_back(true_final);
    nodes_generated++;
  }
}

void KnnHeuristic::gen_final_nodes(const SearchNodePtr node, const Point& rootPoint) {
    for (int gid: end_polygons[node->next_polygon]) {
      const Point& goal = goals[gid];
      SearchNodePtr final_node = new (node_pool->allocate()) SearchNode(*node);
      final_node->set_reached();
      final_node->set_goal_id(gid);
      final_node->f = final_node->g + get_h_value(rootPoint, goal, node->left, node->right);
      open_list.push(final_node);
      nodes_generated++;
      nodes_pushed++;
    }
}
}
