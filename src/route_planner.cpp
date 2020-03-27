#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // Converts coordinates to actual nodes in the map
    start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node = &m_Model.FindClosestNode(end_x, end_y);
}


float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*end_node);
}


void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors();
    for (RouteModel::Node *current_neighbor: current_node->neighbors) {
        current_neighbor->parent = current_node;
        current_neighbor->g_value = current_node->g_value + current_neighbor->distance(*current_node);
        current_neighbor->h_value = CalculateHValue(current_neighbor);
        open_list.push_back(current_neighbor);
        current_neighbor->visited = true;
    }
}


RouteModel::Node *RoutePlanner::NextNode() {
    std::sort(open_list.begin(), open_list.end(), [](auto first, auto second){
        float fValue1 = first->g_value + first->h_value;
        float fValue2 = second->g_value + second->h_value;
        return fValue1 > fValue2;
    });

    auto nextNode = open_list.back();

    open_list.pop_back();

    return nextNode;
}


std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    while (current_node->parent != nullptr){
        path_found.push_back(*current_node);
        distance += current_node->distance(*current_node->parent);
        current_node = current_node->parent;
    }

    path_found.push_back(*current_node);

    std::reverse(path_found.begin(), path_found.end());

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;
}


void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = start_node;
    start_node->visited = true;

    open_list.push_back(current_node);

    while(!open_list.empty()) {
        current_node = NextNode();

        if (current_node == end_node) {
            m_Model.path = ConstructFinalPath(end_node);
            return;
        }

        AddNeighbors(current_node);
    }
}