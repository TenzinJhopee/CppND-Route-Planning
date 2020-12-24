#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node = &m_Model.FindClosestNode(end_x, end_y);

    // TODO 2: Use the m_Model.FindClosestNode method to find the closest nodes to the starting and ending coordinates.
    // Store the nodes you find in the RoutePlanner's start_node and end_node attributes.

}


std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node){
//std::cout<<"function START ::"<<__FUNCTION__<<std::endl;
distance = 0.0f;
std::vector<RouteModel::Node> path_found;
RouteModel::Node parent;

while (current_node -> parent !=nullptr){

    path_found.push_back(*current_node);
    parent = *(current_node->parent);
    distance += current_node -> distance(parent);
    current_node = current_node->parent;
}
path_found.push_back(*current_node);
distance *=m_Model.MetricScale();
//std::cout<<"function STOP ::"<<__FUNCTION__<<std::endl;
return path_found;

};

/*
Perform A* alghorithm
*/

void RoutePlanner::AStarSearch(){
//std::cout<<"function START ::"<<__FUNCTION__<<std::endl;
start_node ->visited =true;
open_list.push_back(start_node);
RouteModel::Node *current_node = nullptr;

while(open_list.size()>0) {

    current_node = NextNode();

    if (current_node ->distance(*end_node) == 0){

        
        m_Model.path = ConstructFinalPath(current_node);
        return;
    }
    AddNeighbors(current_node); 
}
//std::cout<<"function STOP ::"<<__FUNCTION__<<std::endl;
};


// TODO 3: Implement the CalculateHValue method.
// Tips:
// - You can use the distance to the end_node for the h value.
// - Node objects have a distance method to determine the distance to another node.

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*end_node);
}


RouteModel::Node* RoutePlanner::NextNode(){
//std::cout<<"function START ::"<<__FUNCTION__<<std::endl;
    std::sort(open_list.begin(), open_list.end(), [](const auto &_1st, const auto &_2nd){
        return _1st->h_value + _1st->g_value < _2nd->h_value +_2nd->g_value;
    });

    RouteModel::Node *lowest_node = open_list.front();
    open_list.erase(open_list.begin());
   // std::cout<<"function STOP ::"<<__FUNCTION__<<std::endl;
    return lowest_node;
};

/void RoutePlanner::AddNeighbors(RouteModel::Node * current_node){
//std::cout<<"function START ::"<<__FUNCTION__<<std::endl;
current_node->FindNeighbors();

for(auto neighbor : current_node->neighbors){
    neighbor->parent = current_node;
    neighbor ->g_value = current_node->g_value + current_node->distance(*neighbor);
    neighbor-> h_value = CalculateHValue(neighbor);
    open_list.push_back(neighbor);
    neighbor->visited = true;

}

};

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

    // TODO: Implement your solution here.

}