#include "stud_lib_kmint/a_star/a_star.hpp"

#include <kmint/random.hpp>

namespace stud_lib_kmint {

node_list a_star::find_path(map_node& start, const map_node& end,
                            map_graph& graph, const heuristic& heuristic) {
    graph.untag_all();

    map<int, pair<int, int>> weights;  // nodeID, weight, parentID
    for (auto& node : graph) {
        int w = node.node_id() == start.node_id()
                    ? 0
                    : std::numeric_limits<int>::max();

        weights[node.node_id()] = {w, -1};  // std::make_pair<int, int>(w, -1);
    }

    // final path
    node_list path;

    // closed list
    vector<int> closed_list;

    // open list
    node_priority_queue open_list(
        [&weights](const map_node& lhs, const map_node& rhs) {
            return weights[lhs.node_id()].first > weights[rhs.node_id()].first;
        });
    open_list.push(start);
    // weights[start.node_id()].first = 0;

    // loop over open list
    while (!open_list.empty()) {
        map_node& shortest = open_list.top();
        open_list.pop();
        closed_list.push_back(shortest.node_id());

        for (auto& edge : shortest) {
            // if not closed
            if (std::find(closed_list.begin(), closed_list.end(),
                          edge.to().node_id()) == closed_list.end()) {
                // tag as visited
                edge.to().tag(node_tag::visited);

                // aka G
                int weight_up_to_this_node =
                    weights.at(shortest.node_id()).first +
                    (edge.weight() == 1 ? 1 : 4);  // complete avoidance test
                // aka H
                int estimate_weight_to_end = heuristic(edge.to(), end);
                // aka F
                int estimated_total_weight_to_end =
                    weight_up_to_this_node + estimate_weight_to_end;

                if (estimated_total_weight_to_end <
                    weights.at(edge.to().node_id()).first) {
                    open_list.push(edge.to());
                    weights[edge.to().node_id()].first =
                        estimated_total_weight_to_end;
                    weights[edge.to().node_id()].second = shortest.node_id();
                }
            }

            // if reached end
            if (edge.to().node_id() == end.node_id()) {
                // trace back
                reference_wrapper<map_node> parent = graph[end.node_id()];
                parent.get().tag(node_tag::path);
                path.push_back(parent);

                while (weights.at(parent.get().node_id()).second != -1) {
                    parent = graph[weights.at(parent.get().node_id()).second];
                    parent.get().tag(node_tag::path);
                    path.push_back(parent);
                }

                return path;
            }
        }
    }

    throw std::runtime_error("no path found");
    return path;
}

}  // namespace stud_lib_kmint
