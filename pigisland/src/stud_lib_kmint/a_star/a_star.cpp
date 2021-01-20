// #include "..\include\stud_lib_kmint\a_star\a_star.hpp"
#include "stud_lib_kmint/a_star/a_star.hpp"

#include <kmint/random.hpp>

namespace stud_lib_kmint {

node_list a_star::find_path(map_node& start, const map_node& end,
                            const map_graph& graph,
                            const heuristic& heuristic) {
    map<int, pair<int, int>> weights;  // nodeID, weight, parentID
    for (auto& node : graph) {
        weights[node.node_id()] =
            std::make_pair<int, int>(std::numeric_limits<int>::max(), 0);
    }

    node_list path;

    // closed list
    node_list closed_list;

    // open list
    std::reference_wrapper<map_node> tmp = start;

    auto cmp = [&weights](const map_node& lhs, const map_node& rhs) {
        return weights[lhs.node_id()].first < weights[rhs.node_id()].first;
    };

    std::priority_queue<std::reference_wrapper<map_node>, node_list,
                        decltype(cmp)>
        open_list(cmp);
    open_list.push(tmp);  // <-- ??

    // loop over open list
    while (!open_list.empty()) {
        map_node& shortest = open_list.top();
        open_list.pop();

        for (auto& edge : shortest) {
            // if reached end
            if (edge.to().node_id() == end.node_id()) {
            }

            // if not closed
            // if (std::find(closed_list.begin(), closed_list.end(), edge.to())
            // != closed_list.end()) {
            //  int weight_up_to_this_node = 0; // aka G
            //  int estimate_weight_to_end = heuristic(edge.to(), end); // aka H
            //  int estimated_total_weight_to_end =
            //      weight_up_to_this_node + estimated_total_weight_to_end; //
            //      aka F

            //  if (estimated_total_weight_to_end <
            //  weights.at(edge.to().node_id()).first) {
            //    open_list.push(edge.to());
            //  }
            // }
        }
    }

    // random
    path.push_back(start[kmint::random_int(0, start.num_edges())].to());

    return path;
}

}  // namespace stud_lib_kmint