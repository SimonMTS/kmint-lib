#include <exception>
#include <limits>
#include <map>
#include <queue>
#include <stdexcept>
#include <utility>
#include <vector>

#include "kmint/map/map.hpp"
// #include "../../libkmint/include/kmint/map/map.hpp"

namespace stud_lib_kmint {
using kmint::graph::node_tag;
using kmint::map::map_graph;
using kmint::map::map_node;
using std::map;
using std::pair;
using std::reference_wrapper;
using std::vector;
using heuristic = int (*)(map_node&, const map_node&);
using node_list = std::vector<std::reference_wrapper<map_node>>;
using node_priority_queue =
    std::priority_queue<std::reference_wrapper<map_node>, node_list,
                        std::function<bool(const map_node&, const map_node&)>>;

class a_star {
   public:
    static node_list find_path(map_node& start, const map_node& end,
                               map_graph& graph, const heuristic& heuristic);
};
}  // namespace stud_lib_kmint