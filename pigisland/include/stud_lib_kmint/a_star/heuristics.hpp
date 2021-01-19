#include "kmint/map/map.hpp"
using kmint::map::map_node;

namespace stud_lib_kmint {
class heuristics {
 public:
  static int manhattan_distance(map_node& start, const map_node& end);
  static int diagonal_distance(map_node& start, const map_node& end);
  static int euclidean_distance(map_node& start, const map_node& end);
};
}  // namespace stud_lib_kmint