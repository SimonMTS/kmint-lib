#include "kmint/pigisland/boat.hpp"
#include "kmint/pigisland/node_algorithm.hpp"
#include "kmint/pigisland/resources.hpp"
#include "kmint/random.hpp"
#include <stud_lib_kmint/a_star/a_star.hpp>
#include <stud_lib_kmint/a_star/heuristics.hpp>
namespace kmint {
namespace pigisland {
  boat::boat(map::map_graph& g, map::map_node& initial_node)
    : play::map_bound_actor{ initial_node },
      graph_{ g },
      drawable_{ *this, graphics::image{boat_image()} } {}


  void boat::act(delta_time dt) {
    t_passed_ += dt;
    if (to_seconds(t_passed_) >= 1) {
      auto& harbor = find_random_mooring_place(graph_);

      stud_lib_kmint::node_list path = stud_lib_kmint::a_star::find_path(
          node(), harbor, graph_,
          stud_lib_kmint::heuristics::manhattan_distance);

      this->node(path[0].get());
      t_passed_ = from_seconds(0);
    }
  }

} // namespace pigisland
} // namespace kmint
