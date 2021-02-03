#include "stud_lib_kmint/a_star/heuristics.hpp"

int stud_lib_kmint::heuristics::manhattan_distance(map_node& start,
                                                   const map_node& end) {
    int pxdist = 16;
    int x = std::abs(start.location().x() - end.location().x());
    int y = std::abs(start.location().y() - end.location().y());
    int res = (x + y) / pxdist;

    return res / 10;  // divide by 10 so it doesnt over power
}

int stud_lib_kmint::heuristics::euclidean_distance(map_node& start,
                                                   const map_node& end) {
    int pxdist = 16;
    int x = start.location().x() - end.location().x();
    int y = start.location().y() - end.location().y();
    int res = sqrt(pow(x, 2) + pow(y, 2)) / pxdist;

    return res / 10;  // divide by 10 so it doesnt over power
}