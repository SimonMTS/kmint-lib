#include "pigisland/pig.hpp"

#include "kmint/random.hpp"
#include "pigisland/resources.hpp"

namespace kmint {
namespace pigisland {

pig::pig(math::vector2d location)
    : play::free_roaming_actor{location}, drawable_{*this, pig_image()} {}

}  // namespace pigisland

}  // namespace kmint
