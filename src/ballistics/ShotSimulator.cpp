#include "ShotSimulator.h"
#include "SurfaceProfile.h"
#include "WeaponProfile.h"
#include "diagnostics/Error.h"
#include "geometry/Intersection.h"
#include "profiles/Ruleset.h"
#include <limits>

namespace cs2::traceline {
void simulate_shot(const Scene &, const cs2tl_shot_request &request, cs2tl_shot_result &result) {
    const auto size = result.struct_size;
    result = {};
    result.struct_size = size;
    result.outcome = CS2TL_SHOT_NOT_COMPUTED;
    result.remaining_damage = std::numeric_limits<float>::quiet_NaN();
    result.target_damage = std::numeric_limits<float>::quiet_NaN();
    result.ruleset_version = unimplemented_ruleset;
    require(request.segment.struct_size >= sizeof(cs2tl_segment), CS2TL_INVALID_ARGUMENT,
            "Invalid embedded segment size.");
    (void)make_ray(request.segment);
    require(request.weapon && request.weapon->struct_size >= sizeof(WeaponProfile), CS2TL_INVALID_ARGUMENT,
            "Weapon profile is required.");
    require(request.reserved == 0 && (!request.surface_count || request.surfaces), CS2TL_INVALID_ARGUMENT,
            "Invalid shot request.");
    for (uint32_t i = 0; i < request.surface_count; ++i) {
        require(request.surfaces[i].struct_size >= sizeof(SurfaceProfile), CS2TL_INVALID_ARGUMENT,
                "Invalid surface profile size.");
    }
    require(!request.target || request.target->struct_size >= sizeof(cs2tl_target_profile),
            CS2TL_INVALID_ARGUMENT, "Invalid target profile size.");
    throw Error(CS2TL_NOT_IMPLEMENTED, "Ballistics is reserved; no penetration damage model is implemented.");
}
} // namespace cs2::traceline
