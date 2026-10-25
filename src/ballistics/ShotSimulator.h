#pragma once
#include "scene/Scene.h"
namespace cs2::traceline {
void simulate_shot(const Scene &scene, const cs2tl_shot_request &request, cs2tl_shot_result &result);
}
