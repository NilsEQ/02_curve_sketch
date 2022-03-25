#pragma once

#include "cgp/cgp.hpp"
#include "../skinning/skinning.hpp"
#include "../skeleton/skeleton.hpp"


void load_skinning_data(std::string const& directory, cgp::skeleton_pose_structure& skeleton_data, cgp::rig_structure& rig, cgp::mesh& shape, GLuint& texture_id);
