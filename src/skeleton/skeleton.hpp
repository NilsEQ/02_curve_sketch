#pragma once

#include "cgp/cgp.hpp"


namespace cgp
{
	// Helper structure storing an animated skeleton
	struct skeleton_pose_structure
	{
		buffer<int> parent_index;          // Connectivity of the skeleton. Stores at index i, the index of the parent joint
		buffer<affine_rt> rest_pose_local; // Rest pose storage of the rigid transforms (rotation and translation) of the joints in local coordinates

		buffer<affine_rt> current_pose_local; // Storage of all rigid transforms of the joints

		// Number of joints in the skeleton
		size_t number_joint() const;

		// Return the rigid transforms of the joints of the rest and current pose in global coordinates
		buffer<affine_rt> rest_pose_global() const;
		buffer<affine_rt> current_pose_global() const;


		// Apply scaling to the entire skeleton (scale the translation part of the rigid transforms)
		void scale(float s);

		buffer<int> bodyline;

	};

	// Convert a skeleton defined in local coordinates to global coordinates
	buffer<affine_rt> skeleton_local_to_global(buffer<affine_rt> const& local, buffer<int> const& parent_index);
}