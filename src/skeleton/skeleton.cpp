#include "skeleton.hpp"

namespace cgp
{


	size_t skeleton_pose_structure::number_joint() const
	{
		return parent_index.size();
	}


	void skeleton_pose_structure::scale(float s)
	{
		size_t const N_joint = number_joint();
		for (size_t k = 0; k < N_joint; ++k) {
			rest_pose_local[k].translation *= s;
			current_pose_local[k].translation *= s;
		}
	}
	

	buffer<affine_rt> skeleton_pose_structure::rest_pose_global() const
	{
		return skeleton_local_to_global(rest_pose_local, parent_index);
	}

	buffer<affine_rt> skeleton_pose_structure::current_pose_global() const
	{
		return skeleton_local_to_global(current_pose_local, parent_index);
	}

	buffer<affine_rt> skeleton_local_to_global(buffer<affine_rt> const& local, buffer<int> const& parent_index)
	{
		assert_cgp(parent_index.size()==local.size(), "Incoherent size of skeleton data");
		size_t const N = parent_index.size();
		buffer<affine_rt> global;
		global.resize(N);
		global[0] = local[0];

		for (size_t k = 1; k < N; ++k)
			global[k] = global[parent_index[k]] * local[k];
	
		return global;
	}

}