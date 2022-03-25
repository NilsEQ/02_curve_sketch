#include "skinning.hpp"

namespace cgp
{
	void normalize_weights(buffer<buffer<float>>& weights)
	{
		size_t const N = weights.size();
		for (size_t k = 0; k < N; ++k) {
			float s = 0.0f;
			for(float w : weights[k]) s += w;
			assert_cgp_no_msg(s>1e-5f);
			for(float& w : weights[k]) w /= s;
		}
	}


	// Linear Blend Skinning
	void skinning_LBS_compute(
		buffer<vec3>& position_skinned,  // position to deform
		buffer<vec3>& normal_skinned,    // normal to deform
		buffer<affine_rt> const& skeleton_current,    // rigid transforms for skeleton joints in current pose
		buffer<affine_rt> const& skeleton_rest_pose,  // rigid transforms of skeleton joints in rest pose
		buffer<vec3> const& position_rest_pose,       // vertex positions of the mesh in rest pose
		buffer<vec3> const& normal_rest_pose,         // normal coordinates of the mesh in rest pose
		rig_structure const& rig)                     // information of the skinning weights (joints and weights associated to a vertex)
	{
		size_t const N_vertex = position_rest_pose.size();
		size_t const N_joint = skeleton_current.size();

		// Sanity check on sizes of buffers
		assert_cgp_no_msg(position_skinned.size()==N_vertex);
		assert_cgp_no_msg(normal_skinned.size()==N_vertex);
		assert_cgp_no_msg(normal_rest_pose.size()==N_vertex);
		assert_cgp_no_msg(skeleton_rest_pose.size()==N_joint);
		assert_cgp_no_msg(rig.joint.size()==N_vertex);
		assert_cgp_no_msg(rig.weight.size()==N_vertex);

		// To do
		//   Compute the Linear Blend Skinning ...

		buffer<mat4> transmats;
		transmats.resize(N_joint);
		for (int i = 0; i < N_joint; i++) {
			transmats[i] = skeleton_current[i].matrix() * inverse(skeleton_rest_pose[i].matrix());
		}

		for (int i = 0; i < N_vertex; i++) {
			size_t N_friends = rig.joint[i].size();
			mat4 mymat;
			for (size_t j = 0; j < N_friends; j++) {
				size_t k = rig.joint[i][j];
				float w = rig.weight[i][j];

				mymat += w * transmats[k];
			}

			position_skinned[i] = (mymat * vec4(position_rest_pose[i], 1.0f)).xyz();
			normal_skinned[i] = (mymat * vec4(normal_rest_pose[i], 0.0f)).xyz();
		}
	}

}