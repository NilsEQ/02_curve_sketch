#include "skeleton_drawable.hpp"

namespace cgp
{
	skeleton_drawable::skeleton_drawable()
		:segments(), joint_frame(), joint_sphere(), data()
	{}

	skeleton_drawable::skeleton_drawable(buffer<affine_rt> const& skeleton, buffer<int> const& parent_index, buffer<int> const& bodyline)
		: segments(), joint_frame(), joint_sphere(), data(skeleton)
	{
		size_t const N = parent_index.size();
		buffer<vec3> edges;
		buffer<vec3> edges_body_line;


		for (size_t k = 1; k < N; ++k) {
			size_t const parent = parent_index[k];
			assert_cgp_no_msg(parent >= 0 && parent < N);
			edges.push_back(skeleton[k].translation);
			edges.push_back(skeleton[parent].translation);

		}

		for (size_t k : bodyline) {
			size_t const parent = parent_index[k];
			assert_cgp_no_msg(parent >= 0 && parent < N);
			edges_body_line.push_back(skeleton[k].translation);
			edges_body_line.push_back(skeleton[parent].translation);

		}

		segments.initialize(edges, "Bone-segment");
		segments_body_line.initialize(edges_body_line, "Body-line");
		segments_body_line.color.x = 1.;
		joint_frame.initialize(mesh_primitive_frame(), "Skeleton-frame");
		joint_sphere.initialize(mesh_primitive_sphere(), "Joint-sphere");
	}

	void skeleton_drawable::clear()
	{
		segments.clear();
		joint_frame.clear();
		joint_sphere.clear();
		data.clear();
	}

	void skeleton_drawable::update(buffer<affine_rt> const& skeleton, buffer<int> const& parent_index, buffer<int> const& bodyline)
	{
		data = skeleton;

		size_t const N = skeleton.size();
		buffer<vec3> edges;
		for (size_t k = 1; k < N; ++k){
			size_t const parent = parent_index[k];
			assert_cgp_no_msg(parent>=0 && parent<N);
			edges.push_back(skeleton[k].translation);
			edges.push_back(skeleton[parent].translation);
		}

		buffer<vec3> edges_body_line;
		for (size_t k : bodyline) {
			size_t const parent = parent_index[k];
			assert_cgp_no_msg(parent >= 0 && parent < N);
			edges_body_line.push_back(skeleton[k].translation);
			edges_body_line.push_back(skeleton[parent].translation);

		}

		segments.update(edges);
		segments_body_line.update(edges_body_line);

	}
}