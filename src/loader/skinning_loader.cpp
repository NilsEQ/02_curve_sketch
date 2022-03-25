#include "skinning_loader.hpp"

using namespace cgp;


//Map correspondance between skinning weights and vertices (that have been duplicated to load the texture coordinates)
template <typename T>
buffer<T> map_correspondance(buffer<T> value, buffer<buffer<int> > const& correspondance)
{
    // find the maximal index used for destination
    int max_index = 0;
    for(size_t k1=0; k1<correspondance.size(); ++k1)
        for(size_t k2=0; k2<correspondance[k1].size(); ++k2)
            if(max_index<correspondance[k1][k2])
                max_index = correspondance[k1][k2];

    buffer<T> new_value;
    new_value.resize(max_index+1);

    // Apply correspondance (copy value[k] in its destination)
    for(size_t k=0; k<correspondance.size(); ++k)
        for(size_t k2=0; k2<correspondance[k].size(); ++k2)
            new_value[correspondance[k][k2]] = value[k];
    return new_value;
}



void load_skinning_data(std::string const& directory, skeleton_pose_structure& skeleton_data, rig_structure& rig, mesh & shape, GLuint& texture_id)
{
	buffer<buffer<int> > vertex_correspondance;
	shape = mesh_load_file_obj(directory+"mesh.obj", vertex_correspondance);
	shape.fill_empty_field();
	texture_id = opengl_load_texture_image(directory+"texture.png");
	
	buffer<vec3> skeleton_translation; read_from_file(directory+"skeleton_geometry_translation_local.txt", skeleton_translation);
	buffer<quaternion> skeleton_quaternion; read_from_file(directory+"skeleton_geometry_quaternion_local.txt", skeleton_quaternion);
	
	buffer<int> parent_index; read_from_file(directory+"skeleton_parent_index.txt", parent_index);

	
	buffer<buffer<float>> weights; read_from_file(directory+"rig_weights.txt", weights);
	buffer<buffer<int>> joints; read_from_file(directory+"rig_bones.txt", joints);

	

	weights = map_correspondance(weights, vertex_correspondance);
	joints = map_correspondance(joints, vertex_correspondance);

	assert_cgp_no_msg(skeleton_translation.size()==skeleton_quaternion.size());
	assert_cgp_no_msg(parent_index.size()==skeleton_translation.size());
	assert_cgp_no_msg(weights.size()==joints.size());
	assert_cgp_no_msg(shape.position.size()==joints.size());
	

	size_t const N_joint = skeleton_translation.size();
	skeleton_data.rest_pose_local.resize(N_joint);
	skeleton_data.current_pose_local.resize(N_joint);
	for(size_t k=0; k<N_joint; ++k)
	{
		quaternion const q = normalize(skeleton_quaternion[k]);
		affine_rt const T = affine_rt(rotation_transform(q), skeleton_translation[k]);
		skeleton_data.rest_pose_local[k] = T;
		skeleton_data.current_pose_local[k] = T;
	}

	skeleton_data.parent_index = parent_index;

	rig.weight = weights;
	rig.joint = joints;

}