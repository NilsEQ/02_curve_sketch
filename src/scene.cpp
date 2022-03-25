#include "scene.hpp"

#include "loader/skinning_loader.hpp"

using namespace cgp;


void scene_structure::initialize()
{

	global_frame.initialize(mesh_primitive_frame(), "Frame");

	environment.projection = camera_projection::orthographic(- 1, 1, -1, 1, -1, 1);
	environment.camera.look_at({ 0.0f,0.0f,0.0f }, { 0,0,-1 }, {0,1,0});



	mesh shape;
	GLuint texture_id = mesh_drawable::default_texture;
	load_skinning_data("assets/marine/", skeleton_data, rig, shape, texture_id);
	normalize_weights(rig.weight);
	float const scaling = 0.005f;
	for (auto& p : shape.position) p *= scaling;
	skeleton_data.scale(scaling);

	skeleton_data.bodyline.resize(16);
	skeleton_data.bodyline = { 6, 5, 4, 3,2, 11, 12, 13, 14,19,20, 21, 22, 23, 24, 25 };
	n_joints = skeleton_data.bodyline.size();
	update_new_content(shape, texture_id);


	rotation_transform rot = rotation_transform::from_axis_angle({ 0,0,1 }, -rotation_angle);
	cgp::vec3 axis;
	float angle;
	rotation_transform::convert_quaternion_to_axis_angle(environment.camera.orientation_camera.data, axis, angle);
	axis = rot * axis;
	skeleton_data.current_pose_local[1].rotation = rotation_transform::convert_axis_angle_to_quaternion(axis, Pi);



	skeleton_data.current_pose_local[0].translation.y = 0;


	std::cout << skeleton_data.current_pose_local[0].translation << std::endl;

}


void scene_structure::update_posing()
{

	skinning_data.skeleton_current = skeleton_data.current_pose_global();
	visual_data.skeleton_current.update(skinning_data.skeleton_current, skeleton_data.parent_index, skeleton_data.bodyline);

	// Compute skinning deformation
	skinning_LBS_compute(skinning_data.position_skinned, skinning_data.normal_skinned,
		skinning_data.skeleton_current, skinning_data.skeleton_rest_pose,
		skinning_data.position_rest_pose, skinning_data.normal_rest_pose,
		rig);

	visual_data.surface_skinned.update_position(skinning_data.position_skinned);
	visual_data.surface_skinned.update_normal(skinning_data.normal_skinned);
	visual_data.skeleton_rest_pose;
}

void scene_structure::display()
{

	for(int k=0; k<sketch_drawable.size(); ++k)
		draw(sketch_drawable[k], environment);

	update_posing();

	draw(visual_data.skeleton_current, environment);


	if (gui.display_skin)
		draw(visual_data.surface_skinned, environment);



	//n += 0.1;

	//cgp::vec3 axis;
	//float angle;
	//rotation_transform::convert_quaternion_to_axis_angle(environment.camera.orientation_camera.data, axis, angle);

	//rotation_transform rot1 = rotation_transform::from_axis_angle({ 0,0,1 }, - rotation_angle);

	//cgp::vec3 axis1 = rot1 * axis;



	//skeleton_data.current_pose_local[1].rotation = rotation_transform::convert_axis_angle_to_quaternion(axis1, n);
	//skeleton_data.current_pose_local[12].rotation = rotation_transform::convert_axis_angle_to_quaternion(axis1, 0);
	//skeleton_data.current_pose_local[14].rotation = rotation_transform::convert_axis_angle_to_quaternion(axis1, 14);
	//skeleton_data.current_pose_local[19].rotation = rotation_transform::convert_axis_angle_to_quaternion(axis1, 1);
	//skeleton_data.current_pose_local[20].rotation = rotation_transform::convert_axis_angle_to_quaternion(axis1, 1);
	//skeleton_data.current_pose_local[21].rotation = rotation_transform::convert_axis_angle_to_quaternion(axis1, 1);
	//skeleton_data.current_pose_local[22].rotation = rotation_transform::convert_axis_angle_to_quaternion(axis1, 1);
	//skeleton_data.current_pose_local[23].rotation = rotation_transform::convert_axis_angle_to_quaternion(axis1, 1);





	


}

void scene_structure::update_new_content(mesh const& shape, GLuint texture_id)
{
	visual_data.surface_skinned.clear();
	visual_data.surface_skinned.initialize(shape, "Skinned surface");
	visual_data.surface_skinned.texture = texture_id;

	visual_data.surface_rest_pose.clear();
	visual_data.surface_rest_pose.initialize(shape, "Rest pose");
	visual_data.surface_rest_pose.texture = texture_id;

	skinning_data.position_rest_pose = shape.position;
	skinning_data.position_skinned = skinning_data.position_rest_pose;
	skinning_data.normal_rest_pose = shape.normal;
	skinning_data.normal_skinned = skinning_data.normal_rest_pose;

	skinning_data.skeleton_current = skeleton_data.rest_pose_global();
	skinning_data.skeleton_rest_pose = skinning_data.skeleton_current;

	visual_data.skeleton_current.clear();
	visual_data.skeleton_current = skeleton_drawable(skinning_data.skeleton_current, skeleton_data.parent_index, skeleton_data.bodyline);

	visual_data.skeleton_rest_pose.clear();
	visual_data.skeleton_rest_pose = skeleton_drawable(skinning_data.skeleton_rest_pose, skeleton_data.parent_index,skeleton_data.bodyline);



}



// Compute the 3D position of a position given by its screen coordinates
static vec3 unproject(camera_projection const& P, vec2 p_screen)
{
	// Simple un-project assuming that the viewpoint is an orthogonal projection
	vec4 const p_proj = P.matrix_inverse() * vec4(p_screen, 0.0f, 1.0f);
	return vec3(p_proj.xy(), 0.0f);
}


void scene_structure::mouse_click()
{
	if (inputs.mouse.click.last_action == last_mouse_cursor_action::click_left) 
	{
		// Create new stroke (curve_dynamic_drawable)
		int const N_stroke = sketch_drawable.size();
		if (N_stroke > 0) {
			sketch_drawable[N_stroke - 1].clear();
			sketch_drawable.resize(N_stroke - 1);
		}

		int k_sketch = sketch_drawable.size();
		sketch_drawable.push_back(curve_dynamic_drawable());
		sketch_drawable[k_sketch].initialize("Sketch " + str(k_sketch));


		// Add the new clicked position
		vec3 const p = unproject(environment.projection, inputs.mouse.position.current);
		sketch_drawable[k_sketch].push_back(p);

		mem.clear();

		mem.push_back(p);


	}

	if (inputs.mouse.click.last_action == last_mouse_cursor_action::release_left) {

		if (mem.size() <= 1) return;

		cgp::mat4 W = { 1.f, 0.f, 0.f, 0.f,
						1.f, 1.f, 1.f, 1.f,
						0.f, 1.f, 0.f, 0.f,
						0.f, 1.f, 2.f, 3.f };

		int n = mem.size();

		cgp::vec3 p_b0 = mem[0];
		cgp::vec3 p_b1 = mem[1];

		cgp::vec3 p_m0 = mem[n/2 - 1];
		cgp::vec3 p_m1 = mem[n/2];
		cgp::vec3 p_m2 = mem[n/2+ 1];

		cgp::vec3 p_e0 = mem[n -2];
		cgp::vec3 p_e1 = mem[n -1];


		cgp::vec3 tanb =  { p_b1.x - p_b0.x,p_b1.y - p_b0.y,p_b1.z - p_b0.z};
		tanb = normalize(tanb);
		cgp::vec3 tanm = { p_m2.x - p_m0.x,p_m2.y - p_m0.y,p_m2.z - p_m0.z};
		tanm = normalize(tanm);
		cgp::vec3 tane = { p_e1.x - p_e0.x,p_e1.y - p_e0.y,p_e1.z - p_e0.z };
		tanm = normalize(tanm);


		cgp::vec4 to_fit_x = {p_b0.x,p_m1.x,tanb.x,tanm.x};
		hermite_1x = inverse(W) * to_fit_x;
		cgp::vec4 to_fit_y = {p_b0.y,p_m1.y,tanb.y,tanm.y};
		hermite_1y = inverse(W) * to_fit_y;
		cgp::vec4 to_fit_z = {p_b0.z,p_m1.z,tanb.z,tanm.z};
		hermite_1z = inverse(W) * to_fit_z;

		to_fit_x = { p_m1.x,p_e1.x,tanm.x,tane.x };
		hermite_2x = inverse(W) * to_fit_x;
		to_fit_y = { p_m1.y,p_e1.y,tanm.y,tane.y };
		hermite_2y = inverse(W) * to_fit_y;
		to_fit_z = { p_m1.z,p_e1.z,tanm.z,tane.z };
		hermite_2z = inverse(W) * to_fit_z;


		int const N_stroke = sketch_drawable.size();
		if (N_stroke > 0) {
			sketch_drawable[N_stroke - 1].clear();
			sketch_drawable.resize(N_stroke - 1);
		}

		int k_sketch = sketch_drawable.size();
		sketch_drawable.push_back(curve_dynamic_drawable());
		sketch_drawable[k_sketch].initialize("Sketch");

		for (int i = 0; i < 250; i++) {
			float t = float(i) / 250.0;
			vec3 const p = compute_hermite1(t);
			sketch_drawable[k_sketch].push_back(p);
		}

		for (int i = 0; i < 250; i++) {
			float t = float(i) / 250.0;
			vec3 const p = compute_hermite2(t);
			sketch_drawable[k_sketch].push_back(p);
		}
	}
}

vec3 scene_structure::compute_hermite1(float t) {
	return { hermite_1x[0] + hermite_1x[1] * t + hermite_1x[2] * pow(t,2) + hermite_1x[3] * pow(t,3),
			 hermite_1y[0] + hermite_1y[1] * t + hermite_1y[2] * pow(t,2) + hermite_1y[3] * pow(t,3),
			 hermite_1z[0] + hermite_1z[1] * t + hermite_1z[2] * pow(t,2) + hermite_1z[3] * pow(t,3) };
}

vec3 scene_structure::compute_hermite2(float t) {
	return { hermite_2x[0] + hermite_2x[1] * t + hermite_2x[2] * pow(t,2) + hermite_2x[3] * pow(t,3),
			 hermite_2y[0] + hermite_2y[1] * t + hermite_2y[2] * pow(t,2) + hermite_2y[3] * pow(t,3),
			 hermite_2z[0] + hermite_2z[1] * t + hermite_2z[2] * pow(t,2) + hermite_2z[3] * pow(t,3) };
}


void scene_structure::mouse_move()
{
	if (inputs.mouse.click.left && is_sketching) {
		// Add the new clicked position
		int k_sketch = sketch_drawable.size() - 1;
		vec3 const p = unproject(environment.projection, inputs.mouse.position.current);
		sketch_drawable[k_sketch].push_back(p);

		mem.push_back(p);
	}
}


void scene_structure::keyboard() {
	if (inputs.keyboard.left == 1){
		rotation_angle += 0.1;
		rotation_transform rot = rotation_transform::from_axis_angle({ 0,0, 1. }, 0.1);
		skeleton_data.current_pose_local[1].rotation.data *= rot.data;
	}

	if (inputs.keyboard.right == 1) {
		rotation_angle -= 0.1;
		rotation_transform rot = rotation_transform::from_axis_angle({ 0,0, 1. }, -0.1);
		skeleton_data.current_pose_local[1].rotation.data *= rot.data;
	}



}

void scene_structure::fit_model(){

	//initializing the plane we're working in
	cgp::vec3 axis;
	float angle;
	rotation_transform::convert_quaternion_to_axis_angle(environment.camera.orientation_camera.data, axis, angle);
	rotation_transform rot = rotation_transform::from_axis_angle({ 0,0,1 }, - rotation_angle);
	axis = rot * axis;

	buffer<int> joints = skeleton_data.bodyline;


	//initilizing the parameters to optimize
	vec3 xr = skeleton_data.current_pose_local[0].translation;
	skeleton_data.current_pose_local[2].rotation = rotation_transform::convert_axis_angle_to_quaternion(axis, Pi);

	buffer<float> thetas;
	buffer<vec2> ws;

	for (int i = 0; i < n_joints -1; i++) {
		if (i > 0) {
			int joint = joints[i];
			skeleton_data.current_pose_local[joint].rotation = rotation_transform::convert_axis_angle_to_quaternion(axis, 0.);
			if (joint == 2 || joint == 11) {
				skeleton_data.current_pose_local[joint].rotation = rotation_transform::convert_axis_angle_to_quaternion(axis, Pi);
				thetas.push_back(Pi);
			}
			else thetas.push_back(0.0);

		}

		vec2 wi;
		wi.x = 2 * i / (n_joints - 1); wi.y = 2 * (i + 1) / (n_joints - 1);
		ws.push_back(wi);
	}



	for (int k = 0; k < n_steps; k++) {
		xr = optimize_xr(xr, ws, axis);
		thetas = optimize_thetas(thetas, ws, axis);

		skeleton_data.current_pose_local[0].translation = xr;

		for (int i = 1; i < n_joints - 1; i++) {
			int joint = joints[i];
			skeleton_data.current_pose_local[joint].rotation = rotation_transform::convert_axis_angle_to_quaternion(axis, thetas[i-1]);
		}

		ws = optimize_warped_param(ws, axis);
	}



}


vec3 scene_structure::optimize_xr(vec3 xr, buffer<vec2> ws, vec3 axis){
	buffer<buffer<vec3>> x_loa = construct_x_loa(ws);

	buffer<affine_rt> global_transfo = skeleton_data.current_pose_global();
	buffer<buffer<vec3>> x_b0= construct_x_b(global_transfo);

	float E1 = energy1(x_b0, x_loa);

	buffer<buffer<vec3>> x_b1;

	//to_optimize xr
	vec3 grad_xr;
	
	skeleton_data.current_pose_local.data[0].translation.x = xr.x + dx;
	global_transfo = skeleton_data.current_pose_global();
	x_b1 = construct_x_b(global_transfo);
	grad_xr.x = (energy1(x_b1, x_loa) - E1) / dx;
	skeleton_data.current_pose_local.data[0].translation.x = xr.x;

	skeleton_data.current_pose_local.data[0].translation.y = xr.y + dx;
	global_transfo = skeleton_data.current_pose_global();
	x_b1 = construct_x_b(global_transfo);
	grad_xr.y = (energy1(x_b1, x_loa) - E1) / dx;
	skeleton_data.current_pose_local.data[0].translation.y = xr.y;

	std::cout << grad_xr << std::endl;




	//optimize xr
	xr.x -= alpha * grad_xr.x;
	xr.y -= alpha * grad_xr.y;

	return xr;
}




cgp::buffer<float> scene_structure::optimize_thetas(cgp::buffer<float> thetas, buffer<vec2> ws, vec3 axis) {

	buffer<buffer<vec3>> x_loa = construct_x_loa(ws);

	buffer<affine_rt> global_transfo = skeleton_data.current_pose_global();
	buffer<buffer<vec3>> x_b0 = construct_x_b(global_transfo);

	float E1 = energy1(x_b0, x_loa);

	buffer<buffer<vec3>> x_b1;

	//to optimize thetas
	buffer<float> grad_thetas;

	for (int i = 1; i < n_joints - 1; i++) {
		int joint = skeleton_data.bodyline[i];
		float theta = thetas[i - 1];
		skeleton_data.current_pose_local.data[joint].rotation = rotation_transform::convert_axis_angle_to_quaternion(axis, theta + dtheta);
		global_transfo = skeleton_data.current_pose_global();
		x_b1 = construct_x_b(global_transfo);
		grad_thetas.push_back((energy1(x_b1, x_loa) - E1) / dtheta);

		skeleton_data.current_pose_local.data[joint].rotation = rotation_transform::convert_axis_angle_to_quaternion(axis, theta);

	}

	//optimize thetas
	for (int i = 1; i < n_joints - 1; i++) {
		thetas[i - 1] -= alpha * grad_thetas[i - 1];
	}

	return thetas;
}


buffer<vec2> scene_structure::optimize_warped_param(buffer<vec2> ws, cgp::vec3 axis) {
	buffer<affine_rt> global_transfo = skeleton_data.current_pose_global();
	buffer<buffer<vec3>> x_b = construct_x_b(global_transfo);

	buffer<buffer<vec3>> x_loa1 = construct_x_loa(ws);
	buffer<buffer<vec3>> x_loa2;

	float E1 = energy2(x_b, x_loa1, ws);

	buffer<vec2> grad_w;

	for (int i = 0; i < n_joints - 1; i++) {

		vec2 grad_wi;

		ws[i].x += dw;
		x_loa2 = construct_x_loa(ws);
		grad_wi.x = (energy2(x_b, x_loa2,ws) - E1) / dw;
		ws[i].x -= dw;

		ws[i].y += dw;
		x_loa2 = construct_x_loa(ws);
		grad_wi.y = (energy2(x_b, x_loa2, ws) - E1) / dw;
		ws[i].y -= dw;

		grad_w.push_back(grad_wi);
	}

	
	for (int i = 0; i < n_joints - 1; i++) {
		ws[i] -= alpha * grad_w[i];
	}

	return ws;

}

buffer<buffer<vec3>> scene_structure::construct_x_b(buffer<affine_rt> global_transfo) {
	buffer<buffer<vec3>>x_b; 

	for (int i = 0; i < n_joints - 1; i++) {
		buffer<vec3> x;
		int joint1 = skeleton_data.bodyline[i];
		int joint2 = skeleton_data.bodyline[i+1];
		vec3 x1 = global_transfo[joint1] * vec3{0.,0.,0.};
		vec3 x2= global_transfo[joint2] * vec3{ 0.,0.,0. };
		for (int j = 0; j <= n_samples; j++) {
			x.push_back(x1* (1 - j/n_samples) + x2* j/n_samples);
		}
		x_b.push_back(x);
	}
	return(x_b);
}


buffer<buffer<vec3>> scene_structure::construct_x_loa(buffer<vec2> ws) {
	buffer<buffer<vec3>>x_loa;
	for (int i = 0; i < n_joints - 1; i++) {
		buffer<vec3> x;
		float w0 = ws[i].x;
		float w1 = ws[i].y;
		for (int j = 0; j <= n_samples; j++) {
			float t = w0 * (1 - j/n_samples) + w1* j/n_samples;
			if (t <= 1.) x.push_back(compute_hermite1(t));
			else x.push_back(compute_hermite2(t - 1));
		}
		x_loa.push_back(x);
	}
	return(x_loa);
}

float scene_structure::energy1(buffer<buffer<vec3>>x_b, buffer<buffer<vec3>>x_loa) {
	float E = 0.;

	for (int i = 0; i < n_joints - 1; i++) {
		for (int j = 0; j <= n_samples; j++) {
			E += (1. / ((float) n_samples + 1.)) * std::pow(norm(x_b[i][j] - x_loa[i][j]), 2);

			if (j < n_samples) {
				vec3 T_b = (x_b[i][j + 1] - x_b[i][j]) * n_samples;
				vec3 T_loa = (x_loa[i][j + 1] - x_loa[i][j]) * n_samples;
				E += (0.01 / ((float)n_samples + 1.)) * std::pow(norm(T_b - T_loa), 2);
			}
		}
	}

	return E;
}

float scene_structure::energy2(buffer<buffer<vec3>>x_b, buffer<buffer<vec3>>x_loa, buffer<vec2> ws) {
	float E = 0.;

	for (int i = 0; i < n_joints - 1; i++) {
		if (i != 0) E += std::pow(ws[i - 1].y - ws[i].x, 2);

		for (int j = 0; j <= n_samples; j++) {
			E += (1. / ((float)n_samples + 1.)) * std::pow(norm(x_b[i][j] - x_loa[i][j]), 2);


			if (j < n_samples && 0 < j) {
				vec3 double_der = (x_b[i][j + 1] - 2*x_b[i][j] + x_b[i][j - 1]) * std::pow(n_samples,2);
				E += (1 / ((float)n_samples + 1.)) * std::pow(norm(double_der), 2);
			}
		}
	}

	return E;
}


void scene_structure::display_gui()
{

	bool fit = ImGui::Button("Fit model");
	if (fit) {
		fit_model();
	}

	bool cancel = ImGui::Button("Cancel last stroke");
	if (cancel)
	{
		// remove last stroke
		int const N_stroke = sketch_drawable.size();
		if (N_stroke > 0) {
			sketch_drawable[N_stroke - 1].clear();
			sketch_drawable.resize(N_stroke - 1);
		}
	}


	if (ImGui::Button("Show skin")) {
		gui.display_skin = true;
	}

	if (ImGui::Button("Show skeleton")) {
		gui.display_skin = false;
	}

}
