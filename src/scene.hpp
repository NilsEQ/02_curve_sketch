#pragma once

#include "cgp/cgp.hpp"

#include "skeleton/skeleton.hpp"
#include "skeleton/skeleton_drawable.hpp"
#include "skinning/skinning.hpp"

struct gui_parameters {
	bool display_skin = true;
};


struct visual_shapes_parameters
{
	cgp::mesh_drawable surface_skinned;
	cgp::mesh_drawable surface_rest_pose;
	cgp::skeleton_drawable skeleton_current;
	cgp::skeleton_drawable skeleton_rest_pose;
};

struct skinning_current_data
{
	cgp::buffer<cgp::vec3> position_rest_pose;
	cgp::buffer<cgp::vec3> position_skinned;
	cgp::buffer<cgp::vec3> normal_rest_pose;
	cgp::buffer<cgp::vec3> normal_skinned;

	cgp::buffer<cgp::affine_rt> skeleton_current;
	cgp::buffer<cgp::affine_rt> skeleton_rest_pose;
};

// The structure of the custom scene
struct scene_structure {


	gui_parameters gui; // Standard GUI element storage

	cgp::timer_interval timer;


	cgp::mesh_drawable global_frame;          // The standard global frame
	cgp::scene_environment_basic environment; // Standard environment controler
	cgp::inputs_interaction_parameters inputs; // storage for the current values of the inputs (mouse, keyboard, window dimension) that can be use for interaction purpose

	// Store the curve sketched on screen. 
	//   Each new stroke (continuous click+motion of the mouse) is a new element of the buffer
	cgp::buffer<cgp::curve_dynamic_drawable> sketch_drawable; 
	cgp::buffer<cgp::curve_dynamic_drawable> sketch_hermite;

	// ****************************** //
	// Elements and shapes of the scene
	// ****************************** //


	visual_shapes_parameters visual_data;
	cgp::skeleton_pose_structure skeleton_data;
	cgp::rig_structure rig;
	skinning_current_data skinning_data;

	float rotation_angle = 0.;

	// ****************************** //
	// Functions
	// ****************************** //

	void initialize();  // Standard initialization to be called before the animation loop
	void display();     // The frame display to be called within the animation loop
	void display_gui(); // The display of the GUI, also called within the animation loop
	bool is_sketching = true;

	// Add new points in the sketch_drawable
	void mouse_move();

	// Add a new stroke in the sketch_drawable
	void mouse_click();

	void keyboard();


	//our functions to fit the pose
	int n_joints;
	int n_samples = 4;
	int n_steps = 100;
	void fit_model();
	cgp::vec3 optimize_xr(cgp::vec3 xr, cgp::buffer<cgp::vec2> ws, cgp::vec3 axis);
	cgp::buffer<float> optimize_thetas(cgp::buffer<float> thetas, cgp::buffer<cgp::vec2> ws, cgp::vec3 axis);
	cgp::buffer<cgp::vec2> optimize_warped_param(cgp::buffer<cgp::vec2> ws, cgp::vec3 axis);
	cgp::buffer<cgp::buffer<cgp::vec3>> construct_x_loa(cgp::buffer<cgp::vec2> ws);
	cgp::buffer<cgp::buffer<cgp::vec3>> construct_x_b(cgp::buffer<cgp::affine_rt> global_transfo);
	float energy1(cgp::buffer<cgp::buffer<cgp::vec3>>x_b, cgp::buffer<cgp::buffer<cgp::vec3>>x_loa);
	float energy2(cgp::buffer<cgp::buffer<cgp::vec3>>x_b, cgp::buffer<cgp::buffer<cgp::vec3>>x_loa, cgp::buffer<cgp::vec2> ws);

	float dx = 0.01;
	float dtheta = 0.01;
	float dw = 0.1;
	float alpha = 0.01;



	//Tools to compute hermite coordinates
	cgp::buffer<cgp::vec3> mem;
	cgp::vec4 hermite_1x;
	cgp::vec4 hermite_1y;
	cgp::vec4 hermite_1z;

	cgp::vec4 hermite_2x;
	cgp::vec4 hermite_2y;
	cgp::vec4 hermite_2z;

	cgp::vec3 compute_hermite1(float t);
	cgp::vec3 compute_hermite2(float t);

	float n = 0.;


	void update_posing();
	void update_new_content(cgp::mesh const& shape, GLuint texture_id);


};





