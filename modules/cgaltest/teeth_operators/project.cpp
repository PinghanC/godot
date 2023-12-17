#include "project.h"

namespace dentist {
	inline bool ends_with(std::string const &value, std::string const &ending) {
		if (ending.size() > value.size())
			return false;
		return std::equal(ending.rbegin(), ending.rend(), value.rbegin());
	}
	Project::Project()
	{
		// options by default
		m_view[SceneUpperTeeth] = true;
		m_view[SceneLowerTeeth] = true;
		m_view[SceneMeshEdges] = false;
		m_view[SceneUpperInsideRay] = false;
		m_view[SceneUpperInsideFunction] = false;
		m_view[SceneUpperInsideFaces] = true;
		m_view[SceneLowerInsideRay] = false;
		m_view[SceneLowerInsideFunction] = false;
		m_view[SceneLowerInsideFaces] = true;
		m_view[SceneUpperLocalConvexity] = false;
		m_view[SceneUpperLaplacianConvexity] = false;
		m_view[SceneLowerLocalConvexity] = false;
		m_view[SceneLowerLaplacianConvexity] = false;
		m_view[SceneUpperConvexityIsoEdges] = true;
		m_view[SceneLowerConvexityIsoEdges] = true;
		m_view[SceneSelection] = true;
		m_view[SceneSegmentation] = true;
		m_view[SceneValidatedSegmentation] = true;
		m_view[SceneGeodesicDistance] = false;
		m_view[ScenePadOutlines] = true;

		m_view[SceneToothBound] = false;
		m_view[SceneToothCutPoints] = false;
		m_view[SceneToothCutPlane] = true;

		m_view[ScenePairedFittingPlanes] = false;
		m_view[ScenePairedFittingLines] = false;
		m_view[ScenePairedFittingPoints] = false;
		m_view[ScenePairedPlanes] = true;
		m_view[ScenePairedWires] = true;
		m_view[ScenePairedWireShapes] = false;
		m_view[ScenePairedWireExtrudeShapes] = true;
		m_view[ScenePadShapes] = true;

		for (int i = 0; i < 2 * teeth_count; i++)
		{
			upper_seg_attribus.push_back(DataList());
			lower_seg_attribus.push_back(DataList());
		}

		for (int i = 0; i < teeth_count; i++)
		{
			upper_geodesic_attribus.push_back(DataList());
			lower_geodesic_attribus.push_back(DataList());
		}

		for (int i = 0; i < 2 * teeth_count; i++)
		{
			upper_pad_outlines_attribus.push_back(DataList());
			lower_pad_outlines_attribus.push_back(DataList());
		}

		for (int i = 0; i < teeth_count; i++)
		{
			upper_tooth_bounds_attribus.push_back(DataList());
			lower_tooth_bounds_attribus.push_back(DataList());
			upper_tooth_cut_points_attribus.push_back(DataList());
			lower_tooth_cut_points_attribus.push_back(DataList());
		}

		for (int i = 0; i < 2 * teeth_count; i++)
		{
			upper_tooth_cut_planes_attribus.push_back(DataList());
			lower_tooth_cut_planes_attribus.push_back(DataList());
		}

		for (int i = 0; i < 2 * pair_count; i++)
		{
			upper_pair_fitting_planes_attribus.push_back(DataList());
			lower_pair_fitting_planes_attribus.push_back(DataList());

			upper_pair_cut_planes_attribus.push_back(DataList());
			lower_pair_cut_planes_attribus.push_back(DataList());
		}

		for (int i = 0; i < pair_count; i++)
		{
			upper_pair_fitting_lines_attribus.push_back(DataList());
			lower_pair_fitting_lines_attribus.push_back(DataList());

			upper_pair_fitting_points_attribus.push_back(DataList());
			lower_pair_fitting_points_attribus.push_back(DataList());

			upper_pair_wires_attribus.push_back(DataList());
			lower_pair_wires_attribus.push_back(DataList());
		}

		for (int i = 0; i < 2 * pair_count; i++)
		{
			upper_wire_shapes_attribus.push_back(DataList());
			lower_wire_shapes_attribus.push_back(DataList());
			upper_wire_extrude_shapes_attribus.push_back(DataList());
			lower_wire_extrude_shapes_attribus.push_back(DataList());
		}

		for (int i = 0; i < 2 * teeth_count; i++)
		{
			upper_pad_shapes_attribus.push_back(DataList());
			lower_pad_shapes_attribus.push_back(DataList());
		}
	}

	Project::~Project()
	{
	}

	/******************* Visualization **********************/

	void Project::toggle_view(SceneObjectType type) {
		m_view[type] = !m_view[type];
	}

	/******************* I/O **********************/

	bool Project::load_upper_teeth(const std::string& filename)
	{
		bool flag = false;

		if (ends_with(filename,".stl"))
			flag = m_func.load_upper_teeth_mesh(filename);
		else
			std::cout << "Format not supported!" << std::endl;

		if (flag)
			compute_elements(FlagMesh | FlagUpper);
		return flag;
	}

	bool Project::load_lower_teeth(const  std::string& filename)
	{
		bool flag = false;

		if (ends_with(filename,".stl"))
			flag = m_func.load_lower_teeth_mesh(filename);
		else
			std::cout << "Format not supported!" << std::endl;

		if (flag)
			compute_elements(FlagMesh | FlagLower);
		return flag;
	}

	void Project::get_bounding_shape(double& x, double& y, double& z, double& r)
	{
		Point center = m_func.get_center();
		r = m_func.get_radius();
		x = center.x();
		y = center.y();
		z = center.z();
	}

	/******************* Update **********************/
	void Project::set_mouse_pos(double x, double y, double z, double vx, double vy, double vz)
	{
		m_mouse_pos = dentist::Point(x, y, z);
		m_mouse_vec = dentist::Vector(vx, vy, vz);
	}

	bool Project::update_mouse_selection()
	{
		bool flag = m_func.update_selection(m_mouse_pos, m_mouse_vec, m_select_pos, selected_points);
		return flag;
		//if(flag)
			 //are_buffers_initialized = false;
	}

	void Project::compute_elements(unsigned int flags)
	{
		clear_elements(flags);

		if ((flags & FlagMesh) && (flags & FlagUpper))
		{
			m_func.update_teeth_mesh(upper_teeth_facets, upper_teeth_normals, false);
		}

		if ((flags & FlagMesh) && (flags & FlagLower))
		{
			m_func.update_teeth_mesh(lower_teeth_facets, lower_teeth_normals, true);
		}

		if ((flags & FlagInside))
		{
			m_func.update_inside_function(upper_inside_ray_facets,
				upper_inside_ray_normals,
				upper_inside_ray_colors,
				upper_inside_func_colors,
				upper_inside_facets,
				upper_inside_normals,
				false);
			m_func.update_inside_function(lower_inside_ray_facets,
				lower_inside_ray_normals,
				lower_inside_ray_colors,
				lower_inside_func_colors,
				lower_inside_facets,
				lower_inside_normals,
				true);

		}

		if ((flags & FlagConvex))
		{
			m_func.update_convex_function(upper_local_convex_facets,
				upper_local_convex_normals,
				upper_local_convex_colors,
				upper_convex_func_colors,
				false);
			m_func.update_convex_function(lower_local_convex_facets,
				lower_local_convex_normals,
				lower_local_convex_colors,
				lower_convex_func_colors,
				true);
		}

		if ((flags & FlagConvexIso))
		{
			m_func.update_convexity_isovalue(upper_convex_iso_edges,
				m_mesh_options.convex_isovalue,
				false);
			m_func.update_convexity_isovalue(lower_convex_iso_edges,
				m_mesh_options.convex_isovalue,
				true);
		}

		if ((flags & FlagSegment))
		{
			m_func.update_segmentation(selected_seg_facets, selected_seg_normals);
		}

		if ((flags & FlagValid) && (flags & FlagUpper))
		{
			for (int i = 0; i < 6; i++)
				m_func.update_validated_segmentation(i, false, upper_seg_attribus[2 * i], upper_seg_attribus[2 * i + 1]);
		}

		if ((flags & FlagValid) && (flags & FlagLower))
		{
			for (int i = 0; i < 6; i++)
				m_func.update_validated_segmentation(i, true, lower_seg_attribus[2 * i], lower_seg_attribus[2 * i + 1]);
		}

		if ((flags & FlagPadGeodesic))
		{
			for (int i = 0; i < 6; i++)
			{
				if (m_mesh_options.upper_flags[i])
					m_func.update_tooth_geodesic_function(i, false, upper_geodesic_attribus[i]);

				if (m_mesh_options.lower_flags[i])
					m_func.update_tooth_geodesic_function(i, true, lower_geodesic_attribus[i]);
			}
		}

		if ((flags & FlagPadOutline))
		{
			for (int i = 0; i < 6; i++)
			{
				if (m_mesh_options.upper_flags[i])
					m_func.update_pad_outline(i, false, upper_pad_outlines_attribus[2 * i], upper_pad_outlines_attribus[2 * i + 1]);

				if (m_mesh_options.lower_flags[i])
					m_func.update_pad_outline(i, true, lower_pad_outlines_attribus[2 * i], lower_pad_outlines_attribus[2 * i + 1]);
			}
		}

		if ((flags & FlagToothCut))
		{
			for (int i = 0; i < teeth_count; i++)
			{
				if (m_mesh_options.upper_flags[i])
				{
					m_func.update_tooth_cut_plane(i, false,
						upper_tooth_bounds_attribus[i],
						upper_tooth_cut_points_attribus[i],
						upper_tooth_cut_planes_attribus[2 * i],
						upper_tooth_cut_planes_attribus[2 * i + 1]);
				}


				if (m_mesh_options.lower_flags[i])
				{
					m_func.update_tooth_cut_plane(i, true,
						lower_tooth_bounds_attribus[i],
						lower_tooth_cut_points_attribus[i],
						lower_tooth_cut_planes_attribus[2 * i],
						lower_tooth_cut_planes_attribus[2 * i + 1]);
				}
			}
		}

		if ((flags & FlagPairCut))
		{
			for (int i = 0; i < pair_count; i++)
			{
				if (m_mesh_options.upper_flags[i] && m_mesh_options.upper_flags[i + 1])
				{
					m_func.update_pair_cut_plane(i, false,
						upper_pair_fitting_planes_attribus[2 * i],
						upper_pair_fitting_planes_attribus[2 * i + 1],
						upper_pair_fitting_lines_attribus[i],
						upper_pair_fitting_points_attribus[i],
						upper_pair_cut_planes_attribus[2 * i],
						upper_pair_cut_planes_attribus[2 * i + 1],
						upper_pair_wires_attribus[i]);
				}


				if (m_mesh_options.lower_flags[i] && m_mesh_options.lower_flags[i + 1])
				{
					m_func.update_pair_cut_plane(i, true,
						lower_pair_fitting_planes_attribus[2 * i],
						lower_pair_fitting_planes_attribus[2 * i + 1],
						lower_pair_fitting_lines_attribus[i],
						lower_pair_fitting_points_attribus[i],
						lower_pair_cut_planes_attribus[2 * i],
						lower_pair_cut_planes_attribus[2 * i + 1],
						lower_pair_wires_attribus[i]);
				}
			}
		}

		if ((flags & FlagPairWire))
		{
			for (int i = 0; i < pair_count; i++)
			{
				if (m_mesh_options.upper_flags[i] && m_mesh_options.upper_flags[i + 1])
				{
					m_func.update_pair_wire_shape(i, false,
						upper_wire_shapes_attribus[2 * i],
						upper_wire_shapes_attribus[2 * i + 1],
						upper_wire_extrude_shapes_attribus[2 * i],
						upper_wire_extrude_shapes_attribus[2 * i + 1]);
				}


				if (m_mesh_options.lower_flags[i] && m_mesh_options.lower_flags[i + 1])
				{
					m_func.update_pair_wire_shape(i, true,
						lower_wire_shapes_attribus[2 * i],
						lower_wire_shapes_attribus[2 * i + 1],
						lower_wire_extrude_shapes_attribus[2 * i],
						lower_wire_extrude_shapes_attribus[2 * i + 1]);
				}
			}
		}

		if ((flags & FlagPadShape))
		{
			for (int i = 0; i < teeth_count; i++)
			{
				if (m_mesh_options.upper_flags[i])
				{
					m_func.update_pad_shape(i, false,
						upper_pad_shapes_attribus[2 * i],
						upper_pad_shapes_attribus[2 * i + 1]);
				}


				if (m_mesh_options.lower_flags[i])
				{
					m_func.update_pad_shape(i, true,
						lower_pad_shapes_attribus[2 * i],
						lower_pad_shapes_attribus[2 * i + 1]);
				}
			}
		}

		//are_buffers_initialized = false;
	}

	void Project::clear_elements(unsigned int flags)
	{
		if ((flags & FlagMesh) && (flags & FlagUpper))
		{
			upper_teeth_facets.clear();
			upper_teeth_normals.clear();
		}

		if ((flags & FlagMesh) && (flags & FlagLower))
		{
			lower_teeth_facets.clear();
			lower_teeth_normals.clear();
		}

		if ((flags & FlagInside))
		{
			upper_inside_ray_facets.clear();
			upper_inside_ray_normals.clear();
			upper_inside_ray_colors.clear();
			upper_inside_func_colors.clear();
			upper_inside_facets.clear();
			upper_inside_normals.clear();
			lower_inside_ray_facets.clear();
			lower_inside_ray_normals.clear();
			lower_inside_ray_colors.clear();
			lower_inside_func_colors.clear();
			lower_inside_facets.clear();
			lower_inside_normals.clear();
		}

		if ((flags & FlagConvex))
		{
			upper_local_convex_facets.clear();
			upper_local_convex_normals.clear();
			upper_local_convex_colors.clear();
			upper_convex_func_colors.clear();
			lower_local_convex_facets.clear();
			lower_local_convex_normals.clear();
			lower_local_convex_colors.clear();
			lower_convex_func_colors.clear();

		}

		if ((flags & FlagConvexIso))
		{
			upper_convex_iso_edges.clear();
			lower_convex_iso_edges.clear();
		}

		if ((flags & FlagSegment))
		{
			selected_seg_facets.clear();
			selected_seg_normals.clear();
		}

		if ((flags & FlagValid) && (flags & FlagUpper))
		{
			for (int i = 0; i < upper_seg_attribus.size(); i++)
				upper_seg_attribus[i].clear();
		}

		if ((flags & FlagValid) && (flags & FlagLower))
		{
			for (int i = 0; i < lower_seg_attribus.size(); i++)
				lower_seg_attribus[i].clear();
		}

		if ((flags & FlagPadGeodesic))
		{
			for (int i = 0; i < 6; i++)
			{
				if (m_mesh_options.upper_flags[i])
				{
					upper_geodesic_attribus[i].clear();
				}

				if (m_mesh_options.lower_flags[i])
				{
					lower_geodesic_attribus[i].clear();
				}
			}
		}

		if ((flags & FlagPadOutline))
		{
			for (int i = 0; i < 6; i++)
			{
				if (m_mesh_options.upper_flags[i])
				{
					upper_pad_outlines_attribus[2 * i].clear();
					upper_pad_outlines_attribus[2 * i + 1].clear();
				}

				if (m_mesh_options.lower_flags[i])
				{
					lower_pad_outlines_attribus[2 * i].clear();
					lower_pad_outlines_attribus[2 * i + 1].clear();
				}
			}
		}

		if ((flags & FlagToothCut))
		{
			for (int i = 0; i < teeth_count; i++)
			{
				if (m_mesh_options.upper_flags[i])
				{
					upper_tooth_bounds_attribus[i].clear();
					upper_tooth_cut_points_attribus[i].clear();
					upper_tooth_cut_planes_attribus[2 * i + 0].clear();
					upper_tooth_cut_planes_attribus[2 * i + 1].clear();
				}

				if (m_mesh_options.lower_flags[i])
				{
					lower_tooth_bounds_attribus[i].clear();
					lower_tooth_cut_points_attribus[i].clear();
					lower_tooth_cut_planes_attribus[2 * i + 0].clear();
					lower_tooth_cut_planes_attribus[2 * i + 1].clear();
				}
			}
		}

		if ((flags & FlagPairCut))
		{
			for (int i = 0; i < pair_count; i++)
			{
				if (m_mesh_options.upper_flags[i] && m_mesh_options.upper_flags[i + 1])
				{
					upper_pair_fitting_planes_attribus[2 * i + 0].clear();
					upper_pair_fitting_planes_attribus[2 * i + 1].clear();
					upper_pair_fitting_lines_attribus[i].clear();
					upper_pair_fitting_points_attribus[i].clear();
					upper_pair_cut_planes_attribus[2 * i + 0].clear();
					upper_pair_cut_planes_attribus[2 * i + 1].clear();
					upper_pair_wires_attribus[i].clear();
				}

				if (m_mesh_options.lower_flags[i] && m_mesh_options.lower_flags[i + 1])
				{
					lower_pair_fitting_planes_attribus[2 * i + 0].clear();
					lower_pair_fitting_planes_attribus[2 * i + 1].clear();
					lower_pair_fitting_lines_attribus[i].clear();
					lower_pair_fitting_points_attribus[i].clear();
					lower_pair_cut_planes_attribus[2 * i + 0].clear();
					lower_pair_cut_planes_attribus[2 * i + 1].clear();
					lower_pair_wires_attribus[i].clear();
				}
			}
		}

		if ((flags & FlagPairWire))
		{
			for (int i = 0; i < pair_count; i++)
			{
				if (m_mesh_options.upper_flags[i] && m_mesh_options.upper_flags[i + 1])
				{
					upper_wire_shapes_attribus[2 * i + 0].clear();
					upper_wire_shapes_attribus[2 * i + 1].clear();
					upper_wire_extrude_shapes_attribus[2 * i + 0].clear();
					upper_wire_extrude_shapes_attribus[2 * i + 1].clear();
				}

				if (m_mesh_options.lower_flags[i] && m_mesh_options.lower_flags[i + 1])
				{
					lower_wire_shapes_attribus[2 * i + 0].clear();
					lower_wire_shapes_attribus[2 * i + 1].clear();
					lower_wire_extrude_shapes_attribus[2 * i + 0].clear();
					lower_wire_extrude_shapes_attribus[2 * i + 1].clear();
				}
			}
		}

		if ((flags & FlagPadShape))
		{
			for (int i = 0; i < teeth_count; i++)
			{
				if (m_mesh_options.upper_flags[i])
				{
					upper_pad_shapes_attribus[2 * i + 0].clear();
					upper_pad_shapes_attribus[2 * i + 1].clear();
				}

				if (m_mesh_options.lower_flags[i])
				{
					lower_pad_shapes_attribus[2 * i + 0].clear();
					lower_pad_shapes_attribus[2 * i + 1].clear();
				}
			}
		}

		//are_buffers_initialized = false;
	}

	void Project::clear_data()
	{
		m_func.reset();
		clear_elements(FlagAll);
	}

	/******************* Function **********************/

	void Project::compute_laplacian_based_inside()
	{
		m_func.compute_laplacian_based_inside(m_mesh_options.inside_smooth_range,
			m_mesh_options.inside_laplacian_param,
			m_mesh_options.inside_isovalue);

		compute_elements(FlagInside);
	}

	void Project::compute_laplacian_based_convexity()
	{
		m_func.compute_laplacian_based_convexity(m_mesh_options.convex_neighbor_radius,
			m_mesh_options.convex_smooth_range,
			m_mesh_options.convex_laplacian_param);

		compute_elements(FlagConvex | FlagConvexIso);
	}

	void Project::update_convexity_function_isovalue()
	{
		compute_elements(FlagConvexIso);
	}

	void Project::compute_selected_segmentation()
	{
		m_func.compute_selected_segmentation(m_select_pos,
			m_mesh_options.convex_isovalue);

		compute_elements(FlagSegment);
	}

	void Project::clear_selected_points()
	{
		m_select_pos.clear();
		selected_points.clear();
		//are_buffers_initialized = false;
	}

	void Project::validate_selected_segmentation()
	{
		int status = m_func.validate_selected_segmentation();

		if (status == 0)
			return;

		if (status == 1)
			compute_elements(FlagSegment | FlagValid | FlagUpper);
		else if (status == -1)
			compute_elements(FlagSegment | FlagValid | FlagLower);
	}

	void Project::compute_geodesic_pad_outlines()
	{
		for (int i = 0; i < m_mesh_options.upper_flags.size(); i++)
		{
			if (m_mesh_options.upper_flags[i])
				m_func.compute_geodesic_pad_outlines(i, false, m_mesh_options.geodesic_step, m_mesh_options.geodesic_isovalue);
		}

		for (int i = 0; i < m_mesh_options.lower_flags.size(); i++)
		{
			if (m_mesh_options.lower_flags[i])
				m_func.compute_geodesic_pad_outlines(i, true, m_mesh_options.geodesic_step, m_mesh_options.geodesic_isovalue);
		}

		compute_elements(FlagPadGeodesic | FlagPadOutline);
	}

	void Project::recompute_geodesic_pad_outlines()
	{
		for (int i = 0; i < m_mesh_options.upper_flags.size(); i++)
		{
			if (m_mesh_options.upper_flags[i])
				m_func.recompute_geodesic_pad_outlines(i, m_mesh_options.geodesic_isovalue, false);
		}

		for (int i = 0; i < m_mesh_options.lower_flags.size(); i++)
		{
			if (m_mesh_options.lower_flags[i])
				m_func.recompute_geodesic_pad_outlines(i, m_mesh_options.geodesic_isovalue, true);
		}

		compute_elements(FlagPadOutline);
	}

	void Project::compute_tooth_split_planes()
	{
		for (int i = 0; i < m_mesh_options.upper_flags.size(); i++)
		{
			if (m_mesh_options.upper_flags[i])
				m_func.compute_tooth_cut_plane(i, false);
		}

		for (int i = 0; i < m_mesh_options.lower_flags.size(); i++)
		{
			if (m_mesh_options.lower_flags[i])
				m_func.compute_tooth_cut_plane(i, true);
		}

		compute_elements(FlagToothCut);
	}

	void Project::compute_paired_wires()
	{
		for (int i = 0; i < pair_count; i++)
		{
			if (m_mesh_options.upper_flags[i] && m_mesh_options.upper_flags[i + 1])
				m_func.compute_paired_wires(i, false, m_mesh_options.pair_distance);

			if (m_mesh_options.lower_flags[i] && m_mesh_options.lower_flags[i + 1])
				m_func.compute_paired_wires(i, true, m_mesh_options.pair_distance);
		}

		compute_elements(FlagPairCut);
	}

	void Project::compute_wire_shapes()
	{
		for (int i = 0; i < pair_count; i++)
		{
			if (m_mesh_options.upper_flags[i] && m_mesh_options.upper_flags[i + 1])
				m_func.compute_wire_shapes(i, false, m_mesh_options.geodesic_step, m_mesh_options.pair_width);

			if (m_mesh_options.lower_flags[i] && m_mesh_options.lower_flags[i + 1])
				m_func.compute_wire_shapes(i, true, m_mesh_options.geodesic_step, m_mesh_options.pair_width);
		}

		compute_elements(FlagPairWire);
	}

	void Project::compute_pad_shapes()
	{
		for (int i = 0; i < teeth_count; i++)
		{
			if (m_mesh_options.upper_flags[i])
				m_func.compute_pad_shapes(i, false, m_mesh_options.pad_height, m_mesh_options.pad_wire_height);

			if (m_mesh_options.lower_flags[i])
				m_func.compute_pad_shapes(i, true, m_mesh_options.pad_height, m_mesh_options.pad_wire_height);
		}

		compute_elements(FlagPadShape);
	}

	/******************* Options **********************/

	void Project::setInput_point_size(double d)
	{
		m_mesh_options.input_point_size = (float)d;
	}

	void Project::setColor_point_size(double d)
	{
		m_mesh_options.color_point_size = (float)d;
	}

	void Project::setMesh_edge_width(double d)
	{
		m_mesh_options.mesh_edge_width = (float)d;
	}

	void Project::setVector_ratio(double d)
	{
		m_mesh_options.vector_ratio = (float)d;
	}

	void Project::setInside_laplacian_parameter(double d)
	{
		m_mesh_options.inside_laplacian_param = d;
	}

	void Project::setInside_smooth_range(int i)
	{
		m_mesh_options.inside_smooth_range = i;
	}

	void Project::setInside_isovalue(double d)
	{
		m_mesh_options.inside_isovalue = d;
	}

	void Project::setConvex_neighbor_radius(double d)
	{
		m_mesh_options.convex_neighbor_radius = d;
	}

	void Project::setConvex_smooth_range(int i)
	{
		m_mesh_options.convex_smooth_range = i;
	}

	void Project::setConvex_laplacian_thresh(double d)
	{
		m_mesh_options.convex_laplacian_param = d;
	}

	void Project::setConvex_isovalue(double d)
	{
		m_mesh_options.convex_isovalue = d;
	}

	void Project::setGeodesic_step(int i)
	{
		m_mesh_options.geodesic_step = i;
	}

	void Project::setGeodesic_isovalue(double d)
	{
		m_mesh_options.geodesic_isovalue = d;
	}

	void Project::setUpperFlag(int ind, int status)
	{
		m_mesh_options.upper_flags[ind] = (bool)status;
	}

	void Project::setLowerFlag(int ind, int status)
	{
		m_mesh_options.lower_flags[ind] = (bool)status;
	}

	void Project::setPair_distance(double d)
	{
		m_mesh_options.pair_distance = d;
	}

	void Project::setPair_width(double d)
	{
		m_mesh_options.pair_width = d;
	}

	void Project::setPad_height(double d)
	{
		m_mesh_options.pad_height = d;
	}

	void Project::setPad_wire_height(double d)
	{
		m_mesh_options.pad_wire_height = d;
	}

}
