#include "scene.h"
//#include "color.h"

// // Qt
// #include <QBarSet>
// #include <QBarSeries>
// #include <QStringList>
// #include <QBarCategoryAxis>
// #include <QLineSeries>
// #include <QMatrix4x4>
// #include <QtCharts>
// #include <QOpenGLWidget>
// #include <QPainter>
// #include <QAreaSeries>

// // OpenGL
// #include <QOpenGLShader>
void Scene::_bind_methods() {
	ClassDB::bind_method(D_METHOD("load_upper_teeth", "file_path"), &Scene::load_upper_teeth);
	ClassDB::bind_method(D_METHOD("load_lower_teeth", "file_path"), &Scene::load_lower_teeth);
	ClassDB::bind_method(D_METHOD("compute_laplacian_based_inside"), &Scene::compute_laplacian_based_inside);
	ClassDB::bind_method(D_METHOD("compute_laplacian_based_convexity"), &Scene::compute_laplacian_based_convexity);
	ClassDB::bind_method(D_METHOD("compute_selected_segmentation"), &Scene::compute_selected_segmentation);
	ClassDB::bind_method(D_METHOD("update_convexity_function_isovalue"), &Scene::update_convexity_function_isovalue);
	ClassDB::bind_method(D_METHOD("validate_selected_segmentation"), &Scene::validate_selected_segmentation);
	ClassDB::bind_method(D_METHOD("clear_selected_points"), &Scene::clear_selected_points);
	ClassDB::bind_method(D_METHOD("compute_geodesic_pad_outlines"), &Scene::compute_geodesic_pad_outlines);
	ClassDB::bind_method(D_METHOD("recompute_geodesic_pad_outlines"), &Scene::recompute_geodesic_pad_outlines);
	ClassDB::bind_method(D_METHOD("compute_tooth_split_planes"), &Scene::compute_tooth_split_planes);
	ClassDB::bind_method(D_METHOD("compute_paired_wires"), &Scene::compute_paired_wires);
	ClassDB::bind_method(D_METHOD("compute_wire_shapes"), &Scene::compute_wire_shapes);
}

Scene::Scene()
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

	for(int i = 0; i < 2 * teeth_count; i++)
    {
        upper_seg_attribus.push_back(DataList());
        lower_seg_attribus.push_back(DataList());
    }

    for(int i = 0; i < teeth_count; i++)
    {
        upper_geodesic_attribus.push_back(DataList());
        lower_geodesic_attribus.push_back(DataList());
    }

	for(int i = 0; i < 2 * teeth_count; i++)
    {
        upper_pad_outlines_attribus.push_back(DataList());
        lower_pad_outlines_attribus.push_back(DataList());
    }

    for(int i = 0; i < teeth_count; i++)
    {
        upper_tooth_bounds_attribus.push_back(DataList());
        lower_tooth_bounds_attribus.push_back(DataList());
        upper_tooth_cut_points_attribus.push_back(DataList());
        lower_tooth_cut_points_attribus.push_back(DataList());
    }

    for(int i = 0; i < 2 * teeth_count; i++)
    {
        upper_tooth_cut_planes_attribus.push_back(DataList());
        lower_tooth_cut_planes_attribus.push_back(DataList());
    }

    for(int i = 0; i < 2 * pair_count; i++)
    {
        upper_pair_fitting_planes_attribus.push_back(DataList());
        lower_pair_fitting_planes_attribus.push_back(DataList());

        upper_pair_cut_planes_attribus.push_back(DataList());
        lower_pair_cut_planes_attribus.push_back(DataList());
    }

    for(int i = 0; i < pair_count; i++)
    {
        upper_pair_fitting_lines_attribus.push_back(DataList());
        lower_pair_fitting_lines_attribus.push_back(DataList());

        upper_pair_fitting_points_attribus.push_back(DataList());
        lower_pair_fitting_points_attribus.push_back(DataList());

        upper_pair_wires_attribus.push_back(DataList());
        lower_pair_wires_attribus.push_back(DataList());
    }

    for(int i = 0; i < 2 * pair_count; i++)
    {
        upper_wire_shapes_attribus.push_back(DataList());
        lower_wire_shapes_attribus.push_back(DataList());
        upper_wire_extrude_shapes_attribus.push_back(DataList());
        lower_wire_extrude_shapes_attribus.push_back(DataList());
    }

    
    // default params 
    gl_init = false;
}

Scene::~Scene()
{
    // for(int i = 0; i < 158; i++)
    //     vao[i].destroy();

    // for(int i = 0; i < 247; i++)
    //     buffers[i].destroy();
}

/******************* Visualization **********************/

// void Scene::toggle_view(SceneObjectType type) {
// 	m_view[type] = !m_view[type];
// }


// void Scene::render_program_point(QOpenGLVertexArrayObject& vao, bool visible, const DataList data, QColor color)
// {
// 	if(visible && data.size() > 0)
// 	{
// 		vao.bind();
// 		rendering_program.bind();
// 		rendering_program.setUniformValue(colorLocation, color);
// 		rendering_program.setUniformValue("point_size", m_mesh_options.color_point_size);
// 		gl->glDrawArrays(GL_POINTS, 0, static_cast<GLsizei>(data.size() / 3));
// 		rendering_program.release();
// 		vao.release();
// 	}
// }

// void Scene::render_program_line(QOpenGLVertexArrayObject& vao, bool visible, const DataList data, QColor color)
// {
// 	if(visible && data.size() > 0)
// 	{
// 		vao.bind();
// 		rendering_program.bind();
// 		rendering_program.setUniformValue(colorLocation, color);
// 		gl->glLineWidth(m_mesh_options.mesh_edge_width); 
// 		gl->glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(data.size() / 3));
// 		rendering_program.release();
// 		vao.release();
// 	}
// }


// void Scene::render_program_mesh(QOpenGLVertexArrayObject& vao, bool visibile, const DataList data, QColor color, QColor line_color)
// {
// 	if(visibile && data.size() > 0)
// 	{
// 		vao.bind();
//         gl->glEnable(GL_BLEND);
//         gl->glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
// 		rendering_program_text.bind();
// 		rendering_program_text.setUniformValue(colorLineLocation_text, line_color);
// 		rendering_program_text.setUniformValue(widthLineLocation_text, m_mesh_options.mesh_edge_width);
// 		rendering_program_text.setUniformValue(colorLocation_text, color);
// 		gl->glDrawArrays(GL_TRIANGLES, 0, static_cast<GLsizei>(data.size() / 3));
// 		rendering_program_text.release();
// 		vao.release();
// 	}
// }

// void Scene::render_program_function(QOpenGLVertexArrayObject& vao, bool visible, const DataList data, QColor line_color)
// {
// 	if(visible && data.size() > 0)
// 	{
// 		vao.bind();
// 		rendering_program_facet.bind();
// 		rendering_program_facet.setUniformValue(colorLineLocation_text, line_color);
// 		rendering_program_facet.setUniformValue(widthLineLocation_text, m_mesh_options.mesh_edge_width);
// 		gl->glDrawArrays(GL_TRIANGLES, 0, static_cast<GLsizei>(data.size() / 3));
// 		rendering_program_facet.release();
// 		vao.release();
// 	}
// }

// void Scene::render(CGAL::QGLViewer* viewer)
// {
//     if(!gl_init)
//         init_gl();

//     if(!are_buffers_initialized)
//         initialize_buffers();

//     gl->glEnable(GL_DEPTH_TEST);
    

//     attrib_buffers(viewer);

//     for(int i = 0; i < pair_count; i++)
//     {
//         render_program_mesh(vao[wire_extrude_shapes_vao_index + i], m_view[ScenePairedWireExtrudeShapes] && m_mesh_options.upper_flags[i] && m_mesh_options.upper_flags[i+1],
// 		                    upper_wire_extrude_shapes_attribus[2 * i], Color(0.1f, 0.5f, 0.3f), Color(0.f, 0.f, 0.f));
// 		render_program_mesh(vao[wire_extrude_shapes_vao_index + pair_count + i], m_view[ScenePairedWireExtrudeShapes] && m_mesh_options.lower_flags[i] && m_mesh_options.lower_flags[i+1],
// 		                    lower_wire_extrude_shapes_attribus[2 * i], Color(0.1f, 0.5f, 0.3f), Color(0.f, 0.f, 0.f));
//         render_program_mesh(vao[wire_shapes_vao_index + i], m_view[ScenePairedWireShapes] && m_mesh_options.upper_flags[i] && m_mesh_options.upper_flags[i+1],
// 		                    upper_wire_shapes_attribus[2 * i], Color(0.5f, 0.1f, 0.3f, 0.5f), Color(0.f, 0.f, 0.f, 0.5f));
// 		render_program_mesh(vao[wire_shapes_vao_index + pair_count + i], m_view[ScenePairedWireShapes] && m_mesh_options.lower_flags[i] && m_mesh_options.lower_flags[i+1],
// 		                    lower_wire_shapes_attribus[2 * i], Color(0.5f, 0.1f, 0.3f, 0.5f), Color(0.f, 0.f, 0.f, 0.5f));

//         render_program_line(vao[pair_wires_vao_index + i], m_view[ScenePairedWires] && m_mesh_options.upper_flags[i] && m_mesh_options.upper_flags[i+1], 
//                             upper_pair_wires_attribus[i], Color(1.0f, 0.0f, 0.f));
//         render_program_line(vao[pair_wires_vao_index + pair_count + i], m_view[ScenePairedWires] && m_mesh_options.lower_flags[i] && m_mesh_options.lower_flags[i+1], 
//                             lower_pair_wires_attribus[i], Color(1.0f, 0.0f, 0.f));

//         render_program_mesh(vao[pair_cut_planes_vao_index + i], m_view[ScenePairedPlanes] && m_mesh_options.upper_flags[i] && m_mesh_options.upper_flags[i+1],
// 		                    upper_pair_cut_planes_attribus[2 * i], Color(0.f, 0.8f, 0.8f, 0.5f), Color(0.f, 0.f, 0.f, 0.5f));
// 		render_program_mesh(vao[pair_cut_planes_vao_index + pair_count + i], m_view[ScenePairedPlanes] && m_mesh_options.lower_flags[i] && m_mesh_options.lower_flags[i+1],
// 		                    lower_pair_cut_planes_attribus[2 * i], Color(0.f, 0.8f, 0.8f, 0.5f), Color(0.f, 0.f, 0.f, 0.5f));

//         render_program_point(vao[pair_fitting_points_vao_index + i], m_view[ScenePairedFittingPoints] && m_mesh_options.upper_flags[i] && m_mesh_options.upper_flags[i+1], 
//                             upper_pair_fitting_points_attribus[i], Color(1.f, 0.f, 1.f));
//         render_program_point(vao[pair_fitting_points_vao_index + pair_count + i], m_view[ScenePairedFittingPoints] && m_mesh_options.lower_flags[i] && m_mesh_options.lower_flags[i+1], 
//                             lower_pair_fitting_points_attribus[i], Color(1.f, 0.f, 1.f));

//         render_program_line(vao[pair_fitting_lines_vao_index + i], m_view[ScenePairedFittingLines] && m_mesh_options.upper_flags[i] && m_mesh_options.upper_flags[i+1], 
//                             upper_pair_fitting_lines_attribus[i], Color(0.0f, 0.0f, 1.0f));
//         render_program_line(vao[pair_fitting_lines_vao_index + pair_count + i], m_view[ScenePairedFittingLines] && m_mesh_options.lower_flags[i] && m_mesh_options.lower_flags[i+1], 
//                             lower_pair_fitting_lines_attribus[i], Color(0.0f, 0.0f, 1.0f));

//         render_program_mesh(vao[pair_fitting_planes_vao_index + i], m_view[ScenePairedFittingPlanes] && m_mesh_options.upper_flags[i] && m_mesh_options.upper_flags[i+1],
// 		                    upper_pair_fitting_planes_attribus[2 * i], Color(0.8f, 0.f, 0.f, 0.5f), Color(0.f, 0.f, 0.f, 0.5f));
// 		render_program_mesh(vao[pair_fitting_planes_vao_index + pair_count + i], m_view[ScenePairedFittingPlanes] && m_mesh_options.lower_flags[i] && m_mesh_options.lower_flags[i+1],
// 		                    lower_pair_fitting_planes_attribus[2 * i], Color(0.8f, 0.f, 0.f, 0.5f), Color(0.f, 0.f, 0.f, 0.5f));
//     }

// 	for(int i = 0; i < teeth_count; i++)
// 	{
//         render_program_mesh(vao[tooth_cut_planes_vao_index + i], m_view[SceneToothCutPlane] && m_mesh_options.upper_flags[i],
// 		                    upper_tooth_cut_planes_attribus[2 * i], Color(0.5f, 0.8f, 0.f, 0.5f), Color(0.f, 0.f, 0.f));
// 		render_program_mesh(vao[tooth_cut_planes_vao_index + teeth_count + i], m_view[SceneToothCutPlane] && m_mesh_options.lower_flags[i],
// 		                    lower_tooth_cut_planes_attribus[2 * i], Color(0.5f, 0.8f, 0.f, 0.5f), Color(0.f, 0.f, 0.f));

//         render_program_point(vao[tooth_cut_points_vao_index + i], m_view[SceneToothCutPoints] && m_mesh_options.upper_flags[i], 
//                             upper_tooth_cut_points_attribus[i], Color(0.5f, 0.8f, 0.f));
//         render_program_point(vao[tooth_cut_points_vao_index + teeth_count + i], m_view[SceneToothCutPoints] && m_mesh_options.lower_flags[i], 
//                             lower_tooth_cut_points_attribus[i], Color(0.5f, 0.8f, 0.f));

//         render_program_line(vao[tooth_bounds_vao_index + i], m_view[SceneToothBound] && m_mesh_options.upper_flags[i], 
//                             upper_tooth_bounds_attribus[i], Color(0.0f, 0.0f, 1.0f));
//         render_program_line(vao[tooth_bounds_vao_index + teeth_count + i], m_view[SceneToothBound] && m_mesh_options.lower_flags[i], 
//                             lower_tooth_bounds_attribus[i], Color(0.0f, 0.0f, 1.0f));

// 		render_program_mesh(vao[pad_outlines_vao_index + i], m_view[ScenePadOutlines] && m_mesh_options.upper_flags[i],
// 		                    upper_pad_outlines_attribus[2 * i], Color(0.f, 0.8f, 0.8f), Color(0.f, 0.f, 0.f));
// 		render_program_mesh(vao[pad_outlines_vao_index + teeth_count + i], m_view[ScenePadOutlines] && m_mesh_options.lower_flags[i],
// 		                    lower_pad_outlines_attribus[2 * i], Color(0.f, 0.8f, 0.8f), Color(0.f, 0.f, 0.f));

// 		render_program_function(vao[geodesic_vao_index + i], m_view[SceneGeodesicDistance] && m_mesh_options.upper_flags[i], 
// 		                        upper_geodesic_attribus[i], Color(0.f, 0.f, 0.f));
// 		render_program_function(vao[geodesic_vao_index + teeth_count + i], m_view[SceneGeodesicDistance] && m_mesh_options.lower_flags[i],
// 		                        lower_geodesic_attribus[i], Color(0.f, 0.f, 0.f));

// 		render_program_mesh(vao[seg_attribus_vao_index + i], m_view[SceneValidatedSegmentation] && m_mesh_options.upper_flags[i],
// 		                    upper_seg_attribus[2 * i], Color(1.f, 0.8f, 0.f), Color(0.f, 0.f, 0.f));
// 		render_program_mesh(vao[seg_attribus_vao_index + teeth_count + i], m_view[SceneValidatedSegmentation] && m_mesh_options.lower_flags[i],
// 		                    lower_seg_attribus[2 * i], Color(1.f, 0.8f, 0.f), Color(0.f, 0.f, 0.f));
// 	}

// 	render_program_mesh(vao[15], m_view[SceneSegmentation], selected_seg_facets, Color(0.5f, 0.0f, 0.f), Color(0.f, 0.f, 0.f));
// 	render_program_point(vao[14], m_view[SceneSelection], selected_points, Color(1.0f, 0.0f, 0.0f));
// 	render_program_line(vao[13], m_view[SceneLowerConvexityIsoEdges], lower_convex_iso_edges, Color(0.0f, 0.0f, 0.0f));
// 	render_program_line(vao[12], m_view[SceneUpperConvexityIsoEdges], upper_convex_iso_edges, Color(0.0f, 0.0f, 0.0f));
	
// 	render_program_function(vao[11], m_view[SceneLowerLaplacianConvexity], lower_convex_func_colors, Color(0.f, 0.f, 0.f));
// 	render_program_function(vao[10], m_view[SceneLowerLocalConvexity], lower_local_convex_facets, Color(0.f, 0.f, 0.f));
// 	render_program_function(vao[9], m_view[SceneUpperLaplacianConvexity], upper_convex_func_colors, Color(0.f, 0.f, 0.f));
// 	render_program_function(vao[8], m_view[SceneUpperLocalConvexity], upper_local_convex_facets, Color(0.f, 0.f, 0.f));
	
// 	render_program_mesh(vao[7], m_view[SceneLowerInsideFaces], lower_inside_facets, Color(0.0f, 0.8f, 0.0f), Color(0.f, 0.f, 0.f));
// 	render_program_function(vao[6], m_view[SceneLowerInsideFunction], lower_inside_func_colors, Color(0.f, 0.f, 0.f));
// 	render_program_function(vao[5], m_view[SceneLowerInsideRay], lower_inside_ray_facets, Color(0.f, 0.f, 0.f));

// 	render_program_mesh(vao[4], m_view[SceneUpperInsideFaces], upper_inside_facets, Color(0.0f, 0.8f, 0.0f), Color(0.f, 0.f, 0.f));
// 	render_program_function(vao[3], m_view[SceneUpperInsideFunction], upper_inside_func_colors, Color(0.f, 0.f, 0.f));
// 	render_program_function(vao[2], m_view[SceneUpperInsideRay], upper_inside_ray_facets, Color(0.f, 0.f, 0.f));
    
// 	render_program_mesh(vao[1], m_view[SceneLowerTeeth], lower_teeth_facets, Color(0.5f, 0.5f, 1.f), Color(0.f, 0.f, 0.f));
// 	render_program_mesh(vao[0], m_view[SceneUpperTeeth], upper_teeth_facets, Color(0.5f, 0.5f, 1.f), Color(0.f, 0.f, 0.f));

// }

/******************* I/O **********************/

void Scene::load_upper_teeth(const  String &filename)
{
    bool flag = false;

	//if (filename.endsWith(".stl", Qt::CaseInsensitive))
        flag = m_func.load_upper_teeth_mesh((filename.utf8().get_data()));
    //else
    //    std::cout << "Format not supported!" << std::endl;

    if(flag)
        compute_elements(FlagMesh | FlagUpper);
}

void Scene::load_lower_teeth(const String &filename)
{
    bool flag = false;

	//if (filename.endsWith(".stl", Qt::CaseInsensitive))
        flag = m_func.load_lower_teeth_mesh((filename.utf8().get_data()));
    //else
    //    std::cout << "Format not supported!" << std::endl;

    if(flag)
        compute_elements(FlagMesh | FlagLower);
}

void Scene::get_bounding_shape(double& x, double& y, double& z, double& r)
{
    Point center = m_func.get_center();
    r = m_func.get_radius();
    x = center.x();
    y = center.y();
    z = center.z();
}

/******************* Update **********************/
void Scene::set_mouse_pos(double x, double y, double z, double vx, double vy, double vz) 
{  
    m_mouse_pos = dentist::Point(x, y, z); 
    m_mouse_vec = dentist::Vector(vx, vy, vz);
}

void Scene::update_mouse_selection()
{
    bool flag = m_func.update_selection(m_mouse_pos, m_mouse_vec, m_select_pos, selected_points);
    
    if(flag)
        are_buffers_initialized = false;
}

/******************* OpenGL **********************/
// void Scene::init_gl()
// {
//     gl = new QOpenGLFunctions();
//     gl->initializeOpenGLFunctions();

//     compile_shaders();
//     gl_init = true;
// }

// void Scene::compile_shaders()
// {
//     for(int i = 0; i < 247; i++)
//         buffers[i].create();

//     for(int i = 0; i < 158; i++)
//         vao[i].create();
    
//     //----------- Point / Vector Rendering Program -------------//
//     QOpenGLShader *vertex_shader = new QOpenGLShader(QOpenGLShader::Vertex);
//     if(!vertex_shader->compileSourceFile(shader_path+"point.vs"))
//         std::cerr << "Compiling vertex source failed" << std::endl;
//     QOpenGLShader *fragment_shader= new QOpenGLShader(QOpenGLShader::Fragment);
//     if(!fragment_shader->compileSourceFile(shader_path+"point.fs"))
//         std::cerr << "Compiling fragment source FAILED" << std::endl;
    
//     if(!rendering_program.addShader(vertex_shader))
//         std::cerr << "Adding vertex shader failed" << std::endl;
//     if(!rendering_program.addShader(fragment_shader))
//         std::cerr << "adding fragment shader FAILED" << std::endl;

//     if(!rendering_program.link())
//         std::cerr << "linking Program FAILED" << std::endl;
    
//     rendering_program.bind();

//     //----------- Facet Rendering Program -------------//
//     QOpenGLShader *vertex_shader_facet = new QOpenGLShader(QOpenGLShader::Vertex);
//     if(!vertex_shader_facet->compileSourceFile(shader_path+"facet.vs"))
//         std::cerr << "Compiling vertex source facet failed" << std::endl;
//     QOpenGLShader *geometry_shader_facet = new QOpenGLShader(QOpenGLShader::Geometry);
//     if(!geometry_shader_facet->compileSourceFile(shader_path+"facet.gs"))
//         std::cerr << "Compiling geometry source facet failed" << std::endl;
//     QOpenGLShader *fragment_shader_facet = new QOpenGLShader(QOpenGLShader::Fragment);
//     if(!fragment_shader_facet->compileSourceFile(shader_path+"facet.fs"))
//         std::cerr << "Compiling fragment source facet FAILED" << std::endl;
    
//     if(!rendering_program_text.addShader(vertex_shader_facet))
//         std::cerr << "Adding vertex shader facet failed" << std::endl;
//     if(!rendering_program_text.addShader(geometry_shader_facet))
//         std::cerr << "Adding geometry shader facet failed" << std::endl;
//     if(!rendering_program_text.addShader(fragment_shader_facet))
//         std::cerr << "adding fragment shader facet FAILED" << std::endl;

//     if(!rendering_program_text.link())
//         std::cerr << "linking facet Program FAILED" << std::endl;
    
//     rendering_program_text.bind();

//     //----------- Function Rendering Program -------------//
//     QOpenGLShader *vertex_shader_func = new QOpenGLShader(QOpenGLShader::Vertex);
//     if(!vertex_shader_func->compileSourceFile(shader_path+"func.vs"))
//         std::cerr << "Compiling vertex source failed" << std::endl;
//     QOpenGLShader *fragment_shader_func = new QOpenGLShader(QOpenGLShader::Fragment);
//     if(!fragment_shader_func->compileSourceFile(shader_path+"func.fs"))
//         std::cerr << "Compiling fragment source FAILED" << std::endl;
    
//     if(!rendering_program_func.addShader(vertex_shader_func))
//         std::cerr << "Adding vertex shader failed" << std::endl;
//     if(!rendering_program_func.addShader(fragment_shader_func))
//         std::cerr << "adding fragment shader FAILED" << std::endl;

//     if(!rendering_program_func.link())
//         std::cerr << "linking Program FAILED" << std::endl;
    
//     rendering_program_func.bind();

//     //----------- Colored Facet Rendering Program -------------//
//     QOpenGLShader *vertex_shader_facet_color = new QOpenGLShader(QOpenGLShader::Vertex);
//     if(!vertex_shader_facet_color->compileSourceFile(shader_path+"facet_color.vs"))
//         std::cerr << "Compiling vertex source facet color failed" << std::endl;
//     QOpenGLShader *geometry_shader_facet_color = new QOpenGLShader(QOpenGLShader::Geometry);
//     if(!geometry_shader_facet_color->compileSourceFile(shader_path+"facet_color.gs"))
//         std::cerr << "Compiling geometry source facet color failed" << std::endl;
//     QOpenGLShader *fragment_shader_facet_color = new QOpenGLShader(QOpenGLShader::Fragment);
//     if(!fragment_shader_facet_color->compileSourceFile(shader_path+"facet_color.fs"))
//         std::cerr << "Compiling fragment source facet color FAILED" << std::endl;
    
//     if(!rendering_program_facet.addShader(vertex_shader_facet_color))
//         std::cerr << "Adding vertex shader facet color failed" << std::endl;
//     if(!rendering_program_facet.addShader(geometry_shader_facet_color))
//         std::cerr << "Adding geometry shader facet color failed" << std::endl;
//     if(!rendering_program_facet.addShader(fragment_shader_facet_color))
//         std::cerr << "adding fragment shader facet color FAILED" << std::endl;

//     if(!rendering_program_facet.link())
//         std::cerr << "linking facet color Program FAILED" << std::endl;
    
//     rendering_program_facet.bind();
// }

// void Scene::init_buffer(QOpenGLVertexArrayObject& vao, QOpenGLBuffer& buffer, QOpenGLShaderProgram& program, const DataList& data, const char* attribute_name)
// {
// 	vao.bind();

// 	buffer.bind();
// 	buffer.allocate(data.data(), static_cast<int>(data.size() * sizeof(float)));

// 	int location = program.attributeLocation(attribute_name);
// 	program.bind();
// 	program.enableAttributeArray(location);
// 	program.setAttributeBuffer(location, GL_FLOAT, 0, 3);
// 	program.release();

// 	buffer.release();
    
// 	vao.release();
// }

// void Scene::initialize_buffers()
// {
//     // Upper Teeth Mesh
// 	init_buffer(vao[0], buffers[0], rendering_program_text, upper_teeth_facets, "vertex");
// 	init_buffer(vao[0], buffers[1], rendering_program_text, upper_teeth_normals, "normal");

//     // Lower Teeth Mesh
// 	init_buffer(vao[1], buffers[2], rendering_program_text, lower_teeth_facets, "vertex");
// 	init_buffer(vao[1], buffers[3], rendering_program_text, lower_teeth_normals, "normal");

// 	// Upper inside ray
// 	init_buffer(vao[2], buffers[4], rendering_program_facet, upper_inside_ray_facets, "vertex");
// 	init_buffer(vao[2], buffers[5], rendering_program_facet, upper_inside_ray_normals, "normal");
// 	init_buffer(vao[2], buffers[6], rendering_program_facet, upper_inside_ray_colors, "color");

// 	// Upper inside function
// 	init_buffer(vao[3], buffers[4], rendering_program_facet, upper_inside_ray_facets, "vertex");
// 	init_buffer(vao[3], buffers[5], rendering_program_facet, upper_inside_ray_normals, "normal");
// 	init_buffer(vao[3], buffers[7], rendering_program_facet, upper_inside_func_colors, "color");

// 	// Upper Inside Surface
// 	init_buffer(vao[4], buffers[8], rendering_program_text, upper_inside_facets, "vertex");
// 	init_buffer(vao[4], buffers[9], rendering_program_text, upper_inside_normals, "normal");

// 	// Lower inside ray
// 	init_buffer(vao[5], buffers[10], rendering_program_facet, lower_inside_ray_facets, "vertex");
// 	init_buffer(vao[5], buffers[11], rendering_program_facet, lower_inside_ray_normals, "normal");
// 	init_buffer(vao[5], buffers[12], rendering_program_facet, lower_inside_ray_colors, "color");

// 	// Lower inside function
// 	init_buffer(vao[6], buffers[10], rendering_program_facet, lower_inside_ray_facets, "vertex");
// 	init_buffer(vao[6], buffers[11], rendering_program_facet, lower_inside_ray_normals, "normal");
// 	init_buffer(vao[6], buffers[13], rendering_program_facet, lower_inside_func_colors, "color");

// 	// Lower Inside Surface
// 	init_buffer(vao[7], buffers[14], rendering_program_text, lower_inside_facets, "vertex");
// 	init_buffer(vao[7], buffers[15], rendering_program_text, lower_inside_normals, "normal");

// 	// Upper local convexity
// 	init_buffer(vao[8], buffers[16], rendering_program_facet, upper_local_convex_facets, "vertex");
// 	init_buffer(vao[8], buffers[17], rendering_program_facet, upper_local_convex_normals, "normal");
// 	init_buffer(vao[8], buffers[18], rendering_program_facet, upper_local_convex_colors, "color");

// 	// Upper laplacian convexity
// 	init_buffer(vao[9], buffers[16], rendering_program_facet, upper_local_convex_facets, "vertex");
// 	init_buffer(vao[9], buffers[17], rendering_program_facet, upper_local_convex_normals, "normal");
// 	init_buffer(vao[9], buffers[19], rendering_program_facet, upper_convex_func_colors, "color");

// 	// Lower local convexity
// 	init_buffer(vao[10], buffers[20], rendering_program_facet, lower_local_convex_facets, "vertex");
// 	init_buffer(vao[10], buffers[21], rendering_program_facet, lower_local_convex_normals, "normal");
// 	init_buffer(vao[10], buffers[22], rendering_program_facet, lower_local_convex_colors, "color");

// 	// Lower laplacian convexity
// 	init_buffer(vao[11], buffers[20], rendering_program_facet, lower_local_convex_facets, "vertex");
// 	init_buffer(vao[11], buffers[21], rendering_program_facet, lower_local_convex_normals, "normal");
// 	init_buffer(vao[11], buffers[23], rendering_program_facet, lower_convex_func_colors, "color");

// 	// Upper convex isoedges
// 	init_buffer(vao[12], buffers[24], rendering_program, upper_convex_iso_edges, "vertex");

// 	// Lower convex isoedges
// 	init_buffer(vao[13], buffers[25], rendering_program, lower_convex_iso_edges, "vertex");

// 	// Mouse click
// 	init_buffer(vao[14], buffers[26], rendering_program, selected_points, "vertex");

// 	// Segmentation Surface
// 	init_buffer(vao[15], buffers[27], rendering_program_text, selected_seg_facets, "vertex");
// 	init_buffer(vao[15], buffers[28], rendering_program_text, selected_seg_normals, "normal");

// 	for(int i = 0; i < teeth_count; i++)
//     {
// 		// Upper segment face
// 		init_buffer(vao[seg_attribus_vao_index + i], buffers[seg_attribus_buffer_index + 2 * i + 0], rendering_program_text, upper_seg_attribus[2 * i + 0], "vertex");
// 		init_buffer(vao[seg_attribus_vao_index + i], buffers[seg_attribus_buffer_index + 2 * i + 1], rendering_program_text, upper_seg_attribus[2 * i + 1], "normal");
// 		// Lower segment face
// 		init_buffer(vao[seg_attribus_vao_index + teeth_count + i], buffers[seg_attribus_buffer_index + (teeth_count + i) * 2 + 0], rendering_program_text, lower_seg_attribus[2 * i + 0], "vertex");
// 		init_buffer(vao[seg_attribus_vao_index + teeth_count + i], buffers[seg_attribus_buffer_index + (teeth_count + i) * 2 + 1], rendering_program_text, lower_seg_attribus[2 * i + 1], "normal");

//         // Upper geodesic function
// 		init_buffer(vao[geodesic_vao_index + i], buffers[seg_attribus_buffer_index + 2 * i + 0], rendering_program_facet, upper_seg_attribus[2 * i + 0], "vertex");
// 		init_buffer(vao[geodesic_vao_index + i], buffers[seg_attribus_buffer_index + 2 * i + 1], rendering_program_facet, upper_seg_attribus[2 * i + 1], "normal");
// 		init_buffer(vao[geodesic_vao_index + i], buffers[geodesic_buffer_index + i], rendering_program_facet, upper_geodesic_attribus[i], "color");
//         // Lower geodesic function
// 		init_buffer(vao[geodesic_vao_index + teeth_count + i], buffers[seg_attribus_buffer_index + (teeth_count + i) * 2 + 0], rendering_program_facet, lower_seg_attribus[2 * i + 0], "vertex");
// 		init_buffer(vao[geodesic_vao_index + teeth_count + i], buffers[seg_attribus_buffer_index + (teeth_count + i) * 2 + 1], rendering_program_facet, lower_seg_attribus[2 * i + 1], "normal");
// 		init_buffer(vao[geodesic_vao_index + teeth_count + i], buffers[geodesic_buffer_index + teeth_count + i], rendering_program_facet, lower_geodesic_attribus[i], "color");

// 		// Upper pad faces
// 		init_buffer(vao[pad_outlines_vao_index + i], buffers[pad_outlines_buffer_index + 2 * i + 0], rendering_program_text, upper_pad_outlines_attribus[2 * i + 0], "vertex");
// 		init_buffer(vao[pad_outlines_vao_index + i], buffers[pad_outlines_buffer_index + 2 * i + 1], rendering_program_text, upper_pad_outlines_attribus[2 * i + 1], "normal");
// 		// Lower pad faces
// 		init_buffer(vao[pad_outlines_vao_index + teeth_count + i], buffers[pad_outlines_buffer_index + (teeth_count + i) * 2 + 0], rendering_program_text, lower_pad_outlines_attribus[2 * i + 0], "vertex");
// 		init_buffer(vao[pad_outlines_vao_index + teeth_count + i], buffers[pad_outlines_buffer_index + (teeth_count + i) * 2 + 1], rendering_program_text, lower_pad_outlines_attribus[2 * i + 1], "normal");

//         // Upper tooth bound segments
//         init_buffer(vao[tooth_bounds_vao_index + i], buffers[tooth_bounds_buffer_index + i], rendering_program, upper_tooth_bounds_attribus[i], "vertex");
//         // Lower tooth bound segments
//         init_buffer(vao[tooth_bounds_vao_index + teeth_count + i], buffers[tooth_bounds_buffer_index + teeth_count + i], rendering_program, lower_tooth_bounds_attribus[i], "vertex");

//         // Upper tooth cut points
//         init_buffer(vao[tooth_cut_points_vao_index + i], buffers[tooth_cut_points_buffer_index + i], rendering_program, upper_tooth_cut_points_attribus[i], "vertex");
//         // Lower tooth cut points
//         init_buffer(vao[tooth_cut_points_vao_index + teeth_count + i], buffers[tooth_cut_points_buffer_index + teeth_count + i], rendering_program, lower_tooth_cut_points_attribus[i], "vertex");

//         // Upper cut plane
// 		init_buffer(vao[tooth_cut_planes_vao_index + i], buffers[tooth_cut_planes_buffer_index + 2 * i + 0], rendering_program_text, upper_tooth_cut_planes_attribus[2 * i + 0], "vertex");
// 		init_buffer(vao[tooth_cut_planes_vao_index + i], buffers[tooth_cut_planes_buffer_index + 2 * i + 1], rendering_program_text, upper_tooth_cut_planes_attribus[2 * i + 1], "normal");
// 		// Lower cut plane
// 		init_buffer(vao[tooth_cut_planes_vao_index + teeth_count + i], buffers[tooth_cut_planes_buffer_index + (teeth_count + i) * 2 + 0], rendering_program_text, lower_tooth_cut_planes_attribus[2 * i + 0], "vertex");
// 		init_buffer(vao[tooth_cut_planes_vao_index + teeth_count + i], buffers[tooth_cut_planes_buffer_index + (teeth_count + i) * 2 + 1], rendering_program_text, lower_tooth_cut_planes_attribus[2 * i + 1], "normal");
//     }

//     for(int i = 0; i < pair_count; i++)
//     {
//         // Upper pair fitting plane
// 		init_buffer(vao[pair_fitting_planes_vao_index + i], buffers[pair_fitting_planes_buffer_index + 2 * i + 0], rendering_program_text, upper_pair_fitting_planes_attribus[2 * i + 0], "vertex");
// 		init_buffer(vao[pair_fitting_planes_vao_index + i], buffers[pair_fitting_planes_buffer_index + 2 * i + 1], rendering_program_text, upper_pair_fitting_planes_attribus[2 * i + 1], "normal");
// 		// Lower pair fitting plane
// 		init_buffer(vao[pair_fitting_planes_vao_index + pair_count + i], buffers[pair_fitting_planes_buffer_index + (pair_count + i) * 2 + 0], rendering_program_text, lower_pair_fitting_planes_attribus[2 * i + 0], "vertex");
// 		init_buffer(vao[pair_fitting_planes_vao_index + pair_count + i], buffers[pair_fitting_planes_buffer_index + (pair_count + i) * 2 + 1], rendering_program_text, lower_pair_fitting_planes_attribus[2 * i + 1], "normal");

//         // Upper pair fitting lines
//         init_buffer(vao[pair_fitting_lines_vao_index + i], buffers[pair_fitting_lines_buffer_index + i], rendering_program, upper_pair_fitting_lines_attribus[i], "vertex");
//         // Lower pair fitting lines
//         init_buffer(vao[pair_fitting_lines_vao_index + pair_count + i], buffers[pair_fitting_lines_buffer_index + pair_count + i], rendering_program, lower_pair_fitting_lines_attribus[i], "vertex");

//         // Upper pair fitting points
//         init_buffer(vao[pair_fitting_points_vao_index + i], buffers[pair_fitting_points_buffer_index + i], rendering_program, upper_pair_fitting_points_attribus[i], "vertex");
//         // Lower pair fitting points
//         init_buffer(vao[pair_fitting_points_vao_index + pair_count + i], buffers[pair_fitting_points_buffer_index + pair_count + i], rendering_program, lower_pair_fitting_points_attribus[i], "vertex");

//         // Upper pair cut plane
// 		init_buffer(vao[pair_cut_planes_vao_index + i], buffers[pair_cut_planes_buffer_index + 2 * i + 0], rendering_program_text, upper_pair_cut_planes_attribus[2 * i + 0], "vertex");
// 		init_buffer(vao[pair_cut_planes_vao_index + i], buffers[pair_cut_planes_buffer_index + 2 * i + 1], rendering_program_text, upper_pair_cut_planes_attribus[2 * i + 1], "normal");
// 		// Lower pair cut plane
// 		init_buffer(vao[pair_cut_planes_vao_index + pair_count + i], buffers[pair_cut_planes_buffer_index + (pair_count + i) * 2 + 0], rendering_program_text, lower_pair_cut_planes_attribus[2 * i + 0], "vertex");
// 		init_buffer(vao[pair_cut_planes_vao_index + pair_count + i], buffers[pair_cut_planes_buffer_index + (pair_count + i) * 2 + 1], rendering_program_text, lower_pair_cut_planes_attribus[2 * i + 1], "normal");

//         // Upper pair wires
//         init_buffer(vao[pair_wires_vao_index + i], buffers[pair_wires_buffer_index + i], rendering_program, upper_pair_wires_attribus[i], "vertex");
//         // Lower pair wires
//         init_buffer(vao[pair_wires_vao_index + pair_count + i], buffers[pair_wires_buffer_index + pair_count + i], rendering_program, lower_pair_wires_attribus[i], "vertex");

//         // Upper wire surface shapes
// 		init_buffer(vao[wire_shapes_vao_index + i], buffers[wire_shapes_buffer_index + 2 * i + 0], rendering_program_text, upper_wire_shapes_attribus[2 * i + 0], "vertex");
// 		init_buffer(vao[wire_shapes_vao_index + i], buffers[wire_shapes_buffer_index + 2 * i + 1], rendering_program_text, upper_wire_shapes_attribus[2 * i + 1], "normal");
// 		// Lower wire surface shapes
// 		init_buffer(vao[wire_shapes_vao_index + pair_count + i], buffers[wire_shapes_buffer_index + (pair_count + i) * 2 + 0], rendering_program_text, lower_wire_shapes_attribus[2 * i + 0], "vertex");
// 		init_buffer(vao[wire_shapes_vao_index + pair_count + i], buffers[wire_shapes_buffer_index + (pair_count + i) * 2 + 1], rendering_program_text, lower_wire_shapes_attribus[2 * i + 1], "normal");

//         // Upper wire extrude shapes
// 		init_buffer(vao[wire_extrude_shapes_vao_index + i], buffers[wire_extrude_shapes_buffer_index + 2 * i + 0], rendering_program_text, upper_wire_extrude_shapes_attribus[2 * i + 0], "vertex");
// 		init_buffer(vao[wire_extrude_shapes_vao_index + i], buffers[wire_extrude_shapes_buffer_index + 2 * i + 1], rendering_program_text, upper_wire_extrude_shapes_attribus[2 * i + 1], "normal");
// 		// Lower wire extrude shapes
// 		init_buffer(vao[wire_extrude_shapes_vao_index + pair_count + i], buffers[wire_extrude_shapes_buffer_index + (pair_count + i) * 2 + 0], rendering_program_text, lower_wire_extrude_shapes_attribus[2 * i + 0], "vertex");
// 		init_buffer(vao[wire_extrude_shapes_vao_index + pair_count + i], buffers[wire_extrude_shapes_buffer_index + (pair_count + i) * 2 + 1], rendering_program_text, lower_wire_extrude_shapes_attribus[2 * i + 1], "normal");
//     }

//     are_buffers_initialized = true;
// }

// void Scene::attrib_buffers(CGAL::QGLViewer* viewer)
// {
//     QMatrix4x4 mvpMatrix;
//     QMatrix4x4 mvMatrix;
//     QMatrix4x4 vpMatrix;
//     double mat[16];
//     viewer->camera()->getModelViewProjectionMatrix(mat);
//     for(int i=0; i < 16; i++)
//         mvpMatrix.data()[i] = (float) mat[i];
//     viewer->camera()->getModelViewMatrix(mat);
//     for(int i=0; i < 16; i++)
//         mvMatrix.data()[i] = (float) mat[i];
//     int mat1[4];
//     viewer->camera()->getViewport(mat1);
//     vpMatrix.fill(0.);
//     vpMatrix.data()[0] = (float) mat1[2] * 0.5;
//     vpMatrix.data()[3] = (float) mat1[2] * 0.5 + mat1[0];
//     vpMatrix.data()[5] = (float) mat1[3] * 0.5;
//     vpMatrix.data()[7] = (float) mat1[3] * 0.5 + mat1[1];
//     vpMatrix.data()[10] = 0.5;
//     vpMatrix.data()[11] = 0.5;
//     vpMatrix.data()[15] = 1.;

//     // Point / Vector Rendering
//     rendering_program.bind();
//     mvpLocation = rendering_program.uniformLocation("mvp_matrix");
//     colorLocation = rendering_program.uniformLocation("color");
//     rendering_program.setUniformValue(mvpLocation, mvpMatrix);
//     rendering_program.release();

//     // Facet Rendering
//     QVector4D ambient(0.25f, 0.20725f, 0.20725f, 0.922f);
//     QVector4D diffuse( 1.0f, 0.829f, 0.829f, 0.922f );
//     QVector4D specular( 0.6f, 0.6f, 0.6f, 1.0f );
//     QVector4D position( 0.0f, 0.0f, 3.0f, 1.0f );
//     GLfloat shininess = 11.264f;

//     rendering_program_text.bind();
//     mvpLocation_text = rendering_program_text.uniformLocation("mvp_matrix");
//     mvLocation_text = rendering_program_text.uniformLocation("mv_matrix");
//     colorLocation_text = rendering_program_text.uniformLocation("color");
//     colorLineLocation_text = rendering_program_text.uniformLocation("line_color");
//     widthLineLocation_text = rendering_program_text.uniformLocation("line_width");
//     vpLocation_text = rendering_program_text.uniformLocation("ViewportMatrix");
//     flagEdgeLocation_text = rendering_program_text.uniformLocation("flag_edge");

//     lightLocation_text[0] = rendering_program_text.uniformLocation("light_pos");
//     lightLocation_text[1] = rendering_program_text.uniformLocation("light_diff");
//     lightLocation_text[2] = rendering_program_text.uniformLocation("light_spec");
//     lightLocation_text[3] = rendering_program_text.uniformLocation("light_amb");
//     lightLocation_text[4] = rendering_program_text.uniformLocation("spec_power");

//     rendering_program_text.setUniformValue(lightLocation_text[0], position);
//     rendering_program_text.setUniformValue(lightLocation_text[1], diffuse);
//     rendering_program_text.setUniformValue(lightLocation_text[2], specular);
//     rendering_program_text.setUniformValue(lightLocation_text[3], ambient);
//     rendering_program_text.setUniformValue(lightLocation_text[4], shininess);
//     rendering_program_text.setUniformValue(mvpLocation_text, mvpMatrix);
//     rendering_program_text.setUniformValue(mvLocation_text, mvMatrix);
//     rendering_program_text.setUniformValue(vpLocation_text, vpMatrix);
// 	rendering_program_text.setUniformValue(flagEdgeLocation_text, static_cast<int>(m_view[SceneMeshEdges]));
//     rendering_program_text.release();

//     // Facet Color Rendering
//     rendering_program_facet.bind();
//     mvpLocation_facet = rendering_program_facet.uniformLocation("mvp_matrix");
//     mvLocation_facet = rendering_program_facet.uniformLocation("mv_matrix");
//     colorLineLocation_facet = rendering_program_facet.uniformLocation("line_color");
//     widthLineLocation_facet = rendering_program_facet.uniformLocation("line_width");
//     vpLocation_facet = rendering_program_facet.uniformLocation("ViewportMatrix");
//     flagEdgeLocation_facet = rendering_program_facet.uniformLocation("flag_edge");

//     lightLocation_facet[0] = rendering_program_facet.uniformLocation("light_pos");
//     lightLocation_facet[1] = rendering_program_facet.uniformLocation("light_diff");
//     lightLocation_facet[2] = rendering_program_facet.uniformLocation("light_spec");
//     lightLocation_facet[3] = rendering_program_facet.uniformLocation("light_amb");
//     lightLocation_facet[4] = rendering_program_facet.uniformLocation("spec_power");

//     rendering_program_facet.setUniformValue(lightLocation_facet[0], position);
//     rendering_program_facet.setUniformValue(lightLocation_facet[1], diffuse);
//     rendering_program_facet.setUniformValue(lightLocation_facet[2], specular);
//     rendering_program_facet.setUniformValue(lightLocation_facet[3], ambient);
//     rendering_program_facet.setUniformValue(lightLocation_facet[4], shininess);
//     rendering_program_facet.setUniformValue(mvpLocation_facet, mvpMatrix);
//     rendering_program_facet.setUniformValue(mvLocation_facet, mvMatrix);
//     rendering_program_facet.setUniformValue(vpLocation_facet, vpMatrix);
// 	rendering_program_facet.setUniformValue(flagEdgeLocation_facet, static_cast<int>(m_view[SceneMeshEdges]));
//     rendering_program_facet.release();

//     // Func Rendering
//     rendering_program_func.bind();
//     mvpLocation_func = rendering_program_func.uniformLocation("mvp_matrix");
//     rendering_program_func.setUniformValue(mvpLocation_func, mvpMatrix);
//     rendering_program_func.release();
// }

void Scene::compute_elements(unsigned int flags)
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
        m_func.update_inside_function(  upper_inside_ray_facets,
                                        upper_inside_ray_normals,
                                        upper_inside_ray_colors,
                                        upper_inside_func_colors,
                                        upper_inside_facets,
                                        upper_inside_normals,
                                        false);
        m_func.update_inside_function(  lower_inside_ray_facets,
                                        lower_inside_ray_normals,
                                        lower_inside_ray_colors,
                                        lower_inside_func_colors,
                                        lower_inside_facets,
                                        lower_inside_normals,
                                        true);
        
    }

    if ((flags & FlagConvex))
    {
        m_func.update_convex_function(  upper_local_convex_facets,
                                        upper_local_convex_normals,
                                        upper_local_convex_colors,
                                        upper_convex_func_colors,
                                        false);
        m_func.update_convex_function(  lower_local_convex_facets,
                                        lower_local_convex_normals,
                                        lower_local_convex_colors,
                                        lower_convex_func_colors,
                                        true);
    }

    if ((flags & FlagConvexIso))
    {
        m_func.update_convexity_isovalue(   upper_convex_iso_edges, 
                                            m_mesh_options.convex_isovalue,
                                            false);
        m_func.update_convexity_isovalue(   lower_convex_iso_edges, 
                                            m_mesh_options.convex_isovalue,
                                            true);
    }

    if ((flags & FlagSegment))
    {
        m_func.update_segmentation(selected_seg_facets, selected_seg_normals);
    }

    if ((flags & FlagValid) && (flags & FlagUpper))
    {
        for(int i = 0; i < 6; i++)
            m_func.update_validated_segmentation(i, false, upper_seg_attribus[2 * i], upper_seg_attribus[2 * i + 1]);
    }

    if ((flags & FlagValid) && (flags & FlagLower))
    {
        for(int i = 0; i < 6; i++)
            m_func.update_validated_segmentation(i, true, lower_seg_attribus[2 * i], lower_seg_attribus[2 * i + 1]);
    }

    if ((flags & FlagPadGeodesic))
    {
        for(int i = 0; i < 6; i++)
        {
            if(m_mesh_options.upper_flags[i])
                m_func.update_tooth_geodesic_function(i, false, upper_geodesic_attribus[i]);

            if(m_mesh_options.lower_flags[i])
                m_func.update_tooth_geodesic_function(i, true, lower_geodesic_attribus[i]);
        }
    }

    if ((flags & FlagPadOutline))
    {
        for(int i = 0; i < 6; i++)
        {
            if(m_mesh_options.upper_flags[i])
                m_func.update_pad_outline(i, false, upper_pad_outlines_attribus[2 * i], upper_pad_outlines_attribus[2 * i + 1]);

            if(m_mesh_options.lower_flags[i])
                m_func.update_pad_outline(i, true, lower_pad_outlines_attribus[2 * i], lower_pad_outlines_attribus[2 * i + 1]);
        }
    }

    if ((flags & FlagToothCut))
    {
        for(int i = 0; i < teeth_count; i++)
        {
            if(m_mesh_options.upper_flags[i])
            {
                m_func.update_tooth_cut_plane(  i, false, 
                                                upper_tooth_bounds_attribus[i], 
                                                upper_tooth_cut_points_attribus[i], 
                                                upper_tooth_cut_planes_attribus[2 * i], 
                                                upper_tooth_cut_planes_attribus[2 * i + 1]);
            }
                

            if(m_mesh_options.lower_flags[i])
            {
                m_func.update_tooth_cut_plane(  i, true, 
                                                lower_tooth_bounds_attribus[i], 
                                                lower_tooth_cut_points_attribus[i], 
                                                lower_tooth_cut_planes_attribus[2 * i], 
                                                lower_tooth_cut_planes_attribus[2 * i + 1]);
            }
        }
    }

    if ((flags & FlagPairCut))
    {
        for(int i = 0; i < pair_count; i++)
        {
            if(m_mesh_options.upper_flags[i] && m_mesh_options.upper_flags[i+1])
            {
                m_func.update_pair_cut_plane(   i, false, 
                                                upper_pair_fitting_planes_attribus[2 * i], 
                                                upper_pair_fitting_planes_attribus[2 * i + 1],
                                                upper_pair_fitting_lines_attribus[i],
                                                upper_pair_fitting_points_attribus[i],
                                                upper_pair_cut_planes_attribus[2 * i],
                                                upper_pair_cut_planes_attribus[2 * i + 1],
                                                upper_pair_wires_attribus[i] );
            }
                

            if(m_mesh_options.lower_flags[i] && m_mesh_options.lower_flags[i+1])
            {
                m_func.update_pair_cut_plane(   i, true, 
                                                lower_pair_fitting_planes_attribus[2 * i], 
                                                lower_pair_fitting_planes_attribus[2 * i + 1],
                                                lower_pair_fitting_lines_attribus[i],
                                                lower_pair_fitting_points_attribus[i],
                                                lower_pair_cut_planes_attribus[2 * i],
                                                lower_pair_cut_planes_attribus[2 * i + 1],
                                                lower_pair_wires_attribus[i] );
            }
        }
    }

    if ((flags & FlagPairWire))
    {
        for(int i = 0; i < pair_count; i++)
        {
            if(m_mesh_options.upper_flags[i] && m_mesh_options.upper_flags[i+1])
            {
                m_func.update_pair_wire_shape(  i, false, 
                                                upper_wire_shapes_attribus[2 * i], 
                                                upper_wire_shapes_attribus[2 * i + 1],
                                                upper_wire_extrude_shapes_attribus[2 * i], 
                                                upper_wire_extrude_shapes_attribus[2 * i + 1]);
            }
                

            if(m_mesh_options.lower_flags[i] && m_mesh_options.lower_flags[i+1])
            {
                m_func.update_pair_wire_shape(  i, true, 
                                                lower_wire_shapes_attribus[2 * i], 
                                                lower_wire_shapes_attribus[2 * i + 1],
                                                lower_wire_extrude_shapes_attribus[2 * i], 
                                                lower_wire_extrude_shapes_attribus[2 * i + 1]);
            }
        }
    }

    are_buffers_initialized = false;
}

void Scene::clear_elements(unsigned int flags)
{
    if((flags & FlagMesh) && (flags & FlagUpper))
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
        for(int i = 0; i < upper_seg_attribus.size(); i++)
            upper_seg_attribus[i].clear();
    }

    if ((flags & FlagValid) && (flags & FlagLower))
    {
        for(int i = 0; i < lower_seg_attribus.size(); i++)
            lower_seg_attribus[i].clear();
    }

    if ((flags & FlagPadGeodesic))
    {
        for(int i = 0; i < 6; i++)
        {
            if(m_mesh_options.upper_flags[i])
            {
                upper_geodesic_attribus[i].clear();
            }

            if(m_mesh_options.lower_flags[i])
            {
                lower_geodesic_attribus[i].clear();
            }
        }
    }

    if ((flags & FlagPadOutline))
    {
        for(int i = 0; i < 6; i++)
        {
            if(m_mesh_options.upper_flags[i])
            {
                upper_pad_outlines_attribus[2 * i].clear();
                upper_pad_outlines_attribus[2 * i + 1].clear();
            }

            if(m_mesh_options.lower_flags[i])
            {
                lower_pad_outlines_attribus[2 * i].clear();
                lower_pad_outlines_attribus[2 * i + 1].clear();
            }
        }
    }
    
    if ((flags & FlagToothCut))
    {
        for(int i = 0; i < teeth_count; i++)
        {
            if(m_mesh_options.upper_flags[i])
            {
                upper_tooth_bounds_attribus[i].clear();
                upper_tooth_cut_points_attribus[i].clear();
                upper_tooth_cut_planes_attribus[2 * i + 0].clear();
                upper_tooth_cut_planes_attribus[2 * i + 1].clear();
            }

            if(m_mesh_options.lower_flags[i])
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
        for(int i = 0; i < pair_count; i++)
        {
            if(m_mesh_options.upper_flags[i] && m_mesh_options.upper_flags[i+1])
            {
                upper_pair_fitting_planes_attribus[2 * i + 0].clear();
                upper_pair_fitting_planes_attribus[2 * i + 1].clear();
                upper_pair_fitting_lines_attribus[i].clear();
                upper_pair_fitting_points_attribus[i].clear();
                upper_pair_cut_planes_attribus[2 * i + 0].clear();
                upper_pair_cut_planes_attribus[2 * i + 1].clear();
                upper_pair_wires_attribus[i].clear();
            }

            if(m_mesh_options.lower_flags[i] && m_mesh_options.lower_flags[i+1])
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
        for(int i = 0; i < pair_count; i++)
        {
            if(m_mesh_options.upper_flags[i] && m_mesh_options.upper_flags[i+1])
            {
                upper_wire_shapes_attribus[2 * i + 0].clear();
                upper_wire_shapes_attribus[2 * i + 1].clear();
                upper_wire_extrude_shapes_attribus[2 * i + 0].clear();
                upper_wire_extrude_shapes_attribus[2 * i + 1].clear();
            }

            if(m_mesh_options.lower_flags[i] && m_mesh_options.lower_flags[i+1])
            {
                lower_wire_shapes_attribus[2 * i + 0].clear();
                lower_wire_shapes_attribus[2 * i + 1].clear();
                lower_wire_extrude_shapes_attribus[2 * i + 0].clear();
                lower_wire_extrude_shapes_attribus[2 * i + 1].clear();
            }
        }
    }

    are_buffers_initialized = false;
}

void Scene::clear_data()
{
    m_func.reset();
    clear_elements(FlagAll);
}

/******************* Function **********************/

void Scene::compute_laplacian_based_inside()
{
    m_func.compute_laplacian_based_inside(  m_mesh_options.inside_smooth_range, 
                                            m_mesh_options.inside_laplacian_param, 
                                            m_mesh_options.inside_isovalue); 

    compute_elements(FlagInside);
}

void Scene::compute_laplacian_based_convexity()
{
    m_func.compute_laplacian_based_convexity(   m_mesh_options.convex_neighbor_radius,
                                                m_mesh_options.convex_smooth_range, 
                                                m_mesh_options.convex_laplacian_param);

    compute_elements(FlagConvex | FlagConvexIso);
}

void Scene::update_convexity_function_isovalue()
{
    compute_elements(FlagConvexIso);
}

void Scene::compute_selected_segmentation()
{
    m_func.compute_selected_segmentation(   m_select_pos,
                                            m_mesh_options.convex_isovalue);

    compute_elements(FlagSegment);
}

void Scene::clear_selected_points()
{
    m_select_pos.clear();
    selected_points.clear();
    are_buffers_initialized = false;
}

void Scene::validate_selected_segmentation()
{
    int status = m_func.validate_selected_segmentation();

    if(status == 0)
        return;

    if(status == 1)
        compute_elements(FlagSegment | FlagValid | FlagUpper);
    else if(status == -1)
        compute_elements(FlagSegment | FlagValid | FlagLower);
}

void Scene::compute_geodesic_pad_outlines()
{
    for(int i = 0; i < m_mesh_options.upper_flags.size(); i++)
    {
        if(m_mesh_options.upper_flags[i])
            m_func.compute_geodesic_pad_outlines(i, false, m_mesh_options.geodesic_step, m_mesh_options.geodesic_isovalue);
    }

    for(int i = 0; i < m_mesh_options.lower_flags.size(); i++)
    {
        if(m_mesh_options.lower_flags[i])
            m_func.compute_geodesic_pad_outlines(i, true, m_mesh_options.geodesic_step, m_mesh_options.geodesic_isovalue);
    }

    compute_elements(FlagPadGeodesic | FlagPadOutline);
}

void Scene::recompute_geodesic_pad_outlines()
{
    for(int i = 0; i < m_mesh_options.upper_flags.size(); i++)
    {
        if(m_mesh_options.upper_flags[i])
            m_func.recompute_geodesic_pad_outlines(i, m_mesh_options.geodesic_isovalue, false);
    }

    for(int i = 0; i < m_mesh_options.lower_flags.size(); i++)
    {
        if(m_mesh_options.lower_flags[i])
            m_func.recompute_geodesic_pad_outlines(i, m_mesh_options.geodesic_isovalue, true);
    }

    compute_elements(FlagPadOutline);
}

void Scene::compute_tooth_split_planes()
{
    for(int i = 0; i < m_mesh_options.upper_flags.size(); i++)
    {
        if(m_mesh_options.upper_flags[i])
            m_func.compute_tooth_cut_plane(i, false);
    }

    for(int i = 0; i < m_mesh_options.lower_flags.size(); i++)
    {
        if(m_mesh_options.lower_flags[i])
            m_func.compute_tooth_cut_plane(i, true);
    }

    compute_elements(FlagToothCut);
}

void Scene::compute_paired_wires()
{
    for(int i = 0; i < pair_count; i++)
    {
        if(m_mesh_options.upper_flags[i] && m_mesh_options.upper_flags[i+1])
            m_func.compute_paired_wires(i, false, m_mesh_options.pair_distance);

        if(m_mesh_options.lower_flags[i] && m_mesh_options.lower_flags[i+1])
            m_func.compute_paired_wires(i, true, m_mesh_options.pair_distance);
    }

    compute_elements(FlagPairCut);
}

void Scene::compute_wire_shapes()
{
    for(int i = 0; i < pair_count; i++)
    {
        if(m_mesh_options.upper_flags[i] && m_mesh_options.upper_flags[i+1])
            m_func.compute_wire_shapes(i, false, m_mesh_options.geodesic_step, m_mesh_options.pair_width);
        
        if(m_mesh_options.lower_flags[i] && m_mesh_options.lower_flags[i+1])
            m_func.compute_wire_shapes(i, true, m_mesh_options.geodesic_step, m_mesh_options.pair_width);
    }

    compute_elements(FlagPairWire);
}

/******************* Options **********************/

void Scene::setInput_point_size(double d)
{
    m_mesh_options.input_point_size = (float)d;
}

void Scene::setColor_point_size(double d)
{
    m_mesh_options.color_point_size = (float)d;
}

void Scene::setMesh_edge_width(double d)
{
    m_mesh_options.mesh_edge_width = (float)d;
}

void Scene::setVector_ratio(double d)
{
    m_mesh_options.vector_ratio = (float)d;
}

void Scene::setInside_laplacian_parameter(double d)
{
    m_mesh_options.inside_laplacian_param = d;
}

void Scene::setInside_smooth_range(int i)
{
    m_mesh_options.inside_smooth_range = i;
}

void Scene::setInside_isovalue(double d)
{
    m_mesh_options.inside_isovalue = d;
}

void Scene::setConvex_neighbor_radius(double d)
{
    m_mesh_options.convex_neighbor_radius = d;
}

void Scene::setConvex_smooth_range(int i)
{
    m_mesh_options.convex_smooth_range = i;
}

void Scene::setConvex_laplacian_thresh(double d)
{
    m_mesh_options.convex_laplacian_param = d;
}

void Scene::setConvex_isovalue(double d)
{
    m_mesh_options.convex_isovalue = d;
}

void Scene::setGeodesic_step(int i)
{
    m_mesh_options.geodesic_step = i;
}

void Scene::setGeodesic_isovalue(double d)
{
    m_mesh_options.geodesic_isovalue = d;
}

void Scene::setUpperFlag(int ind, int status)
{
    m_mesh_options.upper_flags[ind] = (bool)status;
}

void Scene::setLowerFlag(int ind, int status)
{
    m_mesh_options.lower_flags[ind] = (bool)status;
}

void Scene::setPair_distance(double d)
{
    m_mesh_options.pair_distance = d;
}

void Scene::setPair_width(double d)
{
    m_mesh_options.pair_width = d;
}