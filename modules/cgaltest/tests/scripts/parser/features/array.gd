func test():
	# Indexing from the beginning:
	var cgal_instance = cgalAABB.new()
	var occP1 = cgal_instance.getPoint()
	print(occP1)
	var occ_instance = OccPoint.new();
	var occP2 = occ_instance.getPoint()
	print(occP2)
	var scene_instance = Scene.new() #registered from cgal module
	scene_instance.load_upper_teeth("modules/cgaltest/tests/data/134109188_shell_occlusion_u_cut_1.stl")
	scene_instance.load_lower_teeth("modules/cgaltest/tests/data/134109188_shell_occlusion_l_cut_1.stl")

