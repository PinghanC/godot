func test():
	# Indexing from the beginning:
	var cgal_instance = cgalAABB.new()
	var occP1 = cgal_instance.getPoint()
	print(occP1)
	var occ_instance = OccPoint.new();
	var occP2 = occ_instance.getPoint()
	print(occP2)
	