// occ_test.cpp
#include "cgalAABB.h" 


Vector3 cgalAABB::getPoint() {
    
	 auto box = m_inside_tree.bbox();
     return Vector3(0.1, 0.2, 0.3);

}
 
void cgalAABB::_bind_methods() {
	ClassDB::bind_method(D_METHOD("getPoint"), &cgalAABB::getPoint);
}
