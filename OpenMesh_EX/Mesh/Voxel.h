#pragma once
#include <OpenMesh/Core/IO/MeshIO.hh>

class faceData;

 class Voxel
{
public:
	Voxel();
	~Voxel();

	OpenMesh::Vec3f center;
	std::vector<OpenMesh::Vec3f> Vertex;
	std::vector<std::pair<int, int>> Edge;
	std::vector < std::vector< int> > Face;
	float offset_x , offset_y, offset_z ;
	float x_scale , y_scale , z_scale ;
	bool isintersect;
	std::vector<int> adj_face_count;
	float xmax, xmin, ymax, ymin, zmax, zmin;
	OpenMesh::Vec3f translatepos;
	OpenMesh::Vec3f translatescale;
	int intersect_count;
	OpenMesh::Vec3f V_color;
	int represent_normal;
	std::vector<int> inter_face_count;
	void initial();


	std::vector<OpenMesh::Vec3f>offset;
	void translate(OpenMesh::Vec3d pos);
	void Change_Scale(float sc, float ysc, float zsc);
	void update_vertex();
	void computebbox();
	void print_face_idx();
	void fdinter_push(int idx);
	void Voxel::Setposition(OpenMesh::Vec3f pos);
	bool Voxel::is_intersectwithface(faceData f);
	void adjface_push(int idx);
};




 