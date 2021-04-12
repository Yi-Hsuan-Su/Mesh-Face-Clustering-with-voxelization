#pragma once
#include <OpenMesh/Core/IO/MeshIO.hh>

 class Voxel
{
public:
	Voxel();
	~Voxel();

	OpenMesh::Vec3d center;
	std::vector<OpenMesh::Vec3d> Vertex;
	std::vector<std::pair<int, int>> Edge;
	std::vector < std::vector< int> > Face;
	float offset_x , offset_y, offset_z ;
	float x_scale , y_scale , z_scale ;
	float xmax, xmin, ymax, ymin, zmax, zmin;
	void initial();

	void translate(OpenMesh::Vec3d po);
	void Change_Scale(float sc, float ysc, float zsc);
	void update_vertex();
	void computebbox();
	void print_face_idx();
};


