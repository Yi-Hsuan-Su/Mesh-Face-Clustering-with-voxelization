#include "Voxel.h"
#include "GUA_OM.h"

Voxel::Voxel()
{
	center = { 0,0,0 };
	offset_x = 1;
	offset_y = 1;
	offset_z = 1;
	x_scale = 1.0;
	y_scale = 1.0;
	z_scale = 1.0;

	initial();
	computebbox();
}


Voxel::~Voxel() 
{

}

void Voxel::initial()
{
	Vertex.resize(8);
	Face.resize(6);
	Edge.resize(12);

	Vertex[0] = {   center[0]-1, center[1]-1, center[2]+1 };
	Vertex[1] = {  center[0]-1,center[1]-1,center[2]-1 };
	Vertex[2] = {  center[0]+1,  center[1]-1,center[2]-1 };
	Vertex[3] = {  center[0]+1,center[1]-1	,center[2]+1 };
	Vertex[4] = {  center[0]-1,center[1]+1, center[2]+1 };
	Vertex[5] = {  center[0]-1,center[1]+1,center[2]-1 };
	Vertex[6] = {  center[0]+1,center[1]+ 1,center[2]-1 };
	Vertex[7] = {  center[0]+1,center[1]+ 1,center[2]+1 };

	Face[0] = std::vector<int> { 0,3,2,1 };
	Face[1] = std::vector<int>{ 4,7,6,5 };
	Face[2] = std::vector<int>{ 4,0,1,5 };
	Face[3] = std::vector<int>{ 0,3,7,4 };
	Face[4] = std::vector<int>{ 2,6,7,3 };
	Face[5] = std::vector<int>{ 6,5,1,2 };

	Edge[0] = { 0,3 };
	Edge[1] = { 3,2 };
	Edge[2] = { 2,1 };
	Edge[3] = { 1,0 };
	Edge[4] = { 4,7 };
	Edge[5] = { 7,6 };
	Edge[6] = { 6,5 };
	Edge[7] = { 4,0 };
	Edge[8] = { 1,5 };
	Edge[9] = { 3,7 };
	Edge[10] = { 2,6 };
	Edge[11] = { 4,5 };

	isintersect = false;

}

void Voxel::update_vertex() 
{	
	offset_x *= x_scale;
	offset_y *= y_scale;
	offset_z *= z_scale;

	Vertex[0] = { center[0] - offset_x , center[1] - offset_y , center[2] + offset_z };
	Vertex[1] = { center[0] - offset_x ,center[1] - offset_y ,center[2] - offset_z };
	Vertex[2] = { center[0] + offset_x ,  center[1] - offset_y ,center[2] - offset_z };
	Vertex[3] = { center[0] + offset_x ,center[1] - offset_y	,center[2] + offset_z };
	Vertex[4] = { center[0] - offset_x ,center[1] + offset_y , center[2] + offset_z };
	Vertex[5] = { center[0] - offset_x ,center[1] + offset_y ,center[2] - offset_z };
	Vertex[6] = { center[0] + offset_x ,center[1] + offset_y ,center[2] - offset_z };
	Vertex[7] = { center[0] + offset_x ,center[1] + offset_y ,center[2] + offset_z };

	computebbox();
}

void Voxel::Change_Scale(float xsc  , float ysc , float zsc) 
{
	x_scale = xsc;
	y_scale = ysc;
	 z_scale = zsc;
	update_vertex();
}


void Voxel::computebbox()
{
	for (int i = 0; i < Vertex.size(); i++)
	{
		if (Vertex[i][0] > xmax)
		{
			xmax = Vertex[i][0];
		}
		if (Vertex[i][0] < xmin)
		{
			xmin = Vertex[i][0];
		}
		//----------------
		if (Vertex[i][1] > ymax)
		{
			ymax = Vertex[i][1];
		}
		if (Vertex[i][1] < ymin)
		{
			ymin = Vertex[i][1];
		}
		//---------------------
		if (Vertex[i][2] > zmax)
		{
			zmax = Vertex[i][2];
		}
		if (Vertex[i][2] < zmin)
		{
			zmin = Vertex[i][2];
		}

	}
}


void Voxel::fdinter_push(int idx)
{
	bool ck = false;
	for (int i = 0; i < inter_face_count.size(); i++)
	{
		if (inter_face_count[i] == idx)
		{
			ck = true;
			break;
		}
		else
		{
			ck = false;
		}
	}

	if (ck == true)
	{

	}
	else
	{
		inter_face_count.push_back(idx);
	}
}

void Voxel::print_face_idx() 
{

	for (int i = 0; i < Face.size(); i++) 
	{
		for(int j =0 ; j < 4;j++)
		{
			std::cout << Face[i][j];
		}
		std::cout<< std::endl;
	}

}

void Voxel::Setposition(OpenMesh::Vec3f pos)
{
	center = pos;
	translatepos = pos;
	update_vertex();
}


void Voxel::translate(OpenMesh::Vec3d pos)
{
	center = translatepos;
	translatescale = OpenMesh::Vec3f(1 / x_scale, 1 / y_scale, 1 / z_scale);
	for (int i = 0; i < offset.size(); i++)
	{
		Vertex[i] = (center + (offset[i] * translatescale));//
	}

}
bool Voxel::is_intersectwithface(faceData f)
{

	for (int i = 0; i < Edge.size(); i++)
	{
		OpenMesh::Vec3f root = OpenMesh::Vec3f();
		root = line_inertsect_plane((OpenMesh::Vec3f)f.getFaceNormal(), (OpenMesh::Vec3f)f.getfcenter(), Vertex[Edge[i].first], Vertex[Edge[i].second]);
		
		if (root[0] != -1000)
		{
			if (isintriangle(f, root))
			{
				intersect_count++;
				return true;
			}
		}

	}
	return false;
}

void Voxel::adjface_push(int idx)
{
	bool ck = false;
	for (int i = 0; i < adj_face_count.size(); i++)
	{
		if (adj_face_count[i] == idx)
		{
			ck = true;
			break;
		}
		else
		{
			ck = false;
		}
	}

	if (ck == true)
	{
		represent_normal = idx;
	}
	else
	{
		adj_face_count.push_back(idx);
	}
}