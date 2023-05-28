#include "GUA_OM.h"
#include <algorithm>
using namespace OpenMesh;
namespace OMT
{
	/*======================================================================*/
	Model::Model()
	{
		request_vertex_status();
		request_edge_status();
		request_face_status();
	}

	Model::~Model()
	{
		release_vertex_status();
		release_edge_status();
		release_face_status();
	}
}
/*======================================================================*/
namespace OMP
{
	Model::Model()
	{
		Mesh.request_vertex_status();
		Mesh.request_edge_status();
		Mesh.request_face_status();
	}
	Model::~Model()
	{
		Mesh.release_vertex_status();
		Mesh.release_edge_status();
		Mesh.release_face_status();
	}
	/*======================================================================*/
	bool Model::ReadFile(std::string _fileName)
	{
		bool isRead = false;
		OpenMesh::IO::Options opt;
		if (OpenMesh::IO::read_mesh(Mesh, _fileName, opt))
		{
			//read mesh from filename OK!
			isRead = true;
		}
		if (isRead)
		{
			// If the file did not provide vertex normals and mesh has vertex normal ,then calculate them
			if (!opt.check(OpenMesh::IO::Options::VertexNormal) && Mesh.has_vertex_normals())
			{
				Mesh.update_normals();
			}
		}
		return isRead;
	}
	bool Model::SaveFile(std::string _fileName)
	{
		bool isSave = false;
		OpenMesh::IO::Options opt;
		if (OpenMesh::IO::write_mesh(Mesh, _fileName, opt))
		{
			//read mesh from filename OK!
			isSave = true;
		}
		return isSave;
	}
	/*======================================================================*/
	void Model::Render_solid()
	{
		FIter f_it;
		FVIter	fv_it;
		glPushAttrib(GL_LIGHTING_BIT);
		glEnable(GL_POLYGON_OFFSET_FILL);
		glEnable(GL_DEPTH_TEST);
		glPolygonOffset(2.0, 2.0);
		glBegin(GL_POLYGON);
		//glColor4f(1.0, 0.5, 1.0, 0.5);
		for (f_it = Mesh.faces_begin(); f_it != Mesh.faces_end(); ++f_it)
		{
			for (fv_it = Mesh.fv_iter(f_it); fv_it; ++fv_it)
			{
				glNormal3dv(Mesh.normal(fv_it.handle()).data());
				glVertex3dv(Mesh.point(fv_it.handle()).data());
			}
		}
		glEnd();
		glDisable(GL_POLYGON_OFFSET_FILL);
	}
	void Model::Render_wireframe()
	{
		MyMesh::HalfedgeHandle _hedge;
		EIter e_it = Mesh.edges_begin();

		glDisable(GL_LIGHTING);
		glEnable(GL_DEPTH_TEST);
		glColor3f(0.0, 0.0, 0.0);
		glLineWidth(1);
		glBegin(GL_LINES);
		for (e_it = Mesh.edges_begin(); e_it != Mesh.edges_end(); ++e_it)
		{
			_hedge = Mesh.halfedge_handle(e_it.handle(), 1);

			glVertex3dv(Mesh.point(Mesh.from_vertex_handle(_hedge)).data());
			glVertex3dv(Mesh.point(Mesh.to_vertex_handle(_hedge)).data());
		}
		glEnd();
		glEnable(GL_LIGHTING);
	}
	/*======================================================================*/
	void Model::RenderSpecifiedPoint()
	{
		glPushAttrib(GL_LIGHTING_BIT);
		glDisable(GL_LIGHTING);
		glEnable(GL_DEPTH_TEST);
		glPointSize(5.0f);
		glBegin(GL_POINTS);
		vector<sp_p>::iterator p_itr = sp_p_list.begin();
		for (p_itr; p_itr != sp_p_list.end(); ++p_itr)
		{
			glColor3f(p_itr->r, p_itr->g, p_itr->b);
			glVertex3dv(p_itr->pt.data());
		}
		glEnd();
		glEnable(GL_LIGHTING);
		glDisable(GL_POLYGON_OFFSET_FILL);
	}
	void Model::RenderSpecifiedVertex()
	{
		glPushAttrib(GL_LIGHTING_BIT);
		glDisable(GL_LIGHTING);
		glEnable(GL_DEPTH_TEST);
		glPointSize(5.0f);
		glBegin(GL_POINTS);
		vector< sp_v >::iterator v_itr = sp_v_list.begin();
		for (v_itr; v_itr != sp_v_list.end(); ++v_itr)
		{
			glColor3f(v_itr->r, v_itr->g, v_itr->b);
			glVertex3dv(Mesh.point(v_itr->vh).data());
		}
		glEnd();
		glEnable(GL_LIGHTING);
		glDisable(GL_POLYGON_OFFSET_FILL);
	}
	void Model::RenderSpecifiedFace()
	{
		glDisable(GL_CULL_FACE);
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		glPushAttrib(GL_LIGHTING_BIT);
		glEnable(GL_POLYGON_OFFSET_FILL);
		glPolygonOffset(0.5, 1.0);
		glDisable(GL_LIGHTING);
		glEnable(GL_DEPTH_TEST);
		glBegin(GL_QUADS);
		FVIter fv_itr;
		vector< sp_f >::iterator f_itr;
		for (f_itr = sp_f_list.begin(); f_itr != sp_f_list.end(); ++f_itr)
		{
			glColor3f(f_itr->r, f_itr->g, f_itr->b);
			for (fv_itr = Mesh.fv_iter(f_itr->fh); fv_itr; ++fv_itr)
			{
				glNormal3dv(Mesh.normal(fv_itr.handle()).data());
				glVertex3dv(Mesh.point(fv_itr.handle()).data());
			}
		}
		glEnd();
		glEnable(GL_LIGHTING);
		glDisable(GL_POLYGON_OFFSET_FILL);
		glPolygonMode(GL_FRONT, GL_FILL);
		glEnable(GL_CULL_FACE);
	}


	/*======================================================================*/
	void Model::add_sp_p(Point   _p, float _r, float _g, float _b)
	{
		sp_p input_data;
		input_data.pt = _p;
		input_data.r = _r;
		input_data.g = _g;
		input_data.b = _b;
		sp_p_list.push_back(input_data);
	}
	void Model::add_sp_v(VHandle _v, float _r, float _g, float _b)
	{
		sp_v input_data;
		input_data.vh = _v;
		input_data.r = _r;
		input_data.g = _g;
		input_data.b = _b;
		sp_v_list.push_back(input_data);
	}
	void Model::add_sp_f(FHandle _f, float _r, float _g, float _b)
	{
		sp_f input_data;
		input_data.fh = _f;
		input_data.r = _r;
		input_data.g = _g;
		input_data.b = _b;
		sp_f_list.push_back(input_data);
	}
	void Model::clear_sp_p()
	{
		sp_p_list.clear();
	}
	void Model::clear_sp_v()
	{
		sp_v_list.clear();
	}
	void Model::clear_sp_f()
	{
		sp_f_list.clear();
	}
	/*======================================================================*/
	VHandle Model::addVertex(Point _p)
	{
		int find_result = findVertex(_p);
		if (find_result != -1)
		{
			return Mesh.vertex_handle(find_result);
		}
		else
		{
			return Mesh.add_vertex(_p);
		}
	}
	FHandle Model::addFace(VHandle _v0, VHandle _v1, VHandle _v2, VHandle _v3)
	{
		vector<VHandle> face_vhandles;

		face_vhandles.clear();
		face_vhandles.push_back(_v0);
		face_vhandles.push_back(_v1);
		face_vhandles.push_back(_v2);
		face_vhandles.push_back(_v3);
		return Mesh.add_face(face_vhandles);
	}
	void Model::deleteFace(FHandle _f)
	{
		Mesh.delete_face(_f);
		Mesh.garbage_collection();
	}
	void Model::deleteFace(VHandle _v0, VHandle _v1, VHandle _v2, VHandle _v3)
	{
		/*
		v1				v0
		*<--------------*
		|				|
		|				|
		|				|
		|		f		|
		|				|
		|				|
		|				|
		* --------------*
		v2				v3
		*/

		HEHandle v0v1 = Mesh.find_halfedge(_v0, _v1);
		if (v0v1.is_valid())
		{
			FHandle fh = Mesh.face_handle(v0v1);
			if (fh.is_valid())
			{
				Mesh.delete_face(fh);
				Mesh.garbage_collection();
			}
		}
	}
	/*======================================================================*/
	bool Model::IsVertexVertex(VHandle _vj, VHandle _vi)
	{
		for (VVIter vvit = Mesh.vv_iter(_vi); vvit; ++vvit)
			if (vvit.handle() == _vj)
				return true;
		return false;
	}
	/*======================================================================*/
	int Model::quad_subdivision1(int _face_id)
	{
		/*----------------------------------------------------------------------*/
		//�o�q�O���F�ѨMindex���D
		VHandle vq, vw, vt, vr;
		vq = addVertex(Point(0, 0, 100));
		vw = addVertex(Point(1, 0, 100));
		vt = addVertex(Point(1, 1, 100));
		vr = addVertex(Point(0, 1, 100));
		addFace(vq, vw, vt, vr);
		/*----------------------------------------------------------------------*/
		/*�����ݭnsubdivision��face*/
		//��ltable
		bool* table = new bool[Mesh.n_faces()];
		for (unsigned i = 0; i < Mesh.n_faces(); i++)
		{
			table[i] = false;
		}

		vector< FHandle > candidate_faces, last_found_face;
		last_found_face.push_back(Mesh.face_handle(_face_id));
		table[_face_id] = true;

		while (last_found_face.size() != 0)
		{
			vector< FHandle > new_found_faces;
			for (vector< FHandle >::iterator crnt_f = last_found_face.begin(); crnt_f != last_found_face.end(); ++crnt_f)
			{
				for (FFIter ff_itr = Mesh.ff_iter(*crnt_f); ff_itr; ++ff_itr)
				{
					if (table[ff_itr.handle().idx()] != true)
					{
						new_found_faces.push_back(ff_itr.handle());
						table[ff_itr.handle().idx()] = true;
					}
				}
			}
			for (vector< FHandle >::iterator f_itr = last_found_face.begin(); f_itr != last_found_face.end(); ++f_itr)
			{
				candidate_faces.push_back(*f_itr);
			}
			last_found_face.assign(new_found_faces.begin(), new_found_faces.end());
		}
		delete table;
		/*----------------------------------------------------------------------*/
		/*��candidate faces��subdivision*/
		/*
			v0		vd		v3
			*-------*-------*
			|		|		|
			|		|		|
			|	  ve|		|
		  va*-------*-------*vc
			|		|		|
			|		|		|
			|		|		|
			*-------*-------*
			v1		vb		v2
		*/
		for (vector< FHandle >::iterator face_itr = candidate_faces.begin(); face_itr != candidate_faces.end(); ++face_itr)
		{
			VHandle v[4], va, vb, vc, vd, ve;
			FVIter fv_itr = Mesh.fv_iter(*face_itr);
			for (int i = 0; fv_itr; ++fv_itr)
			{
				v[i++] = fv_itr.handle();
			}

			deleteFace(v[0], v[1], v[2], v[3]);
			va = addVertex((Mesh.point(v[0]) + Mesh.point(v[1])) / 2);
			vb = addVertex((Mesh.point(v[1]) + Mesh.point(v[2])) / 2);
			vc = addVertex((Mesh.point(v[2]) + Mesh.point(v[3])) / 2);
			vd = addVertex((Mesh.point(v[3]) + Mesh.point(v[0])) / 2);
			ve = addVertex((Mesh.point(v[0]) + Mesh.point(v[1]) + Mesh.point(v[2]) + Mesh.point(v[3])) / 4);
			addFace(vd, v[0], va, ve);
			addFace(va, v[1], vb, ve);
			addFace(vb, v[2], vc, ve);
			addFace(vc, v[3], vd, ve);
		}
		/*----------------------------------------------------------------------*/
		deleteFace(vq, vw, vt, vr);//�o��u�O���F�ѨMindex���D
		/*----------------------------------------------------------------------*/
		return 0;
	}
	int Model::quad_subdivision2(int _face_id)
	{
		/*----------------------------------------------------------------------*/
		//�o�q�O���F�ѨMindex���D
		VHandle vq, vw, vt, vr;
		vq = addVertex(Point(0, 0, 100));
		vw = addVertex(Point(1, 0, 100));
		vt = addVertex(Point(1, 1, 100));
		vr = addVertex(Point(0, 1, 100));
		addFace(vq, vw, vt, vr);
		/*----------------------------------------------------------------------*/
		/*�����ݭnsubdivision��face*/
		//��ltable
		bool* table = new bool[Mesh.n_faces()];
		for (unsigned i = 0; i < Mesh.n_faces(); i++)
		{
			table[i] = false;
		}

		vector< FHandle > candidate_faces, last_found_face;
		last_found_face.push_back(Mesh.face_handle(_face_id));
		table[_face_id] = true;

		while (last_found_face.size() != 0)
		{
			vector< FHandle > new_found_faces;
			for (vector< FHandle >::iterator crnt_f = last_found_face.begin(); crnt_f != last_found_face.end(); ++crnt_f)
			{
				for (FFIter ff_itr = Mesh.ff_iter(*crnt_f); ff_itr; ++ff_itr)
				{
					if (table[ff_itr.handle().idx()] != true)
					{
						new_found_faces.push_back(ff_itr.handle());
						table[ff_itr.handle().idx()] = true;
					}
				}
			}
			for (vector< FHandle >::iterator f_itr = last_found_face.begin(); f_itr != last_found_face.end(); ++f_itr)
			{
				candidate_faces.push_back(*f_itr);
			}
			last_found_face.assign(new_found_faces.begin(), new_found_faces.end());
		}
		delete table;
		/*----------------------------------------------------------------------*/
		/*��candidate faces��subdivision*/
		/*
			v0		vh		vg		v3
			*-------*-------*-------*
			|		|		|		|
			|		|		|		|
			|	  vi|	  vl|		|
		 va *-------*-------*-------*vf
			|		|		|		|
			|		|		|		|
			|	  vj|	  vk|		|
		 vb *-------*-------*-------*ve
			|		|		|		|
			|		|		|		|
			|		|		|		|
			*-------*-------*-------*
			v1		vc		vd		v2
		*/
		for (vector< FHandle >::iterator face_itr = candidate_faces.begin(); face_itr != candidate_faces.end(); ++face_itr)
		{
			VHandle v[4], va, vb, vc, vd, ve, vf, vg, vh, vi, vj, vk, vl;
			FVIter fv_itr = Mesh.fv_iter(*face_itr);
			for (int i = 0; fv_itr; ++fv_itr)
			{
				v[i++] = fv_itr.handle();
			}

			deleteFace(v[0], v[1], v[2], v[3]);
			va = addVertex((Mesh.point(v[0]) * 2 + Mesh.point(v[1])) / 3);
			vb = addVertex((Mesh.point(v[0]) + Mesh.point(v[1]) * 2) / 3);
			vc = addVertex((Mesh.point(v[1]) * 2 + Mesh.point(v[2])) / 3);
			vd = addVertex((Mesh.point(v[1]) + Mesh.point(v[2]) * 2) / 3);
			ve = addVertex((Mesh.point(v[2]) * 2 + Mesh.point(v[3])) / 3);
			vf = addVertex((Mesh.point(v[2]) + Mesh.point(v[3]) * 2) / 3);
			vg = addVertex((Mesh.point(v[3]) * 2 + Mesh.point(v[0])) / 3);
			vh = addVertex((Mesh.point(v[3]) + Mesh.point(v[0]) * 2) / 3);

			vi = addVertex((Mesh.point(vh) * 2 + Mesh.point(vc)) / 3);
			vj = addVertex((Mesh.point(vh) + Mesh.point(vc) * 2) / 3);
			vk = addVertex((Mesh.point(vd) * 2 + Mesh.point(vg)) / 3);
			vl = addVertex((Mesh.point(vd) + Mesh.point(vg) * 2) / 3);
			addFace(v[0], va, vi, vh);
			addFace(va, vb, vj, vi);
			addFace(vb, v[1], vc, vj);
			addFace(vc, vd, vk, vj);
			addFace(vd, v[2], ve, vk);
			addFace(ve, vf, vl, vk);
			addFace(vf, v[3], vg, vl);
			addFace(vg, vh, vi, vl);
			addFace(vi, vj, vk, vl);
		}
		/*----------------------------------------------------------------------*/
		deleteFace(vq, vw, vt, vr);//�o��u�O���F�ѨMindex���D
		/*----------------------------------------------------------------------*/
		return 0;
	}
	/*======================================================================*/
	int Model::findVertex(Point _p)
	{
		for (VIter v_itr = Mesh.vertices_begin(); v_itr != Mesh.vertices_end(); ++v_itr)
			if (Mesh.point(v_itr) == _p)
				return v_itr.handle().idx();
		return -1;
	}
	/*======================================================================*/
};
/*======================================================================*/
void Tri_Mesh::Render_Solid()
{
	FIter f_it;
	FVIter	fv_it;
	//glPushAttrib(GL_LIGHTING_BIT);
	glEnable(GL_POLYGON_OFFSET_FILL);
	glEnable(GL_LIGHTING);
	glPolygonOffset(2.0, 2.0);
	glBegin(GL_TRIANGLES);
	glColor4f(0.81, 0.74, 0.33, 0.3);
	for (f_it = faces_begin(); f_it != faces_end(); ++f_it)
	{
		for (fv_it = fv_iter(f_it); fv_it; ++fv_it)
		{
			glNormal3dv(normal(fv_it.handle()).data());
			glVertex3dv(point(fv_it.handle()).data());
		}
	}
	glEnd();

	glDisable(GL_POLYGON_OFFSET_FILL);


}


double* transformation(float* prom, float* vm, float x, float y, float z)
{
	float w;
	float tmp[4];
	double* res = new double[4];

	tmp[0] = vm[0] * x + vm[4] * y + vm[8] * z + vm[12] * 1;
	tmp[1] = vm[1] * x + vm[5] * y + vm[9] * z + vm[13] * 1;
	tmp[2] = vm[2] * x + vm[6] * y + vm[10] * z + vm[14] * 1;
	tmp[3] = vm[3] * x + vm[7] * y + vm[11] * z + vm[15] * 1;
	//printf("W: %f \n\n", w);


	res[0] = prom[0] * tmp[0] + prom[4] * tmp[1] + prom[8] * tmp[2] + prom[12] * tmp[3];
	res[1] = prom[1] * tmp[0] + prom[5] * tmp[1] + prom[9] * tmp[2] + prom[13] * tmp[3];
	res[2] = prom[2] * tmp[0] + prom[6] * tmp[1] + prom[10] * tmp[2] + prom[14] * tmp[3];
	w = prom[3] * tmp[0] + prom[7] * tmp[1] + prom[11] * tmp[2] + prom[15] * tmp[3];
	res[3] = w;
	w = abs((int)w);
	if (w != 0)
	{
		res[0] = res[0] / w;
		res[1] = res[1] / w;
		res[2] = res[2] / w;
		res[3] = res[3] / w;
	}

	return res;
}

void Tri_Mesh::Render_Voxel()
{
	if (voxel.size() > 0)
	{


		//glColor3f(0.0, 1.0, 0);
		glPointSize(5.0);
		glBegin(GL_POINTS);
		for (int i = 0; i < voxel.size(); i++)
		{
			for (int j = 0; j < voxel[i].size(); j++)
			{
				for (int k = 0; k < voxel[i][j].size(); k++)
				{
					glColor3d(voxel[i][j][k].V_color[0], voxel[i][j][k].V_color[1], voxel[i][j][k].V_color[2]);
					//std::cout << i << " " << j << " " << k << std::endl;
					for (int l = 0; l < voxel[i][j][k].Vertex.size(); l++)
					{
						glVertex3d(voxel[i][j][k].Vertex[l][0], voxel[i][j][k].Vertex[l][1], voxel[i][j][k].Vertex[l][2]);
					}

				}
			}
		}
		glEnd();



		glLineWidth(2.0);
		glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
		glBegin(GL_QUADS);
		//voxel.print_face_idx();		
		for (int i = 0; i < voxel.size(); i++)
		{
			for (int j = 0; j < voxel[i].size(); j++)
			{
				for (int k = 0; k < voxel[i][j].size(); k++)
				{

					glColor3d(voxel[i][j][k].V_color[0], voxel[i][j][k].V_color[1], voxel[i][j][k].V_color[2]);

					for (int l = 0; l < voxel[i][j][k].Face.size(); l++)
					{
						for (int m = 0; m < voxel[i][j][k].Face[l].size(); m++)
						{
							glVertex3d(voxel[i][j][k].Vertex[voxel[i][j][k].Face[l][m]][0], voxel[i][j][k].Vertex[voxel[i][j][k].Face[l][m]][1], voxel[i][j][k].Vertex[voxel[i][j][k].Face[l][m]][2]);
						}
					}

				}
			}
		}

		glEnd();
	}
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
}

void Tri_Mesh::Render_SolidWireframe()
{
	FIter f_it;
	FVIter	fv_it;
	VertexIter VIter;
	int count = 0;
	/*glPointSize(8.0);
	glBegin(GL_POINTS);
	glColor4f(1.0, 0.0, 0.0, 1.0);
	glVertex3dv(point(vertex_handle(0)));
	glEnd();*/

	//setFaceColor();
	glDisable(GL_LIGHTING);
	glPushAttrib(GL_LIGHTING_BIT);
	glEnable(GL_POLYGON_OFFSET_FILL);
	glEnable(GL_DEPTH_TEST);
	glPolygonOffset(2.0, 2.0);
	glBegin(GL_TRIANGLES);
	for (f_it = faces_begin(); f_it != faces_end(); ++f_it)
	{
		int faceid = f_it.handle().idx(); 
		//face color 
		glColor4f(fc[faceid].Color[0], fc[faceid].Color[1], fc[faceid].Color[2], 1);
		for (fv_it = fv_iter(f_it); fv_it; ++fv_it)
		{
			//glNormal3dv(normal(fv_it.handle()));
			glVertex3dv((point(fv_it.handle()) / getScale()).data());
			OpenMesh::Vec3d p1 = point(fv_it.handle());
			OpenMesh::Vec3d norm = normal(fv_it.handle());
		}
		count++;
	}
	glEnd();
	//glDisable(GL_POLYGON_OFFSET_FILL);
	if (state == ColorState::FACECLUSTER || state == ColorState::NFACECLUSTER) {
		state = 5;
		saperateFace();
		//findBoundary();
		colorPoint.clear();
		glPointSize(8.0);
		glBegin(GL_POINTS);
		
		glColor4f(1.0, 0.0, 0.0, 1.0);

		/*for (VIter = vertices_begin(); VIter != vertices_end(); ++VIter)
		{
			std::map <int, std::map<int, bool>>::iterator iter = boundaryID.begin();
			for (; iter != boundaryID.end(); iter++) {
				if (boundaryID[iter->first].find(VIter.handle().idx()) != boundaryID[iter->first].end()) {
					colorPoint[VIter.handle().idx()] = true;
					glVertex3dv((point(VIter.handle()) / getScale()).data());
				}
			}

			// Get vertex handle
		}*/
		glEnd();
	}
	glPointSize(8.0);
	glBegin(GL_POINTS);
	glColor4f(1.0, 0.0, 0.0, 1.0);

	//if (Allmeshs.size() > 0) {
	//	//std::cout << "------------new_face------------\n";
	//	for (int i = 0; i < Allmeshs.size(); i++) {
	//		OMT::Model temp = Allmeshs[i];
	//		//std::cout << "------face " << i << ":------\n";
	//		for (VIter = Allmeshs[i].vertices_begin(); VIter != Allmeshs[i].vertices_end(); ++VIter) {
	//			Point p = temp.point(VIter.handle());
	//			//std::cout << "vertex " << i << ":   " << p << "\n";
	//			if (temp.is_boundary(VIter))
	//			{
	//				glVertex3dv((temp.point(VIter.handle()) / getScale()).data());

	//			}
	//			//glVertex3dv((temp.point(VIter.handle()) / getScale()).data());
	//		}
	//	}
	//	//std::cout << "------------new_face------------\n";
	//	
	//}
	//else {
	//	for (VIter = vertices_begin(); VIter != vertices_end(); ++VIter)
	//	{
	//		if (colorPoint.find(VIter.handle().idx()) != colorPoint.end())
	//			glVertex3dv((point(VIter.handle()) / getScale()).data());
	//		// Get vertex handle
	//	}
	//}
	glEnd();
	glPushAttrib(GL_LIGHTING_BIT);
	glDisable(GL_LIGHTING);
	glLineWidth(1.0);
	glColor3f(0.0, 0.0, 0.0);
	glBegin(GL_LINES);
	for (OMT::EIter e_it = edges_begin(); e_it != edges_end(); ++e_it)
	{
		OMT::HEHandle _hedge = halfedge_handle(e_it.handle(), 1);

		OMT::Point curVertex = point(from_vertex_handle(_hedge));
		glVertex3dv((curVertex / getScale()).data());

		curVertex = point(to_vertex_handle(_hedge));
		glVertex3dv((curVertex / getScale()).data());
	}

	glEnd();
	/*
	double tmp[3][3];
	int cnt = 0;
	double * vec;
	float promatrix[16];
	float viematrix[16];
	glGetFloatv(GL_PROJECTION_MATRIX, promatrix);
	glGetFloatv(GL_MODELVIEW_MATRIX, viematrix);

	glPointSize(30.0);
	glBegin(GL_POINTS);
			glColor3f(1.0, 0.0, 0.0);
			for(int i = 0 ;i<ctr.size() ; i++)
					vec = transformation(promatrix, viematrix, ctr[i][0], ctr[i][1], ctr[i][2]);
			if (vec)
			{
				glVertex4dv(vec);
			}
			
	for (f_it = faces_begin(); f_it != faces_end(); ++f_it)
	{
		int faceid = f_it.handle().idx();
		cnt = 0;
		for (fv_it = fv_iter(f_it); fv_it; ++fv_it)
		{
			tmp[cnt][0]= (point(fv_it.handle()) / getScale())[0];
			tmp[cnt][1] = (point(fv_it.handle()) / getScale())[1];
			tmp[cnt][2] = (point(fv_it.handle()) / getScale())[2];
			cnt++;
		}
		glVertex3d((tmp[0][0] + tmp[1][0] + tmp[2][0]) / 3, (tmp[0][1] + tmp[1][1] + tmp[2][1]) / 3, (tmp[0][2] + tmp[1][2] + tmp[2][2]) / 3);
	}
	glEnd();*/
	/*
	if (hull.size() > 1)
	{
		glColor3f(1.0, 0, 0);
		glLineWidth(20.0);
		glBegin(GL_LINES);
		for (int i = 0; i < hull.size(); i++)
		{
			glVertex3d(hull[i][0], hull[i][1], hull[i][2]);
		}
		glEnd();
	}*/



	glPopAttrib();
}

void Tri_Mesh::Render_newMesh()
{
	glDisable(GL_LIGHTING);
	glPushAttrib(GL_LIGHTING_BIT);
	glEnable(GL_POLYGON_OFFSET_FILL);
	glEnable(GL_DEPTH_TEST);
	glPolygonOffset(2.0, 2.0);
	glBegin(GL_TRIANGLES);
	for (int i = 0; i < Allmeshs.size(); i++) {
		Tri_Mesh temp = Allmeshs[i];
		for (FIter f_it = temp.faces_begin(); f_it != temp.faces_end(); ++f_it)
		{
			int faceid = f_it.handle().idx();
			glColor4f(0.6, 0.6, 0.6, 1);
			for (FVIter fv_it = temp.fv_iter(f_it); fv_it; ++fv_it)
			{
				//glNormal3dv(normal(fv_it.handle()));
				glVertex3dv((temp.point(fv_it.handle()) / getScale()).data());
			}
		}
	}
	glEnd();

	glPointSize(8.0);
	glBegin(GL_POINTS);
	glColor4f(1.0, 0.0, 0.0, 1.0);
	/*for (int i = 0; i < Allmeshs.size(); i++) {
		Tri_Mesh temp = Allmeshs[i];
		for (VIter v_it = temp.vertices_begin(); v_it != temp.vertices_end(); ++v_it)
		{
			if (temp.is_boundary(v_it))
			{
				glVertex3dv((temp.point(v_it.handle()) / getScale()).data());
			}
		}
	}*/
	glEnd();

	glLineWidth(1.0);
	glColor3f(0.0, 0.0, 0.0);
	glBegin(GL_LINES);
	for (int i = 0; i < Allmeshs.size(); i++) {
		Tri_Mesh temp = Allmeshs[i];
		for (OMT::EIter e_it = temp.edges_begin(); e_it != temp.edges_end(); ++e_it)
		{
			OMT::HEHandle _hedge = temp.halfedge_handle(e_it.handle(), 1);

			OMT::Point curVertex = temp.point(temp.from_vertex_handle(_hedge));
			glVertex3dv((curVertex / getScale()).data());

			curVertex = temp.point(temp.to_vertex_handle(_hedge));
			glVertex3dv((curVertex / getScale()).data());
		}
	}
	glEnd();

	glPopAttrib();
}

void Tri_Mesh::Render_Wireframe()
{
	//glPushAttrib(GL_LIGHTING_BIT);	
	glDisable(GL_LIGHTING);
	glLineWidth(1.0);

	glColor3f(0.0, 0.0, 0.0);

	glBegin(GL_LINES);
	for (OMT::EIter e_it = edges_begin(); e_it != edges_end(); ++e_it)
	{
		OMT::HEHandle _hedge = halfedge_handle(e_it.handle(), 1);

		OMT::Point curVertex = point(from_vertex_handle(_hedge));
		glVertex3dv(curVertex.data());

		curVertex = point(to_vertex_handle(_hedge));
		glVertex3dv(curVertex.data());
	}
	glEnd();

}

void Tri_Mesh::Render_Point()
{
	glPointSize(8.0);
	glColor3f(1.0, 0.0, 0.0);
	glBegin(GL_POINTS);
	for (OMT::VIter v_it = vertices_begin(); v_it != vertices_end(); ++v_it)
	{
		glVertex3dv(point(v_it).data());
	}
	glEnd();
}



void Tri_Mesh::setFaceColor()
{


}

void Tri_Mesh::setCloseFaceColor()
{

}

void Tri_Mesh::calFaceData()
{
	std::vector <faceData>().swap(fd);
	fd.clear();
	OpenMesh::Vec3d symmetric(0.0f, 0.0f, 1.0f);
	FIter f_it;
	FVIter	fv_it;
	VertexIter VIter;

	for (VIter = vertices_begin(); VIter != vertices_end(); VIter++) {
	//	std::cout << VIter.handle().idx() << std::endl;
		objVertices.push_back(point(VIter.handle()));
	}
	for (f_it = faces_begin(); f_it != faces_end(); ++f_it)
	{
		faceData face(f_it.handle().idx());
		face.setFaceNormal(calc_face_normal(f_it.handle()), symmetric);
		OpenMesh::Vec3d fnorm = calc_face_normal(f_it.handle());
		for (fv_it = fv_iter(f_it); fv_it; ++fv_it)
		{
		//	std::cout << fv_it.handle().idx()<< "  ";
			//glNormal3dv(normal(fv_it.handle()));
			//glVertex3dv((point(fv_it.handle()) / scale).data());
			OpenMesh::Vec3d p1 = point(fv_it.handle());
			OpenMesh::Vec3d norm = normal(fv_it.handle());
			face.addvertex(p1, fv_it.handle().idx());
		}
		//std::cout << std::endl;
		face.cal_area();
		face.caclcenter();
		face.checkData();
		face.f_it = f_it;
		//std::cout << face;
		fd.push_back(face);
		if (!closeNormal(OpenMesh::Vec3d(0, -1, 0), face.getFaceNormal()) && face.getArea() > maxArea) {
			maxArea = face.getArea();
			maxAreaID = fd.size() - 1;
		}
		std::map <Point, float>::iterator it;
		for (it = normalcluster.begin(); it != normalcluster.end(); it++) {
			if (closeNormal(it->first, face.getFaceNormal()))
			{
				it->second += face.getArea();
				break;
			}
		}
		if (normalcluster.size() == 0) {
			maxNormalArea.first = face.getFaceNormal();
			maxNormalArea.second = face.getArea();
		}
		else if (it == normalcluster.end())
		{
			normalcluster[face.getFaceNormal()] += face.getArea();
			if (normalcluster[face.getFaceNormal()] > maxNormalArea.second)
			{
				maxNormalArea.first = face.getFaceNormal();
				maxNormalArea.second = face.getArea();
			}
		}
	}
	std::cout << "index: " << maxAreaID << "   Maxarea: " << maxArea;
}

void Tri_Mesh::setCloseFaceColor(int& count, int f_id, std::vector <int>& faceCheck)
{


}

int Tri_Mesh::facefaceCount()
{
	state = ColorState::CONNECT;
	std::map <int, int> faceCheck;
	for (FaceIter f_it = faces_begin(); f_it != faces_end(); f_it++)
		faceCheck[f_it.handle().idx()] = 0;
	int count = 0;
	for (FaceIter f_it = faces_begin(); f_it != faces_end(); f_it++)
		if (faceCheck[f_it.handle().idx()] == 0) {
			faceCheck[f_it.handle().idx()] = count + 1;
			facefaceCount(faceCheck, f_it.handle().idx(), count);
			count++;
		}
	std::vector <faceColor>().swap(fc);
	fc.clear();
	std::map <int, int>::iterator cur;
	int i = 0;
	for (cur = faceCheck.begin(); cur != faceCheck.end(); cur++) {
		fc.push_back(faceColor());
		fc[i].Color[0] = 0.15 * (cur->second % 7);
		fc[i].Color[1] = 0.25 * (cur->second % 5);
		fc[i].Color[2] = 0.12 * (cur->second % 9);
		i++;
	}
	return count;
}

void Tri_Mesh::facefaceCount(std::map <int, int>& faceCheck, int f_id, int count)
{
	FIter f_it = faces_begin();
	int x = 0;
	for (x = 0; x < f_id; f_it++, x++);
	for (FFIter ff_it = ff_iter(f_it); ff_it; ff_it++) {
		if (faceCheck[ff_it.handle().idx()] == 0) {
			faceCheck[ff_it.handle().idx()] = count + 1;
			facefaceCount(faceCheck, ff_it.handle().idx(), count);
		}
	}
}
void Tri_Mesh::faceCluster(bool close)
{
	calFaceData();
	if (close)	state = ColorState::FACECLUSTER;
	else   state = ColorState::NFACECLUSTER;
	std::vector <int> faceCheck(fd.size(), 0);
	std::vector <std::map <int, bool>> clusterVec;
	std::vector <int> represent;
	usedColor.clear();
	int areaID = maxAreaID;
	double area = maxArea;
	bool allCheck = true;
	int count = 0;
	// ���u�Hnormal�����s -> represent
	do {
		count++;
		allCheck = true;
		std::map <int, bool> clusterMap;
		clusterMap[areaID] = false;
		represent.push_back(areaID);
		faceCheck[areaID] = count;
		std::vector <faceColor> color;
		fc[areaID].Color[0] = 0.15 * (areaID % 7);
		fc[areaID].Color[1] = 0.25 * (areaID % 5);
		fc[areaID].Color[2] = 0.12 * (areaID % 9);
		fc[areaID].ColorID = areaID;
		color.push_back(fc[areaID]);
		for (int i = 0; i < fd.size(); i++) {
			if (fd[i].getArea() == 0 || isnan(fd[i].getArea())) faceCheck[i] = -1;
			if (!fd[i].isAllow())  // �L�k�B�z����
			{
				faceCheck[i] = -1;
				fc[i].Color[0] = 0;
				fc[i].Color[1] = 0;
				fc[i].Color[2] = 0;
			}
			if (faceCheck[i] != 0) continue;
			// Normal �۪񪺤��s  �]���P�C��
			if (closeNormal(fd[areaID].getFaceNormal(), fd[i].getFaceNormal()))
			{
				fc[i].Color = fc[areaID].Color;
				fc[i].ColorID = areaID;
				faceCheck[i] = count;
				clusterMap[i] = false;
			}
			else allCheck = false;
		}
		clusterVec.push_back(clusterMap);
		if (!allCheck)  // �p�G�٦����B�z���� �A�q�̤j���n���B�z
		{
			area = 1e-15;
			areaID = 0;
			for (FaceIter f_it = faces_begin(); f_it != faces_end(); f_it++) {
				if (faceCheck[f_it.handle().idx()] == 0 &&
					fd[f_it.handle().idx()].getArea() > area) {
					area = fd[f_it.handle().idx()].getArea();
					areaID = f_it.handle().idx();
				}
			}
		}
	} while (!allCheck);
	std::cout << "cluster: " << count;
	std::vector <int> colorCount;
	//closeFace  
	// �̷ӶZ�������P�~����s -> subClusterID  -> faceVec
	if (close) {
		for (int i = 0; i < represent.size(); i++) {
			colorCount.push_back(0);
			OpenMesh::Vec3d curColor = (fc[represent[i]].Color);
			int curID = represent[i];
			int curCID = fc[represent[i]].ColorID;
			double subMaxArea = 1e-15;
			do {
				clusterVec[i][curID] = true;
				colorCount[colorCount.size() - 1]++;
				std::map <int, bool>::iterator iter = clusterVec[i].begin();
				int nextID = -1;
				bool find = false;
				double subClusterArea = 0;
				std::vector <int> subClusterID;
				subClusterID.push_back(curID);    // ���s�N��
				if (!isnan(fd[curID].getArea()))
					subClusterArea += fd[curID].getArea();
				usedColor[curCID] = true;
				allCheck = true;
				do {
					find = false;

					std::vector <int> oneRingVec;
					oneRingVec.push_back(curID);
					while (oneRingVec.size() != 0)
					{
						int total = 0;
						int notClust = 0;
						FaceHandle fh = face_handle(oneRingVec[0]);
						for (FFIter ff = ff_begin(fh); ff; ++ff, ++total)
						{
							int ffidx = ff.handle().idx();
							if (clusterVec[i].find(ffidx) != clusterVec[i].end() && !clusterVec[i][ffidx])
							{
								fc[ffidx].Color = curColor;
								fc[ffidx].ColorID = curCID;
								clusterVec[i][ffidx] = true;
								find = true;
								subClusterID.push_back(ffidx);
								if (!isnan(fd[ffidx].getArea()))
									subClusterArea += fd[ffidx].getArea();
								oneRingVec.push_back(ffidx);
							}
						}
						oneRingVec.erase(oneRingVec.begin());
					}

					for (; iter != clusterVec[i].end(); iter++)  // �q���e�����s���M��
					{
						if (!iter->second) {     // �p�G�٨S�Q�G�����s
							int j;
							for (j = 0; j < subClusterID.size(); j++) {
								if (faceConnect(fd[iter->first].vID, fd[subClusterID[j]].vID) || closeFace(fd[iter->first].getVertices(), fd[subClusterID[j]].getVertices()))
								{
									// �Z��subCluster�۪񪺤���P�� subClusterID
									fc[iter->first].Color = curColor;
									fc[iter->first].ColorID = curCID;
									iter->second = true;
									find = true;
									subClusterID.push_back(iter->first);
									if (!isnan(fd[iter->first].getArea()))
										subClusterArea += fd[iter->first].getArea();
									break;
								}
							}
							if (j == subClusterID.size()) {
								// �p�G�Z������ �U�@�ӱq�L�}�l
								if (allCheck || fd[iter->first].getArea() > subMaxArea)
								{
									nextID = iter->first;
									subMaxArea = fd[iter->first].getArea();
								}
								allCheck = false;
							}
						}
					}
				} while (find);
				faceVec.push_back(subClusterID);
				faceVecArea.push_back(subClusterArea);
				if (!allCheck)
				{
					// �]�w�U�@�Ӥ��s���C��
					fc[nextID].Color[0] = 0.15 * (nextID % 7);
					fc[nextID].Color[1] = 0.25 * (nextID % 5);
					fc[nextID].Color[2] = 0.12 * (nextID % 9);
					fc[nextID].ColorID = nextID;
					curColor = fc[nextID].Color;
					curCID = nextID;
					curID = nextID;
				}
			} while (!allCheck);
		}
	}

	// deal with the small face
	std::vector <std::map<int, bool>> faceVecMap;
	std::vector <std::map<int, bool>> faceVecConnect;
	std::vector <bool> faceVecCheck;
	for (int i = 0; i < faceVec.size(); i++)
	{
		faceVecCheck.push_back(true);
		faceVecMap.push_back(std::map<int, bool>());
		faceVecConnect.push_back(std::map<int, bool>());
		for (int j = 0; j < faceVec[i].size(); j++)
		{
			faceVecMap[i][faceVec[i][j]] = true;
		}
	}
	for (int i = 0; i < faceVec.size(); i++)
	{
		for (int j = 0; j < faceVec.size(); j++)
		{
			if (i != j)
			{
				std::map<int, bool>::iterator iter;
				bool isConnect = false;
				for (iter = faceVecMap[i].begin(); iter != faceVecMap[i].end(); iter++)
				{
					for (FFIter ff = ff_begin(face_handle(iter->first)); ff; ++ff)
					{
						// if face i and face j is connected
						if (faceVecMap[j].find(ff.handle().idx()) != faceVecMap[j].end())
						{
							faceVecConnect[i][j] = true;
							faceVecConnect[j][i] = true;
							isConnect = true;
							break;
						}
					}
					if (isConnect) break;
				}
			}
		}
	}
	bool find = false;
	do {
		find = false;
		for (int i = 0; i < faceVecConnect.size(); i++)
		{
			if (faceVecConnect[i].size() == 1 && faceVecCheck[i])
			{
				int j = faceVecConnect[i].begin()->first;
				if (faceVecArea[i] < faceVecArea[j] / 20.0)
				{
					for (int x = 0; x < faceVec[i].size(); x++)
					{
						if (faceVecMap[j].find(faceVec[i][x]) == faceVecMap[j].end())
						{
							faceVecMap[j][faceVec[i][x]] = true;
							faceVec[j].push_back(faceVec[i][x]);
							fc[faceVec[i][x]].Color = fc[faceVec[j][0]].Color;
							faceCheck[i] = false;
							find = true;
							break;
						}
					}
				}
				if (find)
				{
					faceVecArea[j] += faceVecArea[i];
					break;
				}
			}
		}
	} while (find);
	std::vector<std::vector<int>> tempfaceVec;
	for (int i = 0; i < faceVec.size(); i++)
	{
		if (faceVecCheck[i])
			tempfaceVec.push_back(faceVec[i]);
	}
	faceVec.clear();
	std::vector<std::vector<int>>().swap(faceVec);
	faceVec.assign(tempfaceVec.begin(), tempfaceVec.end());
}
float Tri_Mesh::differential(clustergraph a, clustergraph b)
{
	float fup, xdown;
	fup = (b.normal[0] - a.normal[0]) + (b.normal[1] - a.normal[1])+ (b.normal[2] - a.normal[2]);
	xdown = caculadistance( a.center, b.center);

	return fup / xdown;
}

OpenMesh::Vec3f Tri_Mesh::getclustercenter(std::vector<int> faceid) 
{
	float sx, sy, sz;
	for (int i = 0; i < faceid.size(); i++) 
	{
		sx += fd[faceid[i]].getfcenter()[0];
		sy += fd[faceid[i]].getfcenter()[1];
		sz += fd[faceid[i]].getfcenter()[2];
	}
	sx /= faceid.size();
	sy /= faceid.size();
	sz /= faceid.size();
	OpenMesh::Vec3f center;
	center[0] = sx;
	center[1] = sy;
	center[2] = sz;
	return center;
}


/*
bool Tri_Mesh::intersect(std::vector<OpenMesh::Vec3d> a, std::vector<OpenMesh::Vec3d> b) {
	if((a.xmin <= b.xmax && a.xmax >= b.xmin) &&
		(a.ymin <= b.ymax && a.ymax >= b.ymin) &&
		(a.zmin <= b.zmax && a.zmax >= b.zmin))
		{
			return true;
		}
	
	return false;
}*/

void Tri_Mesh::contructbbox(clustergraph *cg)
{
	float xmax = -10000, xmin = 10000, ymax = -10000, ymin = 10000, zmax = -1000, zmin = 1000;
	std::vector<OpenMesh::Vec3d> verts;

	for (int i = 0; i < cg->faceid.size(); i++) 
	{
		for (int j = 0; j < 3; j++)
		{
			verts.push_back(fd[cg->faceid[i]].getVertices()[j]);
		}
	}

	/*
	for (int i = 0; i < objVertices.size();i++)
	{
		if (objVertices[i][0]> xmax)
		{
			cg->m_bbox.xmax = objVertices[i][0];
		}
		if (objVertices[i][0] < xmin)
		{
			cg->m_bbox.xmin = objVertices[i][0];
		}
		//----------------
		if (objVertices[i][1] > ymax)
		{
			cg->m_bbox.ymax = objVertices[i][1];
		}
		if (objVertices[i][1] < ymin)
		{
			cg->m_bbox.ymin = objVertices[i][1];
		}
		//---------------------
		if (objVertices[i][2] > zmax)
		{
			cg->m_bbox.zmax = objVertices[i][2];
		}
		if (objVertices[i][2] < zmin)
		{
			cg->m_bbox.zmin = objVertices[i][2];
		}
	
	}
	*/
	/*
	cg->bbox[0] = OpenMesh::Vec3f(xmin, ymin, zmin);
	cg->bbox[1] = OpenMesh::Vec3f(xmax, ymin, zmin);
	cg->bbox[2] = OpenMesh::Vec3f(xmax, ymin, zmax);
	cg->bbox[3] = OpenMesh::Vec3f(xmin, ymin, zmax);
	cg->bbox[4] = OpenMesh::Vec3f(xmax, ymax, zmax);
	cg->bbox[5] = OpenMesh::Vec3f(xmax, ymax, zmin);
	cg->bbox[6] = OpenMesh::Vec3f(xmin, ymax, zmin);
	cg->bbox[7] = OpenMesh::Vec3f(xmin, ymax, zmax);
	*/
	/*
	float delta =sqrtf( pow((xmax - xmin),2 )+ pow((ymax - ymin),2) + pow((zmax - zmin),2)) *(0.5)*(0.05f);

	float bxmin = xmin - delta  ,  bxmax = xmax+delta;
	float bymin = ymin - delta, bymax = ymax + delta;
	float bzmin = zmin - delta, bzmax = zmax + delta;
	
	Tri_Mesh tmpmesh ;


	Tri_Mesh::VertexHandle vhandle[8];

	vhandle[0] = tmpmesh.add_vertex( Tri_Mesh::Point(xmin, ymin, zmin));
	vhandle[1] =  tmpmesh.add_vertex(Tri_Mesh::Point(xmax, ymin, zmin));
	vhandle[2] =  tmpmesh.add_vertex(Tri_Mesh::Point(xmax, ymin, zmax));
	vhandle[3] =  tmpmesh.add_vertex(Tri_Mesh::Point(xmin, ymin, zmax));
	vhandle[4] =  tmpmesh.add_vertex(Tri_Mesh::Point(xmax, ymax, zmax));
	vhandle[5] =  tmpmesh.add_vertex(Tri_Mesh::Point(xmax, ymax, zmin));
	vhandle[6] =  tmpmesh.add_vertex(Tri_Mesh::Point(xmin, ymax, zmin));
	vhandle[7] =  tmpmesh.add_vertex(Tri_Mesh::Point(xmin, ymax, zmax));
	std::vector<Tri_Mesh::VertexHandle>  face_vhandles;
	face_vhandles.clear();
	face_vhandles.push_back(vhandle[0]);
	face_vhandles.push_back(vhandle[1]);
	face_vhandles.push_back(vhandle[2]);
	face_vhandles.push_back(vhandle[3]);
	 tmpmesh.add_face(face_vhandles);

	face_vhandles.clear();
	face_vhandles.push_back(vhandle[1]);
	face_vhandles.push_back(vhandle[5]);
	face_vhandles.push_back(vhandle[4]);
	face_vhandles.push_back(vhandle[2]);
	 tmpmesh.add_face(face_vhandles);
	face_vhandles.clear();
	face_vhandles.push_back(vhandle[1]);
	face_vhandles.push_back(vhandle[0]);
	face_vhandles.push_back(vhandle[6]);
	face_vhandles.push_back(vhandle[5]);
	 tmpmesh.add_face(face_vhandles);
	face_vhandles.clear();
	face_vhandles.push_back(vhandle[4]);
	face_vhandles.push_back(vhandle[5]);
	face_vhandles.push_back(vhandle[6]);
	face_vhandles.push_back(vhandle[7]);
	 tmpmesh.add_face(face_vhandles);
	face_vhandles.clear();
	face_vhandles.push_back(vhandle[0]);
	face_vhandles.push_back(vhandle[3]);
	face_vhandles.push_back(vhandle[7]);
	face_vhandles.push_back(vhandle[6]);
	 tmpmesh.add_face(face_vhandles);
	face_vhandles.clear();
	face_vhandles.push_back(vhandle[2]);
	face_vhandles.push_back(vhandle[4]);
	face_vhandles.push_back(vhandle[7]);
	face_vhandles.push_back(vhandle[3]);
	 tmpmesh.add_face(face_vhandles);

	 bboxmesh.push_back(tmpmesh);

	 std::cout << "Contruct Boundary box successful " << std::endl;
	try
	{
		if (!OpenMesh::IO::write_mesh(tmpmesh, "bboxmesh.obj"))
		{
			std::cerr << "Cannot write mesh to file 'output.off'" << std::endl;
		}
		std::cout << "Contruct Boundary box successful " << std::endl;
	}
	catch (std::exception& x)
	{
		std::cerr << x.what() << std::endl;
	}*/
}

std::vector<Cluster> Tri_Mesh::ada_planedistcluster(std::vector <Cluster > cl, std::vector <Cluster >& output, float thre)
{
	std::map<int, bool> facecheck;
	OpenMesh::Vec3d p_normal;
	OpenMesh::Vec3d p_center;
	float max_dist = 0;
	double dist;

	for (int i = 0; i < cl.size(); i++)
	{
		for (int j = 0; j < cl[i].faceid.size(); j++)
		{
			facecheck[cl[i].faceid[j]] = false;
		}
		max_dist = calplanemaxdist(cl[i].faceid, fd);

		for (int j = 0; j < cl[i].faceid.size(); j++)
		{
			if (facecheck[cl[i].faceid[j]] == false)
			{
				Cluster tmp;
				for (int k = 0; k < cl[i].faceid.size(); k++)
				{
					if (facecheck[cl[i].faceid[k]] == false)
					{
						dist = calplanedist(fd[cl[i].faceid[j]].getFaceNormal(), fd[cl[i].faceid[j]].getfcenter(), fd[cl[i].faceid[k]].getfcenter());
						//std::cout << dist << std::endl;

						if (dist / max_dist<thre)//0.08
						{
							facecheck[cl[i].faceid[k]] = true;
							//output[*outputidx].faceid.push_back(cl[i].faceid[k]);
							tmp.faceid.push_back(cl[i].faceid[k]);
						}
					}
				}
				output.push_back(tmp);
				//*outputidx += 1;
			}
		}

	}

	return output;
}

std::vector <Cluster > Tri_Mesh::planedistcluster(std::vector <Cluster > cl , std::vector <Cluster > &output )
{
	std::map<int, bool> facecheck;
	OpenMesh::Vec3d p_normal;
	OpenMesh::Vec3d p_center;

	double dist;
	for (int i = 0; i <cl.size() ; i++) 
	{
		for (int j = 0; j < cl[i].faceid.size(); j++) 
		{
			facecheck[cl[i].faceid[j]] = false;
		}
		for(int j =0 ; j< cl[i].faceid.size(); j++)
		{
			if (facecheck[cl[i].faceid[j]]==false)
			{
				Cluster tmp;
				for (int k = 0; k < cl[i].faceid.size(); k++) 
				{
					if (facecheck[cl[i].faceid[k]] == false)
					{
						dist = calplanedist(fd[cl[i].faceid[j]].getFaceNormal(), fd[cl[i].faceid[j]].getfcenter(), fd[cl[i].faceid[k]].getfcenter());
						//std::cout << dist << std::endl;
						
						if (dist <0.04)//0.08
						{
							facecheck[cl[i].faceid[k]] = true;
							//output[*outputidx].faceid.push_back(cl[i].faceid[k]);
							tmp.faceid.push_back(cl[i].faceid[k]);
						}
					}
				}
				output.push_back(tmp);
				//*outputidx += 1;
			}
		}

	}



	return output;
}


void Tri_Mesh::InitialAdjGraph()
{
	for (int i = 0; i < result.size(); i++)
	{
		std::vector<float> tmp;
		tmp.resize(result.size());
		adjmatrix.push_back(tmp);
	}

	for(int i =0 ; i<adjmatrix.size();i++)
	{
		for(int j = 0 ; j< adjmatrix[i].size();j++)
		{
			adjmatrix[i][j] = -1;
		}
	}

	for(int i =0 ; i < result.size();i++)
	{
		for (int j = i+1; j < result.size();j++)
		{
			if(isconnected(result[i],result[j]))
			{
				adjmatrix[i][j] = 1;
			}
		}
	}
	/*
	for (int i = 0; i < result.size(); i++)
	{
		for (int j = i; j < result.size(); j++)
		{
			if (adjmatrix[i][j] == 1) 
			{
				std::cout << i << "  " << j << " " << adjmatrix[i][j] << std::endl;
			}
		}
		*/

	for (int i = 0; i < result.size(); i++)
	{
		std::cout << std::right << std::setw(10) << std::fixed << " " << i;
	}
	std::cout << std::endl;
	for (int i = 0; i < result.size(); i++)
	{
		std::cout << i;
		for (int j = 0; j < result.size(); j++)
		{
			std::cout << std::right << std::setw(10) << std::fixed << " " << adjmatrix[i][j];
		}
		std::cout << std::endl;
	}

		std::cout<< std::endl;
	
		std::cout  << "stop" << std::endl;
}

bool Tri_Mesh::isconnected(Cluster A, Cluster B) 
{
	int count = 0;
	for (int i = 0; i < A.faceid.size(); i++) 
	{
		for (int j = 0; j < B.faceid.size(); j++) 
		{
			count = 0;
			
			for (int k = 0; k < 3; k++) 
			{
				for (int l = 0; l < 3; l++)
				{
					if (fd[A.faceid[i]].getVertices()[k] == fd[B.faceid[j]].getVertices()[l])
					{
						count++;
					}
					if (count >= 2)
					{
						return true;
					}
				}
			}
		}
	}

	return false;
}

void Tri_Mesh::InitailWeight(std::vector<Cluster> output)
{
	for (int i = 0; i < output.size(); i++)
	{
		std::vector<float> tmp;
		tmp.resize(output.size());
		adjmatrix.push_back(tmp);
	}
	for (int i = 0; i < output.size(); i++)
	{
		for (int j = 0; j < output.size(); j++)
		{
			adjmatrix[i][j] = -1;
		}
	}

	for (int i = 0; i < output.size(); i++)
	{
		for (int j = 0; j < output.size(); j++)
		{
			if (isconnected(output[i], output[j]))
			{
				float sum_sigma = 0;
				float eudist = 0;
				sum_sigma = output[i].facearea + output[j].facearea;
				//eudist = findfarestdist(fd, output[i], output[j]);
				eudist = pow((fd[output[i].faceid[0]].getfcenter()[0] - fd[output[j].faceid[0]].getfcenter()[0]), 2) + pow((fd[output[i].faceid[0]].getfcenter()[1] - fd[output[j].faceid[0]].getfcenter()[1]), 2) + pow((fd[output[i].faceid[0]].getfcenter()[2] - fd[output[j].faceid[0]].getfcenter()[2]), 2);
				adjmatrix[i][j] = sqrtf((eudist / sum_sigma));
			}
		}
	}
}

void Tri_Mesh::UpdaeWeight(int index_i , int index_j) 
{

		for (int j = index_j; j < result.size(); j++)
		{
			if (isconnected(result[index_i], result[j]))
			{
				float sum_sigma = 0;
				float eudist = 0;
				sum_sigma = result[index_i].facearea + result[j].facearea;
				//eudist = findfarestdist(fd, output[i], output[j]);
				eudist = pow((fd[result[index_i].faceid[0]].getfcenter()[0] - fd[result[j].faceid[0]].getfcenter()[0]), 2) + pow((fd[result[index_i].faceid[0]].getfcenter()[1] - fd[result[j].faceid[0]].getfcenter()[1]), 2) + pow((fd[result[index_i].faceid[0]].getfcenter()[2] - fd[result[j].faceid[0]].getfcenter()[2]), 2);
				adjmatrix[index_i][j] = sqrtf((eudist / sum_sigma));
			}
		}
	
}


void Tri_Mesh::voxelize(float scale)
{
	calFaceData();
	std::vector<OpenMesh::Vec3f> v;
	tstart = std::clock();
	for (int i = 0; i < objVertices.size(); i++)
	{
		v.push_back(OpenMesh::Vec3f(objVertices[i][0], objVertices[i][1], objVertices[i][2]));
	}

	bbox = computebbox(v);

	float xlength, ylength, zlength;

	xlength = abs(bbox[0] - bbox[1]);
	ylength = abs(bbox[2] - bbox[3]);
	zlength = abs(bbox[4] - bbox[5]);

	float xoffset, yoffset, zoffset;


	xcellsize = xlength / (xlength + ylength + zlength) * 100 * scale;
	ycellsize = ylength / (xlength + ylength + zlength) * 100 * scale;
	zcellsize = zlength / (xlength + ylength + zlength) * 100 * scale;
	

	


	xoffset = xlength / (xcellsize - 1);
	yoffset = ylength / (ycellsize - 1);
	zoffset = zlength / (zcellsize - 1);

	voxel.resize(xcellsize);
	for (int i = 0; i < voxel.size(); i++)
	{
		voxel[i].resize(ycellsize);
		for (int j = 0; j < voxel[i].size(); j++)
		{
			voxel[i][j].resize(zcellsize);
		}
	}

	std::cout << std::endl << "x cell size =  " << xcellsize << std::endl;
	std::cout << "y cell size =  " << ycellsize << std::endl;
	std::cout << "z cell size =  " << zcellsize << std::endl;

	for (int i = 0; i < voxel.size(); i++)
	{
		for (int j = 0; j < voxel[i].size(); j++)
		{
			for (int k = 0; k < voxel[i][j].size(); k++)
			{
				//std::cout << i << "  " << j << " " << k << std::endl;
				voxel[i][j][k].Setposition(OpenMesh::Vec3f(bbox[1] + xoffset * i, bbox[3] + yoffset * j, bbox[5] + zoffset * k));
				voxel[i][j][k].Change_Scale(xoffset / 2, yoffset / 2, zoffset / 2);
			}
		}
	}



#pragma omp parallel  //num_threads(12)
	{

#pragma omp  for  collapse(3)  
		for (int i = 0; i < voxel.size(); i++)
		{
			for (int j = 0; j < voxel[i].size(); j++)
			{
				for (int k = 0; k < voxel[i][j].size(); k++)
				{
					if (voxel[i][j][k].isintersect == false)
					{
						for (int l = 0; l < fd.size(); l++)
						{
							//voxel[i][j][k].V_color = fc[l].Color; //fc[l]

							if (voxel[i][j][k].is_intersectwithface(fd[l])) // fd[l]
							{
								
								voxel[i][j][k].V_color =OpenMesh::Vec3f(1.0,0.0,0.0); //fc[l]				
								voxel[i][j][k].isintersect = true;
							}
				
						}
					}
				}
			}
		}
	}

	std::cout << voxel.size() << std::endl;

	for (int i = 0; i < voxel.size(); i++)
	{
		for (int j = 0; j < voxel[i].size(); j++)
		{
			for (int k = 0; k < voxel[i][j].size(); k++)
			{
				if (!voxel[i][j][k].isintersect)
				{
					voxel[i][j].erase(voxel[i][j].begin() + k);
					k = k - 1;
					if (voxel[i][j].size() == 1)
					{
						voxel[i][j].pop_back();
					}
				}
			}
		}
	}
	


	tend = std::clock();


	float  time;
	float min, sec;
	time = (tend - tstart) / CLOCKS_PER_SEC;
	min = time / 60;
	sec = (int)time % 60;
	if (min > 1)
	{
		std::cout << min << "  minutes " << sec << "  seconds " << std::endl;;
	}
	else
	{
		std::cout << sec << "  seconds " << std::endl;
	}
	//std::cout << "stop" << std::endl;


}

void Tri_Mesh::Output_voxelizemesh() 
{
	
	std::vector<OpenMesh::Vec3f> m_points;

	std::string s = "Output/";
	std::string fn = "voxelize_" + std::to_string(1) + ".obj";
	std::string path = s + fn;
	std::ofstream fout(path);
	for(int i=0; i< voxel.size();i++)
	{
		for(int j =0 ; j<voxel[i].size(); j++)
		{
			for(int k =0 ; k< voxel[i][j].size();k++)
			{
				for(int l = 0;l < voxel[i][j][k].Vertex.size(); l++)
				{
					std::vector<OpenMesh::Vec3f>::iterator it = std::find(m_points.begin(), m_points.end(), voxel[i][j][k].Vertex[l]);
					if (it == m_points.end())
					{
						m_points.push_back(voxel[i][j][k].Vertex[l]);
						fout << "v " << voxel[i][j][k].Vertex[l] << std::endl;
					}
				}
			}
		}	
	}
	fout << std::endl;


	for (int i = 0; i < voxel.size(); i++)
	{
		for (int j = 0; j < voxel[i].size(); j++)
		{
			for (int k = 0; k < voxel[i][j].size(); k++)
			{
				for (int l = 0; l < voxel[i][j][k].Face.size(); l++)
				{
					fout << "f ";
					for (int m = 0; m < 4; m++)
					{
						std::vector<OpenMesh::Vec3f>::iterator it = std::find(m_points.begin(), m_points.end(), voxel[i][j][k].Vertex[ voxel[i][j][k].Face[l][m] ]);
						if (it != m_points.end())
						{
							fout << " " << std::distance(m_points.begin(), it)+1;
						}
					}
					fout << std::endl;
				}
			}
		}
	}

	fout.close();

}


void Tri_Mesh::struct_cluster() 
{
	tstart = std::clock();
	calFaceData();
	std::vector<Cluster> normcacl;
	int index = 0;
	//�k�V�q���s
	//-----------------------------------------------------------
	int getindex = 0;
	for (int i = 0; i < fd.size(); i++) 
	{

		getindex = find_normal(normcacl, fd[i].getFaceNormal());
		if (getindex != -1)
		{
			normcacl[getindex].count++;
			normcacl[getindex].faceid.push_back(fd[i].getID());
		}
		else
		{
			Cluster tmp;
			tmp.norm = fd[i].getFaceNormal();
			tmp.faceid.push_back(fd[i].getID());
			tmp.count += 1;
			normcacl.push_back(tmp);
		}
	}
	qsort(&normcacl[0], normcacl.size(), sizeof(Cluster), pcompare);



	/*
	std::cout << std::endl << "CLuster size  "  <<normcacl.size() << std::endl;

	std::vector<OpenMesh::Vec3d> hull;
	//hull = Andrew_monotone_chain(normcacl[0], fd);
	for (int i = 0; i < normcacl.size(); i++)
	{
		if (OpenMesh::dot(fd[normcacl[i].faceid[0]].getFaceNormal(), OpenMesh::Vec3d(0, 1, 0)) >= 0.9)
		{
			hull = AlphaShapes(normcacl[i], fd); //4
		}
	}
	for (int i = 0; i < hull.size(); i++)
	{
		std::cout << "v " << hull[i] << std::endl;
	}

	*/

	std::vector<Cluster> cadarea;
	//���n���s
	//-------------------------------------------------------
	float maxarea ;
	int maxareaid;
	for (int i = 0; i < normcacl.size(); i++)
	{
		maxarea = -100;
		for (int j = 0; j < normcacl[i].faceid.size(); j++)
		{
			if (maxarea < fd[normcacl[i].faceid[j]].getArea())
			{
				maxarea = fd[normcacl[i].faceid[j]].getArea();
				maxareaid = normcacl[i].faceid[j];
			}
		}
		Cluster tmp;
		tmp.faceid.push_back(maxareaid);
		tmp.facearea = fd[maxareaid].getArea();
		tmp.norm = fd[maxareaid].getFaceNormal();
		cadarea.push_back(tmp);
		//cadarea[i].faceid.push_back(maxareaid);
		//cadarea[i].facearea = fd[maxareaid].getArea();
		//cadarea[i].norm = fd[maxareaid].getFaceNormal();
		//std::cout << "ID  " << maxareaid << "  Area " << maxarea << std::endl;
	}

	qsort(&cadarea[0], index, sizeof(Cluster), acompare);

	for (int i = 0; i < normcacl.size(); i++)
	{
		for (int j = 0; j < normcacl[i].faceid.size(); j++)
		{
			getindex = find_normal(cadarea, fd[normcacl[i].faceid[j]].getFaceNormal());
			if (getindex != -1)
			{
				cadarea[getindex].count++;
				cadarea[getindex].faceid.push_back(fd[normcacl[i].faceid[j]].getID());
			}
			else
			{
				Cluster tmp;
				tmp.norm = fd[i].getFaceNormal();
				tmp.faceid.push_back(fd[i].getID());
				tmp.count += 1;
				cadarea.push_back(tmp);
			}
		}
	}

	
	normcacl.clear();
	normcacl = cadarea;
	cadarea.clear();
	


	//���s��ı��
	//---------------------------------------------------------------------
	if (false)
	{
		for (int i = 0; i < fd.size(); i++)
		{
			std::vector <faceColor> color;
			fc[i].Color[0] = 1;
			fc[i].Color[1] = 1;
			fc[i].Color[2] = 1;
			fc[i].ColorID = 11111;
		}

		for (int i = 0; i < normcacl.size(); i++)
		{
			std::vector <faceColor> color;
			fc[normcacl[i].faceid[0]].Color[0] = colormap[i][0];
			fc[normcacl[i].faceid[0]].Color[1] = colormap[i][1];
			fc[normcacl[i].faceid[0]].Color[2] = colormap[i][2];
			fc[normcacl[i].faceid[0]].ColorID = normcacl[i].faceid[0];
			color.push_back(fc[normcacl[i].faceid[0]]);
		}
		for (int i = 0; i < normcacl.size(); i++)
		{
			for (int j = 0; j < normcacl[i].faceid.size(); j++)
			{
				fc[normcacl[i].faceid[j]].Color = fc[normcacl[i].faceid[0]].Color;
				fc[normcacl[i].faceid[j]].ColorID = normcacl[i].faceid[0];
			}
		}
	}
	//-----------------------------------------------------------------
	OpenMesh::Vec3d targetnoraml[8] = {	{-1,0,0},
																			{1,0,0},
																			{0,0,1},
																			{0,0,-1},
																			{1,0,1},
																			{1,0,-1},
																			{-1,0,-1},
																			{-1,0,1},
																		    };
	OpenMesh::Vec3d y_negative = { 0,-1,0 };
	OpenMesh::Vec3d y_positive = { 0,1,0 };
	std::vector<Cluster> facadecluster;
	std::vector<Cluster> roofcluster;
	std::vector<Cluster> groundcluster;
	int colorusage = 0;
	// �Ψ����s
	//-------------------------------------------------
	for (int k = 0; k < 8; k++)
	{
		for (int i = 0; i < normcacl.size(); i++)
		{

			int maxarea = -10000;
			int maxid = -1;
			for (int j = 0; j < normcacl[i].faceid.size(); j++)
			{
				if (maxarea < fd[normcacl[i].faceid[j]].getArea())
				{
					maxarea = fd[normcacl[i].faceid[j]].getArea();
					maxid = j;
				}				
			}
			std::swap(normcacl[i].faceid[maxid], normcacl[i].faceid[0]);
			/*
			if (dot(fd[normcacl[i].faceid[0]].getFaceNormal(), targetnoraml[k]) >= 0.9)
			{
				facadecluster.push_back(normcacl[i]);
				normcacl.erase(normcacl.begin() + i);
				i = 0;
				if (normcacl.size() == 1)
				{
					normcacl.pop_back();
				}
			}*/
			OpenMesh::Vec3d tmpnorm = fd[normcacl[i].faceid[0]].getFaceNormal();

			if (tmpnorm[0]>=-1 && tmpnorm[0]<=1  && tmpnorm[1]>=-0.1 && tmpnorm[1]<=0.1 && tmpnorm[2]>=-1 && tmpnorm[2]<=1)
			{
				facadecluster.push_back(normcacl[i]);
				normcacl.erase(normcacl.begin() + i);
				i = 0;
				/*
				if (normcacl.size() == 1)
				{
					normcacl.pop_back();
				}*/
			}

		}
	}
	//�γ����s
	//---------------------------------------------------
	for (int i = 0; i < normcacl.size(); i++)
	{

		int maxarea = -10000;
		int maxid = -1;
		for (int j = 0; j < normcacl[i].faceid.size(); j++)
		{
			if (maxarea < fd[normcacl[i].faceid[j]].getArea())
			{
				maxarea = fd[normcacl[i].faceid[j]].getArea();
				maxid = j;
			}
		}
		std::swap(normcacl[i].faceid[maxid], normcacl[i].faceid[0]);
		/*
		if (dot(fd[normcacl[i].faceid[0]].getFaceNormal(), y_negative) >= 0.7)
		{
			roofcluster.push_back(normcacl[i]);
			normcacl.erase(normcacl.begin() + i);
			i = 0;
			if (normcacl.size() == 1)
			{
				normcacl.pop_back();
			}
		}
		*/
		OpenMesh::Vec3d tmpnorm = fd[normcacl[i].faceid[0]].getFaceNormal();
		if (tmpnorm[0] > -1 && tmpnorm[0] < 1 && tmpnorm[1] >=-1 && tmpnorm[1]<-0.1 && tmpnorm[2]>-1 && tmpnorm[2] < 1)
		{
			roofcluster.push_back(normcacl[i]);
			normcacl.erase(normcacl.begin() + i);
			i = 0;
			/*
			if (normcacl.size() == 1)
			{
				normcacl.pop_back();
			}*/
		}
	}
	//�������s
	//--------------------------------------------------------------

	for (int i = 0; i < normcacl.size(); i++)
	{

		int maxarea = -10000;
		int maxid = -1;
		for (int j = 0; j < normcacl[i].faceid.size(); j++)
		{
			if (maxarea < fd[normcacl[i].faceid[j]].getArea())
			{
				maxarea = fd[normcacl[i].faceid[j]].getArea();
				maxid = j;
			}
		}
		std::swap(normcacl[i].faceid[maxid], normcacl[i].faceid[0]);
		/*
		if (dot(fd[normcacl[i].faceid[0]].getFaceNormal(),y_positive) >= 0.9)
		{
			groundcluster.push_back(normcacl[i]);
			normcacl.erase(normcacl.begin() + i);
			i = 0;
			if (normcacl.size() == 1)
			{
				normcacl.pop_back();
			}
		}*/
		
		OpenMesh::Vec3d tmpnorm = fd[normcacl[i].faceid[0]].getFaceNormal();
		if (tmpnorm[0] > -1 && tmpnorm[0] < 1 && tmpnorm[1] >0.1 && tmpnorm[1]<=1 && tmpnorm[2]>-1 && tmpnorm[2] < 1)
		{
			groundcluster.push_back(normcacl[i]);
			normcacl.erase(normcacl.begin() + i);
			i = 0;
			if (normcacl.size() == 1)
			{
				normcacl.pop_back();
			}
		}
	}
	//���s��ı��
	//----------------------------------
	//�Ψ�
	if (false)
	{
		
		for (int i = 0; i < fd.size(); i++)
		{
			std::vector <faceColor> color;
			fc[i].Color[0] = 1;
			fc[i].Color[1] = 1;
			fc[i].Color[2] = 1;
			fc[i].ColorID = 11111;
		}

		for (int i = 0; i < facadecluster.size(); i++)
		{
			std::vector <faceColor> color;
			fc[facadecluster[i].faceid[0]].Color[0] = colormap[colorusage++][0];
			fc[facadecluster[i].faceid[0]].Color[1] = colormap[colorusage++][1];
			fc[facadecluster[i].faceid[0]].Color[2] = colormap[colorusage++][2];
			fc[facadecluster[i].faceid[0]].ColorID = facadecluster[i].faceid[0];
			color.push_back(fc[facadecluster[i].faceid[0]]);
		}

		for (int i = 0; i < facadecluster.size(); i++)
		{
			for (int j = 0; j < facadecluster[i].faceid.size(); j++)
			{
				fc[facadecluster[i].faceid[j]].Color = fc[facadecluster[i].faceid[0]].Color;
				fc[facadecluster[i].faceid[j]].ColorID = facadecluster[i].faceid[0];
			}
		}
	}		
	//�γ� 
	if (false)
	{
		
		for (int i = 0; i < fd.size(); i++)
		{
			std::vector <faceColor> color;
			fc[i].Color[0] = 1;
			fc[i].Color[1] = 1;
			fc[i].Color[2] = 1;
			fc[i].ColorID = 11111;
		}
		
		for (int i = 0; i <roofcluster.size(); i++)
		{
			std::vector <faceColor> color;
			fc[roofcluster[i].faceid[0]].Color[0] = colormap[colorusage++][0];
			fc[roofcluster[i].faceid[0]].Color[1] = colormap[colorusage++][1];
			fc[roofcluster[i].faceid[0]].Color[2] = colormap[colorusage++][2];
			fc[roofcluster[i].faceid[0]].ColorID =roofcluster[i].faceid[0];
			color.push_back(fc[roofcluster[i].faceid[0]]);
		}

		for (int i = 0; i <roofcluster.size(); i++)
		{
			for (int j = 0; j <roofcluster[i].faceid.size(); j++)
			{
				fc[roofcluster[i].faceid[j]].Color = fc[roofcluster[i].faceid[0]].Color;
				fc[roofcluster[i].faceid[j]].ColorID =roofcluster[i].faceid[0];
			}
		}
	}
	//����
	if (false)
	{
		
		for (int i = 0; i < fd.size(); i++)
		{
			std::vector <faceColor> color;
			fc[i].Color[0] = 1;
			fc[i].Color[1] = 1;
			fc[i].Color[2] = 1;
			fc[i].ColorID = 11111;
		}
		
		for (int i = 0; i <groundcluster.size(); i++)
		{
			std::vector <faceColor> color;
			fc[groundcluster[i].faceid[0]].Color[0] = colormap[colorusage++][0];
			fc[groundcluster[i].faceid[0]].Color[1] = colormap[colorusage++][1];
			fc[groundcluster[i].faceid[0]].Color[2] = colormap[colorusage++][2];
			fc[groundcluster[i].faceid[0]].ColorID =groundcluster[i].faceid[0];
			color.push_back(fc[groundcluster[i].faceid[0]]);
		}

		for (int i = 0; i <groundcluster.size(); i++)
		{
			for (int j = 0; j <groundcluster[i].faceid.size(); j++)
			{
				fc[groundcluster[i].faceid[j]].Color = fc[groundcluster[i].faceid[0]].Color;
				fc[groundcluster[i].faceid[j]].ColorID =groundcluster[i].faceid[0];
			}
		}
	}
	//-----------------------------------
	if (true)
	{
		
		std::vector<OpenMesh::Vec3f> v;
		
		for(int i =0 ; i < objVertices.size() ; i++)
		{
			v.push_back(OpenMesh::Vec3f(objVertices[i][0] , objVertices[i][1] , objVertices[i][2]));
		}

		bbox =computebbox(v);
		
		float xlength, ylength, zlength;
		
		xlength = abs(bbox[0] - bbox[1]);
		ylength = abs(bbox[2] - bbox[3]);
		zlength = abs(bbox[4] - bbox[5]);

		float xoffset, yoffset, zoffset;
		
		 xcellsize = xlength / (xlength + ylength + zlength)*100+1;
		ycellsize = ylength / (xlength + ylength + zlength)*100+1;
		zcellsize = zlength / (xlength + ylength + zlength)*100+1;
	/*
		xcellsize = 10;
		ycellsize = 10;
		zcellsize =10;
		*/

		/*
		xoffset = xlength / ((xcellsize%2==0? xcellsize: xcellsize+1));
		yoffset = ylength / ((ycellsize % 2 == 0 ? ycellsize : ycellsize + 1));
		zoffset = zlength / ((zcellsize % 2 == 0 ? zcellsize : zcellsize + 1));
		*/
		xoffset = xlength / ( xcellsize -1);
		yoffset = ylength / (ycellsize-1 );
		zoffset = zlength /( zcellsize-1 );
		
		voxel.resize(xcellsize);
		for (int i = 0; i < voxel.size(); i++) 
		{
			voxel[i].resize(ycellsize);
			for (int j = 0; j < voxel[i].size(); j++)
			{
				voxel[i][j].resize(zcellsize);
			}
		}
	


		
		std::cout << std::endl<< "x cell size =  " << xcellsize << std::endl;
		std::cout << "y cell size =  " << ycellsize << std::endl;
		std::cout << "z cell size =  " << zcellsize << std::endl;

		for (int i = 0; i < voxel.size(); i++)
		{
			for (int j = 0; j < voxel[i].size(); j++)
			{
				for (int k = 0; k < voxel[i][j].size(); k++) 
				{
					//std::cout << i << "  " << j << " " << k << std::endl;
					voxel[i][j][k].translate(OpenMesh::Vec3d(bbox[1] + xoffset * i, bbox[3] + yoffset  * j, bbox[5] + zoffset  * k));
					voxel[i][j][k].Change_Scale(xoffset/2, yoffset/2, zoffset/2);
				
				}
			}
		}

		bool ck = false;
		voxel[0][0].erase(voxel[0][0].begin() + 1);
		/*
		for (int i = 0; i < voxel.size(); i++)
		{
			for (int j = 0; j < voxel[i].size(); j++)
			{
				for (int k = 0; k < voxel[i][j].size(); k++)
				{
					if (k > 0)
					{
						voxel[i][j].erase(voxel[i][j].begin() + k);
						k = 0;
					}
				}
			}
		}
		*/

		//voxel.translate();
		
		/*
		ada_planedistcluster(facadecluster, facade_output, 0.01);
		ada_planedistcluster(roofcluster, roof_output , 0.01);
		ada_planedistcluster(groundcluster, ground_output,0.05);
		
		filtmincluster(fd, facade_output);
		filtmincluster(fd, roof_output);
		filtmincluster(fd, ground_output);
		
		planedistcluster(facadecluster, facade_output);
		planedistcluster(roofcluster, roof_output);
		planedistcluster(groundcluster, ground_output);
		*/

		for (int i = 0; i < facade_output.size(); i++)
		{
			if (facade_output[i].faceid.size() != 0)
			{
				result.push_back(facade_output[i]);
			}
		}
		for (int i = 0; i < roof_output.size(); i++)
		{
			if (roof_output[i].faceid.size() != 0)
			{
				result.push_back(roof_output[i]);
			}
		}
		for (int i = 0; i < ground_output.size(); i++)
		{
			if (ground_output[i].faceid.size() != 0)
			{
				result.push_back(ground_output[i]);
			}
		}

		for (int i = 0; i <result.size(); i++) 
		{
			float area = 0;
			
			for (int j = 0; j < result[i].faceid.size(); j++)
			{
				area += fd[result[i].faceid[j]].getArea();
			}
			result[i].facearea = area;
		}
	
		/*
	hull = Andrew_monotone_chain(normcacl[0], fd);
	hull = AlphaShapes(ground_output[4], fd); //4

		for (int i = 0; i < hull.size(); i++)
		{
			std::cout << "v " << hull[i] << std::endl;
		}
		*/
	
		

		//InitialAdjGraph();
	//	InitailWeight(result);

		std::cout << "FInal Cluster size " << result.size() << std::endl;
		//���s��ı��
		//--------------------------
		if (true)
		{

			for (int i = 0; i < fd.size(); i++)
			{
				std::vector <faceColor> color;
				fc[i].Color[0] = 1;
				fc[i].Color[1] = 1;
				fc[i].Color[2] = 1;
				fc[i].ColorID = 11111;
			}

			std::cout << "fc " << fc.size() << std::endl;

			for (int i = 0; i < result.size(); i++)
			{
				std::vector <faceColor> color;
				//std::cout << "index i " << i<< std::endl;
			//	std::cout <<"index "<< result[i].faceid[0] << std::endl;
				fc[result[i].faceid[0]].Color[0] = colormap[colorusage][0];
				fc[result[i].faceid[0]].Color[1] = colormap[colorusage][1];
				fc[result[i].faceid[0]].Color[2] = colormap[colorusage++][2];
				fc[result[i].faceid[0]].ColorID = result[i].faceid[0];
				color.push_back(fc[result[i].faceid[0]]);
			}

			for (int i = 0; i < result.size(); i++)
			{
				for (int j = 0; j < result[i].faceid.size(); j++)
				{
					fc[result[i].faceid[j]].Color = fc[result[i].faceid[0]].Color;
					fc[result[i].faceid[j]].ColorID = result[i].faceid[0];
				}
			}
		}
		//---------------------------------------------------�{���p�ɵ���
		tend = std::clock();
		float  time;
		float min, sec;
		time = (tend - tstart) / CLOCKS_PER_SEC;
		min = time / 60;
		sec = (int)time % 60;
		if (min > 1)
		{
			std::cout << min << "  minutes " << sec << "  seconds " << std::endl;;
		}
		else
		{
			std::cout << sec << "  seconds " << std::endl;
		}
	}
}


void Tri_Mesh::MergeCluster() 
{
	float minw=10000;
	int index_i=-1 , index_j = -1;
	
	for (int i = 0; i < result.size(); i++) 
	{
		for (int j = i; j < result.size(); j++)
		{
			if (i != j)
			{
				if (minw > adjmatrix[i][j])
				{
					minw = adjmatrix[i][j];
					index_i = i;
					index_j = j;
				}
			}
		}
	}
	UpdaeWeight(index_i , index_j);
	std::cout << "min w " << minw<< std::endl;

		for (int i = 0; i < result[index_j].faceid.size(); i++)
		{
			result[index_i].faceid.push_back(result[index_j].faceid[i]);
		}
		result.erase(result.begin() + index_j);
	

	std::cout << "FInal Cluster size " << result.size() << std::endl;
	//���s��ı��
	//--------------------------


		for (int i = 0; i < result.size(); i++)
		{
			for (int j = 0; j < result[i].faceid.size(); j++)
			{
				fc[result[i].faceid[j]].Color = fc[result[i].faceid[0]].Color;
				fc[result[i].faceid[j]].ColorID = result[i].faceid[0];
			}
		}
	
}


void Tri_Mesh::facepopCluster()
{
	tstart = std::clock();
	calFaceData();

	std::vector<Cluster> normcacl;
	int index = 0;

	int getindex = 0;
	for (int i = 0; i < fd.size(); i++) {

		getindex = find_normal(normcacl, fd[i].getFaceNormal());
		if (getindex != -1)
		{
			normcacl[getindex].count++;
			normcacl[getindex].faceid.push_back(fd[i].getID());
		}
		else
		{
			Cluster tmp;
			tmp.norm = fd[i].getFaceNormal();
			tmp.faceid.push_back(fd[i].getID());
			tmp.count += 1;
			normcacl.push_back(tmp);
			//normcacl[index].norm = fd[i].getFaceNormal();
		    //	normcacl[index].faceid.push_back(fd[i].getID());
			//normcacl[index].count++;
			//index++;
		}
	}
	qsort(&normcacl[0], normcacl.size(), sizeof(Cluster), pcompare);
	std::cout << std::endl << "pop count" << "Face : " << fd.size() << std::endl;
	/*

	for (int i = 0; i < index; i++) {
		std::cout << "index" << i << " : " << normcacl[i].norm << "  times :  " << normcacl[i].count;
		std::cout << "  Face ID : ";
		for (int j = 0; j < normcacl[i].faceid.size(); j++)
		{
			//std::cout << normcacl[i].faceid[j] << "    " ;
		}
		std::cout << std::endl;

	}*/
	int sum = 0;
	for (int i = 0; i <normcacl.size(); i++)
	{
		sum += normcacl[i].count;
	}



	std::cout << " count face " << sum << std::endl;

	state = ColorState::NFACECLUSTER;
	std::vector <int> faceCheck(fd.size(), 0);
	std::vector <std::map <int, bool>> clusterVec;
	std::vector <int> represent;
	usedColor.clear();
	bool allCheck = true;
	int count = 0;
	float maxdot = -100;
	int maxdotid = 0;
	int maxcadid=0;
	double maxarea = -100;
	int maxareaid = 0;
	int outputidx = 0;
	std::vector<Cluster> cadarea;
	std::vector<Cluster> output ;
	// ���u�Hnormal�����s -> represent
	std::map <int, bool> clusterMap;
	
	for (int i = 0; i <normcacl.size(); i++)
	{
		maxarea = -100;
		for (int j = 0; j < normcacl[i].faceid.size(); j++)
		{
			if (maxarea < fd[normcacl[i].faceid[j]].getArea())
			{
				maxarea = fd[normcacl[i].faceid[j]].getArea();
				maxareaid = normcacl[i].faceid[j];
			}
		}
		Cluster tmp;
		tmp.faceid.push_back(maxareaid);
		tmp.facearea = fd[maxareaid].getArea();
		tmp.norm = fd[maxareaid].getFaceNormal();
		cadarea.push_back(tmp);
		//cadarea[i].faceid.push_back(maxareaid);
		//cadarea[i].facearea = fd[maxareaid].getArea();
		//cadarea[i].norm = fd[maxareaid].getFaceNormal();
		//std::cout << "ID  " << maxareaid << "  Area " << maxarea << std::endl;
	}

	qsort(&cadarea[0], index, sizeof(Cluster), acompare);



	for (int i = 0; i < cadarea.size(); i++)
	{
		std::cout << "face ID :  " << cadarea[i].faceid[0] << " Normal  " << cadarea[i].norm << " facearea " << cadarea[i].facearea << std::endl;
	}
	std::cout << std::endl;


	
	for (int i = 0; i < fd.size(); i++)
	{
		std::vector <faceColor> color;
		fc[i].Color[0] = 1;
		fc[i].Color[1] = 1;
		fc[i].Color[2] = 1;
		fc[i].ColorID = 11111;
	}
	/*
	count = 0;
	
	for (int i = 0; i < index; i++)
	{

		clusterMap[normcacl[i].faceid[0]] = false;
		represent.push_back(normcacl[i].faceid[0]);
		//faceCheck[normcacl[i].faceid[0]] = count++;
		std::vector <faceColor> color;
		fc[normcacl[i].faceid[0]].Color[0] = colormap[i][0];
		fc[normcacl[i].faceid[0]].Color[1] = colormap[i][1];
		fc[normcacl[i].faceid[0]].Color[2] = colormap[i][2];
		fc[normcacl[i].faceid[0]].ColorID = normcacl[i].faceid[0];
		color.push_back(fc[normcacl[i].faceid[0]]);
	}
	*/
	

	

	do {
		//count++;
		allCheck = true;

		for (int i = 0; i < fd.size(); i++) {
			if (fd[i].getArea() == 0 || isnan(fd[i].getArea())) faceCheck[i] = -1;
			if (!fd[i].isAllow())  // �L�k�B�z����
			{
				faceCheck[i] = -1;
				fc[i].Color[0] = 0;
				fc[i].Color[1] = 0;
				fc[i].Color[2] = 0;
			}
			if (faceCheck[i] != 0) continue;
			// Normal �۪񪺤��s  �]���P�C��
			maxdot = -100;
			for (int j = 0; j < cadarea.size(); j++)
			{
				if (maxdot < OpenMesh::dot(cadarea[j].norm, fd[i].getFaceNormal())) {

					maxdot = OpenMesh::dot(cadarea[j].norm, fd[i].getFaceNormal());
					maxdotid = cadarea[j].faceid[0];
					maxcadid = j;
					//	std::wcout <<"ID  "<< maxdotid << std::endl;
				}
			}
			if (maxdotid != i)
			{
				cadarea[maxcadid].faceid.push_back(i);
			}

				//fc[i].Color = fc[maxdotid].Color;
				//fc[i].ColorID = maxdotid;
				faceCheck[i] = count;
				clusterMap[i] = false;
			
		}
		clusterVec.push_back(clusterMap);

	} while (!allCheck);
	
	/*
	for (int i = 0; i < cadarea.size(); i++)
	{

		clusterMap[cadarea[i].faceid[0]] = false;
		represent.push_back(cadarea[i].faceid[0]);
		//faceCheck[cadarea[i].faceid[0]] = count++;
		std::vector <faceColor> color;
		fc[cadarea[i].faceid[0]].Color[0] = colormap[i][0];
		fc[cadarea[i].faceid[0]].Color[1] = colormap[i][1];
		fc[cadarea[i].faceid[0]].Color[2] = colormap[i][2];
		fc[cadarea[i].faceid[0]].ColorID = cadarea[i].faceid[0];
		color.push_back(fc[cadarea[i].faceid[0]]);
	}


	for (int i = 0; i < cadarea.size(); i++)
	{
		for (int j = 0; j < cadarea[i].faceid.size(); j++)
		{
			fc[cadarea[i].faceid[j]].Color = fc[cadarea[i].faceid[0]].Color;
			fc[cadarea[i].faceid[j]].ColorID = cadarea[i].faceid[0];
			//std::cout << "ID   " << fc[output[i].faceid[j]].ColorID << "   color   " << fc[output[i].faceid[j]].Color << std::endl;
		}
	}*/

	std::cout << "cluster: " <<cadarea.size()<<std::endl;
	bool sw =true;
	if (sw)
	{
		//ada_planedistcluster(cadarea, output);
		planedistcluster(cadarea ,output);
		std::cout << std::endl << "Clusters " << output.size() << std::endl;

		//filtmincluster(fd, output);
		/*
		for (int i = 0; i < output.size(); i++) 
		{
			std::vector<float> tmp ;
			tmp.resize (output.size());
			adjmatrix.push_back(tmp);
		}
		for (int i = 0; i < output.size(); i++) 
		{
			for (int j = 0; j < output.size(); j++) 
			{
				adjmatrix[i][j] = -1;
			}
		}

		
		std::vector<clustergraph>clustergraph_arr;
		clustergraph_arr.resize(output.size());
		for(int i = 0 ; i< clustergraph_arr.size() ; i++)
		{
			clustergraph_arr[i].normal = fd[output[i].faceid[0]].getFaceNormal();
			clustergraph_arr[i].center = getclustercenter(output[i].faceid);
			clustergraph_arr[i].faceid = output[i].faceid;
			contructbbox(&clustergraph_arr[i]);
		}
		for (int i = 0; i < output.size(); i++) 
		{
			for (int j = 0; j < output.size(); j++) 
			{
				if (intersect(clustergraph_arr[i].m_bbox, clustergraph_arr[j].m_bbox)) 
				{
					adjmatrix[i][j] = differential(clustergraph_arr[i], clustergraph_arr[j]);
				}
			}
		}
	
		for (int i = 0; i < output.size(); i++)
		{
			std::cout << " " << i;
		}
		std::cout << std::endl;
		for (int i = 0; i < output.size(); i++)
		{
			std::cout << i;
			for (int j = 0; j < output.size(); j++)
			{
				std::cout << " " << adjmatrix[i][j];
			}
			std::cout << std::endl;
		}
		*/
		/*
		std::cout << std::endl << "Refine " << output.size() << std::endl;

		for(int i =0 ; i < fc.size(); i++)
		{
			fc[i].Color[0] = 1;
			fc[i].Color[1] = 1;
			fc[i].Color[2] = 1;
			fc[i].ColorID = -1;
		}

	std::cout << std::endl << "Refine " << output.size() << std::endl;
	*/
		for (int i = 0; i < output.size(); i++)
		{
			//std::cout << "index   :   " << i << std::endl;

				std::vector <faceColor> color;
				fc[output[i].faceid[0]].Color[0] = colormap[i][0];
				fc[output[i].faceid[0]].Color[1] = colormap[i][1];
				fc[output[i].faceid[0]].Color[2] = colormap[i][2];
				fc[output[i].faceid[0]].ColorID = output[i].faceid[0];
				color.push_back(fc[output[i].faceid[0]]);
			//std::cout << fd[output[i].faceid[0]].getArea() << std::endl;
		}
		
		for (int i = 0; i <output.size() ; i++)
		{
			for (int j = 0; j < output[i].faceid.size(); j++)
			{
			fc[output[i].faceid[j]].Color = fc[output[i].faceid[0]].Color;
			fc[output[i].faceid[j]].ColorID = output[i].faceid[0];
				//std::cout << "ID   " << fc[output[i].faceid[j]].ColorID << "   color   " << fc[output[i].faceid[j]].Color << std::endl;
			}
		}
		//std::cout <<std::endl<< "Clusters " << output.size() << std::endl;

	}
	tend = std::clock();
	/*
	for (int i = 0; i < output[0].faceid.size(); i++) 
	{
		for (int j = 0; j < 3; j++)
		{
			std::cout <<"v "<< fd[output[0].faceid[i]].getVertices()[j] << std::endl;
		}
	}*/
	
	//contructbbox();
    float  time   ;
	float min, sec;
	time = (tend - tstart)/CLOCKS_PER_SEC ;
	min = time / 60;
	sec =(int) time % 60;
	if (min >1)
	{
		std::cout << min << "  minutes " << sec << "  seconds " << std::endl;;
	}
	else 
	{
		std::cout <<  sec << "  seconds " <<std::endl;
	}
}


void Tri_Mesh::hypothesismesh()
{
	//bboxmesh[0].calFaceData();
	bboxmesh[0].facefaceCount();
	bboxmesh[0].facepopCluster();
}

void Tri_Mesh::findBoundary()
{
	boundaryID.clear();
	clusterID.clear();
	fn.clear();
	std::map <int, bool>::iterator iter;
	for (iter = usedColor.begin(); iter != usedColor.end(); iter++)
	{
		findBoundary(iter->first);
	}
}

void Tri_Mesh::CheckFaceAllow()
{
	for (int i = 0; i < fd.size(); i++) {
		if (fd[i].getArea() < faceIgnore)
			fd[i].setAllowFace(false);
		else fd[i].setAllowFace(true);
	}
}


bool Tri_Mesh::writeBoundaryFile(std::string fileName)
{
	std::fstream file, obj;
	file.open(fileName + ".txt", std::ios::out);
	if (!file)
	{
		std::cout << "Can't open " << fileName << std::endl;
		return false;
	}
	std::map <int, std::map <int, bool>>::iterator iiter;
	std::map <int, bool>::iterator iter;
	std::map <int, Point>::iterator pniter;
	for (iiter = boundaryID.begin(); iiter != boundaryID.end(); iiter++)
	{
		file << "v " << iiter->first << std::endl;
		file << "n " << fn[iiter->first] << std::endl;
		file << "b";
		for (iter = boundaryID[iiter->first].begin(); iter != boundaryID[iiter->first].end(); iter++)
		{
			file << " " << iter->first;
		}
		file << "\nf";
		for (iter = clusterID[iiter->first].begin(); iter != clusterID[iiter->first].end(); iter++)
		{
			file << " " << iter->first;
		}
		file << std::endl;
	}
	/*for (int i = 0; i < faceVec.size(); i++) {
		file << "ff";
		for (int j = 0; j < faceVec[i].size(); j++) {
			file << " " << faceVec[i][j];
		}
		file << std::endl;
	}*/
	file.close();
	obj.open(fileName + ".obj", std::ios::out);
	if (!obj)
	{
		std::cout << "!Can't open " << fileName << std::endl;
		return false;
	}
	for (VIter viter = vertices_begin(); viter != vertices_end(); viter++) {
		double* p = new double[3];
		p = point(viter.handle()).data();
		std::cout << "Index: " << viter.handle().idx() << " " << p[0] << " " << p[1] << " " << p[2] << std::endl;
		obj << "v " << p[0] << " " << p[1] << " " << p[2] << std::endl;
	}
	for (FIter fiter = faces_begin(); fiter != faces_end(); fiter++)
	{
		std::cout << "Index: " << fiter.handle().idx();
		obj << "f";
		for (FVIter fviter = fv_iter(fiter); fviter; fviter++) {
			std::cout << " " << fviter.handle().idx() + 1;
			obj << " " << fviter.handle().idx() + 1;
		}
		std::cout << std::endl;
		obj << std::endl;
	}
	obj.close();
	return true;
}

bool Tri_Mesh::saveNewModel()
{
	std::fstream obj;
	obj.open("newModel.obj", std::ios::out);
	if (!obj)
	{
		std::cout << "!Can't open " << "newModel" << std::endl;
		return false;
	}

	/*for (int i = 0; i < Allmeshs.size(); i++)
	{
		Tri_Mesh temp = Allmeshs[i];
		obj << "# face " << i << std::endl;
		for (VIter viter = temp.vertices_begin(); viter != temp.vertices_end(); viter++) {
			double* p = new double[3];
			p = temp.point(viter.handle()).data();
			std::cout << "Index: " << viter.handle().idx() << " " << p[0] << " " << p[1] << " " << p[2] << std::endl;
			obj << "v " << p[0] << " " << p[1] << " " << p[2] << std::endl;
		}
		for (FIter fiter = temp.faces_begin(); fiter != temp.faces_end(); fiter++)
		{
			std::cout << "Index: " << fiter.handle().idx();
			obj << "f";
			for (FVIter fviter = temp.fv_iter(fiter); fviter; fviter++) {
				std::cout << " " << fviter.handle().idx() + 1;
				obj << " " << fviter.handle().idx() + 1;
			}
			std::cout << std::endl;
			obj << std::endl;
		}
	}*/

	std::stringstream ss;
	int Vcount = 0;
	std::vector <int> offsets;
	for (int i = 0; i < Allmeshs.size(); i++)
	{
		offsets.push_back(Vcount);
		Tri_Mesh temp = Allmeshs[i];
		obj << "# face " << i << std::endl;
		ss << "# face ";
		ss << i;
		ss << std::endl;
		for (VIter viter = temp.vertices_begin(); viter != temp.vertices_end(); viter++) {
			double* p = new double[3];
			Vcount++;
			p = temp.point(viter.handle()).data();
			//std::cout << "Index: " << viter.handle().idx() << " " << p[0] << " " << p[1] << " " << p[2] << std::endl;
			obj << "v " << p[0] << " " << p[1] << " " << p[2] << std::endl;
			ss << "v " << p[0] << " " << p[1] << " " << p[2] << std::endl;
		}
	}
	for (int i = 0; i < Allmeshs.size(); i++)
	{
		Tri_Mesh temp = Allmeshs[i];
		obj << "# face " << i << std::endl;
		ss << "# face " << i << std::endl;
		for (FIter fiter = temp.faces_begin(); fiter != temp.faces_end(); fiter++)
		{
			//std::cout << "Index: " << fiter.handle().idx();
			obj << "f";
			ss << "f";
			for (FVIter fviter = temp.fv_iter(fiter); fviter; fviter++) {
				//std::cout << " " << fviter.handle().idx() + 1 + offsets[i];
				obj << " " << fviter.handle().idx() + 1 + offsets[i];
				ss << " " << fviter.handle().idx() + 1 + offsets[i];
			}
			//std::cout << std::endl;
			obj << std::endl;
			ss << std::endl;
		}
	}
	obj.close();
	objFile += ss.str();
	std::cout << objFile;
	return true;
}

bool Tri_Mesh::saveNewModelMtl(char* path, char* fileName)
{
	std::string objpath(path);
	std::string mtlpath(path);
	std::string name(fileName);
	objpath += (name + ".obj");
	mtlpath += (name + ".mtl");
	std::cout << objpath << std::endl;

	std::cout << mtlpath << std::endl;
	std::fstream obj;

	obj.open(objpath, std::ios::out);
	if (!obj)
	{
		std::cout << "!Can't open " << "newModel" << std::endl;
		return false;
	}

	std::stringstream ss;
	int Vcount = 0;
	std::vector <int> offsets;
	obj << "mtllib newModelMtl.mtl" << std::endl;
	for (int i = 0; i < Allmeshs.size(); i++)
	{
		obj << "g default" << std::endl;
		offsets.push_back(Vcount);
		OMT::Model temp = Allmeshs[i];
		obj << "# face " << i << std::endl;
		ss << "# face ";
		ss << i;
		ss << std::endl;
		for (VIter viter = temp.vertices_begin(); viter != temp.vertices_end(); viter++) {
			double* p = new double[3];
			Vcount++;
			p = temp.point(viter.handle()).data();
			//std::cout << "Index: " << viter.handle().idx() << " " << p[0] << " " << p[1] << " " << p[2] << std::endl;
			obj << "v " << p[0] << " " << p[1] << " " << p[2] << std::endl;
			ss << "v " << p[0] << " " << p[1] << " " << p[2] << std::endl;
		}

		obj << "# face " << i << std::endl;
		ss << "# face " << i << std::endl;
		obj << "s off" << std::endl;
		obj << "g default" << i << std::endl;
		obj << "usemtl default" << 0 << std::endl;
		for (FIter fiter = temp.faces_begin(); fiter != temp.faces_end(); fiter++)
		{
			//std::cout << "Index: " << fiter.handle().idx();
			obj << "f";
			ss << "f";
			for (FVIter fviter = temp.fv_iter(fiter); fviter; fviter++) {
				//std::cout << " " << fviter.handle().idx() + 1 + offsets[i];
				obj << " " << fviter.handle().idx() + 1 + offsets[i];
				ss << " " << fviter.handle().idx() + 1 + offsets[i];
			}
			//std::cout << std::endl;
			obj << std::endl;
			ss << std::endl;
		}
	}

	obj.close();
	objFile += ss.str();
	//std::cout << objFile;

	std::fstream mtl;
	mtl.open(mtlpath, std::ios::out);
	if (!mtl)
	{
		std::cout << "!Can't open " << "newMaterial" << std::endl;
		return false;
	}

	mtl << "newmtl default" << 0 << std::endl;
	mtl << "Kd 0.000 0.000 0.000" << std::endl;
	mtl << "Ka 0.000 0.000 0.000" << std::endl;
	mtl << "Tf 1.000 1.000 1.000" << std::endl;
	mtl << "Ni 1.00" << std::endl;
	mtl << "Ks 1.000 1.000 1.000" << std::endl;

	mtl.close();
	return true;
}

bool Tri_Mesh::saveConnectFace(char* path, char* fileName)
{
	std::string filePath(path);
	std::string name(fileName);
	filePath += (name + ".txt");

	std::fstream connectFile;
	connectFile.open(filePath, std::ios::out);
	if (!connectFile)
	{
		std::cout << "!Can't open " << "connectFile" << std::endl;
		return false;
	}

	std::vector <std::vector<int>> connectface;
	for (int i = 0; i < Allmeshs.size(); i++)
		connectface.push_back(std::vector<int>());

	for (int i = 0; i < Allmeshs.size(); i++)
	{
		for (int j = i + 1; j < Allmeshs.size(); j++)
		{
			std::map <int, int>::iterator iter;
			for (iter = containID[i].begin(); iter != containID[i].end(); ++iter)
			{
				if (containID[j].find(iter->second) != containID[j].end())
				{
					connectface[i].push_back(j);
					connectface[j].push_back(i);
					break;
				}
			}
		}
	}

	for (int i = 0; i < connectface.size(); i++)
	{
		connectFile << i;
		for (int j = 0; j < connectface[i].size(); j++)
		{
			connectFile << " " << connectface[i][j];
		}
		connectFile << std::endl;
	}

	connectFile.close();
	return true;
}

void Tri_Mesh::passObjtoUnity(char* str, int len)
{
	str = new char[len + 1];
	strcpy(str, objFile.c_str());
}

void Tri_Mesh::saperateFace()
{
	bool recalculateFace = true;
	Allmeshs.clear();
	containID.clear();
	std::vector <VHandle> vhandles;
	std::vector <FHandle> fhandles;
	for (VIter v = vertices_begin(); v != vertices_end(); ++v)
		vhandles.push_back(v.handle());
	for (FIter f = faces_begin(); f != faces_end(); ++f)
		fhandles.push_back(f.handle());
	//std::cout << "------------face_saperate------------\n";
	for (int i = 0; i < faceVec.size(); i++)
	{
		std::map <int, int> eachContain;

		// each Big Face
		Tri_Mesh newMesh;
		std::vector <VHandle> newVH;
		std::map <int, int> checkUse;
		//std::cout << "------face " << i << ":------\n";
		for (int j = 0; j < faceVec[i].size(); j++)
		{
			FaceHandle temp = fhandles[faceVec[i][j]];
			std::vector <VHandle> newFH;
			for (FVIter fv = fv_iter(temp); fv; ++fv)
			{
				eachContain[fv.handle().idx()] = fv.handle().idx();

				Point p = point(fv.handle());
				//std::cout << "vertex " << i << ":   " << p << "\n";
				if (checkUse.find(fv.handle().idx()) == checkUse.end())
				{
					VertexHandle tempv = newMesh.add_vertex(Point(p));
					newVH.push_back(tempv);
					newFH.push_back(newVH[newVH.size() - 1]);
					checkUse[fv.handle().idx()] = newVH.size() - 1;
				}
				else {
					newFH.push_back(newVH[checkUse[fv.handle().idx()]]);
				}
			}
			containID.push_back(eachContain);

			newMesh.add_face(newFH);
		}

		if (recalculateFace)
		{
			int count = 0;
			newMesh.centerPoint = Point(0, 0, 0);
			std::map <int, bool> useCenter;
			for (FIter f = newMesh.faces_begin(); f != newMesh.faces_end(); ++f)
			{
				int bcount = 0;
				Point temp = Point(0, 0, 0);
				for (FVIter fv = newMesh.fv_begin(f); fv; ++fv)
				{
					if (newMesh.is_boundary(fv))
						bcount++;
					else temp = newMesh.point(fv);
				}

				if (bcount == 2)
				{
					newMesh.centerPoint += temp;
					count++;
				}

			}
			newMesh.centerPoint /= count;


			// new Mesh first processing
			// addMesh: new Mesh first processing
			Tri_Mesh addMesh;
			newVH.clear();
			checkUse.clear();
			newVH.push_back(addMesh.add_vertex(Point(newMesh.centerPoint)));

			for (FIter f = newMesh.faces_begin(); f != newMesh.faces_end(); ++f)
			{
				int Boundcount = 0;

				std::vector <VHandle> newFH;

				for (FVIter fv = newMesh.fv_iter(f); fv; ++fv)
				{
					if (newMesh.is_boundary(fv))
						Boundcount++;
				}
				if (Boundcount >= 2) {
					for (FVIter fv = newMesh.fv_iter(f); fv; ++fv)
					{
						if (newMesh.is_boundary(fv)) {
							if (checkUse.find(fv.handle().idx()) == checkUse.end()) {
								newVH.push_back(addMesh.add_vertex(newMesh.point(fv.handle())));
								checkUse[fv.handle().idx()] = newVH.size() - 1;
								newFH.push_back(newVH[newVH.size() - 1]);
							}
							else newFH.push_back(newVH[checkUse[fv.handle().idx()]]);
						}
						else newFH.push_back(newVH[0]);
					}
					FHandle check = addMesh.add_face(newFH);
					if (check.idx() == -1)
					{
						std::vector <VHandle> reFH;
						for (FVIter fv = newMesh.fv_iter(f); fv; ++fv)
						{
							if (newMesh.is_boundary(fv))
							{
								newVH.push_back(addMesh.add_vertex(newMesh.point(fv.handle())));
								reFH.push_back(newVH[newVH.size() - 1]);
							}
							else {
								newVH.push_back(addMesh.add_vertex(Point(newMesh.centerPoint)));
								reFH.push_back(newVH[newVH.size() - 1]);
							}
						}
						FHandle reCheck = addMesh.add_face(reFH);
						if (reCheck.idx() == -1)
						{
							std::cout << "reCheck fail\n";
						}
					}

				}
			}


			// new Mesh second processing
			std::map <int, std::pair<int, std::map<int, bool>>> boundaryConnect;
			for (VIter v = addMesh.vertices_begin(); v != addMesh.vertices_end(); ++v)
			{
				if (addMesh.is_boundary(v))
				{
					boundaryConnect[v.handle().idx()] = std::pair<int, std::map<int, bool>>();
					boundaryConnect[v.handle().idx()].first = v.handle().idx();
					for (VVIter vv = addMesh.vv_begin(v); vv; ++vv)
					{
						if (addMesh.is_boundary(vv))
						{
							boundaryConnect[v.handle().idx()].second[vv.handle().idx()] = true;
						}
					}
				}
			}
			//if (boundaryConnect.find(connect[i]) == boundaryConnect.end())
			//{
			//	boundaryConnect[connect[i]] = std::pair<int, std::map<int, bool>>();
			//	boundaryConnect[connect[i]].first = connect[i];
			//}
			//if (i != j)
			//{
			//	boundaryConnect[connect[i]].second[connect[j]] = true;
			//}

			Allmeshs.push_back(addMesh);
		}
		else
		{
			Allmeshs.push_back(newMesh);
		}
	}

	//std::cout << "------------face_saperate------------\n";
}

bool Tri_Mesh::isintriangle( Tri_Mesh *mesh , FIter fiter, OpenMesh::Vec3d pt )
{
	OpenMesh::Vec3d na, nb, nc;
	float ra, rb, rc, rs;
	float a, b, c , s;
	std::vector <OpenMesh::Vec3d> fv;
	
	for (FVIter fvite = mesh->fv_iter(fiter); fvite; ++fvite)
	{
		//std::cout << "id  " <<  fvite.handle().idx() << "   ";
		fv.push_back(mesh->point(fvite));
	}

	
	na = OpenMesh::cross( fv[0]-pt,  fv[1]-pt);
	nb = OpenMesh::cross(fv[1] - pt, fv[2] - pt);
	nc = OpenMesh::cross(fv[2] - pt, fv[0] - pt);
	ra = 0.5 * sqrtf(pow(na[0], 2)  + pow(na[1], 2) + pow(na[2], 2));
	rb = 0.5 * sqrtf(pow(nb[0], 2) + pow(nb[1], 2) + pow(nb[2], 2));
	rc = 0.5  * sqrtf(pow(nc[0], 2) + pow(nc[1], 2) + pow(nc[2], 2));
	
	
	rs =mesh->fd[fiter.handle().idx()].getArea();

	std::cout << "rs " << rs << std::endl;
	std::cout << "abc " << ra + rb + rc << std::endl;
	
	a = ra / rs;
	b = rb / rs;
	c = rc / rs;
	s = a + b + c;
	if (   0.95< s && s <=1.0 && a > 0 && b > 0 && c > 0)
	{
		std::cout << "s " << s << std::endl;
		return false;
	}
	else 
	{
		
		//std::cout << "a " << a << std::endl;
		//std::cout << "b " << b << std::endl;
		//std::cout << "c " << c << std::endl;
		//std::cout << "s " << c << std::endl;
		std::cout << std::endl;
		return true;
	}
}

void Tri_Mesh::remesh()
{

	hypmesh = new Tri_Mesh;
	bool out;
	ReadFile("D:/Mesh-Face-Clustering/OpenMesh_EX/test.obj", hypmesh);
	if (hypmesh)
	{
		std::cout << "load success " << std::endl;
	}
	
	hypmesh->calFaceData();


	
	for (FIter f_iter = hypmesh->faces_begin(); f_iter != hypmesh->faces_end(); ++f_iter)
	{
		
	for (int i = 0; i < fd.size(); i++)
		{
			out = isintriangle(hypmesh, f_iter, fd[i].getfcenter() );
			if (!out)
			{
				break;
			}
		}

		if (out)
		{
			hypmesh->delete_face(f_iter, true);
			hypmesh->garbage_collection();
			f_iter = hypmesh->faces_begin();
		}	

	}
	
	std::cout << std::endl << "face size  " << hypmesh->n_faces() << std::endl;
	/*if (SaveFile("D:/Mesh-Face-Clustering/OpenMesh_EX/final.obj ", hypmesh))
	{
		std::cout << "Save file successful " << std::endl;
	}
	else 
	{
		std::cout << "Save file failed " << std::endl;
	}*/
	/*
	int count = 0;
	for (FIter f_iter = faces_begin(); f_iter != faces_end(); ++f_iter) 
	{
		count = 0;
		for( FFIter ffit = ff_begin(f_iter); ffit ; ++ffit)
		{
			count++;
		}
		for (FEIter fe_iter = fe_begin(f_iter); fe_iter; ++fe_iter) 
		{
			HalfedgeHandle ehalf = halfedge_handle(fe_iter, 1);

		}

		std::cout << "count " << count << std::endl;
		if ( count ==1 || count ==0) 
		{
			delete_face(f_iter);
			garbage_collection();
			f_iter = faces_begin();
		}
	
	}
	*/
}

void Tri_Mesh::findBoundary(int fid)
{
	FIter f_it = faces_begin();
	FVIter fv_it1, fv_it2;
	std::map <int, bool> findID;
	std::map <int, bool> findCluster;
	std::vector <Point> tempfn;
	std::vector <int> count;
	for (f_it = faces_begin(); f_it != faces_end(); f_it++) {

		if (fc[f_it.handle().idx()].ColorID == fid)
		{
			std::vector <int> temp;
			for (fv_it1 = fv_iter(f_it); fv_it1; ++fv_it1)
			{
				findCluster[fv_it1.handle().idx()] = true;
				temp.push_back(fv_it1.handle().idx());
			}
			for (int i = 0; i < temp.size(); i++) {
				std::map <int, bool>::iterator m_it;
				if (findID.find(temp[i]) == findID.end())
				{
					for (m_it = findID.begin(); m_it != findID.end(); m_it++)
					{
						float val = (objVertices[m_it->first] - objVertices[temp[i]]).sqrnorm();
						if (val < 0.001)
							break;
					}
					if (m_it == findID.end()) findID[temp[i]] = true;
				}
			}

			for (FFIter ff_it = ff_begin(f_it); ff_it; ff_it++) {
				if (fc[ff_it.handle().idx()].ColorID != fid)
				{
					//if (fd[ff_it.handle().idx()].getArea() >= 0.03f);
					for (fv_it1 = fv_iter(f_it); fv_it1; ++fv_it1)
						for (fv_it2 = fv_iter(ff_it); fv_it2; ++fv_it2)
							if (fv_it1.handle().idx() == fv_it2.handle().idx())
								findID[(fv_it1.handle().idx())] = true;

				}
			}
			if (tempfn.size() == 0)
			{
				tempfn.push_back(calc_face_normal(f_it.handle()));
				count.push_back(1);
			}
			else {
				Point p = calc_face_normal(f_it.handle());
				int i = 0;
				for (i = 0; i < tempfn.size(); i++)
				{
					double dot = p.values_[0] * tempfn[i].values_[0] + p.values_[1] * tempfn[i].values_[1] + p.values_[2] * tempfn[i].values_[2];
					if (dot > 0.9) {
						count[i]++;
						break;
					}
				}
				if (i == tempfn.size()) {
					tempfn.push_back(p);
					count.push_back(1);
				}
			}
		}
	}
	int maxid = 0;
	int maxCount = 0;
	for (int i = 0; i < count.size(); i++) {
		if (count[i] > maxCount) {
			maxid = i;
			maxCount = count[i];
		}
	}
	fn[fid] = tempfn[maxid];
	boundaryID[fid] = findID;
	clusterID[fid] = findCluster;
}

bool ReadFile(std::string _fileName, Tri_Mesh* _mesh)
{
	bool isRead = false;
	OpenMesh::IO::Options opt;
	if (OpenMesh::IO::read_mesh(*_mesh, _fileName, opt))
	{
		//read mesh from filename OK!
		isRead = true;
	}
	if (isRead)
	{
		// If the file did not provide vertex normals and mesh has vertex normal ,then calculate them
		if (!opt.check(OpenMesh::IO::Options::VertexNormal) && _mesh->has_vertex_normals())
		{
			_mesh->update_normals();
		}
	}
	return isRead;
}

bool SaveFile(std::string _fileName, Tri_Mesh* _mesh)
{
	bool isSave = false;
	if (OpenMesh::IO::write_mesh(*_mesh, _fileName))
	{
		isSave = true;
	}
	return isSave;
}
