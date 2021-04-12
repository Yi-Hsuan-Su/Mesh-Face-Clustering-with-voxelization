#pragma once
#ifndef _GUA_OM_H_

#define _GUA_OM_H_

#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <OpenMesh/Core/Mesh/PolyMesh_ArrayKernelT.hh>
#include <OpenMesh/Tools/Utils/getopt.h>
#include <windows.h>
#include <gl/gl.h>
#include <gl/glu.h>
#include "Clustering.h"
#include"MeshReconstruct.h"
#include <fstream>
#include <sstream>
#include "FaceData.h"
#include "Voxel.h"

enum ColorState;

struct Face_InnerAngle
{
	double Vertex_Angle[3];
};

class AllPairHarVal_class
{
public:
	std::vector<double> Har_val;
};

namespace OMT//OpenMesh Triangle mesh
{
	using namespace std;
	/*----------------------------------------------------------------------*/

	/*�w�q�ϥΪ���ǫשM���ݩ�*/
	struct MyTraits : OpenMesh::DefaultTraits
	{
		// let Point and Normal be a vector made from doubles
		typedef OpenMesh::Vec3d Point;
		typedef OpenMesh::Vec3d Normal;

		// add normal property to vertices and faces
		VertexAttributes(OpenMesh::Attributes::Normal);
		FaceAttributes(OpenMesh::Attributes::Normal);

		// Already defined in OpenMesh::DefaultTraits
		// HalfedgeAttributes( OpenMesh::Attributes::PrevHalfedge );

		// Uncomment next line to disable attribute PrevHalfedge
		// HalfedgeAttributes( OpenMesh::Attributes::None );
		//
		// or
		//
		// HalfedgeAttributes( 0 );
	};
	/*----------------------------------------------------------------------*/

	/*�w�q�`��type*/
	typedef OpenMesh::TriMesh_ArrayKernelT<MyTraits>	    MyMesh;
	typedef OpenMesh::Vec3d									Vector3d;	//Vec3D type
	typedef MyMesh::Scalar									Scalar;	//Scalar type
	typedef MyMesh::Point									Point;	//Point type
	typedef MyMesh::Normal									Normal;	//Normal type
	typedef MyMesh::VertexHandle							VHandle;	//VertexHandle type
	typedef MyMesh::HalfedgeHandle							HEHandle;	//HalfedgeHandle type
	typedef MyMesh::EdgeHandle							    EHandle;	//edgeHandle type
	typedef MyMesh::FaceHandle								FHandle;	//FaceHandle type
	//-------------Vertex iterators & circulators-------------
	typedef MyMesh::VertexIter								VIter;	//VertexIter type
	typedef MyMesh::VertexVertexIter						VVIter;	//VertexVertexIter type
	typedef MyMesh::VertexEdgeIter							VEIter;	//VertexEdgeIter type
	typedef MyMesh::VertexFaceIter							VFIter;	//VertexFaceIter type
	typedef MyMesh::EdgeIter								EIter;	//EdgeIterT	type
	typedef MyMesh::FaceIter								FIter;	//FaceIter type
	typedef MyMesh::FaceVertexIter							FVIter;	//FaceVertexIter type
	typedef MyMesh::FaceEdgeIter							FEIter;	//FaceEdgeIter type
	typedef MyMesh::FaceFaceIter							FFIter;	//FaceFaceIter type
	typedef MyMesh::VertexOHalfedgeIter						VOHEIter;	//VertexOutHalfEdge type
	typedef MyMesh::ConstVertexVertexIter					CVVIter;	//ConstVertexVertexIter type
	/*----------------------------------------------------------------------*/

	/*�w�q�B�~��Ƶ��c*/
	using namespace OpenMesh;
	/*----------------------------------------------------------------------*/

	/*�w�q���O*/
	class Model :public MyMesh
	{
	public:
		Model();//constructor
		~Model();//de-constructor
	};
}
/*======================================================================*/

namespace OMP//OpenMesh Polygonal mesh
{
	using namespace std;
	/*----------------------------------------------------------------------*/

	/*�w�q�ϥΪ���ǫשM���ݩ�*/
	struct MyTraits : OpenMesh::DefaultTraits
	{
		// let Point and Normal be a vector made from doubles
		typedef OpenMesh::Vec3d Point;
		typedef OpenMesh::Vec3d Normal;

		// add normal property to vertices and faces
		VertexAttributes(OpenMesh::Attributes::Normal);
		FaceAttributes(OpenMesh::Attributes::Normal);

		// Already defined in OpenMesh::DefaultTraits
		// HalfedgeAttributes( OpenMesh::Attributes::PrevHalfedge );

		// Uncomment next line to disable attribute PrevHalfedge
		// HalfedgeAttributes( OpenMesh::Attributes::None );
		//
		// or
		//
		// HalfedgeAttributes( 0 );
	};
	/*----------------------------------------------------------------------*/

	/*�w�q�`��type*/
	typedef OpenMesh::PolyMesh_ArrayKernelT<MyTraits>	    MyMesh;
	typedef OpenMesh::Vec3d									Vector3d;	//Vec3D type
	typedef MyMesh::Scalar									Scalar;	//Scalar type
	typedef MyMesh::Point									Point;	//Point type
	typedef MyMesh::Normal									Normal;	//Normal type
	typedef MyMesh::VertexHandle							VHandle;	//VertexHandle type
	typedef MyMesh::HalfedgeHandle							HEHandle;	//HalfedgeHandle type
	typedef MyMesh::EdgeHandle							    EHandle;	//edgeHandle type
	typedef MyMesh::FaceHandle								FHandle;	//FaceHandle type
	//-------------Vertex iterators & circulators-------------
	typedef MyMesh::VertexIter								VIter;	//VertexIter type
	typedef MyMesh::VertexVertexIter						VVIter;	//VertexVertexIter type
	typedef MyMesh::VertexEdgeIter							VEIter;	//VertexEdgeIter type
	typedef MyMesh::VertexFaceIter							VFIter;	//VertexFaceIter type
	typedef MyMesh::EdgeIter								EIter;	//EdgeIterT	type
	typedef MyMesh::FaceIter								FIter;	//FaceIter type
	typedef MyMesh::FaceVertexIter							FVIter;	//FaceVertexIter type
	typedef MyMesh::FaceEdgeIter							FEIter;	//FaceEdgeIter type
	typedef MyMesh::FaceFaceIter							FFIter;	//FaceFaceIter type
	typedef MyMesh::VertexOHalfedgeIter						VOHEIter;	//VertexOutHalfEdge type
	typedef MyMesh::ConstVertexVertexIter					CVVIter;	//ConstVertexVertexIter type
	/*----------------------------------------------------------------------*/

	/*�w�q�B�~��Ƶ��c*/
	using namespace OpenMesh;
	//���w�S�O�e�X������Ƶ��c
	struct sp_f
	{
		FHandle fh;
		float r, g, b;
	};
	//���w�S�O�e�X���I����Ƶ��c
	struct sp_v
	{
		VHandle vh;
		float r, g, b;
	};
	//���w�t�~�e�X��m����Ƶ��c
	struct sp_p
	{
		Point pt;
		float r, g, b;
	};


	/*----------------------------------------------------------------------*/
	struct srtPath
	{
		std::vector<OMP::VHandle> path;
	};

	/*�w�q���O*/
	class Model :public MyMesh
	{
		//��Ʀ���
	public:
		MyMesh Mesh;													//OpenMesh instance
		MyMesh bboxmesh;
	private:
		vector< sp_p > sp_p_list;
		vector< sp_v > sp_v_list;
		vector< sp_f > sp_f_list;
		//�禡����
	public:
		Model();//constructor
		~Model();//de-constructor

		bool ReadFile(std::string _fileName);//Ū��mesh���
		bool SaveFile(std::string _fileName);//�x�smesh���

		void Render_solid();			//solid rendering
		void Render_wireframe();		//wireframe rendering

		void RenderSpecifiedPoint();	//�e�X���w��m���I
		void RenderSpecifiedVertex();	//�e�X���w�����I
		void RenderSpecifiedFace();		//�e�X���w����
		void add_sp_p(Point   _p, float _r, float _g, float _b);//���w�B�~�e�X���I
		void add_sp_v(VHandle _v, float _r, float _g, float _b);//���w�B�~�e�X�����I
		void add_sp_f(FHandle _f, float _r, float _g, float _b);//���w�B�~�e�X����
		void clear_sp_p();//�M���B�~�e�X���I
		void clear_sp_v();//�M���B�~�e�X�����I
		void clear_sp_f();//�M���B�~�e�X����

		VHandle addVertex(Point _p);										//�bmodel�W�W�[�s�����I
		FHandle addFace(VHandle _v0, VHandle _v1, VHandle _v2, VHandle _v3);//�bmodel�W�W�[�s����
		void deleteFace(FHandle _f);										//�bmodel�W�R����
		void deleteFace(VHandle _v0, VHandle _v1, VHandle _v2, VHandle _v3);//�bmodel�W�R����

		bool IsVertexVertex(VHandle _vj, VHandle _vi);	//�ˬd�⳻�I�O�_�۾F

		int quad_subdivision1(int _face_id);//1to4 quadrilateral subdivision
		int quad_subdivision2(int _face_id);//1to9 quadrilateral subdivision

	private:
		int findVertex(Point _p);//��X��m_p�O�_�w�g�����I�s�b
	};
}
/*======================================================================*/
class Tri_Mesh :public OMT::Model
{
public:
	Tri_Mesh()
	{
		colormap = new double* [1000];
		for (int i = 0; i < 1000; i++)
		{
			colormap[i] = new double[3];
			for (int j = 0; j < 3; j++)
			{
				colormap[i][j] = (double)rand() / (RAND_MAX + 1.0);
				//std::cout << colormap[i][j] << std::endl;
			}
		}
	}
	//-------Edit Flag-------//
	bool                                       Delete_Flag;

	int                                        Constrain_num;
	int                                        Boundary_num;
	OMT::VHandle                               start_vh, end_vh;
	OMT::VHandle                               ExtremeVh[2];
	int                                        PatchType;

	std::vector<OMT::VHandle>                  Pluspt;
	std::vector<OMT::VHandle>                  Minuspt;
	std::vector<OMT::VHandle>                  Extrme_Pt;

	std::map <int, bool> colorPoint;
	std::vector <std::vector<int>> faceVec;
	std::vector <double> faceVecArea;
	std::vector <faceColor> fc;
	std::vector <faceData> fd;
	std::vector <Point> objVertices;
	std::map <int, std::map<int, bool>> boundaryID;
	std::map <int, std::map<int, bool>> clusterID;
	std::map <int, Point> fn;
	std::map <int, bool> usedColor;
	Cluster repcl[300];
	std::vector<std::map <int, int>> containID;
	std::map <Point, float> normalcluster;
	std::pair<Point, int> maxNormalArea;
	std::vector <std::map<int, bool>> finalcluster;
	std::vector <Tri_Mesh> Allmeshs;
	std::vector<OpenMesh::Vec3d> hull;
	double** colormap;
	float tstart , tend ;
	std::vector<OpenMesh::Vec3d> ctr;
	Point centerPoint;
	std::string objFile = "";
	int state;
	double maxArea;
	double maxAreaID;
	void Render_Solid();
	void Render_Voxel();
	void Render_SolidWireframe();
	void Render_newMesh();
	void Render_Wireframe();
	void Render_Point();
	void setFaceColor();
	void setCloseFaceColor();
	void calFaceData();

	bool isintriangle(Tri_Mesh *mesh ,FIter fiter, OpenMesh::Vec3d pt );
	void remesh();
	Tri_Mesh* hypmesh;
	std::vector <Cluster > ada_planedistcluster(std::vector <Cluster >cl, std::vector <Cluster >& output ,  float thre);
	std::vector <Cluster > planedistcluster(std::vector <Cluster >cl, std::vector <Cluster > &output);
	std::vector<Tri_Mesh> bboxmesh;
	std::vector<float> bbox;
	float distthresh=0;
	struct bbox
	{
		float xmax = -10000;
		float xmin = 1000;
		float ymax = -10000;
		float ymin = 10000;
		float zmax = -1000;
		float zmin = 1000;
	};
	struct clustergraph 
	{
		OpenMesh::Vec3f normal;
		OpenMesh::Vec3f center;
		std::vector<int> faceid;
	};
	std::vector<std::vector<float>> adjmatrix;
	OpenMesh::Vec3f getclustercenter(std::vector<int> faceid);
	void contructbbox(clustergraph *cg);
	//bool intersect(bbox a, bbox b);
	float differential(clustergraph a , clustergraph b);
	int facefaceCount();
	void faceCluster(bool close);
	void findBoundary();
	void CheckFaceAllow();
	void facepopCluster();
	void struct_cluster();
	bool writeBoundaryFile(std::string fileName);
	bool saveNewModel();
	bool saveNewModelMtl(char* path, char* fileName);
	bool saveConnectFace(char* path, char* fileName);
	void passObjtoUnity(char* str, int len);
	void saperateFace();
	void hypothesismesh();
	std::vector<Cluster> facade_output;
	std::vector<Cluster> roof_output;
	std::vector<Cluster>	ground_output;
	std::vector <Cluster> result;
	void InitailWeight(std::vector<Cluster> output);
	void UpdaeWeight(int index_i, int index_j );
	void MergeCluster();
	void InitialAdjGraph();
	bool isconnected(Cluster A, Cluster B);

	std::vector<std::vector<std::vector<Voxel>>> voxel;
	int xcellsize;
	int ycellsize;
	int zcellsize;
	void Output_voxelizemesh();
private:
	void facefaceCount(std::map <int, int>& faceCheck, int f_id, int count);
	void setCloseFaceColor(int& count, int f_id, std::vector <int>& faceCheck);
	void findBoundary(int fid);
};

enum ColorState {
	CONNECT, //0
	CLOSEFACE, //1
	NCLOSEFACE, //2
	NFACECLUSTER, //3
	FACECLUSTER, //4
};

///*======================================================================*/
/*======================================================================*/
bool ReadFile(std::string _fileName, Tri_Mesh* _mesh); //Ū��mesh���
bool SaveFile(std::string _fileName, Tri_Mesh* _mesh); //�x�smesh���
/*��l��view port�]�w�禡*/

#endif
