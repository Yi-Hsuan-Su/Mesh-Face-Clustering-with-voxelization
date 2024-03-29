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

	/*定義使用的精準度和基本屬性*/
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

	/*定義常用type*/
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

	/*定義額外資料結構*/
	using namespace OpenMesh;
	/*----------------------------------------------------------------------*/

	/*定義類別*/
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

	/*定義使用的精準度和基本屬性*/
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

	/*定義常用type*/
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

	/*定義額外資料結構*/
	using namespace OpenMesh;
	//指定特別畫出面的資料結構
	struct sp_f
	{
		FHandle fh;
		float r, g, b;
	};
	//指定特別畫出頂點的資料結構
	struct sp_v
	{
		VHandle vh;
		float r, g, b;
	};
	//指定另外畫出位置的資料結構
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

	/*定義類別*/
	class Model :public MyMesh
	{
		//資料成員
	public:
		MyMesh Mesh;													//OpenMesh instance
		MyMesh bboxmesh;
	private:
		vector< sp_p > sp_p_list;
		vector< sp_v > sp_v_list;
		vector< sp_f > sp_f_list;
		//函式成員
	public:
		Model();//constructor
		~Model();//de-constructor

		bool ReadFile(std::string _fileName);//讀取mesh資料
		bool SaveFile(std::string _fileName);//儲存mesh資料

		void Render_solid();			//solid rendering
		void Render_wireframe();		//wireframe rendering

		void RenderSpecifiedPoint();	//畫出指定位置的點
		void RenderSpecifiedVertex();	//畫出指定的頂點
		void RenderSpecifiedFace();		//畫出指定的面
		void add_sp_p(Point   _p, float _r, float _g, float _b);//指定額外畫出的點
		void add_sp_v(VHandle _v, float _r, float _g, float _b);//指定額外畫出的頂點
		void add_sp_f(FHandle _f, float _r, float _g, float _b);//指定額外畫出的面
		void clear_sp_p();//清空額外畫出的點
		void clear_sp_v();//清空額外畫出的頂點
		void clear_sp_f();//清空額外畫出的面

		VHandle addVertex(Point _p);										//在model上增加新的頂點
		FHandle addFace(VHandle _v0, VHandle _v1, VHandle _v2, VHandle _v3);//在model上增加新的面
		void deleteFace(FHandle _f);										//在model上刪除面
		void deleteFace(VHandle _v0, VHandle _v1, VHandle _v2, VHandle _v3);//在model上刪除面

		bool IsVertexVertex(VHandle _vj, VHandle _vi);	//檢查兩頂點是否相鄰

		int quad_subdivision1(int _face_id);//1to4 quadrilateral subdivision
		int quad_subdivision2(int _face_id);//1to9 quadrilateral subdivision

	private:
		int findVertex(Point _p);//找出位置_p是否已經有頂點存在
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
	void voxelize(float scale);
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
bool ReadFile(std::string _fileName, Tri_Mesh* _mesh); //讀取mesh資料
bool SaveFile(std::string _fileName, Tri_Mesh* _mesh); //儲存mesh資料
/*初始化view port設定函式*/

#endif


inline OpenMesh::Vec3f point_project(OpenMesh::Vec3f normal, OpenMesh::Vec3f center, OpenMesh::Vec3f point)
{
	float a, b, c, d, t;
	float tup, tdwn;
	OpenMesh::Vec3f root;
	a = normal[0];
	b = normal[1];
	c = normal[2];
	d = a * center[0] + b * center[1] + c * center[2];
	tup = -1 * ((a * point[0]) + (b * point[1]) + (c * point[2]) - d);
	tdwn = pow(a, 2) + pow(b, 2) + pow(c, 2);
	t = tup / tdwn;
	root[0] = point[0] + a * t;
	root[1] = point[1] + b * t;
	root[2] = point[2] + c * t;

	return root;
}
inline OpenMesh::Vec3f intersectPoint(OpenMesh::Vec3f rayVector, OpenMesh::Vec3f rayPoint, OpenMesh::Vec3f planeNormal, OpenMesh::Vec3f planePoint)
{
	OpenMesh::Vec3f diff = rayPoint - planePoint;
	double prod1 = dot(diff, planeNormal);
	double prod2 = dot(rayVector, planeNormal);
	double prod3 = prod1 / prod2;
	return rayPoint - rayVector * prod3;
}


inline OpenMesh::Vec3f line_inertsect_plane(OpenMesh::Vec3f normal, OpenMesh::Vec3f center, OpenMesh::Vec3f to, OpenMesh::Vec3f from)
{

	float a, b, c, d, t;

	OpenMesh::Vec3f root, lnorm, pto, pfrom, tnorm, fnorm;
	float tod, fromd;
	float linea, lineb, linec;


	lnorm = from - to;
	lnorm.normalized();


	if (dot(lnorm, normal) == 0)
	{
		root = { -1000,-1000,-1000 };
		return root;
	}

	pto = point_project(normal, center, to);
	pfrom = point_project(normal, center, from);




	tnorm = to - pto;
	fnorm = from - pfrom;
	tnorm.normalized();
	fnorm.normalized();


	if (dot(tnorm, fnorm) < 0.00)
	{
		root = intersectPoint(lnorm, to, normal, center);
	}
	else
	{
		root = { -1000,-1000,-1000 };
	}

	return root;
}



inline float caltrianglearea(OpenMesh::Vec3f verticesA, OpenMesh::Vec3f verticesB, OpenMesh::Vec3f verticesC)
{
	float a = (verticesA - verticesB).length();
	float b = (verticesB - verticesC).length();
	float c = (verticesA - verticesC).length();
	float s = (a + b + c) / 2.0;
	float sa, sb, sc;
	sa = s - a;
	sb = s - b;
	sc = s - c;
	return  sqrt(s * (s - a) * (s - b) * (s - c));
}
inline bool isintriangle(faceData f, OpenMesh::Vec3f pt)
{
	OpenMesh::Vec3f na, nb, nc;
	float ra, rb, rc, rs;
	float a, b, c, s;
	std::vector <OpenMesh::Vec3f> fv;

	for (int i = 0; i < f.getVertices().size(); i++)
	{
		fv.push_back((OpenMesh::Vec3f)f.getVertices()[i]);
	}

	ra = caltrianglearea(fv[0], fv[1], pt);
	rb = caltrianglearea(fv[1], fv[2], pt);
	rc = caltrianglearea(fv[2], fv[0], pt);
	rs = f.getArea();

	a = ra / rs;
	b = rb / rs;
	c = rc / rs;
	s = a + b + c;
	/*
	std::cout << "a  =" << a << std::endl;
	std::cout << "b  =" << b<< std::endl;
	std::cout << "c  =" << c << std::endl;*/

	if (0.95000 < s && s <= 1.010 && a >= 0 && b >= 0 && c >= 0)
	{

		//	std::cout << "s   =  " << s << std::endl;
		return true;
	}
	else
	{

		return false;
	}
}

