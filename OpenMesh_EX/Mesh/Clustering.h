#pragma once
#include <OpenMesh/Core/IO/MeshIO.hh>
#include "FaceData.h"


struct  Cluster
{
	OpenMesh::Vec3d norm;
	int count = 0;
	std::vector<int> faceid;
	double facearea = 0;
};
inline float caculadistance(OpenMesh::Vec3f cur, OpenMesh::Vec3f gl)
{
	// sqrt(pow((gl[0] - cur[0]), 2) + pow((gl[1] - cur[1]), 2) + pow((gl[2] - cur[2]), 2));
	//std::cout << "dist  " << sqrt(pow((gl[0] - cur[0]), 2) + pow((gl[1] - cur[1]), 2) + pow((gl[2] - cur[2]), 2)) << std::endl;
	//std::cout << cur <<  std::endl;
	//std::cout << gl << std::endl;
	return sqrtf(pow((cur[0] - gl[0]), 2) + pow((cur[1] - gl[1]), 2) + pow((cur[2] - gl[2]), 2));
}

inline double calecldistance(OpenMesh::Vec3d cur, OpenMesh::Vec3d gl)
{
	// sqrt(pow((gl[0] - cur[0]), 2) + pow((gl[1] - cur[1]), 2) + pow((gl[2] - cur[2]), 2));
	//std::cout << "dist  " << sqrt(pow((gl[0] - cur[0]), 2) + pow((gl[1] - cur[1]), 2) + pow((gl[2] - cur[2]), 2)) << std::endl;
	//std::cout << cur <<  std::endl;
	//std::cout << gl << std::endl;
	return sqrt( pow((cur[0]- gl[0] ), 2) + pow(( cur[1]- gl[1] ), 2) + pow(( cur[2] - gl[2] ), 2)) ;
}
inline double calplanedist(OpenMesh::Vec3d normal, OpenMesh::Vec3d center, OpenMesh::Vec3d qpoint);
inline double  calplanemaxdist(std::vector<int>  faceid, std::vector<faceData>fd)
{
	float max = -INFINITY;

	for (int i = 0; i < faceid.size(); i++)
	{
		for (int j = 0; j < faceid.size(); j++)
		{
			float dist = calplanedist(fd[faceid[i]].getFaceNormal(), fd[faceid[i]].getfcenter(), fd[faceid[j]].getfcenter());
			if (dist > max)
			{
				max = dist;
			}
		}
	}

	return max;
}

inline double calplanedist( OpenMesh::Vec3d normal , OpenMesh::Vec3d center,OpenMesh::Vec3d qpoint)
{
	double  planeequation;
	double normalize;
	double dist;

		planeequation = normal[0]*(qpoint[0]-center[0]) + normal[1] * (qpoint[1] - center[1]) + normal[2] * (qpoint[2] - center[2]);
		normalize = sqrtf(pow(normal[0], 2) + pow(normal[1], 2) + pow(normal[2], 2));
		dist =abs( planeequation / normalize);
		return dist;
}

inline float findfarestdist(std::vector<faceData> fd ,  Cluster A , Cluster B) 
{
	float maxdist = -1000;
	for (int i = 0; i < A.faceid.size(); i++) 
	{
		for (int j = i; j < B.faceid.size(); j++) 
		{
			OpenMesh::Vec3f cur;
			OpenMesh::Vec3f gl;
			float tmpdist=0;
			
			cur = fd[A.faceid[i]].getfcenter();
			gl   = fd[B.faceid[j]].getfcenter();
			
			tmpdist =pow((cur[0] - gl[0]), 2) + pow((cur[1] - gl[1]), 2) + pow((cur[2] - gl[2]), 2);
			if (tmpdist > maxdist) 
			{
				maxdist = tmpdist;
			}
		}
	}
	return maxdist;
}


inline std::vector<float> computebbox(std::vector< OpenMesh::Vec3f>  verts)
{
	float xmax = -10000, xmin = 10000, ymax = -10000, ymin = 10000, zmax = -1000, zmin = 1000;
	std::vector<float> bound;
	std::vector<OpenMesh::Vec3f> bbox;


	for (int i = 0; i < verts.size(); i++)
	{
		if (verts[i][0] > xmax)
		{
			xmax = verts[i][0];
		}
		if (verts[i][0] < xmin)
		{
			xmin = verts[i][0];
		}
		//----------------
		if (verts[i][1] > ymax)
		{
			ymax = verts[i][1];
		}
		if (verts[i][1] < ymin)
		{
			ymin = verts[i][1];
		}
		//---------------------
		if (verts[i][2] > zmax)
		{
			zmax = verts[i][2];
		}
		if (verts[i][2] < zmin)
		{
			zmin = verts[i][2];
		}

	}

	bbox.resize(8);
	bound.resize(6);
	bbox[0] = OpenMesh::Vec3d(xmin, ymin, zmin);
	bbox[1] = OpenMesh::Vec3d(xmax, ymin, zmin);
	bbox[2] = OpenMesh::Vec3d(xmax, ymin, zmax);
	bbox[3] = OpenMesh::Vec3d(xmin, ymin, zmax);
	bbox[4] = OpenMesh::Vec3d(xmax, ymax, zmax);
	bbox[5] = OpenMesh::Vec3d(xmax, ymax, zmin);
	bbox[6] = OpenMesh::Vec3d(xmin, ymax, zmin);
	bbox[7] = OpenMesh::Vec3d(xmin, ymax, zmax);
	
	bound[0] = xmax;
	bound[1] = xmin;
	bound[2] = ymax;
	bound[3] = ymin;
	bound[4] = zmax;
	bound[5] = zmin;


	return bound;
}

inline void  kmeans_init(int size, int K, int* pick)
{
	int i, j, k, rnd;
	// 隨機找K 個不同資料
	std::cout << " K :" << K << std::endl;
	for (k = 0; k < K; ++k) {
		//	std::cout<<"idx  " <<k<<"  val  " << pick[k] << std::endl;
		rnd = rand() % size; // 隨機取一筆
		//std::cout << "rnd  " << rnd<<std::endl;
		for (i = 0; i < k && pick[i] != rnd; ++i);
		if (i == k) pick[k] = rnd; // 沒重覆
		else --k; // 有重覆, 再找一次
	}

}


inline double update_table(int* ch_pt, int* cent_c, OpenMesh::Vec3d* dis_k, int K, int len, Cluster t, std::vector<OpenMesh::Vec3d> center, std::vector<faceData>fd, int* table)
{
	int i, j, k, min_k;
	double dis, min_dis, t_sse = 0.0;

	*ch_pt = 0;                          // 變動點數設0
	memset(cent_c, 0, sizeof(cent_c)); // 各叢聚資料數清0
	memset(dis_k, 0, sizeof(dis_k));   // 各叢聚距離和清0

	for (i = 0; i < len; ++i) {
		// 尋找所屬重心
		min_dis = calecldistance(fd[t.faceid[i]].getfcenter(), center[0]);
		min_k = 0;
		for (k = 1; k < K; ++k) {
			dis = calecldistance(fd[t.faceid[i]].getfcenter(), center[k]);
			if (dis < min_dis)
				min_dis = dis, min_k = k;
		}
		*ch_pt += (table[i] != min_k); // 更新變動點數
		table[i] = min_k;          // 更新所屬重心
		++cent_c[min_k];           // 累計重心資料數        
		t_sse += min_dis;          // 累計總重心距離
		for (j = 0; j < 3; ++j)     // 更新各叢聚總距離
			dis_k[min_k][j] += fd[t.faceid[i]].getfcenter()[j];

	}
	return t_sse;
}

inline double caclclusterdis(std::vector<OpenMesh::Vec3d> center)
{
	double dist = 0;

	for (int i = 0; i < center.size(); i++)
	{
		for (int j = i + 1; j < center.size(); j++)
		{
			dist += calecldistance(center[i], center[j]);
		}
	}
	return dist;
}
inline void update_cent(int K, std::vector<OpenMesh::Vec3d>center, int* cent_c, OpenMesh::Vec3d* dis_k)
{
	int k, j;
	for (k = 0; k < K; ++k)
		for (j = 0; j < 3; ++j)
			center[k][j] = dis_k[k][j] / cent_c[k];
}

inline void Kmeans(std::vector<faceData>fd, Cluster* t, int* index, Cluster* output, int* outputidx, std::vector<OpenMesh::Vec3d> ctr)
{
	std::vector<OpenMesh::Vec3d> center;
	int len;
	int Max_iter = 20;
	int Min_PT = 0;
	int* pick;
	int K;
	int* cent_c;
	OpenMesh::Vec3d* dis_k;
	int* table;
	int ch_pt;
	int iter = 0;
	double sse1;
	double sse2;
	double minclustdist = 1000;
	int bk;
	// inti kmeans initial center
	for (int n = 0; n < *index; n++)
	{

		len = t[n].faceid.size();

		if (len > 2) { K = 2; }
		else
		{
			K = 1;
		}

		sse1 = 0;
		sse2 = 0;
		cent_c = new int[K];
		table = new int[len];
		pick = new int[K];
		dis_k = new OpenMesh::Vec3d[K];
		iter = 0;
		center.clear();
		std::cout << "n    " << n << std::endl;
		kmeans_init(len, K, pick);

		for (int k = 0; k < K; k++)
		{

			center.push_back(fd[t[n].faceid[pick[k]]].getfcenter());
		}

		//----------------------------------------
		sse2 = update_table(&ch_pt, cent_c, dis_k, K, len, t[n], center, fd, table);
		do {

			std::cout << "ch_pt  " << ch_pt << std::endl;
			sse1 = sse2, ++iter;
			update_cent(K, center, cent_c, dis_k);
			sse2 = update_table(&ch_pt, cent_c, dis_k, K, len, t[n], center, fd, table);
		} while (iter<Max_iter && sse1 != sse2 && ch_pt>Min_PT);
		std::cout << "testing kmeans" << std::endl;
		std::cout << "sse    =  " << sse2 << std::endl;
		//	std::cout << "ch_pt =  " << ch_pt << std::endl;
			//std::cout << "iter     =  " << iter << std::endl;


		bool isempty = true;
		std::cout << n << "   <-n clusters   =  " << bk << std::endl;
		for (int i = 0; i < K; i++)
		{
			std::cout << "Im inside" << std::endl;
			//t[*index].faceid.push_back(pick[i]);
			for (int j = 0; j < len; j++)
			{
				if (pick[table[j]] == pick[i])
				{
					isempty = false;
					output[*outputidx].faceid.push_back(t[n].faceid[j]);
				}
			}
			if (!isempty)
			{
				*outputidx += 1;
				isempty = true;
			}
		}
	}

	/*
	for (int i = 0; i < 3; i++)
	{
		std::cout << "CLuster  " << pick[i] << "  ------------------------------" << std::endl;
		for (int j = 0; j < len; j++)
		{
			if (pick[table[j]] == pick[i])
			{
				std::cout << j << "    " << pick[table[j]] << std::endl;
			}
		}
	}*/
}



//-----------------------

inline int find_faceid(int targetid , Cluster *cl , int index) 
{
		for (int i = 0; i < index; i++)
		{
			for (int j = 0; j < cl[i].faceid.size(); j++)
			{
				if (cl[i].faceid[j] == targetid) {
					return i;
				}
			}
		}

	return -1;
}

inline int find_normal(std::vector<Cluster> norcarray, OpenMesh::Vec3d curvec)
{
	for (int i = 0; i < norcarray.size(); i++) {
		if (OpenMesh::dot(curvec, norcarray[i].norm) >0.90)
		{
			return i;
		}
	}
	return -1;
}


inline int pcompare(const void* a, const void* b)//這函式是 qsort 所需的比較函式
{
	Cluster va, vb;

	va = *(Cluster*)a;
	vb = *(Cluster*)b;

	if (va.count < vb.count) { return 1; }               //傳回 -1 代表 a < b
	else if (va.count == vb.count) { return 0; }      //傳回   0 代表 a = b
	else return -1;                          //傳回  1 代表 a>b
}

inline int acompare(const void* a, const void* b)//這函式是 qsort 所需的比較函式
{
	Cluster va, vb;

	va = *(Cluster*)a;
	vb = *(Cluster*)b;

	if (va.facearea < vb.facearea) { return 1; }               //傳回 -1 代表 a < b
	else if (va.facearea == vb.facearea) { return 0; }      //傳回   0 代表 a = b
	else return -1;                          //傳回  1 代表 a>b
}

inline OpenMesh::Vec3d pointproject(OpenMesh::Vec3d normal, OpenMesh::Vec3d center, OpenMesh::Vec3d point)
{
	float a, b, c, d, t;
	float tup, tdwn;
	OpenMesh::Vec3d root;
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


inline bool covcomparex(OpenMesh::Vec3d a, OpenMesh::Vec3d b)
{
	return (a[1] < b[1]) || (a[1] == b[1] && a[2] < b[2]);
}

inline bool covcomparey(OpenMesh::Vec3d a, OpenMesh::Vec3d b)
{
	return (a[0] < b[0]) || (a[0] == b[0] && a[2] < b[2]);
}

inline bool covcomparez(OpenMesh::Vec3d a, OpenMesh::Vec3d b)
{
	return (a[0] < b[0]) || (a[0] == b[0] && a[1] < b[1]);
}

inline double oricross(OpenMesh::Vec3d o, OpenMesh::Vec3d a, OpenMesh::Vec3d b , int idx)
{
	if (idx == 1)
	{
		return (a[0] - o[0]) * (b[2] - o[2]) - (a[2] - o[2]) * (b[0] - o[0]);
	}
	else if (idx == 2)
	{
		return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]);
	}
	else if (idx == 0)
	{
		return (a[1] - o[1]) * (b[2] - o[2]) - (a[2] - o[2]) * (b[1] - o[1]);
	}
}

inline std::vector<OpenMesh::Vec3d> Andrew_monotone_chain(Cluster A ,  std::vector<faceData>fd)
{
	std::vector<OpenMesh::Vec3d >P, L, U ,CH;
	bool ck=false;
	for(int i =0 ; i <A.faceid.size();i++ )
	{
		for (int j = 0; j < 3; j++)
		{
			ck = false;

			for(int k = 0 ; k < P.size();k++)
			{
				if (P[k] == fd[A.faceid[i]].getVertices()[j])
				{
					ck = true;
				}
			}

			if (ck == false)
			{
				P.push_back(fd[A.faceid[i]].getVertices()[j]);
			}
		}
	}
	L.resize(P.size());
	U.resize(P.size());
	CH.resize(P.size());
	for (int i = 0; i < P.size(); ++i)
	{
		P[i] = pointproject( fd[A.faceid[0]].getFaceNormal(), P[0] , P[i]);
		L[i][0] = L[i][1] = L[i][2] = 0;
		U[i][0] = U[i][1] = U[i][2] = 0;
	}
	
	int idx = 0;

	for (int k = 0; k < 3; k++)
	{
		ck = true;
		for (int i = 1; i < P.size(); ++i)
		{
			if (P[0][k] != P[i][k])
			{
				ck = false;
				break;
			}
		}
		if (ck == true)
		{
			idx = k;
			break;
		}
	}

	if (idx == 0)
	{
		sort(P.begin(), P.end(), covcomparex);
	}
	else if (idx == 1) 
	{
		sort(P.begin(), P.end(), covcomparey);
	}
	else if(idx==2)
	{
		sort(P.begin(), P.end(), covcomparez);
	}


	for(int i =0 ; i < P.size() ; i++)
	{
		std::cout << P[i] << std::endl;
	}
	std::cout << std::endl;



	
	int l = 0, u = 0;   // 下凸包的點數、上凸包的點數
	for (int i = 0; i <P.size(); ++i)
	{
		while (l >= 2 && oricross(L[l - 2], L[l - 1], P[i], idx)<=0) l--;
		while (u >= 2 && oricross(U[u - 2], U[u - 1], P[i], idx)>=0) u--;
		L[l++] = P[i];
		U[u++] = P[i];
	}

	std::vector<OpenMesh::Vec3d> hull;

	for (int i = 0; i <l; i++) 
	{
		hull.push_back(L[i]);
	}
	for (int i = 0; i < u; i++)
	{
		hull.push_back(U[i]);
	}

	return hull;
}


inline  std::vector<OpenMesh::Vec3d> AlphaShapes(Cluster A, std::vector<faceData>fd)
{
	struct Edge
	{
		OpenMesh::Vec3d a;
		OpenMesh::Vec3d b;
	};

	int m_radius =3;

	std::vector<OpenMesh::Vec3d> m_points;  // 点集S
	std::vector<Edge> m_edges;

	m_edges.clear();

	bool ck = false;
	for (int i = 0; i < A.faceid.size(); i++)
	{
		for (int j = 0; j < 3; j++)
		{
			ck = false;

			for (int k = 0; k <  m_points.size(); k++)
			{
				if ( m_points[k] == fd[A.faceid[i]].getVertices()[j])
				{
					ck = true;
				}
			}

			if (ck == false)
			{
				 m_points.push_back(fd[A.faceid[i]].getVertices()[j]);
			}
		}
	}


	std::cout << "origin ----------------------------------------" << std::endl;

	for (int i = 0; i < m_points.size(); i++)
	{
		std::cout << "v " << m_points[i] << std::endl;
	}
	std::cout << std::endl;
	std::cout << "project ----------------------------------------" << std::endl;

	for (int i = 0; i < m_points.size(); ++i)
	{
		m_points[i] = pointproject(fd[A.faceid[0]].getFaceNormal(), m_points[0], m_points[i]);//
	}

	ck = true;
	int idx = 0;
	float sum = 0;
	float sumx = 0, sumy = 0, sumz = 0;
		for (int k = 0; k < 3; k++)
		{
			ck = true;
			for (int i = 1; i < m_points.size(); ++i)
			{
				/*
					if (m_points[0][k] != m_points[i][k])
					{
						ck = false;
						break;
					}						
					*/
				sum += abs(m_points[0][k] - m_points[i][k]);
			}
			if(k==0)
			{
				sumx = sum;
				sum = 0;
			}
			else if (k == 1) 
			{
				sumy = sum;
				sum = 0;
			}
			else if (k == 2)
			{
				sumz = sum;
				sum = 0;
			}
			/*
			if (ck == true) 
			{
				idx = k;
				break;
			}
			*/
		}

		if (sumx < sumy && sumx < sumz) 
		{
			idx = 0;
		}
		else if (sumy < sumx && sumy < sumz) 
		{
			idx = 1;
		}
		else if (sumz < sumx && sumz < sumy)
		{
			idx = 2;
		}

		for (int i = 0; i < m_points.size(); i++)
		{
			std::cout << std::right << std::setw(10) << std::fixed << "v " << m_points[i] << std::endl;
		}
		std::cout << std::endl;


	for (int i = 0; i < m_points.size(); i++)
	{
		// k从i+1开始，减少重复计算
		for (int k = i + 1; k < m_points.size(); k++)
		{
			// 跳过距离大于直径的情况
			if (calecldistance ( m_points[i] , m_points[k]) > 2 * m_radius)
				continue;

			// 两个圆心
		OpenMesh::Vec3d  c1, c2;

			// 线段中点
		OpenMesh::Vec3d  center = 0.5 * (m_points[i] + m_points[k]);

			// 方向向量 d = (x,y)
		OpenMesh::Vec3d dir = m_points[i] - m_points[k];

			// 垂直向量 n = (a,b)  a*dir.x+b*dir.y = 0; a = -(b*dir.y/dir.x)
		OpenMesh::Vec3d  normal;
		if (idx == 0)
		{
			normal[2] = 5;  // 因为未知数有两个，随便给y附一个值5。 

			if (dir[1] != 0)
				normal[1] = -(normal[2] * dir[2]) / dir[1];
			else     // 如果方向平行于y轴
			{
				normal[1] = 1;
				normal[2] = 0;
			}
		}
		else if (idx == 1) 
		{
			normal[2] = 5;  // 因为未知数有两个，随便给y附一个值5。 

			if (dir[0] != 0)
				normal[0] = -(normal[2] * dir[2]) / dir[0];
			else     // 如果方向平行于y轴
			{
				normal[0] = 1;
				normal[2] = 0;
			}
		}
		else if (idx == 2) 
		{
			normal[1] = 5;  // 因为未知数有两个，随便给y附一个值5。 

			if (dir[0] != 0)
				normal[0] = -(normal[1] * dir[1]) / dir[0];
			else     // 如果方向平行于y轴
			{
				normal[0] = 1;
				normal[1] = 0;
			}
		}
			normal.normalize();   // 法向量单位化



			float len = sqrt(m_radius * m_radius - (0.25 * dir.length() * dir.length()));
			c1 = center + len * normal;
			c2 = center - len * normal;

			// b1、b2记录是否在圆C1、C2中找到其他点。
			bool b1 = false, b2 = false;
			for (int m = 0; m < m_points.size(); m++)
			{
				if (m == i || m == k)
					continue;

				if (b1 != true && calecldistance(m_points[m],c1) < m_radius)
					b1 = true;
				if (b2 != true && calecldistance(m_points[m],c2) < m_radius)
					b2 = true;

				// 如果都有内部点，不必再继续检查了
				if (b1 == true && b2 == true)
					break;
			}

			Edge edge;
			if (b1 != true || b2 != true)
			{
				edge.a = m_points[i];
				edge.b = m_points[k];

				m_edges.push_back(edge);
			}

		}
	}

	std::vector<OpenMesh::Vec3d> P;

	for (int i = 0; i < m_edges.size(); i++)
	{
		
		ck = false;
		for (int k = 0; k < P.size(); k++)
		{
			if (P[k] == m_edges[i].a)
			{
				ck = true;
			}
		}
		if (ck == false) 
		{
			P.push_back(m_edges[i].a);
		}
		
		ck = false;
		for (int k = 0; k < P.size(); k++)
		{
			if (P[k] == m_edges[i].b)
			{
				ck = true;
			}
		}
		if (ck == false)
		{
			P.push_back(m_edges[i].b);
		}

	}
	


	return P;
}
