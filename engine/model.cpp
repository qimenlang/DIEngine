#include "Model.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
Model::Model(const char* filename):diffuse_tex()
{
	std::ifstream in;
	in.open(filename,std::ifstream::in);
	if (in.fail()) return;
	std::string line;
	while (!in.eof())
	{
		std::getline(in,line);
		std::istringstream iss(line.c_str());
		char trash;
		if (!line.compare(0, 2, "v "))
		{
			iss >> trash;//v 字符放到了这里
			Vec3f v;
			for (int i = 0; i < 3; i++) iss >> v[i];
			verts_.push_back(v);
		}
		else if (!line.compare(0, 4, "vt  "))
		{
			iss >> trash >> trash;//vt 字符放到了这里
			Vec3f vt;
			for (int i = 0; i < 3; i++) iss >> vt[i];
			vts_.push_back(vt);
		}
		else if (!line.compare(0, 4, "vn  "))
		{
			iss >> trash >> trash;//vn 字符放到了这里
			Vec3f vn;
			for (int i = 0; i < 3; i++) iss >> vn[i];
			vns_.push_back(vn);
		}
		else if(!line.compare(0, 2, "f "))
		{
			iss >> trash;
			std::vector<int> f;
			std::vector<int> vt_f;
			std::vector<int> vn_f;
 			int vn_idx, v_idex, vt_idx;
			while (iss>> v_idex >>trash>> vt_idx >>trash>> vn_idx)
			{
				f.push_back(--v_idex);
				vt_f.push_back(--vt_idx);
				vn_f.push_back(--vn_idx);
			}
			faces_.push_back(f);
			vt_faces_.push_back(vt_f);
			vn_faces_.push_back(vn_f);
		}
	}
	std::cerr << "# v# " << verts_.size() 
		<< " vt#" << vts_.size()
		<< " f#" << faces_.size()
		<< " vt_f#" << vt_faces_.size() << std::endl;

	in.close();

	diffuse_tex.read_tga_file("../obj/african_head/african_head_diffuse.tga");
	nm_tex.read_tga_file("../obj/african_head/african_head_nm.tga");
	spec_tex.read_tga_file("../obj/african_head/african_head_spec.tga");
}

TGAColor Model::diffuse(Vec2f uv)
{
	int w = diffuse_tex.get_width() * uv.x;
	int h = diffuse_tex.get_height() * (1 - uv.y);
	return diffuse_tex.get(w,h);
}

Model::~Model()
{
}

int Model::nverts()
{
	return verts_.size();
}

int Model::nfaces()
{
	return faces_.size();
}

int Model::nvt_faces()
{
	return vt_faces_.size();
}

int Model::nvts()
{
	return vts_.size();
}

Vec3f Model::vert(int i)
{
	if (i < verts_.size())
		return verts_[i];
	else
		return Vec3f();
}

Vec3f Model::uv(int i,int j)
{
	if (vt_faces_[i][j] < vts_.size())
		return vts_[vt_faces_[i][j]];
	else
		return Vec3f();
}

Vec3f Model::norm(int i, int j)
{
	if (vn_faces_[i][j] < vns_.size())
		return vns_[vn_faces_[i][j]];
	else
		return Vec3f();
}

Vec3f Model::norm(Vec2f uv)
{
	int w = nm_tex.get_width() * uv.x;
	int h = nm_tex.get_height() * (1 - uv.y);
	TGAColor c = nm_tex.get(w, h);
	Vec3f res;
	for (int i = 0; i < 3; i++)
		res[2 - i] = (float)c[i] / 255.f * 2.f - 1.f;
	return res;
}

float Model::spec(Vec2f uv)
{
	int w = spec_tex.get_width() * uv.x;
	int h = spec_tex.get_height() * (1 - uv.y);
	return spec_tex.get(w, h)[0]/1.f;
}

std::vector<int> Model::face(int idx)
{
	return faces_[idx];
}
