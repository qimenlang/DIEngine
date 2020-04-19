#pragma once
#include <vector>
#include <memory>
#include "geometry.h"
#include "tgaimage.h"

class Model
{
public:
	Model(const char* filename);
	~Model();

	TGAColor diffuse(Vec2f uv);
	int nverts();
	int nfaces();
	int nvt_faces();
	int nvts();
	Vec3f vert(int i);
	Vec3f uv(int i,int j);
	Vec3f norm(int i, int j);
	Vec3f norm(Vec2f uv);
	float spec(Vec2f uv);
	std::vector<int> face(int idx);
private:
	std::vector<Vec3f> verts_;
	std::vector<Vec3f> vts_;
	std::vector<Vec3f> vns_;
	std::vector<std::vector<int>> faces_;
	std::vector<std::vector<int>> vt_faces_;
	std::vector<std::vector<int>> vn_faces_;
	TGAImage diffuse_tex;
	TGAImage nm_tex;
	TGAImage spec_tex;
};