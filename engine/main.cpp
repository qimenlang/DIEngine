#include <iostream>
#include <memory>
#include "tgaimage.h"
#include "Model.h"
#include "base.h"
#include "yaml-cpp/yaml.h"


const TGAColor white = TGAColor(255, 255, 255, 255);
const TGAColor red = TGAColor(255, 0, 0, 255);
const TGAColor green = TGAColor(0, 255, 0, 255);
extern std::shared_ptr<Model> model;
extern std::vector<int> zbuffer;
extern TGAImage zbufferImage;
extern Matrix Projection;
int main(int argc, char** argv) {


	YAML::Node node;

	node["language"] = "cpp";
	node["version"] = 2;


	YAML::Node primes = YAML::Load("[2, 3, 5, 7, 11]");


	Projection[3][2] = -1.f / (eye - center).norm();
	TGAImage image(width, height, TGAImage::RGB);
	//GouraudShader shader;
	//GouraudSpecShader shader;
	//DiffuseShader shader;
	//DiffuseLightShader shader;
	//模型绘制代码暂时迁移出来
	//DiffuseNormShader shader;
	PhongShader shader;
	shader.uniform_M = Projection * ModelView;
	shader.uniform_MIT = (Projection * ModelView).invert_transpose();
	for (int i = 0; i < model->nfaces(); i++)
	{
		std::vector<int> face = model->face(i);
		Vec4f screen_coords[3];
		for (int j = 0; j < 3; j++)
		{
			screen_coords[j] = shader.vert(i,j);
		}
		rasterization(screen_coords, shader, image, zbufferImage);
	}
	image.flip_vertically(); // i want to have the origin at the left bottom corner of the image
	image.write_tga_file("output.tga");
	zbufferImage.flip_vertically(); // i want to have the origin at the left bottom corner of the image
	zbufferImage.write_tga_file("zbuffer.tga");
	return 0;
}
