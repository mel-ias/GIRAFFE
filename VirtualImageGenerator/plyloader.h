//#pragma once
//
///**************************************
//* Copyright (c) <2014> <Vishesh Gupta> *
//**************************************
//The MIT / X Window System License
//=================================
//Permission is hereby granted, free of charge, to any person obtaining
//a copy of this software and associated documentation files (the
//"Software"), to deal in the Software without restriction, including
//without limitation the rights to use, copy, modify, merge, publish,
//distribute, sublicense, and/or sell copies of the Software, and to
//permit persons to whom the Software is furnished to do so, subject to
//the following conditions:
//The above copyright notice and this permission notice shall be
//included in all copies or substantial portions of the Software.
//THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
//EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
//MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
//NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
//LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
//OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
//WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//*/
//
//#ifndef PLYLOADER_H
//#define	PLYLOADER_H
//
//#include"opencv2\opencv.hpp"
//#include <vector>
//
//struct PLYModel {
//	std::vector<cv::Vec3d> positions;
//	std::vector<cv::Vec3d> normals;
//	std::vector<cv::Vec3d> colors;
//	std::vector<cv::Vec3i> faces;
//	/*glm::vec3 *positions;
//	glm::vec3 *normals;
//	glm::vec3 *colors;*/
//
//	int vertexCount; //number of vertices
//	float bvWidth, bvHeight, bvDepth; //bounding volume dimensions
//	float bvAspectRatio; //bounding volume aspect ratio
//	int faceCount; //number of faces; if reading meshes
//	bool isMesh; // To tell if this is a mesh or not
//	bool ifColor, ifNormal;
//
//	cv::Vec3d min, max, center;
//
//	PLYModel();
//	PLYModel(const char *filename, bool = 1, bool = 1); //To indicate if normal, color informations are present in the file respectively
//	void PLYWrite(const char *filename, bool = 1, bool = 1);// To indicate if normal, color informations are to be written in the file respectively
//	void FreeMemory();
//};
//
//struct Material {
//	cv::Vec4d Ka;
//	cv::Vec4d Kd;
//	cv::Vec4d Ks;
//	float shininess;
//};
//
//struct Light {
//	cv::Vec4d La;
//	cv::Vec4d Ld;
//	cv::Vec4d Ls;
//	cv::Vec4d position;
//};
//
//#endif	/* PLYLOADER_H */
