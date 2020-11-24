
#ifndef FILLIMAGES_H
#define FILLIMAGES_H

#include <opencv2/core/core.hpp>

#include <iostream>
#include <string>
#include <algorithm>
#include "kdtree.h"
#include <omp.h>

#endif




//########################################
inline void berechneVisualisierung(
	cv::Mat& bild,
	//const cv::Mat& original,
	const cv::Mat& maske, // Bild mit Pixeln mit 0 oder 255
	const std::vector<Vek2d>& punkte,
	const std::vector<cv::Scalar>& farbenPunkte,
	//const KdTree2f& kdtree,
	size_t k_for_knn)
{
	assert(!punkte.empty());
	assert(farbenPunkte.size() == punkte.size());
	assert(k_for_knn > 0);
	assert(!bild.empty());
	assert(bild.rows > 0 && bild.cols > 0);
	assert(bild.rows == maske.rows && bild.cols == maske.cols);
	assert(maske.channels() == 1);

	KdTree2d kdtree(punkte, 12, 30);
	kdtree.init();

	if (!kdtree.istInitialisiert()) {
		cout << "Probleme bei Erstellung des KdTree!\n";
		return;
	}

	//original.copyTo(bild);
	uchar* zeigerBild = bild.data;

	int idx_row, idx_row_end = bild.rows;

	std::vector<const uchar*> pMaske;
	for (int i = 0; i < maske.rows; ++i) pMaske.push_back(maske.ptr<uchar>(i));
	uchar wert255 = static_cast<uchar>(255);

	int anzahlKanaele = bild.channels();

	//for(int idx_row = 0, idx_row_end = bild.rows;
	//idx_row < idx_row_end; ++idx_row)
#pragma omp parallel for num_threads(omp_get_num_procs()) schedule(dynamic)
	for (idx_row = 0; idx_row < idx_row_end; ++idx_row)
		for (int idx_col = 0, idx_col_end = bild.cols;
			idx_col < idx_col_end; ++idx_col)
			//if(maske.at<uchar>(idx_row, idx_col) == (uchar)255) {
			if (pMaske[idx_row][idx_col] == wert255) {

				//int idxThread = omp_get_thread_num();
				std::vector<NachbarDatend> nachbarDaten;
				kdtree.knn(nachbarDaten, Vek2d(idx_col, idx_row), k_for_knn);

				float gewicht, summeGewichte(0.0f);
				Vek3f farbe(0, 0, 0);
				for (auto& n : nachbarDaten) {
					unsigned int idxN = n.idx;
					if (n.distanz != 0.0f) gewicht = 1.0f / n.distanz;
					else gewicht = 0.5f;//1.0f;
					summeGewichte += gewicht;
					for (unsigned int k = 0; k < 3; ++k)
						farbe[k] += farbenPunkte[idxN][k] * gewicht;

					//std::cout << farbe.x() << "," << farbe.y() << "," << farbe.z() << std::endl;
				}

				if (summeGewichte != 0.0f) {
					farbe *= (1.0f / summeGewichte);
					//                bild.at<cv::Vec3b>(idx_row, idx_col)
					//                        = cv::Vec3b(farbe[0], farbe[1], farbe[2]);
					int idxPixel = (bild.cols*idx_row + idx_col)*anzahlKanaele;
					for (int k = 0; k < anzahlKanaele; ++k)
						zeigerBild[idxPixel + k] = cv::saturate_cast<uchar>(farbe[k]);
				}
			}
}

// Berechne Visualisierung für Typ 32SC1
//########################################
inline void berechneVisualisierung32SC1(
	cv::Mat& bild,
	//const cv::Mat& original,
	const cv::Mat& maske, // Bild mit Pixeln mit 0 oder 255
	const std::vector<Vek2d>& punkte,
	const std::vector<Vek3i>& farbenPunkte,
	//const KdTree2f& kdtree,
	size_t k_for_knn)

{
	assert(!punkte.empty());
	assert(farbenPunkte.size() == punkte.size());
	assert(k_for_knn > 0);
	assert(!bild.empty());
	assert(bild.rows > 0 && bild.cols > 0);
	assert(bild.rows == maske.rows && bild.cols == maske.cols);
	assert(maske.channels() == 1);

	KdTree2d kdtree(punkte, 12, 30);
	kdtree.init();

	if (!kdtree.istInitialisiert()) {
		cout << "Probleme bei Erstellung des KdTree!\n";
		return;
	}

	//original.copyTo(bild);
	int* zeigerBild = bild.ptr<int>(0);


	int idx_row, idx_row_end = bild.rows;

	std::vector<const uchar*> pMaske;
	for (int i = 0; i < maske.rows; ++i) pMaske.push_back(maske.ptr<uchar>(i));
	uchar wert255 = static_cast<uchar>(255);


	//for(int idx_row = 0, idx_row_end = bild.rows;
	//idx_row < idx_row_end; ++idx_row)
#pragma omp parallel for num_threads(omp_get_num_procs()) schedule(dynamic)
	for (idx_row = 0; idx_row < idx_row_end; ++idx_row)
		for (int idx_col = 0, idx_col_end = bild.cols;
			idx_col < idx_col_end; ++idx_col)
			//if(maske.at<uchar>(idx_row, idx_col) == (uchar)255) {
			if (pMaske[idx_row][idx_col] == wert255) {

				//int idxThread = omp_get_thread_num();
				std::vector<NachbarDatend> nachbarDaten;
				kdtree.knn(nachbarDaten, Vek2d(idx_col, idx_row), k_for_knn);

				float gewicht, summeGewichte(0.0f), farbe(0);

				for (auto& n : nachbarDaten) {
					unsigned int idxN = n.idx;
					if (n.distanz != 0.0f) gewicht = 1.0f / n.distanz;
					else gewicht = 0.5f;//1.0f;
					summeGewichte += gewicht;
					farbe += farbenPunkte[idxN][0] * gewicht;

					//std::cout << farbe.x() << "," << farbe.y() << "," << farbe.z() << std::endl;
				}

				if (summeGewichte != 0.0f) {
					farbe *= (1.0f / summeGewichte);
					//                bild.at<cv::Vec3b>(idx_row, idx_col)
					//                        = cv::Vec3b(farbe[0], farbe[1], farbe[2]);
					int idxPixel = (bild.cols*idx_row + idx_col);
					zeigerBild[idxPixel] = cv::saturate_cast<int>(farbe);
				}
			}
}



/////////////////////////////////////////////////
// Berechne Visualisierung für Radius
inline void berechneVisualisierung(
	cv::Mat& bild,
	//const cv::Mat& original,
	const cv::Mat& maske, // 0 255
	const std::vector<Vek2d>& punkte,
	const std::vector<Vek3i>& farbenPunkte,
	//const KdTree2f& kdtree,
	float radius
) {

	assert(!punkte.empty());
	assert(farbenPunkte.size() == punkte.size());
	assert(radius > 0.0f);
	assert(!bild.empty());
	assert(bild.rows == maske.rows && bild.cols == maske.cols);
	assert(maske.channels() == 1);

	KdTree2d kdtree(punkte, 12, 30);//maxTiefe = 12,//anzahlPunkteAbbruch = 30
	kdtree.init();

	if (!kdtree.istInitialisiert()) {
		cout << "Probleme bei Erstellung des KdTree!\n";
		return;
	}

	//original.copyTo(bild);
	uchar* zeigerBild = bild.data;

	int idx_row, idx_row_end = bild.rows;

	std::vector<const uchar*> pMaske;
	for (int i = 0; i < maske.rows; ++i) pMaske.push_back(maske.ptr<uchar>(i));
	uchar wert255 = static_cast<uchar>(255);

	int anzahlKanaele = bild.channels();

	//for(int idx_row = 0, idx_row_end = bild.rows;
	//idx_row < idx_row_end; ++idx_row)
#pragma omp parallel for num_threads(omp_get_num_procs()) schedule(dynamic)
	for (idx_row = 0; idx_row < idx_row_end; ++idx_row)
		for (int idx_col = 0, idx_col_end = bild.cols;
			idx_col < idx_col_end; ++idx_col)
			//if(maske.at<uchar>(idx_row, idx_col) == (uchar)255) {
			if (pMaske[idx_row][idx_col] == wert255) {

				//int idxThread = omp_get_thread_num();
				std::vector<NachbarDatend> nachbarDaten;
				float radius_ = radius;
				int iter = 0, maxIter = 3;
				do {
					kdtree.nachbarnImUmkreis(nachbarDaten,
						Vek2d(idx_col, idx_row), radius_);
					if (!nachbarDaten.empty()) break;
					radius_ *= 2;
				} while (iter++ < maxIter);

				float gewicht, summeGewichte(0.0f);
				Vek3f farbe(0, 0, 0);
				for (auto& n : nachbarDaten) {
					unsigned int idxN = n.idx;
					if (n.distanz != 0.0f) gewicht = 1.0f / n.distanz;
					else gewicht = 0.5f;//1.0;
					summeGewichte += gewicht;
					for (unsigned int k = 0; k < 3; ++k) {
						farbe[k] += farbenPunkte[idxN][k] * gewicht;
						//std::cout << farbe[k] << ";" ;
					}
				}

				if (summeGewichte != 0.0f) {
					farbe *= (1.0f / summeGewichte);
					int idxPixel = (bild.cols*idx_row + idx_col)*anzahlKanaele;
					for (int k = 0; k < anzahlKanaele; ++k) {
						zeigerBild[idxPixel + k] = cv::saturate_cast<uchar>(farbe[k]);

					}
				}
			}
}








