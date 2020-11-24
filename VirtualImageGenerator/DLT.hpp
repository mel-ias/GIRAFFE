//
//#ifndef DLT_H
//#define DLT_H
//
//#include <iostream>
//#include <fstream>
//#include <vector>
//#include <cmath>
//#include <cstdlib>
//#include <Eigen/Dense>
//#include"opencv2\opencv.hpp"
//
//#endif
//
//// DLT von Ferdindand
//
//using namespace std;
//using namespace Eigen;
//
//
//inline void calcDLT(std::vector<cv::Point2d> imageData, std::vector <cv::Point3d> objectData) {
//
//	// Einlesen der Daten durch (Argument = Name der Datei) und Überprüfung
//
//
//
//	// Einlesen der Vektoren
//
//	std::vector<double> IDin, Xin, Yin, Zin, xin, yin;
//	int counter = 0;
//
//	//data.ignore(numeric_limits<streamsize>::max(), '\n');   // Methode um Zeilen zu überspringen
//
//
//	for (auto p : objectData) {
//
//		counter++;
//
//		IDin.push_back(counter);
//		Xin.push_back(p.x);
//		Yin.push_back(p.y);
//		Zin.push_back(p.z);
//	}
//
//	for (auto p : imageData) {
//
//		xin.push_back(p.x);
//		yin.push_back(p.y);
//	}
//
//	int count = IDin.size();
//
//	// Übergabe der Vektoren in Vektoren der Klasse Eigen
//
//	double* IDptr = &IDin[0];
//	double* Xptr = &Xin[0];
//	double* Yptr = &Yin[0];
//	double* Zptr = &Zin[0];
//	double* xptr = &xin[0];
//	double* yptr = &yin[0];
//
//	Map<VectorXd> ID(IDptr, count);
//	Map<VectorXd> X(Xptr, count);
//	Map<VectorXd> Y(Yptr, count);
//	Map<VectorXd> Z(Zptr, count);
//	Map<VectorXd> x(xptr, count);
//	Map<VectorXd> y(yptr, count);
//
//
//	/*
//
//	// Transformation der Koordinaten wenn Ursprung "oben links" liegt
//
//	VectorXd x(count);
//	VectorXd y(count);
//
//	for (int i = 0; i < count; i++) {
//	x(i) = xh(i) - xx / 2;
//	y(i) = yy / 2 - yh(i);
//	}
//	*/
//
//	// Direkte Lineare Transformation
//	// Designmatrix
//
//	MatrixXd A(count * 2, 11);
//	for (int i = 0; i<count; i++) {
//		A(i * 2, 0) = X(i);
//		A(i * 2, 1) = Y(i);
//		A(i * 2, 2) = Z(i);
//		A(i * 2, 3) = 1;
//		A(i * 2, 4) = 0;
//		A(i * 2, 5) = 0;
//		A(i * 2, 6) = 0;
//		A(i * 2, 7) = 0;
//		A(i * 2, 8) = -x(i)*X(i);
//		A(i * 2, 9) = -x(i)*Y(i);
//		A(i * 2, 10) = -x(i)*Z(i);
//
//		A((i * 2 + 1), 0) = 0;
//		A((i * 2 + 1), 1) = 0;
//		A((i * 2 + 1), 2) = 0;
//		A((i * 2 + 1), 3) = 0;
//		A((i * 2 + 1), 4) = X(i);
//		A((i * 2 + 1), 5) = Y(i);
//		A((i * 2 + 1), 6) = Z(i);
//		A((i * 2 + 1), 7) = 1;
//		A((i * 2 + 1), 8) = -y(i)*X(i);
//		A((i * 2 + 1), 9) = -y(i)*Y(i);
//		A((i * 2 + 1), 10) = -y(i)*Z(i);
//	}
//
//
//	// gekürzter Beobachtungsvektor
//
//	MatrixXd l(count * 2, 1);
//	for (int i = 0; i < count; i++) {
//		l(i * 2, 0) = x(i);
//		l((i * 2 + 1), 0) = y(i);
//	}
//
//	// gekürzter Parametervektor
//
//	MatrixXd Hm1, Hm2, Hm3, L, v;
//	Hm1 = A.transpose()*A;
//	Hm2 = Hm1.inverse();
//	Hm3 = A.transpose()*l;
//
//	L = Hm2*Hm3;
//
//	// Verbesserungen, Verbesserungsquadratsumme, Standardabweichung der Gewichtseinheit
//
//	MatrixXd Omega;
//	int f;
//	double s0;
//
//	f = count * 2 - 11;
//	v = A*L - l;
//	Omega = v.transpose()*v;
//	s0 = sqrt(Omega.value() / f);
//
//
//	// Hilfswerte
//
//	Matrix3d abc;
//	double Lsquared, Lsingle, atc, btc, atb, ata, btb;
//	Lsquared = pow(L(8), 2) + pow(L(9), 2) + pow(L(10), 2);
//	Lsingle = sqrt(Lsquared);
//	atc = L(0)*L(8) + L(1)*L(9) + L(2)*L(10);
//	btc = L(4)*L(8) + L(5)*L(9) + L(6)*L(10);
//	atb = L(0)*L(4) + L(1)*L(5) + L(2)*L(6);
//	ata = pow(L(0), 2) + pow(L(1), 2) + pow(L(2), 2);
//	btb = pow(L(4), 2) + pow(L(5), 2) + pow(L(6), 2);
//
//
//	abc << L(0), L(1), L(2),
//		L(4), L(5), L(6),
//		L(8), L(9), L(10);
//
//	// Hauptpunktlage
//
//	double x0, y0;
//	x0 = atc / Lsquared;
//	y0 = btc / Lsquared;
//
//	// Kamerakonstanten in beiden Achsen des Bildkoordinatensystems (wird in diversen Quellen berechnet) - für Ausgabe wird cy verwendet
//
//	double cx, cy;
//	cx = sqrt(ata / Lsquared - pow(x0, 2));
//	cy = sqrt(btb / Lsquared - pow(y0, 2));
//
//	// Scherung und Maßstab der Achsen des Bildkoordinatensystems
//
//	double d, m;
//	d = (atb*Lsquared - atc*btc) / (ata*Lsquared - pow(atc, 2));
//	m = -(abc.determinant()) / (pow(sqrt(Lsquared), 3)*pow(cx, 2));
//
//	// Projektionszentrum
//
//	MatrixXd Proj;
//	Vector3d hv;
//	hv(0) = -L(3);
//	hv(1) = -L(7);
//	hv(2) = -1;
//
//	Proj = abc.inverse()*hv;
//
//	// Rotationsmatrix
//
//	Matrix3d R;
//	double r11, r12, r13, r21, r22, r23, r31, r32, r33;
//	r13 = L(8) / Lsingle;
//	r23 = L(9) / Lsingle;
//	r33 = L(10) / Lsingle;
//	r11 = (x0*r13 - L(0) / Lsingle) / cx;
//	r21 = (x0*r23 - L(1) / Lsingle) / cx;
//	r31 = (x0*r33 - L(2) / Lsingle) / cx;
//	r12 = (y0*r13 - L(4) / Lsingle) / cy;
//	r22 = (y0*r23 - L(5) / Lsingle) / cy;
//	r32 = (y0*r33 - L(6) / Lsingle) / cy;
//
//	R << r11, r12, r13,
//		r21, r22, r23,
//		r31, r32, r33;
//
//	if (R.determinant() < 1) {
//		R = -R;
//	}
//
//	// Ausgabe
//
//	cout.precision(10);
//	cout << "Kamerakonstante:" << endl << cy << endl << "Bildhauptpunkt:" << endl << x0 << endl << y0 << endl
//		<< "Projektionszentrum:" << endl << Proj << endl << "Rotationsmatrix:" << endl << R << endl;
//
//	// Optionale Ausgabe in Text-Datei
//
//
//	ofstream output;
//	output.open("output.txt");
//	output << "Kamerakonstante:" << endl << cy << endl << "Bildhauptpunkt:" << endl << x0 << endl << y0 << endl
//		<< "Projektionszentrum:" << endl << Proj << endl << "Rotationsmatrix:" << endl << R << endl;
//	output.close();
//
//
//
//
//
//
//
//}