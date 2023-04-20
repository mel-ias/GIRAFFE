#include "Modell_OCV.h"



Modell_OCV::Modell_OCV(LogFile* _logfile) {

	logfile = _logfile;
	logfile->append("");
	logfile->append(TAG + "---- initialisation open computer vision modell ----");


}

Modell_OCV::~Modell_OCV() {
}


/*
* INFOS:
* returns corresponding colors for all points of vector<cv::Point3d> point_cloud using input image cv::Mat& image_for_color
* overrides vector point_cloud_colors because of referencing!
* in case of water line points, use kd tree nearest neighbour approach to find next image point of projected point cloud to input water line point (iterative) 
* use fix_aspect_ratio != 0.0 in case of fx=fy (standard fix_aspect_ratio = 1.0)
* for project points: 
	for( i = 0; i < count; i++ )
	{
		double X = M[i].x, Y = M[i].y, Z = M[i].z;
		double x = R[0]*X + R[1]*Y + R[2]*Z + t[0];
		double y = R[3]*X + R[4]*Y + R[5]*Z + t[1];
		double z = R[6]*X + R[7]*Y + R[8]*Z + t[2];
		double r2, r4, r6, a1, a2, a3, cdist, icdist2;
		double xd, yd, xd0, yd0, invProj;		
		
		z = z ? 1./z : 1;
		x *= z; y *= z;
		
		r2 = x*x + y*y;
		r4 = r2*r2;
		r6 = r4*r2;
		a1 = 2*x*y;
		a2 = r2 + 2*x*x;
		a3 = r2 + 2*y*y;
		cdist = 1 + k[0]*r2 + k[1]*r4 + k[4]*r6; //k = distCoeffs
		icdist2 = 1./(1 + k[5]*r2 + k[6]*r4 + k[7]*r6);
		xd = x*cdist*icdist2 + k[2]*a1 + k[3]*a2 + k[8]*r2+k[9]*r4;
		yd = y*cdist*icdist2 + k[2]*a3 + k[3]*a1 + k[10]*r2+k[11]*r4;
				
		m[i].x = xd*fx + cx;
		m[i].y = yd*fy + cy;
}
*/
std::vector<cv::Point3d> Modell_OCV::getColorFor(std::vector<cv::Point3d>& point_cloud, cv::Mat& image_for_color, std::vector<cv::Vec3b>& point_cloud_colors, std::vector<cv::Point2d>& image_coords_colors, bool fix_aspect_ratio, cv::Mat& cameraMatrix, cv::Mat& distCoeffs, cv::Mat& rvec, cv::Mat& tvec, std::vector<cv::Point2d>& waterlinePoints) {

	
	std::vector<cv::Point2d> image_pixels;
	std::vector<cv::Point3d> waterlinePoints_projected;

	cv::projectPoints(point_cloud, rvec, tvec, cameraMatrix, distCoeffs, image_pixels, cv::noArray(), fix_aspect_ratio);

	//Test
	//std::ofstream outputFile("my_test_file.csv");
	//for (cv::Point2f pixel: image_pixels) {
	//		outputFile << std::fixed << pixel.x << "," << std::fixed << pixel.y << std::endl;
	//}
	//outputFile.close();


	std::cout << "image_pixels size: " << image_pixels.size() << endl;

	// iterate through all pixel coordinates and save corresponding color from image_for_color

	for (int i = 0; i < image_pixels.size(); i++) {
		cv::Point2d pixel = image_pixels[i];

		if (pixel.x > 0 && pixel.y > 0 && pixel.x < image_for_color.cols && pixel.y < image_for_color.rows) {
			point_cloud_colors.push_back(image_for_color.at<cv::Vec3b>(pixel.y, pixel.x));
			image_coords_colors.push_back(cv::Point2d(pixel.x, pixel.y));
		}
		else {
			point_cloud_colors.push_back(cv::Vec3b(0, 0, 0)); //add black if outside of image	
			image_coords_colors.push_back(cv::Point2d(-1, -1)); // negative coordinates not exist thus add -1
		}
	}

	std::cout << "pointcloud_color size: " << point_cloud_colors.size() << endl;

	

	// check water line points, if zero, return here, else continue
	if (waterlinePoints.size() == 0)
		return waterlinePoints_projected;


	// use nearest neighbour kd tree approach to find nearest image point to water line image point to push back corresponding object point
	// look for nearest neighbour of image points and water line points
	// query = water line point 
	// points to search = image points

	std::vector<cv::Point2f> image_points_to_search;
	for (cv::Point2f p : image_pixels)
		image_points_to_search.push_back(cv::Point2f(p.x, p.y)); //Insert all 2D points to this vector

	cv::flann::KDTreeIndexParams indexParams;
	cv::flann::Index kdtree(cv::Mat(image_points_to_search).reshape(1), indexParams);
	
	// iterate through water line points for queries
	for (cv::Point2d p_wl : waterlinePoints) {
		
		std::vector<float> query;
		query.push_back(p_wl.x); //Insert the 2D point we need to find neighbours to the query
		query.push_back(p_wl.y); //Insert the 2D point we need to find neighbours to the query
		std::vector<int> indices;
		std::vector<float> dists;

		kdtree.knnSearch(query, indices, dists, 1);

		waterlinePoints_projected.push_back(point_cloud[indices[0]]);


		//std::cout << "FLANN, look for water line point: " << p_wl.x << "," << p_wl.y << ", nn: " << image_points_to_search[indices[0]] << ",dists: " << dists[0] << std::endl;
	
	}

	return waterlinePoints_projected;
	
		
}






void Modell_OCV::export_point_cloud_recolored(std::string workingDirectory, std::string wD_name, std::vector<cv::Point3d>& point_cloud, std::vector<cv::Vec3b>& point_cloud_colors, std::vector<cv::Point2d>& image_coords_colors, double shifter_x, double shifter_y, double shifter_z) {

	// output point cloud to file, apply coordinate shifting
	std::ofstream outStream;
	outStream.open(workingDirectory + wD_name + "_pcl.txt");

	int i = 0;
	for (i = 0; i < point_cloud.size(); i++) {

		cv::Point3d& p = point_cloud[i];
		cv::Vec3b& c = point_cloud_colors[i];
		cv::Point2d& imgP = image_coords_colors[i];

		//if (static_cast<unsigned int>(c.val[0]) != 0 && static_cast<unsigned int>(c.val[1]) != 0 && static_cast<unsigned int>(c.val[2]) != 0) {
		if ((c.val[0]) > 0.0 || (c.val[1]) > 0.0 || (c.val[2]) > 0.0) {
		
			std::setprecision(4);
			outStream
				<< std::fixed << p.x + shifter_x << "," // apply shift_x
				<< std::fixed << p.y + shifter_y << "," // apply shift_y
				<< std::fixed << p.z + shifter_z << ","
				<< static_cast<unsigned int>(c.val[2]) << ","
				<< static_cast<unsigned int>(c.val[1]) << ","
				<< static_cast<unsigned int>(c.val[0]) << ","
				<< std::fixed << imgP.x << "," // add image coordinates to list
				<< std::fixed << imgP.y << "," << std::endl;



		}
	}

	outStream.close();

	logfile->append(TAG + "saved " + std::to_string(i) + " points!");


}



