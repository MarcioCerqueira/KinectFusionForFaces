#include "MyCVPointCloud.h"

MyCVPointCloud::MyCVPointCloud(int width, int height) {
	cvPointCloud.create(height, width, CV_32FC3);
	f = 525.f;
}

void MyCVPointCloud::load(unsigned short *data, int max_z) {
	
	for(int y = 0; y < cvPointCloud.rows; y++) {

		cv::Vec3f* Mi = cvPointCloud.ptr<cv::Vec3f>(y);

		for(int x = 0; x < cvPointCloud.cols; x++) {

			int index = y * cvPointCloud.cols + x;
			
			float d = data[index];

			if ( d < max_z && d > 0 ){

				Mi[x][0] = ( float(d * (x - 320)) / f );
				Mi[x][1] = ( float(d * (y - 240)) / f );
				Mi[x][2] = d;

			} else {
				
				Mi[x] = 0;
			}
		}
	}

}