#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <stdlib.h>
#include <stdio.h>
#include <sensor_msgs/CompressedImage.h>

using namespace cv;

void imageCallback(const sensor_msgs::CompressedImage& msg){
	//Leggo e stampo l'immagine originale
	cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	cv::imshow("Originale", cv_ptr->image);
	namedWindow( "Originale", CV_WINDOW_AUTOSIZE);

	cv::Mat dest;
	//Imposto i colori in grayscale
	cv::cvtColor(cv_ptr->image, dest, cv::COLOR_BGR2GRAY);
	
	int lowThreshold = 130;
	int highThreshold = 255;
	/*A volte potevano comparire delle linee extra nel risultato finale,
	 * e con la seguente funzione tutti i grigi scuri diventano nero e
	 * i grigi chiari diventano bianco, così da non falsare più il risultato finale
	 */
	cv::threshold(dest, dest, lowThreshold, highThreshold, CV_THRESH_BINARY);
	
	//Applico un blur Gaussiano
	GaussianBlur(dest, dest, Size(3,3), 0, 0, BORDER_DEFAULT);

	//Svolgo la Canny
	int kernel_size = 3;	
	Canny(dest, dest, lowThreshold, highThreshold, kernel_size);
	
	//Mostro l'immagine finale
	cv::imshow("Lavorata", dest);
	waitKey(10);

}

int main(int argc, char **argv){
	ros::init(argc, argv, "HW4");	
	ros::NodeHandle nh;
	ros::Subscriber sub = nh.subscribe("default/camera_node/image/compressed", 1, imageCallback);	
	ros::spin();
}
