/*****************************************************************************************
Dr. Lawlor's modified ArUco marker detector:
	- Finds marker 16 in the webcam image
	- Reconstructs the camera's location relative to the marker
	- Writes the camera location and orientation to "marker.bin"
	- Periodically saves an image to vidcap.jpg and vidcaps/<date>.jpg


ArUco example Copyright 2011 Rafael Muñoz Salinas. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are
permitted provided that the following conditions are met:

   1. Redistributions of source code must retain the above copyright notice, this list of
      conditions and the following disclaimer.

   2. Redistributions in binary form must reproduce the above copyright notice, this list
      of conditions and the following disclaimer in the documentation and/or other materials
      provided with the distribution.

THIS SOFTWARE IS PROVIDED BY Rafael Muñoz Salinas ''AS IS'' AND ANY EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL Rafael Muñoz Salinas OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those of the
authors and should not be interpreted as representing official policies, either expressed
or implied, of Rafael Muñoz Salinas.
********************************************************************************************/
#include <iostream>
#include <fstream>
#include <sstream>
#include "aruco.h"
#include "cvdrawingutils.h"

using namespace cv;
using namespace aruco;

/* Keep the webcam from locking up when you interrupt a frame capture.
http://lawlorcode.wordpress.com/2014/04/08/opencv-fix-for-v4l-vidioc_s_crop-error/ */
volatile int quit_signal=0;
#ifdef __unix__
#include <signal.h>
extern "C" void quit_signal_handler(int signum)
{
	if (quit_signal!=0) exit(0); // just exit already
	quit_signal=1;
	printf("Will quit at next camera frame (repeat to kill now)\n");
}
#endif


bool showGUI=false;
string TheInputVideo;
string TheIntrinsicFile;
float TheMarkerSize=-1;
int ThePyrDownLevel=0;
MarkerDetector MDetector;
VideoCapture vidcap;
vector<Marker> TheMarkers;
Mat TheInputImage,TheInputImageCopy;
CameraParameters cam_param;

pair<double,double> AvrgTime(0,0) ;//determines the average time required for detection


int main(int argc,char **argv)
{
	try {
		int camNo=1;
		for (int argi=1; argi<argc; argi++) {
			if (0==strcmp(argv[argi],"-gui")) showGUI=true;
			else if (0==strcmp(argv[argi],"-cam")) camNo=atoi(argv[++argi]);
			else printf("Unrecognized argument %s\n",argv[argi]);
		}

		//read from camera
		vidcap.open(camNo);

		//check video is open
		if (!vidcap.isOpened()) {
			cerr<<"Could not open video"<<endl;
			return -1;

		}

		TheIntrinsicFile="camera.yml";
		TheMarkerSize=0.5; // big marker is 0.5 meters across

		//read first image to get the dimensions
		vidcap>>TheInputImage;

		//read camera parameters if passed
		if (TheIntrinsicFile!="") {
			cam_param.readFromXMLFile(TheIntrinsicFile);
			cam_param.resize(TheInputImage.size());
		}
		//Configure other parameters
		if (ThePyrDownLevel>0)
			MDetector.pyrDown(ThePyrDownLevel);
		MDetector.setCornerRefinementMethod(MarkerDetector::LINES);
		MDetector.setMinMaxSize(0.02,1.0); // for distant/small markers


		if (showGUI) {
			//Create gui
			cv::namedWindow("in",1);
		}

#ifdef __unix__
		signal(SIGINT,quit_signal_handler); // listen for ctrl-C
#endif
		unsigned int framecount=0;
		uint32_t vidcap_count=0;

		//capture until press ESC or until the end of the video
		while (vidcap.grab()) {
			vidcap.retrieve( TheInputImage);
			if (quit_signal) exit(0);

			double tick = (double)getTickCount();//for checking the speed
			//Detection of markers in the image passed
			MDetector.detect(TheInputImage,TheMarkers,cam_param,TheMarkerSize);
			//check the speed by calculating the mean speed of all iterations
			AvrgTime.first+=((double)getTickCount()-tick)/getTickFrequency();
			AvrgTime.second++;
			cout<<"Time detection="<<1000*AvrgTime.first/AvrgTime.second<<" milliseconds"<<endl;


			// Write to binary file
#include "location_binary.h"
			location_binary bin;
			bin.valid=0;
			bin.x=bin.y=bin.z=0.0;
			bin.angle=0.0;

			for (unsigned int i=0; i<TheMarkers.size(); i++) {
				Marker &marker=TheMarkers[i];

				if (marker.id!=16) {
					// Weird--not the marker we expected
					std::cout<<"Unexpected marker ID "<<marker.id<<"!\n";
					continue;
				}

				// Extract 3x3 rotation matrix
				Mat Rot(3,3,CV_32FC1);
				Rodrigues(marker.Rvec, Rot); // euler angles to rotation matrix

				// Full 4x4 output matrix:
				Mat full(4,4,CV_32FC1);

				// Copy rotation 3x3
				for (int i=0; i<3; i++)
					for (int j=0; j<3; j++)
						full.at<float>(i,j)=Rot.at<float>(i,j);

				// Copy translation vector
				full.at<float>(0,3)=marker.Tvec.at<float>(0,0);
				full.at<float>(1,3)=marker.Tvec.at<float>(1,0);
				full.at<float>(2,3)=marker.Tvec.at<float>(2,0);

				// Final row is identity (nothing happening on W axis)
				full.at<float>(3,0)=0.0;
				full.at<float>(3,1)=0.0;
				full.at<float>(3,2)=0.0;
				full.at<float>(3,3)=1.0;

				// Invert, to convert marker-from-camera into camera-from-marker
				Mat back=full.inv();

			if (false) {
				// Splat to screen, for debugging
				for (int i=0; i<4; i++) {
					for (int j=0; j<4; j++)
						printf("%.2f	",back.at<float>(i,j));
					printf("\n");
				}
			}
				
				bin.valid=1;
				bin.x=back.at<float>(0,3);
				bin.y=back.at<float>(1,3);
				bin.z=back.at<float>(2,3);
				bin.angle=180.0/M_PI*atan2(back.at<float>(1,0),back.at<float>(0,0));

				// Print grep-friendly output
				printf("Camera %.3f %.3f %.3f meters, heading %.1f degrees\n",
				       bin.x,bin.y,bin.z,bin.angle
				      );

			}
			static uint32_t bin_count=0;
			bin.count=bin_count++;
			bin.vidcap_count=vidcap_count;
			FILE *fbin=fopen("marker.bin","rb+"); // "r" mode avoids file truncation
			fwrite(&bin,sizeof(bin),1,fbin); // atomic(?) file write
			fclose(fbin);

			bool vidcap=false;
			if ((framecount++%32) == 0) vidcap=true;
			if (showGUI || vidcap) {
				//print marker info and draw the markers in image
				TheInputImage.copyTo(TheInputImageCopy);
				for (unsigned int i=0; i<TheMarkers.size(); i++) {
					Marker &marker=TheMarkers[i];
					// cout<<TheMarkers[i]<<endl;
					marker.draw(TheInputImageCopy,Scalar(0,0,255),1);

					//draw a 3d cube on each marker if there is 3d info
					if (  cam_param.isValid()) {
						CvDrawingUtils::draw3dCube(TheInputImageCopy,marker,cam_param);
						CvDrawingUtils::draw3dAxis(TheInputImageCopy,marker,cam_param);
					}

				}

				if (true) {
					//print other rectangles that contains invalid markers
					for (unsigned int i=0; i<MDetector.getCandidates().size(); i++) {
						aruco::Marker m( MDetector.getCandidates()[i],999);
						m.draw(TheInputImageCopy,cv::Scalar(255,0,0));
					}
				}

				if (vidcap) { // write to disk
					std::vector<int> params;
					params.push_back(CV_IMWRITE_JPEG_QUALITY);
					params.push_back(30); // <- low quality, save disk space and network bandwidth
					cv::imwrite("vidcap_next.jpg",TheInputImageCopy,params); // dump JPEG
					int ignore;
					ignore=system("mv -f vidcap_next.jpg vidcap.jpg"); // atomic(?) file replace
					ignore=system("cp vidcap.jpg vidcaps/`date '+%F__%H_%M_%S__%N'`.jpg"); // telemetry log
					vidcap_count++;
				}
			}
			if (showGUI) {
				//show input with augmented information and  the thresholded image
				cv::imshow("in",TheInputImageCopy);
				// cv::imshow("thres",MDetector.getThresholdedImage());

				char key=cv::waitKey(1);//wait for key to be pressed
				if (key=='q' || key=='x' || key==0x13) exit(0);
			} /* end showGUI */
		} /* end frame loop */

	} catch (std::exception &ex) {
		cout<<"Vision/ArUco exception: "<<ex.what()<<endl;
	}

}

