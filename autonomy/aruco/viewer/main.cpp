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
#include "errno.h"

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


/**
  Silly hack to detect camera disconnections.
  When you unplug a video camera, it's actually detected right in:
    opencv/modules/highgui/src/cap_libv4l.cpp
  when the VIDIOC_DQBUF ioctl fails.  However, this function 
  somehow fails to correctly report the problem via the error reporting
  return code.  It does log it using perror, so I'm hooking the global perror
  to figure out if this is what went wrong, and exit appropriately.
*/
extern "C" void perror(const char *str) {
	int e=errno;
	std::cout<<"perror errno="<<e<<": "<<str<<"\n";
	if (e==ENODEV) {
		std::cout<<"ERROR!  Camera no longer connected!\n";
		std::cerr<<"ERROR!  Camera no longer connected!\n";
		exit(1);
	}
}

#include "location_binary.h"

/* Extract location data from this valid, detected marker. 
   Does not modify the location for an invalid marker.
*/
void extract_location(location_binary &bin,float true_size,float X_shift,const Marker &marker,int rot=0)
{
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
	
	if (rot==90) {
		for (int i=0; i<3; i++) {
			std::swap(full.at<float>(i,0),full.at<float>(i,2)); // swap X and Z
			full.at<float>(i,0)*=-1; // invert (new) X
		}
	}


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
	double scale=true_size;
	bin.x=back.at<float>(0,3)*scale+X_shift;
	bin.y=back.at<float>(1,3)*scale;
	bin.z=back.at<float>(2,3)*scale;
	bin.angle=180.0/M_PI*atan2(back.at<float>(1,0),back.at<float>(0,0));

	// Print grep-friendly output
	printf("Marker %d: Camera %.3f %.3f %.3f meters, heading %.1f degrees\n",
	       marker.id, bin.x,bin.y,bin.z,bin.angle
	      );
}

/* Refine this marker-derived location, using the Y-offset black spike. 
*/
void refine_location(location_binary &loc,float Y_offset /* <- units: Y distance / true size of marker */, 
	cv::Mat &cameraFrame,const Marker &marker)
{
	std::cout<<"\n\nMarker "<<marker<<"\n\n\n";
	/* Marker corners:
	   [1] [2]
	   [0] [3]
	*/
	cv::Point2f left=marker[0], top=marker[2], right=marker[3];
	cv::Point2f cen=right; // spike is aligned with big marker's right side
	float hfrac=0.7;
	cv::Point2f hdir=hfrac*(right-left); // horizontal range to check for spike
	cv::Point2f vdir=0.12*(top-right); // vertical range to check
	
	cv::Rect cameraRect(cv::Point(),cameraFrame.size());
// Search over the search space for the spike
	int nrow=(int)abs(vdir.y);
	int ncol=(int)abs(hdir.x);
	enum {nrow_check=3};
	int best_col[nrow_check]={0,0,0}; // best column number, indexed by row
	for (int row=0;row<nrow_check;row++) { // scanlines (only check bottom few)
		int darkest_x=-1;
		int darkest_g=200;
		for (int col=0;col<ncol;col++) {
			cv::Point2f p=cen + hdir*(col*(1.0/(ncol-1.0))-0.5) + vdir*(-nrow + 1 + row)*(1.0/(nrow-1.0));
			cv::Point pint((int)p.x,(int)p.y);
			if (cameraRect.contains(pint)) {
				cv::Vec3b bgr=cameraFrame.at<cv::Vec3b>(pint.y,pint.x);
				if (darkest_g>bgr[1]) {
					darkest_g=bgr[1];
					darkest_x=col;
				}
				static cv::Vec3b mark(0,255,255); 
				cameraFrame.at<cv::Vec3b>(pint.y,pint.x)=cv::Vec3b(bgr[0],255,bgr[2]); // green searchspace
				//printf("(%d,%d) ",pint.x,pint.y);
			}
		}
		if (darkest_x>0) {
			printf("row %3d: min %3d at %d (%.2f)\n",row,darkest_g,darkest_x,darkest_x*1.0/(ncol-1)-0.5);
			best_col[row]=darkest_x;
		}
	}
	if (fabs(best_col[1]-best_col[0])>1) return; // we're not reading a consistent value--skip this.
	float avg_col=(best_col[0]+best_col[1])*0.5; // column shift (relative to hdir)
	float ang_col=(avg_col*(1.0/(ncol-1))-0.5)*hfrac/Y_offset; // tangent of angle (approx == angle in radians)
	float radius=sqrt(loc.x*loc.x+loc.y*loc.y); // distance from origin of markers (meters)
	float X=-radius*ang_col; // true X coordinate, in meters
	printf("X: %.2f m.  R: %.2f m. Angle: %.2f radians.  Col: %.1f pixels\n",X,radius, ang_col,avg_col);
	
	// Update location to match new info
	loc.y=radius; // HACK: should take into account X value too
	loc.x=X;
	// FIXME: update loc.angle too (rotate from old x to new X?)
	
	
}



int main(int argc,char **argv)
{
	try {
	bool showGUI=false, useRefine=false;
	string TheInputVideo;
	string TheIntrinsicFile;
	int camNo=1;
	float TheMarkerSize=-1;
	int ThePyrDownLevel=0;
	MarkerDetector MDetector;
	VideoCapture vidcap;
	vector<Marker> TheMarkers;
	Mat TheInputImage,TheInputImageCopy;
	CameraParameters cam_param;
	pair<double,double> AvrgTime(0,0) ;//determines the average time required for detection
	int skipCount=1; // only process frames ==0 mod this
	int skipPhase=0;

		int wid=640, ht=480;
		for (int argi=1; argi<argc; argi++) {
			if (0==strcmp(argv[argi],"-gui")) showGUI=true;
			else if (0==strcmp(argv[argi],"-cam")) camNo=atoi(argv[++argi]);
			else if (0==strcmp(argv[argi],"-refine")) useRefine=true;
			else if (0==strcmp(argv[argi],"-sz")) sscanf(argv[++argi],"%dx%d",&wid,&ht);
			else if (0==strcmp(argv[argi],"-skip")) sscanf(argv[++argi],"%d",&skipCount);
			else printf("Unrecognized argument %s\n",argv[argi]);
		}

		//read from camera
		vidcap.open(camNo);
		
		vidcap.set(CV_CAP_PROP_FRAME_WIDTH, wid);
		vidcap.set(CV_CAP_PROP_FRAME_HEIGHT, ht);

		//check video is open
		if (!vidcap.isOpened()) {
			cerr<<"Could not open video"<<endl;
			return -1;
		}

		TheIntrinsicFile="camera.yml";

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
		MDetector.setCornerRefinementMethod(MarkerDetector::SUBPIX); // LINES);
		MDetector.setMinMaxSize(0.02,1.0); // for distant/small markers (smaller values == smaller markers, but slower too)
		// MDetector.setCornerMethod(SUBPIX); // bounces around more than "LINES"

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
			if (!vidcap.retrieve( TheInputImage) || !vidcap.isOpened()) {
				std::cout<<"ERROR!  Camera "<<camNo<<" no longer connected!\n";
				std::cerr<<"ERROR!  Camera "<<camNo<<" no longer connected!\n";
				exit(1);
			}
			if (quit_signal) exit(0);

			// Skip frames (do no processing)
			skipPhase=(skipPhase+1)%skipCount;
			if (skipPhase!=0) continue;

			double tick = (double)getTickCount();//for checking the speed
			//Detection of markers in the image passed
			MDetector.detect(TheInputImage,TheMarkers,cam_param,1.0);
			
			//check the speed by calculating the mean speed of all iterations
			AvrgTime.first+=((double)getTickCount()-tick)/getTickFrequency();
			AvrgTime.second++;
			cout<<"Time detection="<<1000*AvrgTime.first/AvrgTime.second<<" milliseconds"<<endl;

			// Locations extracted from different markers:
			location_binary binBig, binL, binR;
			for (unsigned int i=0; i<TheMarkers.size(); i++) {
				Marker &marker=TheMarkers[i];

				if (marker.id==236) { // big 'A'
					extract_location(binBig,
						// 0.84, // <- spray painted fabric
						0.77, // <- glued-on foam
						0.0,marker);
					// Big marker has black spike below it	
					if (useRefine)
						refine_location(binBig,0.45/0.84,TheInputImage,marker);
				} else if (marker.id==771) { // left side: 'U'
					extract_location(binL,0.245,
						-0.60,
						marker); 
				} else if (marker.id==816) { // right side: 'F', but rotated 90 degrees!
					extract_location(binR,0.245,
						+0.60, /* <- center-to-center distance */
						marker, 90);
				}

/* OLD May marker setup */
				else if (marker.id==0) { // big
					extract_location(binBig,0.765,0.0,marker);
				} else if (marker.id==16) { // left side
					extract_location(binL,0.25,
						-0.5725,
						marker); 
				} else if (marker.id==787) { // right side
					extract_location(binR,0.25,
						+0.5725, /* <- center-to-center distance */
						marker);
				} else {
					// Weird--not the marker we expected
					std::cout<<"Unexpected marker ID "<<marker.id<<"!\n";
					continue;
				}
			}

			location_binary bin; // invalid by default
			if (binBig.valid) bin=binBig; // use big one if possible
			else if (binL.valid) bin=binL;
			else if (binR.valid) bin=binR;
			// else invalid
			
			
			static uint32_t bin_count=0;
			bin.count=bin_count++;
			bin.vidcap_count=vidcap_count;
			FILE *fbin=fopen("marker.bin","rb+"); // "r" mode avoids file truncation
			if (fbin==NULL) { // file doesn't exist (yet)
				fbin=fopen("marker.bin","w"); // create the file
				if (fbin==NULL) { // 
					printf("Error creating marker.bin output file (disk full?  permissions?)");
					exit(1);
				}
			} 
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

