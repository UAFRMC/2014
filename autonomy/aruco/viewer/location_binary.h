/**
  Exports camera location data from aruco camera viewer, using a simple flat binary file.

Dr. Orion Lawlor, lawlor@alaska.edu, 2014-03.  Public Domain.
*/
#ifndef __LOCATION_BINARY_H
#define __LOCATION_BINARY_H

class location_binary {
public:
	uint32_t valid; // 1 if detected, 0 if did not detect

	// Location of the center of the camera, in meters
	float x; // right from marker
	float y; // out from marker
	float z; // down from marker
	
	// Camera's pointing angle in the X-Y plane, degrees from Y axis
	float angle;
	
	uint32_t count; // incremented at each update (to detect crash/lag/invisible)
	uint32_t vidcap_count; // incremented at each update

	location_binary() {
		valid=0;
		x=y=z=angle=0.0f;
		count=0;
		vidcap_count=0;
	}
};

#endif

