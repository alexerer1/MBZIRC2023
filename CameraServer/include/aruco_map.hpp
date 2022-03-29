#ifndef _ARUCO_MAP_HPP
#define _ARUCO_MAP_HPP

// Marker struct with all marker information
struct ar_marker{
	int id;
	float x;
	float y;
	float z;
	float th;
	float size;
	bool fixed;
};

// Map class
class ar_map{
	private:
		int num_markers;
		int * ids;
		ar_marker * markers;
	public:
		int get_index_from_id(int id);
		int get_num_markers(void);
		ar_marker get_marker_from_index(int index);
		ar_marker get_marker_from_id(int id);
		float get_marker_size_from_id(int id);
		int decode_xml(const char * filename);
		float calc_marker_visibility_radius(float fov, float height, int marker_id);
};

#endif