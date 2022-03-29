
#ifndef _BOX_DETECTION_H_
#define _BOX_DETECTION_H_

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

using namespace cv;
using namespace std;

class BoxDetection {

public:
    struct detection_result {
        bool box_found;
        Point box_center;
        double box_angle;
    };
    struct boxPlacement {
        int x = -1, y = -1;
        double angle = 0;
    };
    boxPlacement box;

    detection_result result;

    const bool debug;

    static const int N_BOX_TYPES = 3;
    enum box_type_t {
        RED_BOX=0, GREEN_BOX=1, BLUE_BOX=2
    };


// private:
public:
    const Size FRAME_SIZE;
    const double HOUGH_RHO_RES = 2,
                HOUGH_THETA_RES = 1 * CV_PI / 180; // (2 degrees)

    template<typename T>
    struct expectation {
        T val, min, max;
        expectation(){};
        expectation(T value, double prc_lim){
            val = value;
            min = (1 - prc_lim) * value;
            max = (1 + prc_lim) * value;
        }
    };

    struct hsv_thresholds {
        int low_H, low_S, low_V, high_H, high_S, high_V;
    };
    const int MAX_H = 180, MAX_SV = 255;

    const hsv_thresholds FILTER_VALUES[N_BOX_TYPES] = {
        {142, 57, 12, 10, 255, 255}, // red
        {66, 55, 27, 87, 240, 238}, // green
        {104, 96, 0, 120, 255, 255} // blue
    };

    const double BOX_WIDTH = 0.2;
    const double BOX_LENGTHS[N_BOX_TYPES] = {0.3,0.6,1.2};
    const double PLATE_WIDTH = 0.15;
    const double PLATE_LENGTHS[N_BOX_TYPES] = {0.2,0.15,0.3}; // Square green box plate
    // const double PLATE_LENGTHS[N_BOX_TYPES] = {0.2,0.3,0.3}; // correct values TODO

    struct box_constants {
        double plate_size[2]; // {short, long}
        double face_size[2]; // {short, long}
        double plate_area;
        double face_area;
        double plate_ratio;
        double face_ratio;
    };

    typedef Vec3d point_t;
    typedef point_t line_t;

    struct filter_item {
        Point2i b1,b2; // bounds defined by (x_min, y_min) and (x_max, y_max)
        int size, merges;
    };
    struct filter_expects {
        expectation<int> area;
        Point2i pos;
        double max_pos_diff;
        double eval;
        hsv_thresholds thres;
    };
    struct filter_result {
        int total_merges, outliers;
        double proper_ratio;
        vector<filter_item> proper_items;
        int item_idx;
        hsv_thresholds thres;
    };

    struct contour_result {
        vector<vector<Point>> contours;
        vector<Point> centers;
        int total_n_contours;
        int selected_idx;
        int selected_area;
        int total_area;
        int total_accepted_area;
        double best_center_dif;
    };

    struct line_point {
        int x,y;
        float p;
        bool operator<(const line_point& a) const{ return p <a.p; }
    };
    struct line_seg{
        Point2i p1, p2;
        line_t line;
        double length;
        int inliers = -1;
        int hough_index = -1;
        
        bool operator==(line_seg const & rhs) const {
            return this->p1 == rhs.p1 && this->p2 == rhs.p2;
        }
    };
    line_seg seg_from_ends(const Point2d &p1, const Point2d &p2){
        line_seg s;
        s.p1 = p1;
        s.p2 = p2;
        s.line[0] = p2.y - p1.y;
        s.line[1] = p1.x - p2.x;
        s.line[2] = p2.x*p1.y - p1.x*p2.y;
        s.length = sqrt(s.line[0]*s.line[0] + s.line[1]*s.line[1]);
        s.line /= s.length;
        return s;
    }

    struct line_pair {
        int ls1, ls2;
        double dist;
    };

    enum face_direction_t {FACE_DIR_UNKNOWN, FACE_DIR_LONG_SHORT, FACE_DIR_SHORT_LONG}; // Whether fave begins with a long or a short side point[0]
    struct face_properties {
        Point2i center;
        double theta;
        double area;
        double ratio;
        enum face_direction_t rect_dir;
    };
    struct face_t {
        Point2i p[4];
        line_t line[4];
        vector<int> seg_idc;
        uint8_t complete = 0;
        face_properties props; // Calculated in compute_face()
        face_t(){seg_idc.reserve(4);}
    };
    // static inline const int f_next[4] = {1,2,3,0};
    // static inline const int f_prev[4] = {3,0,1,2};
    static const int f_next[4];
    static const int f_prev[4];

    struct loose_seg {
        int seg_i = -1;
        double dist, ang_dif, exp_dist;
        bool orthogonal;
    };

    struct box_t {
        double theta, scale;
        Point2i center;
        double max_displacement;
        double max_angle_diff;
    };

    enum detection_type_t {NOT_FOUND=0, DETECT_ITEM_CENTER=1, DETECT_CONTOURS=2, DETECT_LOOSE_SEGMENTS=3, DETECT_FACE_CENTER=4};

    struct item_expectations {
        double exp_scale;
        int exp_hough_thres;
        int exp_n_houghlines;
        expectation<int> exp_n_lines; // TODO delete
        vector<line_t> exp_hough_lines;
        box_constants box_consts;
        box_t box_exp;
    };
    struct item_results {
        int used_hough_thres;
        Vec2i used_canny_thres;
        int n_hough_lines;
        vector<line_t> hough_lines;
        vector<line_seg> segments;
        vector<face_t> faces;
        vector<int> used_segs;
        bool box_was_found;
        face_t found_plate; // TODO put in box ?
        face_t found_top;
        box_t box;
        detection_type_t detection_type;
    };
    struct frame_expectations {
        Rect RoI;
        item_expectations item_exp;
        filter_expects filter_exp;
    };
    struct frame_results {
        int n_items;
        int item_size;
        Point item_center;
        // Point2i contour_center;
        double item_eval;
        Rect item_crop;
        item_results item_res;
        hsv_thresholds used_hsv;
    };

    struct box_state {
        frame_expectations frame_exp;
        frame_results frame_res;
        Point2i center_change;
        int box_stability;          // How many consecutive times the box has been found (or not found if < 0)
        int item_stability;          // How many consecutive times the correct item has been found (or not found if < 0)
        Point2i final_center;
        double final_angle;
        box_type_t box_type;
    };

    struct detection_log {
        int detection_time;
        detection_result result;
        detection_type_t detection_type;
    };

    // inline static const int LOG_SIZE = 1000;
    #define LOG_SIZE 1000
    detection_log log[LOG_SIZE];
    int log_i =0;

// private:
public:
    box_state state;

public:
    BoxDetection(Size img_size, box_type_t box_type);
    BoxDetection(Size img_size, box_type_t box_type, bool print_outs);
    void set_box(box_type_t box);
    void reset();
    // bool detect_box(const Mat3b &frame);
    bool getBox(const Mat3b &frame, char color);
    void write_out_log(String filename);

// private:
public:
    void reset_item_expectations(frame_expectations &frame_exp, box_type_t box_type);
    void reset_box_expectations(frame_expectations &frame_exp);
    box_constants compute_box_constants(box_type_t box);

    frame_results process_frame(const Mat3b &src, const frame_expectations &frame_exp);
    item_results process_item(const Mat1b &src_filtered_gray, const int item_size, const item_expectations &item_exp);
    Point2i item_offset(const box_state &s);
    Rect region_of_interest(Rect roi, Point2i new_center, const double scale, const Point2i margin, const Size2i img_size);
    void update_expectations(box_state &state);

    void filter_HSV(const Mat3b &src, hsv_thresholds hsv, Mat1b &color_mask);
    void filter_RGB_min_value(const Mat3b &src, double val_min, Mat1b &color_mask);
    int find_mask_items(const Mat1b &mask, const int line_thres, vector<filter_item> &items);
    struct filter_constants {int LINE_THRES, MAX_ITEMS;};
    bool evaluate_mask(const Mat1b &hsv_mask, filter_constants C, const filter_expects &exp, filter_result &res);
    filter_result find_filtered_items(const Mat3b &src, const int down_samp, filter_expects filter_exp);

    contour_result find_contours(Mat1b mask, int area_min, const Point2i exp_center);
    contour_result auto_filter_plate(const Mat3b &src, int &value, const int exp_plate_area, Point2i exp_center);
    contour_result auto_filter_box(const Mat3b &src, hsv_thresholds &hsv_thres, int &value, const int MIN_CONT_AREA);
    double filter_and_search_plate(const Mat3b src, Point2i center, int found_box_area, int &exp_filter_thres, Rect &crop, contour_result &fc);

    void find_CannyEdges(const Mat1b &src_gray, const int thres_low, const int thres_high, Mat1b &edge_mask);
    int find_hough_lines(const Mat1b &edges, const int exp_threshold, const int n_lines, const vector<line_t> &exp_lines, vector<line_t> &detected_lines);
    vector<line_seg> find_hough_segments(const vector<line_t> &lines, const Mat1b mask, const double min_length, const double max_thickness);
    vector<line_point> line_subset(const line_t line, const Mat1b src, const double thickness);
    int line_endpoints(vector<line_point> points, Point2i &p_start, Point2i &p_end);

    void find_unique_segments(vector<line_seg> &segs, const double unique_ang_thres, const double unique_dist_thres);
    void connect_segments(vector<line_seg> &segs, vector<bool> &corrected);
    void clean_connections(vector<line_seg> &segs);
    void join_segments(vector<line_seg> &segs);
    
    vector<line_pair> find_segment_pairs(const vector<line_seg> &segs, const double MIN_DIST);
    vector<face_t> find_faces(const vector<line_seg> &segs, const vector<line_pair> &pairs);
    void uniquify_faces(vector<face_t> &faces);
    void compute_face(face_t &face);
    vector<int> search_face(const vector<face_t> &pool, const expectation<double> &ratio, const expectation<double> &area, Point2i exp_center, double max_displacement);
    void search_faces(const item_expectations &item_exp, item_results &res, double plate_exp_area, double top_exp_area);
    vector<int> search_segments(const face_t &face, bool long_side_first, const expectation<double> &short_side, const expectation<double> &long_side, double margin, vector<line_seg> segments);
    vector<int> search_box_segments(box_t &box_res, const box_t &box_exp, double plate_size[2], double face_size[2], double margin, const vector<line_seg> &segments);
    vector<loose_seg> search_segments_along_dir(Point2i center, Vec2d dir, double dist1, double dist2, double margin, const vector<line_seg> &segments);
    loose_seg search_segment(Point2i center, Vec2d dir, double exp_dist, double margin, bool choose_closest, const vector<line_seg> &segments);


    // HELPER INLINE FUNCTIONS //
    inline Point2d line_intersection(line_t line1, line_t line2){
        point_t v_int = line1.cross(line2);
        return {v_int[0]/v_int[2],v_int[1]/v_int[2]};
    }
    inline double line_dot(const line_t line1, const line_t line2){
        return line1[0]*line2[0] + line1[1]*line2[1];
    }
    inline double line_point_dot(const line_t line, const Point p){
        return line[0]*p.x + line[1]*p.y + line[2];
    }
    
    inline int face_side_count(uint8_t complete){
        int sum=0;
        for (int i = 0; i < 4; i++) sum += (complete>>i & 1);
        return sum;
    }
    inline double angle_dif(double exp_theta, double found_theta, double repeat){
        const double diff = found_theta - exp_theta;
        int div = round(diff/repeat);
        return diff - div * repeat;
    }
    inline double angle_face_diff(double exp_theta, const face_properties &props){
        switch(props.rect_dir){
            case FACE_DIR_LONG_SHORT:
                return angle_dif(exp_theta, props.theta+M_PI_2, M_PI);
            case FACE_DIR_SHORT_LONG:
                return angle_dif(exp_theta, props.theta, M_PI);
            case FACE_DIR_UNKNOWN:
                return angle_dif(exp_theta, props.theta, M_PI_2);
        }
    }
};

#endif