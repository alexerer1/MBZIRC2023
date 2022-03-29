
#include "BoxDetection.h"

#include <algorithm>
#include <iostream>
#include <fstream> // for exporting log
#include <chrono>  
using namespace std::chrono; 

const int BoxDetection::f_next[4] = {1,2,3,0};
const int BoxDetection::f_prev[4] = {3,0,1,2};

////////////////// PUBLIC INTERFACE //////////////////

BoxDetection::BoxDetection(Size img_size, box_type_t box_type) : FRAME_SIZE(img_size), debug(false) {
    set_box(box_type);
}
BoxDetection::BoxDetection(Size img_size, box_type_t box_type, bool print_outs) : FRAME_SIZE(img_size), debug(print_outs) {
    set_box(box_type);
}

void BoxDetection::set_box(box_type_t box){
    state.box_type = box;
    state.frame_exp.filter_exp.thres = FILTER_VALUES[box];
    state.frame_exp.item_exp.box_consts = compute_box_constants(box);

    state.center_change = {0,0};
    state.box_stability = 0;
    state.item_stability = 0;

    result.box_angle = 0;
    result.box_center = {FRAME_SIZE.width/2,FRAME_SIZE.height};
    result.box_found = false;

    reset_item_expectations(state.frame_exp, state.box_type);
    reset_box_expectations(state.frame_exp);
}

void BoxDetection::reset(){
    set_box(state.box_type);
}

bool BoxDetection::getBox(const Mat3b &frame, char color){
// bool BoxDetection::detect_box(const Mat3b &frame){
    auto start = high_resolution_clock::now(); 
    Mat3b src = frame(state.frame_exp.RoI);
    state.frame_res = process_frame(src, state.frame_exp);
    update_expectations(state);
    // result.box_found = state.frame_res.item_res.detection_type != NOT_FOUND;
    result.box_found = state.frame_res.item_res.detection_type == DETECT_FACE_CENTER
                    || state.frame_res.item_res.detection_type == DETECT_LOOSE_SEGMENTS;
    if(result.box_found){
        result.box_center = state.final_center;
        result.box_angle = - state.final_angle - M_PI_2;
        result.box_angle = angle_dif(0, result.box_angle, 2*M_PI); 
        if(result.box_angle > M_PI_2) result.box_angle -= M_PI;
        else if(result.box_angle < -M_PI_2) result.box_angle += M_PI;
    }else{
        result.box_center = {frame.cols/2, frame.rows/2};
        result.box_angle = 0;
    }

    box.x = result.box_center.x;
    box.y = result.box_center.y;
    box.angle = result.box_angle;

    auto stop = high_resolution_clock::now(); 
    auto duration = duration_cast<microseconds>(stop - start); 
    if(log_i < LOG_SIZE){
        log[log_i].detection_time = duration.count();
        log[log_i].result = result;
        log[log_i].detection_type = state.frame_res.item_res.detection_type;
        ++log_i;
    }

    return result.box_found;
}

void BoxDetection::write_out_log(String filename){
    ofstream file;
    file.open(filename);
    for (int i = 0; i < log_i; i++)
    {
        file << i 
        << " " << log[i].detection_time
        << " " << log[i].detection_type
        << " " << log[i].result.box_found
        << " " << log[i].result.box_center.x
        << " " << log[i].result.box_center.y
        << " " << log[i].result.box_angle
        << endl;
    }
    file.close();
}

////////////////// SETUP //////////////////

void BoxDetection::reset_item_expectations(frame_expectations &frame_exp, box_type_t box_type){
    // TODO use scale instead
    expectation<double> exp_distance;
    exp_distance.val = 5;
    exp_distance.min = 1;
    exp_distance.max = 10;

    const double f = 800; // TODO
    expectation<int> img_area;
    frame_exp.filter_exp.area.val =  pow(f/exp_distance.val,2) * frame_exp.item_exp.box_consts.face_area;
    frame_exp.filter_exp.area.min =  pow(f/exp_distance.max,2) * frame_exp.item_exp.box_consts.face_area;
    frame_exp.filter_exp.area.max =  pow(f/exp_distance.min,2) * frame_exp.item_exp.box_consts.face_area;

    // frame_exp.filter_exp.area = exp_distance_to_area(exp_distance, frame_exp.item_exp.box_consts.face_area - frame_exp.item_exp.box_consts.plate_area);
    // frame_exp.filter_exp.area = exp_distance_to_area(exp_distance, frame_exp.item_exp.box_consts.face_area);
    // const double exp_item_area = frame_exp.item_exp.exp_scale * (frame_exp.item_exp.box_consts.face_area - frame_exp.item_exp.box_consts.plate_area);

    // frame_exp.filter_exp.area.val = 800;
    // frame_exp.filter_exp.area.min = 200;
    // frame_exp.filter_exp.area.max = (img_size.width*img_size.height)/2;

    frame_exp.filter_exp.pos = {-1,-1};
    frame_exp.filter_exp.max_pos_diff = -1; // deactivate postition requirement
    frame_exp.filter_exp.thres = FILTER_VALUES[box_type];

    frame_exp.RoI = {0,0,FRAME_SIZE.width,FRAME_SIZE.height};
}

void BoxDetection::reset_box_expectations(frame_expectations &frame_exp){
    frame_exp.item_exp.exp_scale = 400; //  = 800/2

    // Box macthing disabled until first box is found
    frame_exp.item_exp.box_exp.max_displacement = -1;
    frame_exp.item_exp.box_exp.max_angle_diff = -1;

    frame_exp.item_exp.box_exp.theta = 0;
    frame_exp.item_exp.exp_n_houghlines = 20;
}

BoxDetection::box_constants BoxDetection::compute_box_constants(box_type_t box){
    box_constants consts;
    consts.face_size[0] = BOX_WIDTH;
    consts.face_size[1] = BOX_LENGTHS[box];
    consts.plate_size[0] = PLATE_WIDTH;
    consts.plate_size[1] = PLATE_LENGTHS[box];
    
    consts.face_area = consts.face_size[0] * consts.face_size[1];
    consts.plate_area = consts.plate_size[0] * consts.plate_size[1];

    consts.face_ratio = consts.face_size[1] / consts.face_size[0];
    consts.plate_ratio = consts.plate_size[1] / consts.plate_size[0];

    return consts;
}

////////////////// HIGH LEVEL METHODS //////////////////

BoxDetection::frame_results BoxDetection::process_frame(const Mat3b &src, const frame_expectations &frame_exp){
    frame_results frame_res;

    normalize(src, src, 0, 255, NORM_MINMAX);

    const int down_samp = 2;
    const int down_fact = pow(2,down_samp);
    const int down_fact_2 = down_fact*down_fact;

    //// NEW CONTOUR STUFF ////

    // Mat3b src_downsamp;
    // resize(src, src_downsamp, Size(), 0.25, 0.25, INTER_NEAREST);

    // Rect plate_crop({0,0},src_downsamp.size());
    // Point2i plate_center = {-1,-1};

    // const int MIN_CONT_AREA = 100;

    // hsv_thresholds hsv_thres = frame_exp.filter_exp.thres;
    // contour_result cr_box = auto_filter_box(src_downsamp, hsv_thres, hsv_thres.low_S, MIN_CONT_AREA);
    // cout <<"thres: "<< hsv_thres.low_S << endl;

    // if(cr_box.selected_idx != -1){
    //     contour_result cr_plt;
    //     int rgb_min_val = 0;
    //     double scale_est = filter_and_search_plate(src_downsamp, cr_box.centers[cr_box.selected_idx], cr_box.selected_area, rgb_min_val, plate_crop, cr_plt);
    //     if(cr_plt.selected_idx != -1){            
    //         scale_est = (scale_est +  sqrt((double)cr_plt.selected_area / state.frame_exp.item_exp.box_consts.plate_area))/2;

    //         plate_center = cr_plt.centers[cr_plt.selected_idx] + plate_crop.tl();
    //         const double center_dif = norm(plate_center - cr_box.centers[cr_box.selected_idx]);
    //         cout << "Scale est: " << scale_est << endl;
    //         if(center_dif < scale_est*0.15){
    //             cout << "Found filter box!" << endl;
    //             Rect countour_crop = boundingRect(cr_box.contours[cr_box.selected_idx]);
    //             frame_res.contour_center = (plate_center+countour_crop.tl()) * down_fact + state.frame_exp.RoI.tl();
    //         }else{
    //             plate_center = {-1,-1};
    //             frame_res.contour_center = {-1,-1};
    //         }

    //     }
    // }
    // else{
    //     cout << "no contours!" << endl;
    //     frame_res.n_items = 0;
    //     frame_res.used_hsv = hsv_thres;
    //     frame_res.item_res.box_was_found = false;
    //     frame_res.item_res.detection_type = NOT_FOUND;
    //     return frame_res;
    // }

    // countour_crop.x *= down_fact;
    // countour_crop.y *= down_fact;
    // countour_crop.width *= down_fact;
    // countour_crop.height *= down_fact;
    // countour_crop &= Rect(0,0,src.cols,src.rows);
    // frame_res.item_crop = countour_crop;
    // frame_res.item_size = cr_box.selected_area * down_fact_2;
    // frame_res.item_center = plate_center * down_fact;
    // frame_res.used_hsv = hsv_thres;

    //////////////////

    const filter_expects &f_exp = frame_exp.filter_exp;
    filter_expects filter_exp_dwn;
    filter_exp_dwn.area.val = f_exp.area.val / down_fact_2;
    filter_exp_dwn.area.min = f_exp.area.min / down_fact_2;
    filter_exp_dwn.area.max = f_exp.area.max / down_fact_2;
    if(filter_exp_dwn.max_pos_diff != -1){
        filter_exp_dwn.pos = f_exp.pos / down_fact;
        filter_exp_dwn.max_pos_diff = f_exp.max_pos_diff / down_fact;
    }
    filter_exp_dwn.thres = f_exp.thres;
    filter_result filt_res = find_filtered_items(src, down_samp, filter_exp_dwn);
    frame_res.used_hsv = filt_res.thres;

    frame_res.n_items = filt_res.proper_items.size(); 

    if(filt_res.item_idx == -1){
        if(debug) cout << "no items!" << endl;
        frame_res.item_res.detection_type = NOT_FOUND;
        return frame_res;
    }

    filter_item &item = filt_res.proper_items[filt_res.item_idx];
    Point2i center = {(item.b1.x + item.b2.x)/2, (item.b1.y + item.b2.y)/2};

    // Adjust size due to downsampling
    item.b1 *= down_fact;
    item.b2 *= down_fact;
    item.size *= down_fact_2;
    center *= down_fact;

    frame_res.item_size = item.size;
    frame_res.item_center = center;
    frame_res.item_eval = filt_res.proper_ratio;

    const int add_edge = 15;
    item.b1.x = std::max(0, item.b1.x - add_edge);
    item.b1.y = std::max(0, item.b1.y - add_edge);
    item.b2.x = std::min(src.cols, item.b2.x + add_edge);
    item.b2.y = std::min(src.rows ,item.b2.y + add_edge);
    Rect crop(item.b1, item.b2);

    frame_res.item_crop = crop;

    const Mat3b src_item_crop = src(frame_res.item_crop);
    Mat1b color_mask, src_crop_gray, src_filtered_gray;
    filter_HSV(src_item_crop, frame_res.used_hsv, color_mask);
    cvtColor( src_item_crop, src_crop_gray, COLOR_BGR2GRAY );
    src_crop_gray.copyTo(src_filtered_gray, color_mask);

    frame_res.item_res = process_item(src_filtered_gray, frame_res.item_size, frame_exp.item_exp);

    return frame_res;
}

BoxDetection::item_results BoxDetection::process_item(const Mat1b &src_filtered_gray, const int item_size, const item_expectations &item_exp){
    item_results res;

    // { // TODO not used
    //     expectation<double> exp_num_boxes;
    //     exp_num_boxes.val = (double)item.size / exp_area.val;
    //     exp_num_boxes.min = (double)item.size / exp_area.max;
    //     exp_num_boxes.max = (double)item.size / exp_area.min;
    //     exp_num_boxes.val = std::max(1,(int)(exp_num_boxes.val));
    //     printf("exp_num_boxes: %f [%f ; %f]\n",exp_num_boxes.val,exp_num_boxes.min,exp_num_boxes.max);

    //     expectation<int> exp_box_area;
    //     exp_box_area.val = item.size / exp_num_boxes.val + item_exp.box_consts.plate_area;
    //     exp_box_area.min = item.size / exp_num_boxes.min + item_exp.box_consts.plate_area;
    //     exp_box_area.max = item.size / exp_num_boxes.max + item_exp.box_consts.plate_area;
    //     printf("exp_box_area: %d [%d ; %d]\n",exp_box_area.val,exp_box_area.min,exp_box_area.max);

    //     const double exp_n_boxes1 = item.size/exp_area.val;
    //     const int exp_n_boxes2 = item.merges+1;
    //     const double exp_n_boxes = (exp_n_boxes1+exp_n_boxes2)/2;
    //     const int box_size = item.size / exp_n_boxes;
    //     const double exp_box_size = item.size/exp_n_boxes;
    //     cout << "exp boxes 1: " << exp_n_boxes1 << endl;
    //     cout << "exp boxes 2: " << exp_n_boxes2 << endl;
    //     cout << "exp boxes: " << exp_n_boxes << endl;
    //     cout << "exp box size: " << exp_box_size << endl;
    // }

    // Canny
    Mat1b canny_edge_mask;

    double x = item_exp.box_consts.plate_area / item_exp.box_consts.face_area;
    double plate_exp_area = item_size / (1.0/x - 1);
    double top_exp_area = item_size / (1 - x);
    if(debug) printf("Assuming 1 box, exp_top: %.0f, exp_plate: %.0f\n",top_exp_area,plate_exp_area);

    double exp_scale = sqrt(top_exp_area / item_exp.box_consts.face_area);
    double est_shortest_line = exp_scale * item_exp.box_consts.plate_size[0];

    const int can_thres_high = std::max(10, static_cast<int>(0.8*est_shortest_line));
    const int can_thres_low = can_thres_high / 2;

    if(debug) printf("Canny thres: %d / %d\n",can_thres_low,can_thres_high);

    find_CannyEdges(src_filtered_gray, can_thres_low, can_thres_high, canny_edge_mask);

    // Hough
    res.used_hough_thres = find_hough_lines(canny_edge_mask, item_exp.exp_hough_thres, item_exp.exp_n_houghlines, item_exp.exp_hough_lines, res.hough_lines);
    res.n_hough_lines = res.hough_lines.size();

    res.segments = find_hough_segments(res.hough_lines, canny_edge_mask, can_thres_low, 2);
    
    const double ang_thres = 8;
    const double dist_thres = 4;
    find_unique_segments(res.segments, ang_thres, dist_thres);

    // Corners
    vector<bool> corrected;
    corrected.reserve(res.segments.size()*2);
    for (int i = 0; i < corrected.size(); i++) corrected[i] = false;
    
    connect_segments(res.segments,corrected);

    // connect_segments2(res.segments, corrected, eval_img);
    clean_connections(res.segments);
    join_segments(res.segments);

    // Faces
    const int MIN_DIST_BETWEEN = 10;
    vector<line_pair> seg_pairs = find_segment_pairs(res.segments, MIN_DIST_BETWEEN);

    res.faces = find_faces(res.segments, seg_pairs);

    uniquify_faces(res.faces);

    for(face_t &f : res.faces){
        compute_face(f); // Sets props of face_t
    }

    //////// FACE SEARCH //////////////// 
    search_faces(item_exp, res, plate_exp_area, top_exp_area); // NOTE: sets found_plate, found_top and used_segs of res

    res.detection_type = NOT_FOUND;
    const double weight_plate = face_side_count(res.found_plate.complete);
    const double weight_top   = face_side_count(res.found_top.complete);
    const double total_weight = weight_plate + weight_top;
    if(total_weight > 0){ // At least one face was found
        if(item_exp.box_exp.max_angle_diff == -1){
            // Assumes plate angle is known, which is not true if square
            const double th_dif_plate = angle_face_diff(0, res.found_plate.props);
            const double th_dif_top = angle_face_diff(0, res.found_top.props);
            res.box.theta = (weight_plate * th_dif_plate + weight_top * th_dif_top) / total_weight;
            if(res.found_plate.props.rect_dir==FACE_DIR_UNKNOWN){
                if(debug) printf("Plate dir unknown when setting angle\n");
            }
            if(debug) printf("set new angle: %.1f, plate dir: %d\n",res.box.theta*180/M_PI,res.found_plate.props.rect_dir);
        }else{
            const double th_dif_plate = angle_face_diff(item_exp.box_exp.theta, res.found_plate.props);
            const double th_dif_top = angle_face_diff(item_exp.box_exp.theta, res.found_top.props);
            res.box.theta = item_exp.box_exp.theta + (weight_plate * th_dif_plate + weight_top * th_dif_top) / total_weight;
        }
        res.box.center = (weight_plate * res.found_plate.props.center + weight_top * res.found_top.props.center) / total_weight;
        const double plate_scale = sqrt(res.found_plate.props.area / item_exp.box_consts.plate_area);
        const double top_scale = sqrt(res.found_top.props.area / item_exp.box_consts.face_area);
        res.box.scale = (weight_plate*plate_scale + weight_top*top_scale) / total_weight;
        res.detection_type = DETECT_FACE_CENTER;
    }
    else if(item_exp.box_exp.max_displacement >= 0 && item_exp.box_exp.max_angle_diff >= 0)
    {
        if(debug) printf("No faces found, search segments! (center: [%d,%d], theta: %.1f, scale: %f)\n",item_exp.box_exp.center.x,item_exp.box_exp.center.y,item_exp.box_exp.theta/M_PI*180,item_exp.exp_scale);
        double plate_s[2], top_s[2];
        plate_s[0] = item_exp.exp_scale * item_exp.box_consts.plate_size[0];
        plate_s[1] = item_exp.exp_scale * item_exp.box_consts.plate_size[1];
        top_s[0] = item_exp.exp_scale * item_exp.box_consts.face_size[0];
        top_s[1] = item_exp.exp_scale * item_exp.box_consts.face_size[1];
        if(debug) printf("plate: %f x %f, top %f x %f\n", plate_s[0], plate_s[1], top_s[0], top_s[1]);
        box_t seg_search_res;
        if(debug) printf("exp center: (%d, %d)\n",item_exp.box_exp.center.x, item_exp.box_exp.center.y);
        vector<int> found_segs = search_box_segments(seg_search_res, item_exp.box_exp, plate_s, top_s, item_exp.box_exp.max_displacement, res.segments);
        if(debug) printf("Found %lu\n",found_segs.size());
        if(found_segs.size() > 2){
            res.box = seg_search_res;
            res.used_segs = found_segs;
            res.detection_type = DETECT_LOOSE_SEGMENTS;
        }
    }

    return res;
}

inline Point2i BoxDetection::item_offset(const box_state &s){
     return s.frame_exp.RoI.tl() + s.frame_res.item_crop.tl();
}

inline Rect BoxDetection::region_of_interest(Rect roi, Point2i new_center, const double scale, const Point2i margin, const Size2i img_size){
    roi.width = scale * roi.width + 2*margin.x;
    roi.height = scale * roi.height + 2*margin.y;
    roi.x = new_center.x - roi.width / 2;
    roi.y = new_center.y - roi.height / 2;
    roi &= Rect(0, 0, img_size.width, img_size.height);
    // roi.x = std::max(roi.x, 0);
    // roi.y = std::max(roi.y, 0);
    // roi.width = std::min(roi.x + roi.width, img_size.width) - roi.x;
    // roi.height = std::min(roi.y + roi.height, img_size.height) - roi.y;
    return roi;
}

void BoxDetection::update_expectations(box_state &state){
    // Update expectations
    frame_expectations &new_expects = state.frame_exp;

    new_expects.item_exp.exp_hough_lines.clear();

    item_results &item_res = state.frame_res.item_res;

    if(state.frame_res.n_items > 0){
        // Item found
        if(state.item_stability > 0 || item_res.detection_type!=NOT_FOUND) state.item_stability++;

        // Update hough thres
        if(state.frame_res.item_res.hough_lines.size()>0){
            double incr = 1.0 - (double)state.frame_exp.item_exp.exp_hough_lines.size() / 8.0 ;
            double decr = ((double)state.frame_res.item_res.hough_lines.size() - 10.0) / 50;
            double hough_eval = incr - decr;
            new_expects.item_exp.exp_n_houghlines = state.frame_res.item_res.hough_lines.size() + 5*hough_eval;
            new_expects.item_exp.exp_hough_thres =  state.frame_res.item_res.used_hough_thres;
            // new_expects.item_exp.exp_hough_thres = state.frame_res.item_res.used_hough_thres - hough_eval;
        }else{
            new_expects.item_exp.exp_n_houghlines = std::min(new_expects.item_exp.exp_n_houghlines+5, 40);
            new_expects.item_exp.exp_hough_thres = state.frame_res.item_res.used_hough_thres;
            // new_expects.item_exp.exp_hough_thres = state.frame_res.item_res.used_hough_thres -5;
        }

        if(state.item_stability > 0){
            filter_expects &filter_exp = new_expects.filter_exp;
            const double alpha = 0.6; //TODO base on wether a box was found
            const double new_area_val = alpha*state.frame_res.item_size + (1-alpha)*filter_exp.area.val;
            filter_exp.area = expectation<int>(new_area_val, 0.3);
            filter_exp.pos = state.frame_res.item_center; // TODO use box expected center
            filter_exp.max_pos_diff = sqrt(filter_exp.area.val); // TODO base on size/distance/movement
            filter_exp.eval = state.frame_res.item_eval;
            filter_exp.thres = state.frame_res.used_hsv;
        }

    }else{
        state.item_stability = std::min(state.item_stability-1, -1);
        if(state.item_stability < -4){
            reset_item_expectations(new_expects, state.box_type);
            state.item_stability = 0;
        }
    }

    if(item_res.detection_type != NOT_FOUND){
        box_t &found_box = item_res.box;

        // Transform center to frame coordinates:
        const Point2i global_center = found_box.center + item_offset(state);

        if(found_box.max_displacement == -1){ // First time box is found
            if(debug) printf("FIRST BOX FOUND\n");
            state.center_change = {0,0};
        }else{
            // Note state.final_center is the box found in last step
            state.center_change = global_center - state.final_center;
            // state.angle_dif = found_box.theta - state.final_angle; // TODO use (note thate might not have been matched)            
        }

        double plate_weight = 0;
        double top_weight = 0;
        double old_weight = 0;

        if(item_res.found_top.complete > 0){
            new_expects.item_exp.exp_scale = sqrt(item_res.found_top.props.area / state.frame_exp.item_exp.box_consts.face_area);
            top_weight = 0.3; // Base on box_stability and no. segments
        }else if(item_res.found_plate.complete > 0){
            new_expects.item_exp.exp_scale = sqrt(item_res.found_plate.props.area / state.frame_exp.item_exp.box_consts.plate_area);
            plate_weight = 0.3; // Base on box_stability and no. segments
        }else{
            new_expects.item_exp.exp_scale = state.frame_exp.item_exp.exp_scale;
        }
        old_weight = 1 - top_weight - plate_weight;

        state.final_angle = found_box.theta;
        state.final_center = global_center;

        new_expects.item_exp.box_exp.theta = found_box.theta;
        new_expects.item_exp.box_exp.center = found_box.center;
        new_expects.item_exp.box_exp.scale = found_box.scale;
        new_expects.item_exp.box_exp.max_displacement = 20; // TODO
        new_expects.item_exp.box_exp.max_angle_diff = 10/180*M_PI; // TODO

        new_expects.item_exp.exp_hough_lines.reserve(state.frame_res.item_res.used_segs.size());
        for (int i : state.frame_res.item_res.used_segs)
        {
            const line_seg &seg = state.frame_res.item_res.segments[i];
            const line_t &hline = state.frame_res.item_res.hough_lines[seg.hough_index];
            new_expects.item_exp.exp_hough_lines.push_back(hline);
        }

        new_expects.RoI = region_of_interest(state.frame_res.item_crop, global_center, 1.5, Point2i(20,20) ,FRAME_SIZE);

        state.box_stability = std::max(state.box_stability+1, 1);
    }
    else
    {
        if(debug) printf("BOX NOT MATCHED\n");

        box_t &new_box_exp = new_expects.item_exp.box_exp;

        if(state.frame_res.n_items == 0){
            // No items found
            // TODO reset thresholds
            new_expects.item_exp.box_exp.max_displacement = -1; // TODO increase instead 

        }else{
            Size2i item_size = state.frame_res.item_crop.size();
            new_box_exp.center = Point2i(item_size/2);
            new_box_exp.max_displacement = std::max(item_size.width, item_size.height)/3;             

            state.final_center = new_box_exp.center + item_offset(state);
            state.frame_res.item_res.detection_type = DETECT_ITEM_CENTER;

            // item_crop is in RoI-coordinates
            Point2i global_center = new_box_exp.center + item_offset(state);
            new_expects.RoI = region_of_interest(state.frame_res.item_crop, global_center, 1.5, Point2i(new_box_exp.max_displacement, new_box_exp.max_displacement) ,FRAME_SIZE);
        }

        // if(state.frame_res.contour_center.x >= 0){
        //     state.final_center = state.frame_res.contour_center;
        //     state.frame_res.item_res.detection_type = DETECT_CONTOURS;
        // }

        if(new_expects.item_exp.box_exp.max_displacement != -1){
            // Theta has been found once:
            new_expects.item_exp.box_exp.max_displacement += 10*M_PI/180;
            if(new_expects.item_exp.box_exp.max_displacement > M_PI)
                new_expects.item_exp.box_exp.max_displacement = -1;
        }

        state.box_stability = std::min(state.box_stability-1, -1);

        // // TODO check if box is close to image edge

        if(state.box_stability < -4){
            reset_box_expectations(new_expects);
            state.box_stability = 0;
        }
    }

}

/////////////// FILTERING //////////////////

void BoxDetection::filter_HSV(const Mat3b &src, hsv_thresholds hsv, Mat1b &color_mask)
{
    Mat3b src_hsv;
    cvtColor(src, src_hsv, COLOR_BGR2HSV);

    if(hsv.low_H > hsv.high_H){
        inRange(src_hsv, Scalar(0, hsv.low_S, hsv.low_V), Scalar(hsv.high_H, hsv.high_S, hsv.high_V), color_mask);
        Mat1b mask2;
        inRange(src_hsv, Scalar(hsv.low_H, hsv.low_S, hsv.low_V), Scalar(MAX_H, hsv.high_S, hsv.high_V), mask2);
        color_mask |= mask2;
    }else{
        inRange(src_hsv, Scalar(hsv.low_H, hsv.low_S, hsv.low_V), Scalar(hsv.high_H, hsv.high_S, hsv.high_V), color_mask);
    }
}

void BoxDetection::filter_RGB_min_value(const Mat3b &src, double val_min, Mat1b &color_mask){
    inRange(src, Scalar(val_min, val_min, val_min), Scalar(255, 255, 255), color_mask);
}


int BoxDetection::find_mask_items(const Mat1b &mask, const int line_thres, vector<filter_item> &items)
{
    
    // lines are described as [start col, end col, item_index]
    int pix_total = 0;
    vector<Vec3i> lines; // {x1, x2, id}
    vector<Vec3i> last_lines;
    lines.reserve(4);
    for (int r = 0; r < mask.rows; r++)
    {
        // Push back each horizantal sub line in row
        int count = 0;
        // int end_count = 0;
        int start;
        for (int c = 0; c < mask.cols; c++)
        {
            if(mask.at<uchar>(r,c) > 0){
                if(count==0){
                    start = c;
                }
                ++count;
                ++pix_total;
                // end_count = 0;
            }
            // else if(end_count < line_thres){
            //     end_count++;
            // }
            else{
                if(count >= line_thres){
                    Vec3i l = {start,c,-1};
                    lines.push_back(l);
                }
                count=0;
                // end_count=0;
            }
        }

        // Check each line with lines from last row
        for(Vec3i &l1 : lines){
            for (int j = 0; j < last_lines.size(); j++)
            {
                Vec3i &l2 = last_lines[j];
                if(l1[1] > l2[0] && l1[0] < l2[1]) // l1 is same item as l2
                {
                    if(l1[2] != l2[2]){
                        if(l1[2] == -1)
                        {
                            // Add line to same item as l2
                            l1[2] = l2[2];
                            items[l1[2]].size += l1[1]-l1[0];
                        }
                        else
                        {
                            // Merge item l1[2] and l2[2]:
                            filter_item &item1 = items[l1[2]];
                            filter_item &item2 = items[l2[2]];

                            item1.b1.x = std::min(item1.b1.x,item2.b1.x);
                            //Note: item1.b1.y is always smaller
                            item1.b2.x = std::max(item1.b2.x,item2.b2.x);
                            item1.b2.y = std::max(item1.b2.y,item2.b2.y);

                            item1.size += item2.size;
                            item1.merges++;

                            item2.size = -1; // Mark item as deleted

                            // Iterate rest of last_lines to update if index points to 'deleted' item
                            for (int k = j+1; k < last_lines.size(); k++)
                                if(last_lines[k][2]==l2[2]) last_lines[k][2]=l1[2];
                            l2[2] = l1[2];
                        }
                        items[l1[2]].b1.x = min(items[l1[2]].b1.x,l1[0]);
                        items[l1[2]].b2.x = max(items[l1[2]].b2.x,l1[1]);
                        items[l1[2]].b2.y = r;
                    }

                }
            }
            if(l1[2]==-1)
            {// New item:
                l1[2] = items.size();
                items.push_back({ {l1[0],r}, {l1[1],r}, l1[1]-l1[0], 0});
            }
        }
        last_lines = lines;
        lines.clear();
    }

    // Delete merged items
    auto it = items.begin();
	while (it != items.end()) {
		if (it->size == -1) {
			it = items.erase(it);
		} else
			it++;
	}
    return pix_total;
}


bool BoxDetection::evaluate_mask(const Mat1b &hsv_mask, filter_constants C, const filter_expects &exp, filter_result &res){

    vector<filter_item> items;
    items.reserve(C.MAX_ITEMS);
    const int pix_total = find_mask_items(hsv_mask, C.LINE_THRES, items);

    if(items.size() == 0 || items.size() > C.MAX_ITEMS) return false;

    int total_inliers = 0;
    res.total_merges = 0;
    res.proper_items.clear();
    res.proper_items.reserve(items.size());
    double min_pos_diff = INFINITY;
    res.item_idx = -1;
    for (int i = 0; i < items.size(); i++)
    {
        filter_item &it = items[i];
        if(it.size < exp.area.min || it.size > exp.area.max) continue;

        res.proper_items.push_back(it);
        res.total_merges += items[i].merges;
        total_inliers += items[i].size;

        if(exp.max_pos_diff != -1){
            Point2i center = {(it.b1.x + it.b2.x)/2, (it.b1.y + it.b2.y)/2};
            const double pos_diff = norm(exp.pos - center);
            if(pos_diff <= exp.max_pos_diff && pos_diff < min_pos_diff){
                min_pos_diff = pos_diff;
                res.item_idx = res.proper_items.size()-1;
            }
        }
    }
    res.proper_ratio = (double)res.proper_items.size() / items.size();
    res.outliers = pix_total - total_inliers;
    return true;
}


BoxDetection::filter_result BoxDetection::find_filtered_items(const Mat3b &src, const int down_samp, filter_expects filter_exp/*, const Mat1b &plate_mask*/)
{
    Mat3b src_downsamp = src.clone();
    for (int i = 0; i < down_samp; i++)
        pyrDown( src_downsamp, src_downsamp);

    // src_downsamp = src_downsamp(Rect(0,0,plate_mask.cols,plate_mask.rows));

    // resize(src, src_downsamp, Size(), 0.25, 0.25, INTER_AREA);

    // const int LINE_THRES = 8/pow(2,down_samp);
    const int LINE_THRES = 1; // TODO base on expected distance / scale
    const filter_constants FC = {LINE_THRES, 10};

    filter_result best_result;
    best_result.proper_ratio = 0;
    best_result.item_idx = -1;
    vector<filter_item> &mask_items = best_result.proper_items;

    // Try current low S:
    if(filter_exp.area.val != -1 && filter_exp.max_pos_diff != -1){
        Mat1b hsv_mask;
        filter_HSV(src_downsamp, filter_exp.thres, hsv_mask);
        // hsv_mask |= plate_mask;
        filter_result res;
        if(evaluate_mask(hsv_mask, FC, filter_exp, res) && res.item_idx != 1){
            // Found expected item
            if(res.proper_ratio > 0.5*filter_exp.eval){
                best_result = res;
                best_result.thres = filter_exp.thres;
            }
        }
    }
    
    // Adjust Low S to find a nice filter
    if(best_result.item_idx == -1)
    {
        best_result.outliers = 100000;
        const int LOW_S_MIN = 10;
        const int LOW_S_MAX = 200;
        const int LOW_S_STEP = 17;

        int &lowS = filter_exp.thres.low_S;
        for (lowS = LOW_S_MIN; lowS < LOW_S_MAX; lowS += LOW_S_STEP)
        // for (int i = 0; i < 10; ++i)
        {
            filter_result res;
            Mat1b hsv_mask;
            filter_HSV(src_downsamp, filter_exp.thres, hsv_mask);
            if(!evaluate_mask(hsv_mask, FC, filter_exp, res)) continue;
            const double eval = res.proper_ratio;
            const int outliers = res.outliers;

            // TODO check if items overlap!
            // TODO take res.item_idx into account !
            bool better_or_equal_fit = res.item_idx != -1 || best_result.item_idx == -1; // TODO debug this
            if(better_or_equal_fit && eval > 0 && (eval > best_result.proper_ratio || (eval==best_result.proper_ratio && outliers < best_result.outliers))){
                best_result = res;
                best_result.thres = filter_exp.thres;
            }

        }

        if(mask_items.size() == 0){
            filter_exp.area.val = -1;
            best_result.item_idx = -1;
            return best_result;
        }
        else if(best_result.item_idx == -1){
            // Sort predicate
            // TODO select based on position as well (center of image)
            struct item_comparator
            {
                inline bool operator() (const filter_item& item1, const filter_item& item2)
                {
                    return (item1.size < item2.size);
                }
            };
            sort(mask_items.begin(), mask_items.end(), item_comparator());
            best_result.item_idx = 0;
        }

    }
    return best_result;
}

BoxDetection::contour_result BoxDetection::find_contours(Mat1b mask, int area_min, const Point2i exp_center){
    const bool select_by_center = exp_center.x != -1;
    contour_result r;
    vector<Vec4i> hierarchy;
    vector<vector<Point>> contours;
    r.centers.reserve(5);
    r.contours.reserve(5);
    r.selected_idx = -1;
    r.selected_area = 0;
    r.total_area = 0;
    r.total_accepted_area = 0;
    r.best_center_dif = 10000;
    findContours( mask, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
    for (int i = 0; i < contours.size(); i++)
    {
        const double area = contourArea(contours[i]);
        r.total_area += area;
        if(area > area_min){
            r.total_accepted_area += area;
            Moments m = moments(contours[i],true);
            Point p(m.m10/m.m00, m.m01/m.m00);
            r.centers.push_back(p);
            r.contours.push_back(contours[i]);
            if(select_by_center){
                const double dist = norm(exp_center-p);
                if(dist < r.best_center_dif){
                    r.best_center_dif = dist;
                    r.selected_area = area;
                    r.selected_idx = r.contours.size()-1;
                }
            }else{
                if(area > r.selected_area){
                    r.selected_area = area;
                    r.selected_idx = r.contours.size()-1;
                }
            }
        }   
    }
    r.total_n_contours = contours.size();
    if(!select_by_center || r.selected_idx==-1) r.best_center_dif = -1;
    return r;
}

BoxDetection::contour_result BoxDetection::auto_filter_plate(const Mat3b &src, int &value, const int exp_plate_area, Point2i exp_center){
    Mat1b mask;
    value = std::min(std::max(value,100),255);
    filter_RGB_min_value(src, value, mask);
    const int MIN_CONT_AREA = 0.5*exp_plate_area;

    contour_result best_r = find_contours(mask, MIN_CONT_AREA, exp_center);
    int best_thres = value;
    double best_size_diff = ((double) best_r.selected_area - exp_plate_area)/exp_plate_area;

    double proper_ratio = (double)best_r.contours.size() / best_r.total_n_contours;
    
    bool decreasing;
    if(best_size_diff > 0.3){
        decreasing = false;
    }else if(best_r.contours.size() == 0 || best_size_diff < -0.3){
        decreasing = true;
    }else if(proper_ratio > 0.25){
        return best_r;        
    }
    else{
        decreasing = true;
    }

    const int max_tries = 5;
    int step_size = 20;
    for (int i = 0; i < max_tries; i++)
    {
        value += decreasing ? -step_size : step_size;
        value = std::min(std::max(value,100),255);
        filter_RGB_min_value(src, value, mask);
        contour_result r = find_contours(mask, MIN_CONT_AREA, exp_center);
        const double size_diff = ((double) r.selected_area - exp_plate_area)/exp_plate_area;
        const double proper_ratio = (double)r.contours.size() / r.total_n_contours;

        if(fabs(size_diff) < best_size_diff){
            best_size_diff = size_diff;
            best_r = r;
            best_thres = value;
        }

        if(size_diff > 0.3){
            if(decreasing) step_size /= 2;
            decreasing = false;
        }else if(r.contours.size() == 0 || size_diff < -0.3){
            if(!decreasing) step_size /= 2;
            decreasing = true;
        }else if(proper_ratio < 0.25){
            break;
        }
    }
    value = best_thres;
    return best_r;
}

BoxDetection::contour_result BoxDetection::auto_filter_box(const Mat3b &src, hsv_thresholds &hsv_thres, int &value, const int MIN_CONT_AREA){
    
    Mat1b mask;
    filter_HSV(src, hsv_thres, mask);
    contour_result r = find_contours(mask, MIN_CONT_AREA, {-1,-1});

    double proper_ratio = (double)r.total_accepted_area / r.total_area;
    
    bool decreasing = (r.contours.size()==0 || (double)r.total_accepted_area / r.total_area < 0.9);

    const int max_tries = 5;
    int step_size = 40;
    for (int i = 0; i < max_tries; i++)
    {
        value += decreasing ? -step_size : step_size;
        value = std::min(std::max(value,0),255);
        filter_HSV(src, hsv_thres, mask);
        r = find_contours(mask, MIN_CONT_AREA, {-1, -1});

        const double new_proper_ratio = (double)r.total_accepted_area / r.total_area;
        if(new_proper_ratio > proper_ratio){
            
        }else{
            decreasing = !decreasing;
            step_size /= 2;
        }
        proper_ratio = new_proper_ratio;
    }

    return r;
}

double BoxDetection::filter_and_search_plate(const Mat3b src, Point2i center, int found_box_area, int &exp_filter_thres, Rect &crop, contour_result &fc){
    const double area_scale = (double)found_box_area / state.frame_exp.item_exp.box_consts.face_area;
    const double size_scale = sqrt(area_scale);
    const double exp_long_side = size_scale * (2*state.frame_exp.item_exp.box_consts.plate_size[1] + 0.3);
    const double half_side = exp_long_side/2;
    const double exp_plate_area = area_scale * state.frame_exp.item_exp.box_consts.plate_area;

    crop = Rect(center.x - half_side, center.y - half_side, exp_long_side, exp_long_side);
    crop &= Rect(0, 0, src.cols, src.rows);

    if(debug) cout << "biggest area: " << found_box_area << endl;
    if(debug) cout << "expected plate size: " << exp_plate_area << endl;

    Mat3b src_down_crop = src(crop);
    const Point2i exp_center = center-crop.tl();
    fc = auto_filter_plate(src_down_crop, exp_filter_thres, exp_plate_area, exp_center);

    return size_scale;
}

////////////////// LINES AND SEGMENTS //////////////////
void BoxDetection::find_CannyEdges(const Mat1b &src_gray, const int thres_low, const int thres_high, Mat1b &edge_mask)
{
    const int blur_size = 3;
    const int kernel_size = 3;
    blur( src_gray, edge_mask, Size(blur_size,blur_size) );
    Canny( edge_mask, edge_mask, thres_low, thres_high, kernel_size );
}

int BoxDetection::find_hough_lines(const Mat1b &edges, const int exp_threshold, const int n_lines, const vector<line_t> &exp_lines, vector<line_t> &detected_lines)
{
    vector<Vec2f> lines;

    int threshold = exp_threshold;
    const double THETA_THRES = 4;
    double RHO_THRES = 3;

    int tries = 0;
    const int MAX_TRIES = 15;

    int change_dir = 0;
    for (int i = 0; i < MAX_TRIES; i++){
        lines.clear();
        HoughLines(edges, lines, HOUGH_RHO_RES, HOUGH_THETA_RES, threshold, 0, 0);
        if(fabs(n_lines - lines.size()) < 3) break;
        else if(lines.size() < n_lines){
            if(change_dir > 0) break;
            threshold -= 5;
            change_dir = -1;
        }else{
            if(change_dir < 0) break;
            threshold += 5;
            change_dir = 1;
        }
    }

    if(lines.size() > 50) return exp_threshold;

    int delete_count = 0; // debug
    while(lines.size() - delete_count > n_lines){
        for (int i = 0; i < lines.size(); i++)
        {
            if(lines[i][0] == -1) continue;
            for (int j = i+1; j < lines.size(); j++)
            {
                if(lines[j][0] == -1) continue;
                if(fabs(lines[i][0]-lines[j][0]) < RHO_THRES && fabs(lines[i][1]-lines[j][1]) < THETA_THRES){
                    lines[j][0] = -1;
                    ++delete_count; // debug
                }
            }
        }
        RHO_THRES += 2;
    }
    if(debug) cout << "deleted " << delete_count << " hough lines (" << lines.size()-delete_count << " left)"<<endl;

    for (int i = 0; i < lines.size(); i++)
    {
        if(lines[i][0] == -1) continue;
        float rho = lines[i][0], theta = lines[i][1];
        const double a = cos(theta), b = sin(theta);
        const double l = sqrt(a*a+b*b);
        detected_lines.push_back(line_t(a/l,b/l,-rho));
    }

    return threshold;
}

vector<BoxDetection::line_seg> BoxDetection::find_hough_segments(const vector<line_t> &lines, const Mat1b mask, const double min_length, const double max_thickness){
    vector<line_seg> segments;
    segments.reserve(lines.size());
    for (int i = 0; i < lines.size(); i++)
    {
        const line_t &hline = lines[i];
        Point2i p1,p2;
        int inliers = line_endpoints(line_subset(hline,mask,max_thickness),p1,p2);
        const double len = norm(p2-p1);
        if(inliers > 0 && len >= min_length){
            // line_seg l({.p1=p1,.p2=p2,.line=hline,.length=len,.inliers=inliers,.hough_index=i});
            // segments.push_back({.p1=p1,.p2=p2,.line=hline,.length=len,.inliers=inliers,.hough_index=i});
            line_seg l;
            l.p1 = p1;
            l.p2 = p2;
            l.line = hline;
            l.length = len;
            l.inliers = inliers;
            l.hough_index = i;
            segments.push_back(l);
        }
        
    }    
    return segments;
}

vector<BoxDetection::line_point> BoxDetection::line_subset(const line_t line, const Mat1b src, const double thickness)
{
    int y_min, y_max;
    vector<line_point> points;
    if(line[0]==0){ // Horizontal line
        cout << "WARNING Horizontal line not implemented!!"<<endl;
        return points;
    }else{
        float c1 = line[2] + copysignf(thickness, line[0]); // will be further in the +x direction
        float c2 = line[2] - copysignf(thickness, line[0]); // will be further in the -x direction

        if(line[1]==0){ // Vertical line
            y_min=0;
            y_max=src.rows;
        }else{
            double yleft  =  -c2/line[1];                   // when x=0
            double yright = (-c1-line[0]*src.cols)/line[1]; // when x=src.cols
            if(yleft < yright){
                y_min = yleft; // floor
                y_max = yright+1; // ceil
            }else{
                y_min = yright;  // floor
                y_max = yleft+1; // ceil
            }
            y_min = y_min<0 ? 0:y_min;
            y_max = y_max>src.rows ? src.rows:y_max;
        } 

        for (int row = y_min; row < y_max; ++row)
        {
            int x1 = floor( -(c1 + line[1]*row)/line[0] );
            int x2 = ceil( -(c2 + line[1]*row)/line[0] );
            x1 = x1<0 ? 0:x1;
            x2 = x2>src.cols ? src.cols:x2;
            for (int col = x1; col < x2; ++col)
            {
                if(src.at<uchar>(row,col) > 0){
                    float p = line[0]*row - line[1]*col; // How far along the line the point is (length of projection on the cw-rotated normal vector)
                    line_point lp = {col,row,p};
                    points.push_back(lp);
                }else{
                }
            }   
        }
    }
    return points;
}

int BoxDetection::line_endpoints(vector<line_point> points, Point2i &p_start, Point2i &p_end)
{
    const double SEG_MAX_PIXEL_DIST = 1.5; //  > sqrt(2)
    const double SEG_MAX_HOLE = 50;
    const int SEG_VALIDS_THRES = 15;
    const int SEG_VALIDS_RESTART = 5;
    const int SEG_RESTART_RATIO = 3;

    if(points.empty()){
        cout << "ERROR Empty point list!"<<endl;
        return -1;
    }

    std::sort(points.begin(),points.end());
    int start=-1,end=-1;
    int consec_valid=0;
    int i;

    // Find start point
    for (i = 1; i < points.size(); ++i)
    {
        if(points[i].p - points[i-1].p > SEG_MAX_PIXEL_DIST)
        { // Invalid point:
            consec_valid=0;
            continue;
        }

        ++consec_valid;
        if(consec_valid > SEG_VALIDS_THRES){
            start = i - consec_valid;
            break;
        }
    }

    // Find end point
    while(end == -1){
        // Set end point as first invalid point
        for (; i < points.size(); ++i)
        {
            if(points[i].p - points[i-1].p > SEG_MAX_PIXEL_DIST)
            { // Invalid point:
                end = i - 1;
                break;
            }
        }
        if(end == -1) end = points.size()-1;

        // Check if line starts again
        double consec_valid = 0;
        double invalid_dist = 0;
        for (; i < points.size(); ++i)
        {
            const double dist_since_last = points[i].p - points[i-1].p;
            const double dist_since_end = points[i].p - points[end].p;

            if(dist_since_last > SEG_MAX_PIXEL_DIST)
            { // Invalid point:
                consec_valid=0;
                invalid_dist += dist_since_last;
                if(dist_since_end > SEG_MAX_HOLE) break; // To far from end point
                continue;
            }

            ++consec_valid;
            if(consec_valid > SEG_VALIDS_RESTART && 
            SEG_RESTART_RATIO*invalid_dist < dist_since_end)
            { // Theres at least SEG_RESTART_RATIO as much valid distance as invalid
                end = -1;
                break;
            }
        }
    }

    if(start == -1) return 0;

    p_start.x = points[start].x;
    p_start.y = points[start].y;
    p_end.x = points[end].x;
    p_end.y = points[end].y;

    return end-start;
}

void BoxDetection::find_unique_segments(vector<line_seg> &segs, const double unique_ang_thres, const double unique_dist_thres)
{
    vector<int> delete_indices;
    delete_indices.reserve(segs.size()/3);
    for (int i = 0; i < segs.size(); i++)
    {
        for (int j = i+1; j < segs.size(); j++)
        {

            point_t mid;
            mid[0] = (segs[i].p1.x + segs[i].p2.x)/2;
            mid[1] = (segs[i].p1.y + segs[i].p2.y)/2;
            mid[2] = 1;

            const double dist = fabs(segs[j].line.ddot(mid));            
            const double ang_diff = fabs(acos(line_dot(segs[i].line, segs[j].line)));

            if(ang_diff < (double)unique_ang_thres/100 && dist < unique_dist_thres){
                int delete_index = j;
                if(segs[i].inliers < segs[j].inliers) delete_index = i;

                if(find(delete_indices.begin(), delete_indices.end(), delete_index) == delete_indices.end()) {
                    delete_indices.push_back(delete_index);
                } 
            }else{
            }
        }
    }

    sort(delete_indices.begin(), delete_indices.end());
    while(delete_indices.size() > 0){
        segs[delete_indices.back()] = segs.back();
        segs.pop_back();
        delete_indices.pop_back();
    }

}

void BoxDetection::connect_segments(vector<line_seg> &segs, vector<bool> &corrected)
{
    vector<line_seg> new_segments;

    int MAX_DIST = 8; // TODO depend on expected size
    for (int i = 0; i < segs.size(); i++)
    {
        Point *pi = &segs[i].p1;
        for (int c = 0; c < 2; c++)
        {
            if(corrected[2*i+c]) {
                pi = &segs[i].p2;
                continue; // next pi
            }

            for (int j = 0; j < segs.size(); j++)
            {
                if(j == i) {
                    continue; //next j
                }

                // Intersection point wbetween lines
                point_t v_int = segs[i].line.cross(segs[j].line);
                Point2i p_int = {static_cast<int>(v_int[0]/v_int[2]), static_cast<int>(v_int[1]/v_int[2])};

                // if(fabs(line_point_dot(segs[j].line, *pi)) < MAX_DIST/2){
                if(norm(*pi - p_int) < MAX_DIST){ // Point is close to intersection

                    // Connect at intersection
                    Point *pj = &segs[j].p1;
                    for (int c2 = 0; c2 < 2; c2++)
                    {
                        // if(fabs(line_point_dot(segs[i].line, *pj)) < MAX_DIST/2){
                        if(norm(*pj - p_int) < MAX_DIST){
                            if(!corrected[2*j+c2]){
                                *pj = p_int;
                                corrected[2*j+c2] = true;
                            }
                            *pi = *pj;
                            corrected[2*i+c] = true;
                            break;
                        }
                        pj = &segs[j].p2;
                    }

                    // Connect to side
                    if(!corrected[2*i+c])
                    {                        
                        const double si = segs[j].line[0] * p_int.y      - segs[j].line[1] * p_int.x; // Intersection point along line j
                        const double s1 = segs[j].line[0] * segs[j].p1.y - segs[j].line[1] * segs[j].p1.x; // pj1 along line j
                        const double s2 = segs[j].line[0] * segs[j].p2.y - segs[j].line[1] * segs[j].p2.x; // pj2 along line j
                        if(s1 < si && si < s2)
                        { // Intersection is between line ends (pj1 and pj2):

                            // Connect pi to side line of j and split j in two
                            *pi = p_int;
                            corrected[2*i+c] = true;
                            // Divide line j in two
                            line_seg lj2 = seg_from_ends(p_int,segs[j].p2);
                            lj2.hough_index = segs[j].hough_index;
                            segs[j].p2 = p_int;
                            corrected[2*j+1] = true;

                            segs.push_back(lj2);
                            corrected.push_back(true);
                            corrected.push_back(false);
                            break;
                        }
                    }else{
                        break;
                    }
                }

            }

            pi = &segs[i].p2;
        }
    }  
}

void BoxDetection::clean_connections(vector<line_seg> &segs){

    for (int i = 0; i < segs.size(); i++)
    {
        if(segs[i].p1 == segs[i].p2){
            segs[i].p1.x = -1; // Mark for deletion
            continue;
        }

        for (int j = i+1; j < segs.size(); j++)
        {
            if(segs[i]==segs[j]){
                segs[i].p1.x = -1; // Mark for deletion
                break;
            }
        }
    }

    segs.erase(std::remove_if(
    segs.begin(), segs.end(),
    [](const line_seg& s) { 
        return s.p1.x == -1;
    }), segs.end());
}

void BoxDetection::join_segments(vector<line_seg> &segs){
    const double PAR_THRES = 0.9;
    vector<line_seg> new_segments;
    for (int i = 0; i < segs.size(); i++)
    {
        for (int j = i+1; j < segs.size(); j++)
        {
            const double dotp = line_dot(segs[i].line, segs[j].line);
            if(fabs(dotp) > PAR_THRES) // Lines are parallel
            {
                line_seg new_seg;
                bool joined = false;
                if(dotp > 0){
                    if(segs[i].p2 == segs[j].p1)
                    {
                        new_seg = seg_from_ends(segs[i].p1, segs[j].p2);
                        new_seg.hough_index = segs[i].hough_index;
                        joined = true;
                    }
                    else if(segs[i].p1 == segs[j].p2)
                    {
                        new_seg = seg_from_ends(segs[i].p2, segs[j].p1);
                        new_seg.hough_index = segs[i].hough_index;
                        joined = true;
                    }
                }else{
                    if(segs[i].p1 == segs[j].p1)
                    {
                        new_seg = seg_from_ends(segs[i].p2, segs[j].p2);
                        new_seg.hough_index = segs[i].hough_index;
                        joined = true;
                    }
                    else if(segs[i].p2 == segs[j].p2)
                    {
                        new_seg = seg_from_ends(segs[i].p1, segs[j].p1);
                        new_seg.hough_index = segs[i].hough_index;
                        joined = true;
                    }
                }
                if(joined){
                    new_segments.push_back(new_seg);
                }
            }
        }
    }
    segs.insert(segs.end(), new_segments.begin(), new_segments.end());
}

vector<BoxDetection::line_pair> BoxDetection::find_segment_pairs(const vector<line_seg> &segs, const double MIN_DIST){
    vector<line_pair> pairs;
    for (int i = 0; i < segs.size(); i++)
    {
        for (int j = i+1; j < segs.size(); j++)
        {
            if(segs[i].hough_index == segs[j].hough_index && segs[i].hough_index != -1) continue;
            // Not from same hough_line
            
            const double ratio = 2;
            if(segs[i].length > segs[j].length * ratio || segs[j].length > segs[i].length * ratio) continue;
            // Approx same length:

            const Point2i center = (segs[i].p1 + segs[i].p2 + segs[j].p1 + segs[j].p2)/4;
            const Point2i p_int = line_intersection(segs[i].line, segs[j].line);
            if(norm(center - p_int) < segs[i].length+segs[j].length) continue;
            // No close intersection:

            if(segs[i].p1 == segs[j].p1 || segs[i].p1 == segs[j].p2
            || segs[i].p2 == segs[j].p1 || segs[i].p2 == segs[j].p2) continue;
            // No shared points:

            Point2i j_mid = (segs[j].p1 + segs[j].p2)/2;
            const double dist_to_mid = fabs(line_point_dot(segs[i].line, j_mid));
            if(dist_to_mid < MIN_DIST) continue;
            
            pairs.push_back({i,j,-1});
        }
    }
    return pairs;
}


////////////////// FACE SEARCH //////////////////

vector<BoxDetection::face_t> BoxDetection::find_faces(const vector<line_seg> &segs, const vector<line_pair> &pairs)
{
    vector<face_t> faces;
    faces.reserve(pairs.size());

    for(line_pair pair : pairs)
    {
        face_t f;

        const int i = pair.ls1;
        const int j = pair.ls2;

        const double ij_dir = line_dot(segs[i].line, segs[j].line); // Wheter i and j pointing smae direction
        const double ij_lr = line_point_dot(segs[i].line, segs[j].p1); // Whether j is on right side of i
        const bool same_dir = ij_dir >= 0;
        const bool j_on_left = ij_lr >= 0;
        const bool flip_i = !j_on_left;
        const bool flip_j = same_dir==j_on_left;

        // Order segments point in circular order
        if(flip_i){
            f.p[0] = segs[i].p2;
            f.p[1] = segs[i].p1;
        }else{
            f.p[0] = segs[i].p1;
            f.p[1] = segs[i].p2;
        }

        if(flip_j){
            f.p[2] = segs[j].p2;
            f.p[3] = segs[j].p1;
        }else{
            f.p[2] = segs[j].p1;
            f.p[3] = segs[j].p2;
        }

        for(int k = 0; k < segs.size(); ++k)
        {
            if(k==i || k==j) continue;
            for (int c = 1; c <= 3; c+=2)
            {
                const Point2i &p_from = f.p[c];
                const Point2i &p_to = f.p[f_next[c]];

                const double cross = (p_to.x - p_from.x) * segs[k].line[1] - (p_to.y - p_from.y) * segs[k].line[0];
                const bool flip_k = cross > 0;
                bool match;
                if(flip_k){
                    match = segs[k].p2 == p_from && segs[k].p1 == p_to;
                }else{
                    match = segs[k].p1 == p_from && segs[k].p2 == p_to;
                }
                if(match){
                    f.complete |= (1<<c);
                    if(flip_k){
                        f.line[c] = -segs[k].line;
                    }else{
                        f.line[c] = segs[k].line;
                    }
                    f.seg_idc.push_back(k);
                    break;
                }

            }

        }

        if(f.complete > 0){
            // Add pair lines
            if(flip_i)
                f.line[0] = -segs[i].line;
            else
                f.line[0] = segs[i].line;

            if(flip_j)
                f.line[2] = -segs[j].line;
            else
                f.line[2] = segs[j].line;

            f.complete |= 5;

            f.seg_idc.push_back(i);
            f.seg_idc.push_back(j);

            faces.push_back(f);
        }
    
    }
    return faces;
}

void BoxDetection::uniquify_faces(vector<face_t> &faces)
{
    vector<int> delete_indices;
    delete_indices.reserve(faces.size());

    const int MIN_COUNT = 4;

    for (int i = 0; i < faces.size(); i++)
    {
        for (int j = i+1; j < faces.size(); j++)
        {
            int sim_count = 0;
            for (int pi = 0; pi < 4; pi++)
            {
                for (int pj = 0; pj < 4; pj++)
                {
                    if(faces[i].p[pi] == faces[j].p[pj]){
                        ++sim_count;
                        break;
                    }
                }
                if(sim_count >= MIN_COUNT) break;                
            }
            
            if(sim_count >= MIN_COUNT){
                const int delete_index = j; // TODO choose based on evaluation
                if(find(delete_indices.begin(), delete_indices.end(), delete_index) == delete_indices.end()) {
                    delete_indices.push_back(delete_index);
                } 
            }
        }
        
    }
    
    sort(delete_indices.begin(), delete_indices.end());
    while(delete_indices.size() > 0)
    {
        faces[delete_indices.back()] = faces.back();
        faces.pop_back();

        delete_indices.pop_back();
    }

}

void BoxDetection::compute_face(face_t &face){
    face_properties &props = face.props;
    
    props.center = (face.p[0]+face.p[1]+face.p[2]+face.p[3])/4;

    double len[4];
    double ang[4];
    // double th_sum = 0;
    for (int i = 0; i < 4; i++)
    {
        len[i] = norm(face.p[f_next[i]] - face.p[i]);
        const line_t &prev_line = face.line[f_prev[i]];
        ang[i] = acos(face.line[i][0]*prev_line[0] + face.line[i][1]*prev_line[1]);
        // th_sum += atan(face.line[i][1] / face.line[i][0]);
    }

    const double l1 = len[0]+len[2];
    const double l2 = len[1]+len[3];

    props.area = l1 / 2 * l2 / 2;
    props.theta = atan((face.line[1][1] - face.line[3][1]) / (face.line[1][0] - face.line[3][0]));

    if(l1 > l2){
        props.rect_dir = FACE_DIR_LONG_SHORT;
        props.ratio = l1 / l2;
    }else{
        props.rect_dir = FACE_DIR_SHORT_LONG;
        props.ratio = l2 / l1;
        // props.theta = atan((face.line[0][1] - face.line[2][1]) / (face.line[0][0] - face.line[2][0]));
    }
    if(props.ratio < 1.1) props.rect_dir = FACE_DIR_UNKNOWN;
}

vector<int> BoxDetection::search_face(const vector<face_t> &pool, const expectation<double> &ratio, const expectation<double> &area, Point2i exp_center, double max_displacement){
    vector<int> found_idc;
    for (int i = 0; i < pool.size(); i++)
    {
        const face_properties &props = pool[i].props;
        if(props.area > area.min && props.area < area.max
        && props.ratio > ratio.min && props.ratio < ratio.max
        && (max_displacement<0 || norm(props.center - exp_center) <= max_displacement)){
            found_idc.push_back(i);
        }

    }
    return found_idc;
}

void BoxDetection::search_faces(const item_expectations &item_exp, item_results &res, double plate_exp_area, double top_exp_area){

    double plate_eval=0, top_eval=0;

    // Search limits //
    expectation<double> search_plate_area(plate_exp_area, 0.4);
    expectation<double> search_plate_ratio(item_exp.box_consts.plate_ratio, 0.1);

    expectation<double> search_top_area(top_exp_area, 0.4);
    expectation<double> search_top_ratio(item_exp.box_consts.face_ratio, 0.1);

    vector<int> plate_search_res =  search_face(res.faces, search_plate_ratio, search_plate_area, item_exp.box_exp.center, item_exp.box_exp.max_displacement);
    vector<int> top_search_res =    search_face(res.faces, search_top_ratio,   search_top_area,   item_exp.box_exp.center, item_exp.box_exp.max_displacement);

    if(search_plate_ratio.max > search_top_ratio.min){
        // Plate may be found as top and vice versa, so treat each of them as both
        plate_search_res.insert(plate_search_res.end(), top_search_res.begin(), top_search_res.end());
        top_search_res.insert(top_search_res.end(), plate_search_res.begin(), plate_search_res.end());
    }

    if(debug) printf("Found %lu plate and %lu tops\n",plate_search_res.size(), top_search_res.size());

    if(plate_search_res.size() == 0 && top_search_res.size() == 0){
        return;
    }
    
    const double max_center_diff = 10; // TODO base on distance
    const double max_scale_diff = 1.2;
    double best_diff = INFINITY;
    int best_match[2] = {-1,-1};

    for (int i = 0; i < plate_search_res.size(); i++)
    {
        for (int j = 0; j < top_search_res.size(); j++)
        {
            if(plate_search_res[i] == top_search_res[j]) continue;
            face_t &plate = res.faces[plate_search_res[i]];
            face_t &top = res.faces[top_search_res[j]];

            const double center_diff = norm(plate.props.center - top.props.center);
            if(center_diff > max_center_diff) continue;
            const double plate_scale = sqrt(plate.props.area / item_exp.box_consts.plate_area);
            const double top_scale   =   sqrt(top.props.area / item_exp.box_consts.face_area);
            const double scale_diff = (plate_scale > top_scale) ? plate_scale / top_scale : top_scale / plate_scale;
            if(scale_diff > max_scale_diff) continue;

            if(center_diff < best_diff){
                best_match[0]=plate_search_res[i];
                best_match[1]=top_search_res[j];
                best_diff = center_diff;
            }
        }
    }
    if(best_match[0] != -1){
        if(debug) printf("Found match plate/top! (%d/%d)\n",best_match[0],best_match[1]);
        res.found_plate = res.faces[best_match[0]];
        res.found_top = res.faces[best_match[1]];
        res.used_segs.reserve( res.found_plate.seg_idc.size() + res.found_top.seg_idc.size());
        res.used_segs.insert(res.used_segs.end(), res.found_plate.seg_idc.begin(), res.found_plate.seg_idc.end());
        res.used_segs.insert(res.used_segs.end(), res.found_top.seg_idc.begin(),   res.found_top.seg_idc.end());
    }
    else
    {
        if(debug) printf("No plate/top match!\n");

        int best_face_i = -1;
        int best_face_n_segs = -1;
        vector<int> best_found_segs;
        bool best_is_plate;

        for (int i = 0; i < plate_search_res.size(); i++)
        {
            face_t &face = res.faces[plate_search_res[i]];
            int total_segs = face_side_count(face.complete);

            double scale_fact = sqrt(face.props.area / item_exp.box_consts.plate_area);
            expectation<double> short_side(item_exp.box_consts.face_size[0] * scale_fact, 0.2);
            expectation<double>  long_side(item_exp.box_consts.face_size[1] * scale_fact, 0.2);
            
            vector<int> found_segs;
            if(face.props.rect_dir==FACE_DIR_UNKNOWN){
                vector<int> found_segs1 = search_segments(face, true, short_side, long_side, 2, res.segments);
                vector<int> found_segs2 = search_segments(face, false, short_side, long_side, 2, res.segments);

                found_segs = (found_segs1.size() > found_segs2.size()) ? found_segs1 : found_segs2; 
                face.props.rect_dir = (found_segs1.size() > found_segs2.size()) ? FACE_DIR_LONG_SHORT : FACE_DIR_SHORT_LONG;
                
                total_segs += found_segs.size();
            }else{
                found_segs = search_segments(face, face.props.rect_dir==FACE_DIR_LONG_SHORT, short_side, long_side, 2, res.segments);
                total_segs += found_segs.size();
            }

            if(total_segs > best_face_n_segs){
                best_face_i = plate_search_res[i];
                best_face_n_segs = total_segs;
                best_found_segs = found_segs;
                best_is_plate = true;
            }
        }

        for (int i = 0; i < top_search_res.size(); i++)
        {
            face_t &face = res.faces[top_search_res[i]];
            int total_segs = face_side_count(face.complete);

            double scale_fact = sqrt(face.props.area / item_exp.box_consts.face_area);
            expectation<double> short_side(item_exp.box_consts.plate_size[0] * scale_fact, 0.1);
            expectation<double>  long_side(item_exp.box_consts.plate_size[1] * scale_fact, 0.1);

            if(res.found_top.props.rect_dir==FACE_DIR_UNKNOWN){
                printf("ERROR top face rect_dir UNKNOWN\n");
            }
            vector<int> found_segs = search_segments(face, face.props.rect_dir==FACE_DIR_LONG_SHORT, short_side, long_side, -2, res.segments);
            total_segs += found_segs.size();

            if(total_segs > best_face_n_segs){
                best_face_i = top_search_res[i];
                best_face_n_segs = total_segs;
                best_found_segs = found_segs;
                best_is_plate = false;
            }   
        }

        if(best_is_plate){
            if(debug) printf("Found plate with %d total segs\n",best_face_n_segs);
            res.found_plate = res.faces[best_face_i];
        }else{
            if(debug) printf("Found top with %d total segs\n",best_face_n_segs);
            res.found_top = res.faces[best_face_i];
        }
        res.used_segs.reserve(best_face_n_segs);
        res.used_segs.insert(res.used_segs.end(), res.faces[best_face_i].seg_idc.begin(), res.faces[best_face_i].seg_idc.end());
        res.used_segs.insert(res.used_segs.end(), best_found_segs.begin(), best_found_segs.end());
    }
}

vector<int> BoxDetection::search_segments(const face_t &face, bool long_side_first, const expectation<double> &short_side, const expectation<double> &long_side, double margin, vector<line_seg> segments){
    vector<int> found_segs;
    // face_t found_face;
    found_segs.reserve(4);
    for (int i = 0; i < 4; i++)
    {
        bool is_long = ((i & 1) != long_side_first);

        int best_seg = -1;
        int best_length = 0;

        // Search top segments:
        for (int j = 0; j < segments.size(); j++)
        {
            const line_seg &seg = segments[j];
            double dotp = line_dot(face.line[i], seg.line);
            if(fabs(dotp) > 0.95){ // TODO // Parrallel:
                double dot2 = line_point_dot(face.line[i], face.props.center); // Always positive! (TODO)
                double dot3 = line_point_dot(seg.line,     face.props.center);
                if( dotp*dot2*dot3 < 0) continue; // Found line is opposit of center
                double face_line_dtc = fabs(dot2);
                double dist_to_center = fabs(dot3);

                // TODO check midpoint dist to center
                if(margin < 0 && dist_to_center > face_line_dtc + margin) continue;
                if(margin > 0 && dist_to_center < face_line_dtc - margin) continue;

                if((!is_long && dist_to_center >= long_side.min/2 && dist_to_center <= long_side.max/2)
                || (is_long && dist_to_center >= short_side.min/2 && dist_to_center <= short_side.max/2))
                {
                    if(seg.length > best_length){
                        best_seg = j;
                        best_length = seg.length;
                    }
                }

            }

        }
        if(best_seg != -1)
            found_segs.push_back(best_seg);

    }
    return found_segs;
}

vector<int> BoxDetection::search_box_segments(box_t &box_res, const box_t &box_exp, double plate_size[2], double face_size[2], double margin, const vector<line_seg> &segments){

    Vec2d dir = {cos(box_exp.theta),sin(box_exp.theta)};
    vector<loose_seg> dir_segs = search_segments_along_dir(box_exp.center, dir, plate_size[0]/2, face_size[0]/2, margin, segments);
    Vec2d dir_rot = {-dir[1],dir[0]};
    vector<loose_seg> dir_rot_segs = search_segments_along_dir(box_exp.center, dir_rot, plate_size[1]/2, face_size[1]/2, margin, segments);

    for(loose_seg &ls : dir_segs) ls.orthogonal = false;
    for(loose_seg &ls : dir_rot_segs) ls.orthogonal = true;

    vector<loose_seg> segs;

    segs.reserve( dir_segs.size() + dir_rot_segs.size() );
    segs.insert( segs.end(), dir_segs.begin(), dir_segs.end() );
    segs.insert( segs.end(), dir_rot_segs.begin(), dir_rot_segs.end() );

    vector<int> found_segs;
    found_segs.reserve(segs.size());

    double ang_sum=0;
    double scale_sum=0;
    for (const loose_seg &ls : segs)
    {
        ang_sum += ls.ang_dif;
        scale_sum += ls.dist / ls.exp_dist;
        found_segs.push_back(ls.seg_i);
    }
    const double avg_ang_diff = ang_sum/segs.size();
    const double new_scale = scale_sum/segs.size();

    int count[2] = {0,0};
    double dist_sum[2] = {0,0};
    for (const loose_seg &ls : segs)
    {
        dist_sum[ls.orthogonal] -= ls.dist - new_scale*ls.exp_dist;
        count[ls.orthogonal]++;
    }

    double move[2] = {0,0};
    if(count[0] > 1) move[0] = dist_sum[0]/count[0];
    if(count[1] > 1) move[1] = dist_sum[1]/count[1];

    box_res.center = box_exp.center + Point2i(dir*move[0]) + Point2i(dir_rot*move[1]);
    box_res.theta = box_exp.theta + avg_ang_diff;
    box_res.scale = new_scale;

    return found_segs;
}

vector<BoxDetection::loose_seg> BoxDetection::search_segments_along_dir(Point2i center, Vec2d dir, double dist1, double dist2, double margin, const vector<line_seg> &segments){
    vector<loose_seg> result;
    result.reserve(4);

    for (int i = 0; i < 4; i++)
    {
        loose_seg res;
        bool bit1 = (i & 1) == 0;
        bool bit2 = (i & 2) == 0;
        res = search_segment(center, (bit2 ? dir : -dir), (bit1 ? dist1 : dist2), margin, bit1, segments);
        if(res.seg_i != -1){
            if(!bit2){
                res.dist = -res.dist;
                res.exp_dist = -res.exp_dist;
            }

            bool dont_add = false;
            for(auto ls = result.begin(); ls != result.end(); ls++){
                if(ls->seg_i == res.seg_i){
                    if(fabs(ls->dist-ls->exp_dist) < fabs(res.dist-res.exp_dist)){
                        // Segment already added and existing is better match
                        dont_add = true;
                    }else{
                        result.erase(ls);
                    }
                    break;
                }
            }
            if(dont_add) continue;
            result.push_back(res);
        }

    }
    return result;
}

BoxDetection::loose_seg BoxDetection::search_segment(Point2i center, Vec2d dir, double exp_dist, double margin, bool choose_closest, const vector<line_seg> &segments){

    loose_seg result;
    for (int i = 0; i < segments.size(); i++)
    {
        const line_seg &seg = segments[i];
        const double dotp = dir[0]*seg.line[0] + dir[1]*seg.line[1];
        if(fabs(dotp) < 0.95) continue; // TODO set threshold

        const double signed_dist = line_point_dot(seg.line, center);
        if(signed_dist>0 != dotp>0) continue; // Segment is on opposite side of center

        const double abs_dist = fabs(signed_dist);

        const double dist_dif = abs_dist - exp_dist;
        if(fabs(dist_dif) > margin) continue;

        if(result.seg_i == -1 || (choose_closest && abs_dist<fabs(result.dist)) || (!choose_closest && abs_dist>fabs(result.dist))){  // fabs not necesarry since signed_dist > 0 TODO delete
            result.seg_i = i;
            result.dist = abs_dist;
            result.exp_dist = exp_dist;
            result.ang_dif = asin(dir[0]*seg.line[1] - dir[1]*seg.line[0]);
        }

    }
    return result;
}