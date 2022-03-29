#ifndef TYPE_DEFINES_HPP
#define TYPE_DEFINES_HPP

// enum CamLocType {
//     NONE,
//     ARUCO,
//     RED_BOX,
//     BALL,
//     FIND_BOX,
//     UPDATE_BOX,
//     FOLLOW_MARKER,
//     J_FIND_BOX,
//     FIND_BALLOON,
//     LAST_LOC_TYPE
// };

enum CamLocType
{
    NONE,
    ARUCO,
    RED_BOX,
    BALL,
    FIND_BOX,
    UPDATE_BOX,
    FIND_WALL,
    FOLLOW_MARKER,
    J_FIND_BOX,
    FIND_BALLOON,
    DRAW_CROSS,
    FIND_NAVAL_BOX,
    LAST_LOC_TYPE
};

struct pose_t
{
    bool valid;
    double x;
    double y;
    double z;
    double th;
    long num_pose;
};

struct target_pos
{
    float x;
    float y;
    float z;
    float th;
};

struct state_t
{
    float xacc;
    float xvel;
    float xpos;
    float yacc;
    float yvel;
    float ypos;
    float head;
    float zvel;
    float zpos;

    float &operator[](size_t idx)
    {
        switch (idx)
        {
        case 0:
            return xacc;
        case 1:
            return xvel;
        case 2:
            return xpos;
        case 3:
            return yacc;
        case 4:
            return yvel;
        case 5:
            return ypos;
        case 6:
            return zvel;
        case 7:
            return zpos;
        case 8:
            return head;
        default:
            return xacc; // Need to throw a pointer to a double
        };
    };
};

#endif
