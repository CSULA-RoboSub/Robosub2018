#include "DVLHelper.h"

//positive is right negative is left
double DVLHelper::getDirection(double x1, double y1, double x2, double y2, double yaw){
    double direction = 0;
    double heading;

    //convert to heading
    if (90 <= yaw && yaw <= 180)
        heading = yaw - 90;
    else
        heading = yaw + 270;

    double directionDegree = rad2deg(std::atan2(y2-y1, x2-x1));
    if (directionDegree >= 0)
        directionDegree = 180 - directionDegree;
    else
        directionDegree = -180 - directionDegree;

    double dvl_yaw;
    if (90 <= directionDegree && directionDegree <= 180)
        dvl_yaw = directionDegree - 90;
    else
        dvl_yaw = directionDegree + 270;


    double yaw_diff = dvl_yaw - heading;
    if (yaw_diff > 0){
        if (yaw_diff > 180)
            // if yaw_diff is greater than 180 then rotation is left and 360-yaw_diff degrees
            direction = -(360 - yaw_diff);
        else
            direction = yaw_diff;
            // direction = 'right'
    }
    else if (yaw_diff < 0){
        if (yaw_diff < -180)
            direction = 360 + yaw_diff;
            // direction = 'right'
        else
            direction = yaw_diff;
            // direction = 'left'
    }
    else{
        direction = 0;
        // direction = 'in front'
    }

    return direction;
}

double DVLHelper::getDistance(double x1, double y1, double x2, double y2){
    double distance = 0;

    double l1 = std::max(x1, x2) - std::min(x1, x2);
    double l2 = std::max(y1, y2) - std::min(y1, y2);
    distance = std::sqrt(l1*l1 + l2*l2);

    return distance;
}
