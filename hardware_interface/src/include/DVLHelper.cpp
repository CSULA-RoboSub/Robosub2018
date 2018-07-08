#include "DVLHelper.h"

//positive is right negative is left
float DVLHelper::getDirection(float x1, float y1, float x2, float y2, float yaw){
    float direction = 0;
    float heading;

    //convert to heading
    if (90 <= yaw && yaw <= 180)
        heading = yaw - 90;
    else
        heading = yaw + 270;
    
    float directionDegree = std::atan2(y2-y1, x2-x1) * (180/PI);
    if (directionDegree >= 0)
        directionDegree = 180 - directionDegree;
    else
        directionDegree = -180 - directionDegree;
        
    float dvl_yaw;
    if (90 <= directionDegree && directionDegree <= 180)
        dvl_yaw = directionDegree - 90;
    else
        dvl_yaw = directionDegree + 270;


    float yaw_diff = dvl_yaw - heading;
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

float DVLHelper::getDistance(float x1, float y1, float x2, float y2){
    float distance = 0;
    
    float l1 = std::max(x1, x2) - std::min(x1, x2);
    float l2 = std::max(y1, y2) - std::min(y1, y2);
    distance = std::sqrt(l1*l1 + l2*l2);

    return distance;
}
