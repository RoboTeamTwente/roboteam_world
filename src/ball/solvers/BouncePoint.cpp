//
// Created by rolf on 10-02-20.
//

#include "BouncePoint.h"
#include <iostream>
std::optional<BouncePoint> calculateBouncePoint(Eigen::Vector3d startPoint, Eigen::Vector3d startVel, double gravity,
        double ballRadius) {
    std::optional<double> bounceTime=calculateBounceTime(startPoint.z()-ballRadius,startVel.z(),gravity);
    if (!bounceTime){
        return std::nullopt;
    }
    Eigen::Vector3d bouncePos=startPoint+(*bounceTime) * startVel;
    bouncePos.z() = ballRadius;
    BouncePoint bouncePoint(bouncePos,*bounceTime);
    return bouncePoint;
}
std::optional<double> calculateBounceTime(double offset, double startVel, double gravity) {
    //We solve (startPos- ballRadius )+startVel*t-0.5 gravity*t^2 = 0for t, taking the highest root we can find.
    std::cout<<offset<<" "<<startVel<< " "<<gravity<<std::endl;
    double D = startVel*startVel + 2*offset*gravity;
    if (D > 0) {
        // two roots: we return the largest one in time always!
        return ( startVel + sqrt(D))/( gravity);
    }
    //for both other root cases we return nullopt: they don't make physical sense to be hit
    return std::nullopt;
}