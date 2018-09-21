
/** @file PathPlaner.cpp
 *  @author Andrey N. Zabegaev <speench@gmail.com>
 */

#include <math.h>
#include <iostream>
#include "spline.h"
#include "PathPlaner.h"

PathPlaner::PathPlaner(std::vector<double> mapS, std::vector<double> mapX, std::vector<double> mapY/*, std::vector<double> mapDX, std::vector<double> mapDY*/) :
    _mapS(mapS), _mapX(mapX), _mapY(mapY)/*, _mapDX(mapDX), _mapDY(mapDY)*/
{
    _predictSpacing = 30.;
    _predictDist = 91.;
    _laneWidth = 4.;
    _limitV = 49. / 2.24; //limit is in m/s
    _V = 0; //start velocity
    _maxAcc = 0.224;
    _maxS = 6945.554;
}

PathPlaner::~PathPlaner()
{
}

void PathPlaner::Iterate(double x0, double y0, double s0, double endS, double d0, double endD, double yaw, std::vector<double> previous_path_x, std::vector<double> previous_path_y,  std::vector<std::vector<double>> otherCars)
{
    yaw = yaw/180.*M_PI;
    
    //selsect work Frenet coordinates to be the last one in already use path, otherwise use current coordinates
    int prevSize = previous_path_x.size();
    double s = s0;
    double d = d0;
    if (prevSize > 0) {
	s = endS;
	d = endD;
    }
    
    int l = round(d/_laneWidth - 0.5);
    
    std::vector<double> refPointsX;
    std::vector<double> refPointsY;
    double refX;
    double refY;
    double refYaw;
    double refXPrev;
    double refYPrev;
    // If previos path is too small we use current coordinate and angle
    if (prevSize < 2) {
	refX = x0;
	refY = y0;
	refYaw = yaw;
	refXPrev = x0 - cos(yaw);
	refYPrev = y0 - sin(yaw);
    }
    else {
	refX = previous_path_x[prevSize-1];
	refY = previous_path_y[prevSize-1];
	refXPrev = previous_path_x[prevSize-2];
	refYPrev = previous_path_y[prevSize-2];
	refYaw = atan2(refY-refYPrev, refX-refXPrev);
    }
    refPointsX.push_back(refXPrev);
    refPointsX.push_back(refX);
    refPointsY.push_back(refYPrev);
    refPointsY.push_back(refY);
    
    //Analize nearby trafic, choose best line and velocity
    double desiredV;
    int line = this->AnalizeTrafic(otherCars, s, l, refYaw, prevSize, desiredV);
    
    //Generate few waypoints in the shoosen line
    for (double i=_predictSpacing; i<_predictDist; i+=_predictSpacing) {
	std::vector<double> nextWP = this->getXY(s + i, _laneWidth*(0.5+line));
	refPointsX.push_back(nextWP[0]);
	refPointsY.push_back(nextWP[1]);
    }
    
    //Convert waypoints to the car coordinate system
    std::vector<double> refXCar;
    std::vector<double> refYCar;
    for (unsigned int i=0; i<refPointsX.size(); ++i) {
	double shiftX = refPointsX[i] - refX;
	double shiftY = refPointsY[i] - refY;
	refXCar.push_back(shiftX*cos(-refYaw) - shiftY*sin(-refYaw));
	refYCar.push_back(shiftX*sin(-refYaw) + shiftY*cos(-refYaw));
    }
    
    // Set spline for control points generation
    tk::spline spl;
    spl.set_points(refXCar, refYCar);
    
    //Clear control points vectors
    _nextX.erase(_nextX.begin(), _nextX.end());
    _nextY.erase(_nextY.begin(), _nextY.end());
    
    //Add unused part of previouse path
    for (unsigned int i=0; i<prevSize; ++i) {
	_nextX.push_back(previous_path_x[i]);
	_nextY.push_back(previous_path_y[i]);
    }
    
    //speed control
    if (_V > desiredV)
	_V -= _maxAcc;
    else
	_V += _maxAcc;
    
    //Generate control points based on spline and convert them to local coordinate system
    double targetXCar = _predictSpacing;
    double targetYCar =spl(targetXCar);
    double targetDist = sqrt(targetXCar*targetXCar + targetYCar*targetYCar);
    double refVXSpacing = targetXCar * (0.02*_V) / targetDist;
    for (unsigned int i=1; i<=50-prevSize; ++i) {
	double xCar = refVXSpacing*i;
	double yCar = spl(xCar);
	double x = xCar*cos(refYaw) - yCar*sin(refYaw) + refX;
	double y = xCar*sin(refYaw) + yCar*cos(refYaw) + refY;
	_nextX.push_back(x);
	_nextY.push_back(y);
    }
    
    return;
}

std::vector<double> & PathPlaner::NextX()
{
    return _nextX;
}

std::vector<double> & PathPlaner::NextY()
{
    return _nextY;
}

int PathPlaner::AnalizeTrafic(std::vector<std::vector<double> > otherCars, double carS, int carL, double yaw, int predictionLen, double &desiredV)
{
    //This is protection for crossin any s-coordinate 0 value
    bool somebodyInLastQuarter = false;
    if (carS > _maxS * 3. / 4.)
	somebodyInLastQuarter = true;
    for (auto otherCar : otherCars)
	if (otherCar[5] > _maxS * 3. / 4.)
	    somebodyInLastQuarter = true;
    bool somebodyInFirstQuarter = false;
    if (carS < _maxS / 4.)
	somebodyInFirstQuarter = true;
    for (auto otherCar : otherCars)
	if (otherCar[5] < _maxS / 4.)
	    somebodyInFirstQuarter = true;
    
    if ((carS < _maxS / 4.) && somebodyInLastQuarter)
	carS += _maxS;
    
    //vector with desired speeds for all lines
    std::vector<double> lineDesiredV(3, _limitV);
    
    //vector with set of waights for all lines
    std::vector<std::vector<double>> lineWeights(3, std::vector<double>(4, 0));
    //set weight of middle line a little lower
    lineWeights[0][1] += 1;
    lineWeights[2][1] += 1;
    
    std::cout <<std::endl << "I " << carS << " " << carS << " " << carL << std::endl;;
    
    bool lookForLineChange = false;
    
    for (auto otherCar : otherCars) {
	int otherCarId = otherCar[0];
	double otherCarX = otherCar[1];
	double otherCarY = otherCar[2];
	double otherCarVX = otherCar[3];
	double otherCarVY = otherCar[4];
	double otherCarS = otherCar[5];
	double otherCarD = otherCar[6];
	
	//This is protection for crossin any s-coordinate 0 value
	if (somebodyInFirstQuarter && somebodyInLastQuarter && (otherCarS < _maxS/4.))
	    otherCarS += _maxS;
	
	double otherCarV = sqrt(otherCarVX*otherCarVX+otherCarVY*otherCarVY);
// 	double otherCarVS = otherCarVX*cos(yaw) - otherCarVY*sin(yaw);
	double otherCarVS = otherCarV;
	double otherCarVD = otherCarVX*sin(yaw) + otherCarVY*cos(yaw);
	
	otherCarS += otherCarVS*0.02*predictionLen;
// 	otherCarD += otherCarVD*0.02*predictionLen;
	
	//get car line
	int otherCarL = round(otherCarD/_laneWidth - 0.5);
	std::cout <<otherCarId  << " " << otherCarD << " " << otherCarS << " " << otherCarL << std::endl;;
	if ((otherCarL < 0) || (otherCarL>2))
	    continue;
	
	// if other car is close to us it makes high weight for this car line
	if ( ((otherCarS - carS) < 15.) && ((otherCarS - carS) > -10.) ) {
	    if ( (lineWeights[otherCarL][3]) < (1./fabs(otherCarS - carS)) ) {
		lineWeights[otherCarL][3] = 1./fabs(otherCarS - carS);
		lineDesiredV[otherCarL] = 0.9*otherCarV;
	    }
	}
	
	// if other car is infront of us, but not too close
	if (((otherCarS - carS) < 60.) && ((otherCarS - carS) >= 15.)) {
	    double k = (otherCarS - carS - 15)/(60-15);
	    lineWeights[otherCarL][2] = std::max(lineWeights[otherCarL][2], 1-(otherCarS - carS - 15)/(50-15));
	    lineDesiredV[otherCarL] = std::min(lineDesiredV[otherCarL], otherCarV + (_limitV-otherCarV)*k);
	}
	
	// this value shows if it is time for look for line change
	if (((otherCarS - carS) < 40.) && ((otherCarS - carS) > 0.) ) {
	    lookForLineChange = true;
	}

    }
    
    // calculate total line weight
    for (unsigned int l=0; l<lineWeights.size(); ++l) {
	int d = 1;
	for (unsigned int i=1; i<lineWeights[0].size(); ++i) {
	    if ((i==2) && (!lookForLineChange))
		continue;
	    lineWeights[l][0] += lineWeights[l][i]*d;
	    d *= 100;
	}
    }
    
    double lowestWeight = 10000000;
    int desL = -1;
    int minL = std::max(0, carL-1);
    int maxL = std::min(2, carL+1);
    
    // choose line
    for (unsigned int l=minL; l<=maxL; ++l) {
	if (lineWeights[l][0] < lowestWeight) {
	    lowestWeight = lineWeights[l][0];
	    desL = l;
	}
    }
//     std::vector<double>::iterator result = std::min_element(lineWeights.begin()+minL, lineWeights.begin()+maxL+1);
//     int desL = std::distance(lineWeights.begin(), result);
    
//     std::cout << lineWeights.size() << " " << desL << std::endl;
    for (auto &l : lineWeights) {
	for (auto &w : l) {
	    std::cout << w << " ";
	}
	std::cout << std::endl;
    }
    
    desiredV = lineDesiredV[desL];
    return desL;
}

double PathPlaner::distance(double x1, double y1, double x2, double y2)
{
    return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

int PathPlaner::ClosestWaypoint(double x, double y)
{
    
    double closestLen = 100000; //large number
    int closestWaypoint = 0;
	
    for(int i = 0; i < _mapX.size(); i++)
    {
	double map_x = _mapX[i];
	double map_y = _mapY[i];
	double dist = distance(x,y,map_x,map_y);
	if(dist < closestLen)
	{
	    closestLen = dist;
	    closestWaypoint = i;
	}
    }
    
    return closestWaypoint;
}


std::vector<double> PathPlaner::getXY(double s, double d)
{
    int prev_wp = -1;
    
    while(s > _mapS[prev_wp+1] && (prev_wp < (int)(_mapS.size()-1) ))
	prev_wp++;
    
    int wp1 = (prev_wp+_mapX.size())%_mapX.size();
    int wp2 = (prev_wp+_mapX.size()+1)%_mapX.size();
    
    double heading = atan2((_mapY[wp2]-_mapY[wp1]),(_mapX[wp2]-_mapX[wp1]));
    // the x,y,s along the segment
    double seg_s = (s-_mapS[prev_wp]);
    double seg_x = _mapX[prev_wp]+seg_s*cos(heading);
    double seg_y = _mapY[prev_wp]+seg_s*sin(heading);
    
    double perp_heading = heading-M_PI/2;

    double x = seg_x + d*cos(perp_heading);
    double y = seg_y + d*sin(perp_heading);

    return {x,y};
}
