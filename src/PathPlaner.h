
/** @file PathPlaner.h
 *  @author Andrey N. Zabegaev <speench@gmail.com>
 */

#ifndef _PathPlaner_h_
#define _PathPlaner_h_

#include <vector>

class PathPlaner
{
public:
    
    /** @brief Constructor
     *  @param[in] mapS map control points frenet s coordinates
     *  @param[in] mapX map control points decart x coordinates
     *  @param[in] mapY map control points decart y coordinates
     */
    PathPlaner(std::vector<double> mapS, std::vector<double> mapX, std::vector<double> mapY);
    
    /** @brief Destructor
     */
    ~PathPlaner();
    
    /** @breif Run one iteration of path planner
     *  @param[in] x0 x car coordinate
     *  @param[in] y0 y car coordinate
     *  @param[in] s0 s car coordinate
     *  @param[in] endS s car coordinate of the last point in path
     *  @param[in] d0 d car coordinate
     *  @param[in] endD d car coordinate of the last point in path
     *  @param[in] yaw car heading angle
     *  @param[in] previous_path_x x coordinates of last car path
     *  @param[in] previous_path_y y coordinates of last car path
     *  @param[in] otherCars vector with data about near cars
     */
    void Iterate(double x0, double y0, double s0, double endS, double d0, double endD, double yaw, std::vector<double> previous_path_x, std::vector<double> previous_path_y, std::vector<std::vector<double>> otherCars);
    
    /** @brief Get x coordinates of generated car path
     *  @return vector with x coordinates
     */
    std::vector<double> &NextX();
    
    /** @brief Get y coordinates of generated car path
     *  @return vector with y coordinates
     */
    std::vector<double> &NextY();
    
protected:
    
    /** @brief Analize nearby trafic and make a dicision of the best line
     *  @param[in] otherCars vector with data about near cars
     *  @param[in] carS our car s coordinate
     *  @param[in] carL our car line
     *  @param[in] yaw our car heading angle
     *  @param[in] predictionLen number of prediction steps we work on
     *  @param[out] desiredV desired velocity in choosen line
     *  @return best line for current trafic conditions
     */
    int AnalizeTrafic(std::vector<std::vector<double>> otherCars, double carS, int carL, double yaw, int predictionLen, double &desiredV);
    
    /** @brief Calculate distance between two points
     *  @param[in] x1 point 1 x coordinate
     *  @param[in] y1 point 1 y coordinate
     *  @param[in] x2 point 2 x coordinate
     *  @param[in] y2 point 2 y coordinate
     *  @return distance between points 1 and 2
     */
    double distance(double x1, double y1, double x2, double y2);
    
    /** @brief Find map control point closeset to the given position
     *  @param[in] x target position x coordinate
     *  @param[in] y target position y coordinate
     *  @return map control point number
     */
    int ClosestWaypoint(double x, double y);
    
    /** @brief Convert Frenet coordinates to Decart
     *  @param[in] s Frenet s coordinate
     *  @param[in] d Frenet d coordinate
     *  @return vector {x,y} with point decart coordinates
     */
    std::vector<double> getXY(double s, double d);
    
    std::vector<double>			_nextX;				//!< vector with generated path x coordinates
    std::vector<double>			_nextY;				//!< vector with generated path y coordinates
    
    std::vector<double>			_mapS;				//!< map control points frenet s coordinates
    std::vector<double>			_mapX;				//!< map control points decart x coordinates
    std::vector<double>			_mapY;				//!< map control points decart y coordinates
//     std::vector<double>			_mapDX;
//     std::vector<double>			_mapDY;
    
    double						_predictSpacing;		//!< prediction control points spacing
    double						_predictDist;			//!< prediction control point max distance
    double						_laneWidth;			//!< lane line width
    double						_limitV;				//!< maximum velocity
    double						_V;					//!< current velocity
    double						_maxAcc;			//!< maximum acceleration
    double						_maxS;				//!< maximum value of frenet s coordinate
    
};

#endif /* _PathPLaner_h_ */
