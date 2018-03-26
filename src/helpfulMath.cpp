#include "helpfulMath.hpp"
/**
 * Returns the distance between two points
 * @param  point1 first point
 * @param  point2 second point
 * @return        distance between points
 */
double calcDistance(sejong::Vect3 point1, sejong::Vect3 point2) {
  double distance1 = pow((point1[0] - point2[0]), 2);
  double distance2 = pow((point1[1] - point2[1]), 2);
  double distance3 = pow((point1[2] - point2[2]), 2);

  return sqrt(distance1 + distance2 + distance3);
}

/**
 * Calculate the midpoint between two points
 * @param  point1 first point
 * @param  point2 second point
 * @return        midpoint of the two points
 */
sejong::Vect3 calcMidpoint(sejong::Vect3 point1, sejong::Vect3 point2) {
  return sejong::Vect3((point1[0] + point2[0]) / 2, (point1[1] + point2[1]) / 2, (point1[2] + point2[2]) / 2);
}
