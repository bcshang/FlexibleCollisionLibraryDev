#ifndef MARKERGENERATION_HPP
#define MARKERGENERATION_HPP
double calcDistance(sejong::Vect3 point1, sejong::Vect3 point2);

sejong::Vect3 calcMidpoint(sejong::Vect3 point1, sejong::Vect3 point2);

visualization_msgs::Marker createMarker(double x_pos, double y_pos, double z_pos);

visualization_msgs::Marker createMarker(sejong::Vect3 pos);

visualization_msgs::Marker createCylinder(sejong::Vect3 pos1, sejong::Vect3 pos2);

#endif
