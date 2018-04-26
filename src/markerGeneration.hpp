#ifndef MARKERGENERATION_HPP
#define MARKERGENERATION_HPP


visualization_msgs::Marker createMarker(double x_pos, double y_pos, double z_pos);

visualization_msgs::Marker createMarker(sejong::Vect3 pos);

visualization_msgs::Marker createCylinder(double radius, double height, int markerID);

visualization_msgs::Marker createBox(double x_size, double y_size, double z_size, int markerID);
#endif
