# How to use FCL!

**Creating an object to collide into**
1. Create a Shape using their constructors look in (fcl/include/fcl/shape/) and include like "fcl/geometry/shape/whatever_shape.h"
2a. Create a CollisionGeometry from the shape
```cpp
std::shared_ptr<fcl::CollisionGeometry<double>> insertName(Shape);
```
2b. Add user data to the geometry (more important) with setUserData()
3. Make a transform with Translation/Rotation parameter for that Shape
[Transform Reference from UNC](http://gamma.cs.unc.edu/FCL/fcl_docs/webpage/generated/classfcl_1_1Transform3f.html)
4a. Create a collision object
```cpp
fcl::CollisionObject<double>* insertName = new fcl::CollisionObject<double>(GeometryName, TransformName);
```
4b. Add user data to the object (less important) with setUserData()

