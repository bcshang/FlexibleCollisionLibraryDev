## Insert Interesting Header


## How to use FCL!
### Creating an object to collide into
1. Create a Shape using their constructors look in (fcl/include/fcl/shape/) and include like "fcl/geometry/shape/whatever_shape.h"

2. Create a CollisionGeometry from the shape
```cpp
std::shared_ptr<fcl::CollisionGeometry<double>> insertName(Shape);
```
(Optional, more important) Add user data to the geometry with setUserData()

3. Make a transform with Translation/Rotation parameter for that Shape
[Transform Reference from UNC](http://gamma.cs.unc.edu/FCL/fcl_docs/webpage/generated/classfcl_1_1Transform3f.html)

4. Create a collision object
```cpp
fcl::CollisionObject<double>* insertName = new fcl::CollisionObject<double>(GeometryName, TransformName);
```
(Optional, less important) Add user data to the object with setUserData()



###Creating the robot to collide into

[Collision Manager Reference](http://gamma.cs.unc.edu/FCL/fcl_docs/webpage/generated/classfcl_1_1BroadPhaseCollisionManager.html)
1. Do the above steps to create collision objects for each robot link
2. Put all collision objects in a vector
3. Declare a collision manager (I chose to go with sweep and prune somewhat arbitrarily)
4. register objects in that vector
5. setup

*Looping now where the robot model is updating*
*You can do multiple things from here depending on how you setup your objects*

**slow version where you recreated every object every iteration**
6. clear the model
7. register objects again
8. update()

**fast version that is probably wrong?**
6. update(collision objects vector)????????????





### Notes about my wrappers

