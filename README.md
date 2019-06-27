# rrt-rrt_star-dubins
Python project to evaluate performance gains of path planning with RRT* on linear and dubins vehicles.  

To create visualizations for RRT:
./rrt-validator python planner.py < space-1.sw
./rrt-validator python dubins_planner.py < space-1.sw
For RRT*, follow the script with 'rrt_star' and a radius of rewiring:
./rrt-validator python planner.py rrt_star 2 < space-1.sw
./rrt-validator python dubins_planner.py rrt_star 2 < space-1.sw

FLANN and dubins library will likely have to be installed. 
