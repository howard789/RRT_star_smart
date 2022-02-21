# RRT_star_smart
robot path planning

RRT*-Smart has made improvements on the basis of RRT*, mainly optimizing the path. The path explored through RRT and 
RRT* is often tortuous and slightly wavy (after all, the nodes are randomly generated), but in fact, the best path in 
the open space is generally a straight line. RRT*-Smart is completely consistent with RRT * in the previous stage of 
operation, but after finding a feasible path from the starting point to the end point, it starts to consider optimizing
 the path and turning the curve into straight. This process actually starts from the leaf node and constantly looks for 
 whether it can be directly connected to the predecessor node without obstacles. If you directly connect one layer forward, 
 there will be more straight lines and less curves. (in order to increase the calculation speed, we might as well 
 directly treat the obstacles as rectangular.) in the process of turning the curve, we can find several anchor points, 
 which are often near the obstacles, and they can't help our descendants optimize directly.
 
 reference:
 https://zhuanlan.zhihu.com/p/161829703
 https://github.com/adnanmunawar/matlab-rrt-variants