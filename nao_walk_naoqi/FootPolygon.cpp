
#include "FootPolygon.h"
        FootPolygon::FootPolygon(RobotParameters& robot_):robot(robot_)
        {
            footLF.setZero();
            footRF.setZero();
            footLH.setZero();
            footRH.setZero();
            footA.setZero();
            polygon.setZero();
            stepXF = robot.getWalkParameter(StepXF);
            stepXH = robot.getWalkParameter(StepXH);
            stepYL = robot.getWalkParameter(StepYL);
            stepYR = robot.getWalkParameter(StepYR);
        }
        void FootPolygon::setPolygon(Vector3f footA_, Affine3f T)
        {
            

            //Set Ankle 3D position w.r.t. Inertial frame of Reference
            footA = footA_;
            //Transform to Inertial frame of Reference
            footLF =   T.linear() * Vector3f(stepXF,stepYL,0.0) + footA;
            footLH =   T.linear() * Vector3f(stepXH,stepYL,0.0) + footA;
            footRF =   T.linear() * Vector3f(stepXF,stepYR,0.0) + footA;
            footRH =   T.linear() * Vector3f(stepXH,stepYR,0.0) + footA;

            polygon(0,0) = footLF(0);
            polygon(0,1) = footLF(1);
            polygon(1,0) = footLH(0);
            polygon(1,1) = footLH(1);
            polygon(2,0) = footRH(0);
            polygon(2,1) = footRH(1);
            polygon(3,0) = footRF(0);
            polygon(3,1) = footRF(1);

          

            numNodes = 4;

        }

          
           // Returns true if the point p lies inside the polygon[] with n vertices 
        bool FootPolygon::checkIn(float x_, float y_)
        {
        // Check if point lies on same side of each segment.  We will check each
        // segment in order.
        // Assumption: vertices are given in either clockwise or
        // counterclockwise manner.
        Vector3f current_poly;
        int current_sign;
        int check_sign;
        bool insidePoly=true;
        for(int k = 0; k<numNodes; k++)
        {
            
            // determine vectors for each segment around boundary
            // starting between p2-p1, p3-p2, etc.        
            // if at the final node, the vector connects back to p1
            if(k==numNodes-1)
            {
                current_poly = Vector3f(polygon(0,0) - polygon(k,0), polygon(0,1) - polygon(k,1), 0.0);
            }
            else
            {
                current_poly = Vector3f(polygon(k+1,0) - polygon(k,0), polygon(k+1,1) - polygon(k,1), 0.0);
            }

            //vector from point in space vertex on polygon (point-p(k))
            Vector3f point_vect = Vector3f(x_ - polygon(k,0), y_ - polygon(k,1), 0.0);

            // determine if the point is inside or outside the polygon:
            // if cross product of all polygon_vectors x point_vectors is
            // negative then the point is inside (if polygon is convex) 
            // take cross_product of point vector and polygon vector       
            Vector3f c = current_poly.cross(point_vect);
            if(c(2)>0)
            {
                current_sign = 1;
            }
            else
            {
                current_sign = -1;
            }
            
            if(k == 0)
                check_sign = current_sign;
            else if(check_sign != current_sign)
                insidePoly = false;

        }
                    return insidePoly;

        }
        bool FootPolygon::pnpoly(float x, float y)
        {
            // If we never cross any lines we're inside.
            bool inside = false;

            // Loop through all the edges.
            for (int i = 0; i < polygon.rows(); ++i)
            {
                // i is the index of the first vertex, j is the next one.
                // The original code uses a too-clever trick for this.
                int j = (i + 1) % polygon.rows();

                // The vertices of the edge we are checking.
                double xp0 = polygon(i, 0);
                double yp0 = polygon(i, 1);
                double xp1 = polygon(j, 0);
                double yp1 = polygon(j, 1);

                // Check whether the edge intersects a line from (-inf,y) to (x,y).

                // First check if the line crosses the horizontal line at y in either direction.
                if ( ((yp0 <= y) && (yp1 > y)) || ((yp1 <= y) && (yp0 > y)))
                {
                    // If so, get the point where it crosses that line. This is a simple solution
                    // to a linear equation. Note that we can't get a division by zero here -
                    // if yp1 == yp0 then the above if be false.
                    double cross = (xp1 - xp0) * (y - yp0) / (yp1 - yp0) + xp0;

                    // Finally check if it crosses to the left of our test point. You could equally
                    // do right and it should give the same result.
                    if (cross < x)
                        inside = !inside;
                }
            }
            return inside;
        }