#include "solution_interface.hpp"
#include <cmath>

using namespace std;

// ============================================================================
// This is the only file you should need to edit.
// Read INSTRUCTIONS.md before you start.
// ============================================================================

//The two hard rules
const double MAX_STEP_DISTANCE = 10.0; // metres
const double CONE_HALF_ANGLE = 30.0; // degrees either side of heading

//~value of Pi for conversions later
const double PI = 3.14159265358979323846;

vector<Point> buildPath(const vector<Point>& candidatePoints, const CarState& car) {
    vector<Point> path;

    //Now I know which points I've already been to
    //NOT a constructor
    vector<bool> visited(candidatePoints.size(), false);

    //Our location + which way we're facing
    double current_x = car.x;
    double current_y = car.y;
    double heading = car.heading;

    //Checking if a point is inside the cone
    double minCosine = cos(CONE_HALF_ANGLE * PI / 180.0);

    while (true) {
        //Unit vector pointing the way we are facing
        double forward_x = cos(heading);
        double forward_y = sin(heading);

        //Look through every point & rmr the closest valid one
        int bestIndex = -1; //since its an index, -1 means no point found
        double bestDistance = 0.0;

        //turns it into an int (java type casting?)
        for (int i = 0; i < (int)candidatePoints.size(); i++) {
            if (visited[i]) {
                //continue skips it
                continue;
            }

            //distance to points
            double dx = candidatePoints[i].x - current_x;
            double dy = candidatePoints[i].y - current_y;
            double distance = sqrt(dx * dx + dy * dy);

            //Edge case: skip a point sitting right on top of us
            if (distance < 0.000001) {
                continue;
            }

            //Checking rule 1
            if (distance > MAX_STEP_DISTANCE) {
                continue;
            }

            //Checking rule 2
            double cosine = (dx * forward_x + dy * forward_y) / distance;
            if (cosine < minCosine) {
                continue;
            }

            //If both rules pass, keep point if it's the closest one we've seen so far 
            if (bestIndex == -1 || distance < bestDistance) {
                bestIndex = i;
                bestDistance = distance;
            }
        }

        //No valid point left ahead of us anymore
        if (bestIndex == -1) {
            break;
        }

        //Move to the chosen point; new heading = direction we just travelled
        Point next = candidatePoints[bestIndex];
        //use arctan function
        heading = atan2(next.y - current_y, next.x - current_x);
        current_x = next.x;
        current_y = next.y;

        visited[bestIndex] = true;
        path.push_back(next);
    }

    return path;
}
