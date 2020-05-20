#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <iostream>

#include <vector>
#include <array>
#include "include/csv.h"
#include "constgps.h"
#include "PathPlanning.cc"

int loadSplineWaypoints(const std::string& wayPointsFilename, std::vector<std::array<double, 2>>& waypoints)
{
    int ret = 0;

    try
    {
        io::CSVReader<2> in(wayPointsFilename);
        in.read_header(io::ignore_extra_column, "x", "y");

        double x;
        double y;

        while(in.read_row(x, y)){
            std::array<double, 2> row = {x, y};
            waypoints.emplace_back(row);
        }
    }
    catch (const std::exception& e)
    { // caught by reference to base
        std::cout << " a standard exception was caught, with message '"<< e.what() << "'\n";
        ret = -1;
    }

#ifdef DEBUG_LOADED_WAYPOINTS
    std::cerr << "Loaded waypoints:" << std::endl;

    for(unsigned int i = 0; i < waypoints.size(); i++)
        std::cerr << i << "x=" << waypoints[i][0] << " y=" << waypoints[i][1] << std::endl;
#endif


    
    return ret;
}


void set_spline_waypoints(const std::vector<std::array<double, 2>>& spline_vector)
{
    std::vector<std::array<double, 2>> sw; ///> spline waypoints
    sw = spline_vector;
    
    // Adding phantom vertices according to: Bartels, Introduction to spline, 1987, p.43    
    std::array<double, 2> startrow = {2*sw[0][0]-sw[1][0], 2*sw[0][1]-sw[1][1]};    
    std::array<double, 2> endrow = {2*sw[sw.size()-1][0]-sw[sw.size()-2][0], 2*sw[sw.size()-1][1]-sw[sw.size()-2][1]};

    sw.insert(sw.begin(), startrow);
    sw.emplace_back(endrow);           
    //        
    
#ifdef DEBUG_LOADED_WAYPOINTS
    std::cerr << "Combined waypoints:" << std::endl;

    for(unsigned int i = 0; i < sw.size(); i++)
        std::cerr << i << "x=" << sw[i][0] << " y=" << sw[i][1] << std::endl;
#endif   
        
}


int main(int argc, char **argv) {
    
    double ret = 0;
    std::string defaulWaypointFile;
    defaulWaypointFile = "waypoints/ws.csv";
    
    std::string wayPointsFilename(defaulWaypointFile);
    
    std::vector<std::array<double, 2>> sw;
    
    ret = loadSplineWaypoints(wayPointsFilename, sw);

    if(ret != 0) {
        fprintf(stderr, "Failed loading waypoints from %s\n", wayPointsFilename.c_str());
        return EXIT_FAILURE;
    }
    
    //set_spline_waypoints(sw);

    std::cout << "Last element:" << sw.size()-1 << std::endl;
    for(unsigned int i = sw.size()-1; i < sw.size(); i++)
        std::cout << i << "x=" << sw[i][0] << " y=" << sw[i][1] << std::endl;
    
    
    
    double K, u;
    K = 1.0/0;
    K = INFINITY;
    
    u = K*1.2 + 6.1;
    
    double alpha = atan2(u,1);
    double thrhld = 30*M_PI/180;
    if (alpha>thrhld)
        alpha=thrhld;
    else if (alpha<-thrhld)
        alpha=-thrhld;    
    
    u = tan(alpha);

    
    printf("alpha: %.2f, u: %.2f \n", alpha*180/M_PI, u);
    if (isinf(K))
        printf("isinf(K): true; copysign(1,0): %.2f\n", copysign(1,0));
    
    
    
    
    PATHPLAN pathplanning;
    pathplanning.set_spline_vertices(sw);
    pathplanning.init();
    
    double x = 1;
    double y = 1;
    double theta = PI/20;
    pathplanning.goawayflag = false;
    pathplanning.isStartingManeuver = true;
    pathplanning.xs = 2;
    pathplanning.ys = 3;
    pathplanning.thetas = PI;
    pathplanning.Nspl = 0;
    pathplanning.startflag = false;
    
    if(pathplanning.Run(x, y, theta)) {
        printf("Pathplanning is good; D = %.5f; K = %.5f; psi = %.5f \n", pathplanning.DKPsi[0], pathplanning.DKPsi[1], pathplanning.DKPsi[2]);
        printf("Heuristic: { %.5f; %.5f; %.5f; %.5f; %.5f \n", pathplanning.heur[0], pathplanning.heur[1], pathplanning.heur[2], pathplanning.heur[3], pathplanning.heur[4]);
    } else {
        printf("Pathplanning exit \n");
    }
    
    return 0;
}
