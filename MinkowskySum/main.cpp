//Libraries for Function
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <vector>
#include "eigen3/Eigen/Eigen"

/* ----------------------------------------------------- */
bool err;
#define IS_TRUE(x) {if (!(x)) {\
    std::cerr<<"⛔️ :"  <<" FAILED" <<std::endl;\
    err = true;}}
/* ----------------------------------------------------- */
// Function
std::vector<Eigen::Vector3d> createMinkowskyPoints(Eigen::Vector3d input,double resolution)
{   
    std::vector< std::vector<int> > EXPANSION_DIRECTIONS = {{-1, -1, -1}, {-1, -1, 0}, {-1, -1, 1}, {-1, 0, -1}, {-1, 0, 0}, {-1, 0, 1}, {-1, 1, -1},
                                                            {-1, 1, 0},   {-1, 1, 1},  {0, -1, -1}, {0, -1, 0},  {0, -1, 1}, {0, 0, -1}, {0, 0, 1},
                                                            {0, 1, -1},   {0, 1, 0},   {0, 1, 1},   {1, -1, -1}, {1, -1, 0}, {1, -1, 1}, {1, 0, -1},
                                                            {1, 0, 0},    {1, 0, 1},   {1, 1, -1},  {1, 1, 0},   {1, 1, 1}};
    
    std::vector< std::vector<int> > EXPANSION_DIRECTIONS_EASY = {{-1, -1, -1}, {-1, -1, 1},
                                                                {-1, 1, -1},{-1, 1, 1},   
                                                                {1, -1, -1},{1, -1, 1}, 
                                                                {1, 1, -1},{1, 1, 1}};
    std::vector<Eigen::Vector3d> points;
    points.push_back(input);
    for (auto vertex: EXPANSION_DIRECTIONS_EASY)
    {
        Eigen::Vector3d point;
        point << input(0)+resolution*vertex[0],input(1)+resolution*vertex[1], input(2)+resolution*vertex[2];
        points.push_back(point);
    }
    return points;
}

/* ----------------------------------------------------- */
// Tests
void handleTest(int test_number, Eigen::Vector3d input, double resolution,std::vector<Eigen::Vector3d> check)
{
    std::vector<Eigen::Vector3d> result = createMinkowskyPoints(input,resolution);
    std::cout<<"TEST "<<test_number<<":"<<std::endl;
    IS_TRUE(result==check);
    if (err==false)
        std::cout<<"✅ : PASSED\n";
}

void execute_tests()
{

    std::vector< std::vector<int> > EXPANSION_DIRECTIONS = {{-1, -1, -1}, {-1, -1, 0}, {-1, -1, 1}, {-1, 0, -1}, {-1, 0, 0}, {-1, 0, 1}, {-1, 1, -1},
                                                            {-1, 1, 0},   {-1, 1, 1},  {0, -1, -1}, {0, -1, 0},  {0, -1, 1}, {0, 0, -1}, {0, 0, 1},
                                                            {0, 1, -1},   {0, 1, 0},   {0, 1, 1},   {1, -1, -1}, {1, -1, 0}, {1, -1, 1}, {1, 0, -1},
                                                            {1, 0, 0},    {1, 0, 1},   {1, 1, -1},  {1, 1, 0},   {1, 1, 1}};
    
    std::vector< std::vector<int> > EXPANSION_DIRECTIONS_EASY = {{-1, -1, -1}, {-1, -1, 1},
                                                                {-1, 1, -1},{-1, 1, 1},   
                                                                {1, -1, -1},{1, -1, 1}, 
                                                                {1, 1, -1},{1, 1, 1}};
    // handleTest(test number, input, reference)
    Eigen::Vector3d input(3);
    double resolution = 1.0;
    input << 0,0,0;
    
    std::vector<Eigen::Vector3d> reference_points;
    // reference_points.push_back(input);
    // for (auto exp_vec : EXPANSION_DIRECTIONS)
    // {
    //     Eigen::Vector3d point(3);
    //     point << exp_vec[0],exp_vec[1],exp_vec[2];
    //     reference_points.push_back(point);    
    // } 
    // handleTest(1,input, resolution,reference_points);
    // // handleTest(test number, input, reference)
    // resolution = 0.6;
    // input << 0,0,0;
    
    // reference_points = {};
    // reference_points.push_back(input);
    // for (auto exp_vec : EXPANSION_DIRECTIONS)
    // {
    //     Eigen::Vector3d point(3);
    //     point << exp_vec[0],exp_vec[1],exp_vec[2];
    //     reference_points.push_back(resolution*point);    
    // } 
    // handleTest(2,input, resolution,reference_points);

    // // handleTest(test number, input, reference)
    // resolution = 1.0;
    // input << 1,1,1;
    
    // reference_points = {};
    // reference_points.push_back(input);
    // for (auto exp_vec : EXPANSION_DIRECTIONS)
    // {
    //     Eigen::Vector3d point(3);
    //     point << input(0)+ resolution*exp_vec[0],input(1)+ resolution*exp_vec[1],input(2)+ resolution*exp_vec[2];
    //     reference_points.push_back(point);    
    // } 
    // handleTest(3,input, resolution,reference_points);
    // // handleTest(test number, input, reference)
    // resolution = 0.6;
    // input << -0.5,-2.0,1;
    
    // reference_points = {};
    // reference_points.push_back(input);
    // for (auto exp_vec : EXPANSION_DIRECTIONS)
    // {
    //     Eigen::Vector3d point(3);
    //     point << input(0)+ resolution*exp_vec[0],input(1)+ resolution*exp_vec[1],input(2)+ resolution*exp_vec[2];
    //     reference_points.push_back(point);    
    // } 
    // handleTest(4,input, resolution,reference_points);

    // handleTest(test number, input, reference)
    resolution = 4.0;
    input << 0,0,0;
    
    reference_points = {};
    reference_points.push_back(input);
    for (auto exp_vec : EXPANSION_DIRECTIONS_EASY)
    {
        Eigen::Vector3d point(3);
        point << input(0)+ resolution*exp_vec[0],input(1)+ resolution*exp_vec[1],input(2)+ resolution*exp_vec[2];
        reference_points.push_back(point);    
    } 
    handleTest(5,input, resolution,reference_points);
}

/* ----------------------------------------------------- */
int main(void)
{ 
    execute_tests();
    if (err == false) std::cout<<"All tests are passed\n";
}
