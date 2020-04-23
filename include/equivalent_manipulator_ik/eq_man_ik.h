#ifndef EQ_MAN_IK_H
#define EQ_MAN_IK_H

#include <stdio.h>
#include <math.h>
#include <vector>

#define PI 3.14159265


class IK_Solver_OW
{
    public:
        IK_Solver_OW(); //Default
        IK_Solver_OW(float ang_0, float ang_1); //Explicit
    
        std::vector<float> solve_ik_2q(std::vector<float> rot_goal); //Sign is not specified for atan2 function
        std::vector<float> solve_ik_4q(std::vector<float> rot_goal); //Sign is specified for atan2 function, double calculations


    private:
        int sign_of(float val);

        float _ang_0;
        float _ang_1;

        float c0;
        float c1;
        float s0;
        float s1;
       
};

#endif