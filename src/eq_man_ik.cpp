#include "equivalent_manipulator_ik/eq_man_ik.h"



/////////Constructors////////////

//Default constructor
IK_Solver_OW::IK_Solver_OW(){
    _ang_0 = 120*(PI_/180);
    _ang_1 = 45*(PI_/180);

    c0 = cos(_ang_0);
    c1 = cos(_ang_1);
    s0 = sin(_ang_0);
    s1 = sin(_ang_1);    
}

//Declarative Constructor
IK_Solver_OW::IK_Solver_OW(float ang_0, float ang_1){
    _ang_0 = ang_0*(PI_/180);
    _ang_1 = ang_1*(PI_/180);

    c0 = cos(_ang_0);
    c1 = cos(_ang_1);
    s0 = sin(_ang_0);
    s1 = sin(_ang_1);    
}

/////////////Methods//////////////

//IK Solver ignoring quadrant specification due to sign of theta3
std::vector<float> IK_Solver_OW::solve_ik_2q(std::vector<float> rot_goal){
    std::vector<float> q_out(3);

    rot_goal.resize(3);

    float alpha = rot_goal[0];
    float beta = rot_goal[1];
    float gamma = rot_goal[2];

    float ca = cos(alpha);
    float sa = sin(alpha);
    float cb = cos(beta);
    float sb = sin(beta);
    float cg = cos(gamma);
    float sg = sin(gamma);


    //EQ Manipulator A
    float x0 = s1*(cb*cg*s1 - c1*sb) + c1*(c1*ca*cb + s1*(ca*cg*sb + sa*sg));
    float y0 = -(cb*sb*sg) - c1*(-(cg*sa) + ca*sb*sg);

    //EQ Manipulator B
    float x1 = c1*(c1*ca*cb + c0+s1*(ca*cg*sb + sa*sg) + s0*s1*(-(cg*sa) + ca*sb*sg))
               + s1*(c0*(c0*cb*cg*s1-c1*sb+cb*s0*s1*sg)
               +  s0*(c1*cb*sa+c0*s1*(cg*sa*sb-ca*sg)+s0*s1*(ca*cg+sa*sb*sg)));
    float y1 = -( c1*(-(s0*(ca*cg*sb+sa*sg))+c0*(-(cg*sa)+ca*sb*sg))) - s1*(c0*(-(cb*cg*s0)+c0*cb*sg) + s0*(-(s0*(cg*sa*sb-ca*sg))+c0*(ca*cg+sa*sb*sg)));

    //EQ Manipulator C
    float x2 = c1*(c1*ca*cb+c0*s1*(ca*cg*sb+sa*sg)-s0*s1*(-(cg*sa)+ca*sb*sg)) + s1*(c0*(c0*cb*cg*s1-c1*sb-cb*s0*s1*sg) - s0*(c1*cb*sa+c0*s1*(cg*sa*sb-ca*sg)-s0*s1*(ca*cg+sa*sb*sg))); 
    float y2 = -(c1*(s0*(ca*cg*sb+sa*sg) + c0*(-(cg*sa)+ca*sb*sg))) - s1*(c0*(cb*cg*s0+c0*cb*sg) - s0*(s0*(cg*sa*sb-ca*sg)+c0*(ca*cg+sa*sb*sg)));



    q_out[0] = atan2(y0,x0);
    q_out[1] = atan2(y1,x1);
    q_out[2] = atan2(y2,x2);

    return q_out;
}


//IK Solver on all 4 quadrants specification due to sign of theta3
std::vector<float> IK_Solver_OW::solve_ik_4q(std::vector<float> rot_goal){
    std::vector<float> q_out(3);

    rot_goal.resize(3);

    float alpha = rot_goal[0];
    float beta = rot_goal[1];
    float gamma = rot_goal[2];

    float ca = cos(alpha);
    float sa = sin(alpha);
    float cb = cos(beta);
    float sb = sin(beta);
    float cg = cos(gamma);
    float sg = sin(gamma);


    //EQ Manipulator A
    float q3a_x = sqrt(pow((cb*s1*sg+c1*(-(cg*sa)+ca*sb*sg)),2) + pow(s1*(cb*cg*s1-c1*sb)+c1*(c1*ca*cb+s1*(ca*cg*sb+sa*sg)),2));
    float q3a_y = s1*(c1*cb*cg+s1*sb)+c1*(-(ca*cb*s1)+c1*(ca*cg*sb+sa*sg));
    float q3a = atan2f(q3a_y,q3a_x);
    int sign_c3a = sign_of(cos(q3a));

    float q2a_x = (s1*(cb*cg*s1-c1*sb)+c1*(c1*ca*cb+s1*(ca*cg*sb+sa*sg)))*sign_c3a;
    float q2a_y = (-(cb*s1*sg)-c1*(-(cg*sa)+ca*sb*sg))*sign_c3a; 
    float q2a = atan2f(q2a_y,q2a_x);


    //EQ Manipulator B
    float q3b_x = sqrt(pow(c1*(-(s0*(ca*cg*sb+sa*sg))+c0*(-(cg*sa)+ca*sb*sg))+ s1*(c0*(-(cb*cg*s0)+c0*cb*sg)+ s0*(-(s0*(cg*sa*sb-ca*sg))+c0*(ca*cg+sa*sb*sg))),2)
                + pow(c1*(c1*ca*ca+c0*s1*(ca*cg*sa+sa*sg)+s0*s1*(-(cg*sa)+ca*sa*sg))+ s1*(c0*(c0*ca*cg*s1-c1*sa+ca*s0*s1*sg)+s0*(c1*ca*sa+ c0*s1*(cg*sa*sa-ca*sg)+s0*s1*(ca*cg+sa*sa*sg))),2));

    float q3b_y = c1*(-(ca*cb*s1)+c0*c1*(ca*cg*sb+sa*sg)+c1*s0*(-(cg*sa)+ca*sb*sg)) + s1*(c0*(c0*c1*cb*cg+s1*sb+c1*cb*s0*sg) + s0*(-(cb*s1*sa)+c0*c1*(cg*sa*sb-ca*sg)+c1*s0*(ca*cg+sa*sb*sg)));
    
    float q3b = atan2f(q3b_y,q3b_x);
    int sign_c3b = sign_of(cos(q3a));

    float q2b_x = (c1*(c1*ca*cb+c0*s1*(ca*cg*sb+sa*sg)+s0*s1*(-(cg*sa)+ca*sb*sg))+ s1*(c0*(c0*cb*cg*s1-c1*sb+cb*s0*s1*sg)+ s0*(c1*cb*sa+c0*s1*(cg*sa*sb-ca*sg)+s0*s1*(ca*cg+sa*sb*sg))))*sign_c3b;
    float q2b_y = (-(c1*(-(s0*(ca*cg*sb+sa*sg))+c0*(-(cg*sa)+ca*sb*sg)))-s1 *(c0*(-(cb*cg*s0)+c0*cb*sg)+ s0*(-(s0*(cg*sa*sb-ca*sg))+c0*(ca*cg+sa*sb*sg))))*sign_c3b;
    float q2b = atan2f(q2b_y,q2b_x);

    //EQ Manipulator C
    float q3c_x = sqrt(pow(c1*(s0*(ca*cg*sb+sa*sg)+c0*(-(cg*sa)+ca*sb*sg))+ s1*(c0*(cb*cg*s0+c0*cb*sg) - s0*(s0*(cg*sa*sb-ca*sg)+c0*(ca*cg+sa*sb*sg))),2) 
                + pow( c1*(c1*ca*cb+c0*s1*(ca*cg*sb+sa*sg)-s0*s1*(-(cg*sa)+ca*sb*sg))+ s1*(c0*(c0*cb*cg*s1-c1*sb-cb*s0*s1*sg)-s0*(c1*cb*sa+ c0*s1*(cg*sa*sb-ca*sg)-s0*s1*(ca*cg+sa*sb*sg))),2));
    float q3c_y = c1*(-(ca*cb*s1)+c0*c1*(ca*cg*sb+sa*sg)-c1*s0*(-(cg*sa)+ca*sb*sg)) + s1*(c0*(c0*c1*cb*cg+s1*sb-c1*cb*s0*sg) - s0*(-(cb*s1*sa)+c0*c1*(cg*sa*sb-ca*sg)-c1*s0*(ca*cg+sa*sb*sg)));
    float q3c = atan2f(q3c_y,q3c_x);
    int sign_c3c = sign_of(cos(q3c));

    float q2c_x = (c1*(c1*ca*cb+c0*s1*(ca*cg*sb+sa*sg)-s0*s1*(-(cg*sa)+ca*sb*sg)) + s1*(c0*(c0*cb*cg*s1-c1*sb-cb*s0*s1*sg) - s0*(c1*cb*sa+c0*s1*(cg*sa*sb-ca*sg)-s0*s1*(ca*cg+sa*sb*sg))))*sign_c3c;
    float q2c_y = (-(c1*(s0*(ca*cg*sb+sa*sg)+c0*(-(cg*sa)+ca*sb*sg))) - s1*(c0*(cb*cg*s0+c0*cb*sg) - s0*(s0*(cg*sa*sb-ca*sg)+c0*(ca*cg+sa*sb*sg))))*sign_c3c;
    float q2c = atan2f(q2c_y,q2c_x);
   

    // q_out[0] = q2a;
    // q_out[1] = q2b;    
    // q_out[2] = q2c;

    // q_out[0] = q2c;
    // q_out[1] = q2a;    
    // q_out[2] = q2b;

    //map to motor axis
    q_out[0] = q2b;
    q_out[1] = q2a;    
    q_out[2] = q2c;


    return q_out;
}



int IK_Solver_OW::sign_of(float val){
    if(val <= 0) return -1;
    else return 1;
}
