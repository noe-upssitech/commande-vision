function [s, L] = visu(q, param_robot, OP1, OP2, OP3, OP4)
   T_RRm = [cos(q(3)) -sin(q(3))  0 q(1)
            sin(q(3))  cos(q(3))  0 q(2)
            0           0          1 param_robot(6);
            0           0          0 1];
         
   T_RmRp = [cos(q(4)) -sin(q(4)) 0 param_robot(1);
             sin(q(4))  cos(q(4)) 0 0;
             0            0           1 param_robot(5);
             0            0           0 1]; 
          
   T_RpRc = [0 0 1 param_robot(2); 0 1 0 param_robot(3); -1 0 0 param_robot(4); 0 0 0 1];
          
   
   T_RRc  = T_RRm * T_RmRp * T_RpRc;
   
   T_RcR  = inv(T_RRc);
   
   CP1 = T_RcR * OP1;
   CP2 = T_RcR * OP2;
   CP3 = T_RcR * OP3;
   CP4 = T_RcR * OP4;
   
   % Proj perspective
   f = param_robot(7);
   X1 = f * CP1(1)/CP1(3);
   Y1 = f * CP1(2)/CP1(3);
   X2 = f * CP2(1)/CP2(3);
   Y2 = f * CP2(2)/CP2(3);
   X3 = f * CP3(1)/CP3(3);
   Y3 = f * CP3(2)/CP3(3);
   X4 = f * CP4(1)/CP4(3);
   Y4 = f * CP4(2)/CP4(3);
   
   s = [X1 Y1 X2 Y2 X3 Y3 X4 Y4]';
   
   % Calcul de L, J
   
   L1 = [0 X1/CP1(3) X1*Y1; -1/CP1(3) Y1/CP1(3) 1+Y1^2];
   L2 = [0 X2/CP2(3) X2*Y2; -1/CP2(3) Y2/CP2(3) 1+Y2^2];
   L3 = [0 X3/CP3(3) X3*Y3; -1/CP3(3) Y3/CP3(3) 1+Y3^2];     
   L4 = [0 X4/CP4(3) X4*Y4; -1/CP4(3) Y4/CP4(3) 1+Y4^2];
   L  = [L1;L2;L3;L4];
end