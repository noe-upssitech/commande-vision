
function drawTarget(OP1,OP2,OP3,OP4, pos_obst, r_obst)
  
x_en = [OP1(1) OP2(1) OP3(1) OP4(1) OP1(1)];
y_en = [OP1(2) OP2(2) OP3(2) OP4(2) OP1(2)];
z_en = [OP1(3) OP2(3) OP3(3) OP4(3) OP1(3)];
plot3(x_en,y_en,z_en,'r','linewidth',3)
plot3(x_en,y_en,z_en,'go','linewidth',5)

for i = 1 : size(pos_obst, 2)
    drawcircle(pos_obst(1, i), pos_obst(2, i), r_obst, 'r-'); 
end 
