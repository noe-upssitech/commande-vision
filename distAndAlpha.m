function [d, alpha] = distAndAlpha(q, pos_obst, param_robot)
    Dx = param_robot(1);

    vec1 = [Dx * cos(q(3)) ; Dx * sin(q(3))];
    vec2 = pos_obst - [q(1); q(2)];
    
    vec = vec2 - vec1;


    d = norm(vec);
    
    % Old
    normal_vec3 = [vec(2) ; -vec(1) ; 0];
    vec3 = [vec1(1); vec1(2); 0];
    alpha2 = asin( norm(cross(vec3, normal_vec3)) / (norm(vec3) * norm(normal_vec3)) );
    
    % New
    w1 = vec1(1); 
    w2 = vec1(2);
    v1 = normal_vec3(1);
    v2 = normal_vec3(2);
    alpha = atan2(w2 * v1 - w1 * v2, w1 * v1 + w2 * v2); 
end