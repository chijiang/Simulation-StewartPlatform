function length = inverse_kinematic(alpha, beta, gamma, base2top, base2leg, top2leg, length0)
    R1 = [1 0 0; 0 cos(alpha) -sin(alpha); 0 sin(alpha) cos(alpha)];
    R2 = [cos(beta) 0 sin(beta); 0 1 0; -sin(beta) 0 cos(beta)];
    R3 = [cos(gamma) -sin(gamma) 0; sin(gamma) cos(gamma) 0; 0 0 1];
    
    R = R1 * R2 * R3;
    L = R*base2top + top2leg - base2leg;
    L = L' * L;
    length = sqrt(L) - length0;
end