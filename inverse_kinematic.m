function [length, speed] = inverse_kinematic(alpha, beta, gamma, top_position, speed_top, pos_base, top2leg)
    % Rotation about x axis
    R1 = [1 0 0; 0 cos(alpha) -sin(alpha); 0 sin(alpha) cos(alpha)];
    % Rotation about y axis
    R2 = [cos(beta) 0 sin(beta); 0 1 0; -sin(beta) 0 cos(beta)];
    % Rotation about z axis
    R3 = [cos(gamma) -sin(gamma) 0; sin(gamma) cos(gamma) 0; 0 0 1];
    % Whole rotation
    R = R1 * R2 * R3;
    
    P = zeros(3,6);
    for i = 1:6
        P(:,i) = top_position;
    end
    % Vectors of the legs incl. the length
    L = P + R * top2leg - pos_base;
    
    % Calculate the lengths of the legs and thier unit vector
    leg_vectors = zeros(3,6);
    length = zeros(6,1);
    for i = 1:6
        length(i) = norm(L(:,i));
        leg_vectors(:, i) = L(:, i) / length(i);
    end
    
    %
    speed = [leg_vectors', (cross(R * top2leg, leg_vectors))'] * speed_top;
end