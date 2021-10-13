function seq_jacob = jacob_gen3_lite_s(q)

    seq_jacob = zeros(6,6);
    rot_corr=[0 -1 0;1 0 0; 0 0 1];
    T_0 = [rot_corr [0.057 ; -0.010 ; 1.0033]
           zeros(1,3) 1];    
    O1 = [0 ; 0 ; 0];
    O2 = [0 ; 0 ; 0.2433];
    O3 = [0 ; 0 ; 0.5233];
    O4 = [0 ; -0.01 ; 0.5233];
    O5 = [0 ; -0.01 ; 0.7683];
    O6 = [0.057 ; -0.01 ; 0.7683];
    
    seq_axe = [[0;0;1],[0;-1;0],[0;1;0],[0;0;1],[1;0;0],[0;0;1]];
    seq_Q = [O1,O2,O3,O4,O5,O6];
    T = rotation(seq_Q(:,1),seq_axe(:,1),q(1));
    seq_jacob(:,1) = V_i(seq_axe(:,1),seq_Q(:,1));
    for i = 2:6
        seq_jacob(:,i) = Ad(T)*V_i(seq_axe(:,i),seq_Q(:,i));
        T=T*rotation(seq_Q(:,i),seq_axe(:,i),q(i));
    end
    
end

function V = V_i(omega, Q)
    V = [omega ; -skewm(omega)*Q];
end



function M = Ad(T)
    M = [T(1:3,1:3) zeros(3,3)
         skewm(T(1:3,4))*T(1:3,1:3) T(1:3,1:3)];
end
