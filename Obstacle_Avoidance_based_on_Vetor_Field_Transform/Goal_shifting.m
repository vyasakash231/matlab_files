function new_Og = Goal_shifting(Og, Oe, n, theta)
    Og = [Og;1]; % convert into homogeneous form 

    T = [1, 0, 0, -Oe(1);
         0, 1, 0, -Oe(2);
         0, 0, 1, -Oe(3);
         0, 0, 0,   1  ];

    d = sqrt(n(2)^2 + n(3)^2);

    Rx = [1,   0   ,    0   , 0;
          0, n(3)/d, -n(2)/d, 0;
          0, n(2)/d,  n(3)/d, 0;
          0,   0   ,    0   , 1];

    Ry = [d   , 0, -n(1), 0;
          0   , 1,   0  , 0;
          n(1), 0,   d  , 0;
          0   , 0,   0  , 1];

    Rz = [cos(theta), sin(theta), 0, 0;
         -sin(theta), cos(theta), 0, 0;
              0     ,     0     , 1, 0;
              0     ,     0     , 0, 1];

      net_Transform = inv(T) * inv(Rx) * inv(Ry) * Rz * Ry * Rx * T;
    
      new_Og = net_Transform * Og;
    new_Og = new_Og(1:3,:); % converting homogeneous form back to cartesian form
end