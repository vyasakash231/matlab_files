function new_transformed_vector = Linear_Velocity_transform(vector_to_transform, Om, n, theta)
      homo_vector_to_transform = [vector_to_transform;1]; % convert into homogeneous form 

      T = [1, 0, 0, -Om(1);
           0, 1, 0, -Om(2);
           0, 0, 1, -Om(3);
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

      Rz = [cos(theta), -sin(theta), 0, 0;
            sin(theta),  cos(theta), 0, 0;
                0     ,      0     , 1, 0;
                0     ,      0     , 0, 1];

      net_Transform = inv(T) * inv(Rx) * inv(Ry) * Rz * Ry * Rx * T;
    
      transformed_vector = net_Transform * homo_vector_to_transform;
    new_transformed_vector = transformed_vector(1:3,:); % converting homogeneous form back to cartesian form
end