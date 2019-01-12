% Compute the error of a pose-pose constraint
% x1 3x1 vector (x,y,theta) of the first robot pose
% x2 3x1 vector (x,y,theta) of the second robot pose
% z 3x1 vector (x,y,theta) of the measurement
%
% You may use the functions v2t() and t2v() to compute
% a Homogeneous matrix out of a (x, y, theta) vector
% for computing the error.
%
% Output
% e 3x1 error of the constraint
% A 3x3 Jacobian wrt x1
% B 3x3 Jacobian wrt x2
function [e, A, B] = linearize_pose_pose_constraint(x1, x2, z)

  % TODO compute the error and the Jacobians of the error
  X1 = v2t(x1);
  X2 = v2t(x2);
  Z = v2t(z);

  e = t2v(inv(Z) * (inv(X1) * X2));

  dt_j = Z(1:2, 1:2)' * X1(1:2, 1:2)';
  dt_i = -dt_j;
  theta1 = x1(3);
  dRT_i_d_theta = [
    -sin(theta1), -cos(theta1);
    cos(theta1), -sin(theta1)
  ]';
  dR_i = Z(1:2, 1:2)' * dRT_i_d_theta * (x2(1:2) - x1(1:2));
  A = [
    0, 0, dR_i(1);
    0, 0, dR_i(2);
    0, 0, -1
  ];
  A(1:2, 1:2) = dt_i;

  B = [
    0, 0, 0;
    0, 0, 0;
    0, 0, 1
  ];
  B(1:2, 1:2) = dt_j;

end;
