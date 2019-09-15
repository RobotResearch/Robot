function [mu, sigma] = prediction_step(mu, sigma, u)

% Updates the belief concerning the robot pose according to the motion model,
% mu: 2N+3 x 1 vector representing the state mean
% sigma: 2N+3 x 2N+3 covariance matrix
% u: odometry reading (r1, t, r2)
% Use u.r1, u.t, and u.r2 to access the rotation and translation values
 
% TODO: Compute the new mu based on the noise-free (odometry-based) motion model
% Remember to normalize theta after the update (hint: use the function normalize_angle available in tools)
 
cmd = [u.t*cos(mu(3)+u.r1); u.t*sin(mu(3)+u.r1); u.r1+u.r2];
L = size(mu,1);
cmd2 = zeros(L,1);
cmd2(1:3) = cmd(1:3);
mu = mu + cmd2;
mu(3) = normalize_angle(mu(3));
 
% TODO: Compute the 3x3 Jacobian Gx of the motion model
Gx = zeros(3,3);
Gx(1,3) = -cmd(2);
Gx(2,3) = cmd(1);

% TODO: Construct the full Jacobian G

Fx = zeros(3,L);
Fx(1:3, 1:3) = eye(3);

Gxt = transpose(Fx)*Gx*Fx + eye(L);

% Motion noise

motionNoise = 0.1;
R3 = [motionNoise, 0, 0;
     0, motionNoise, 0;
     0, 0, motionNoise/10];
R = zeros(size(sigma,1));
R(1:3,1:3) = R3;

% TODO: Compute the predicted sigma after incorporating the motion

sigma = Gxt*sigma*transpose(Gxt)+R;
 
end 