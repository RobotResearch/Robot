function [mu2, sigma2] = prediction_step(mu, sigma, u)
% Updates the belief concerning the robot pose according to the motion model,
% mu: 2N+3 x 1 vector representing the state mean
% sigma: 2N+3 x 2N+3 covariance matrix
% u: odometry reading (r1, t, r2)
% Use u.r1, u.t, and u.r2 to access the rotation and translation values

% TODO: Compute the new mu based on the noise-free (odometry-based) motion model
% Remember to normalize theta after the update (hint: use the function normalize_angle available in tools)
mu2 = mu;
mu2(1) = mu2(1) + u.t * cos(mu(3) + u.r1);
mu2(2) = mu2(2) + u.t * sin(mu(3) + u.r1);
mu2(3) = mu2(3) + u.r1 + u.r2;
% TODO: Compute the 3x3 Jacobian Gx of the motion model

Gx = [1, 0, -u.t * sin(mu(3) + u.r1);
      0, 1, u.t * cos(mu(3) + u.r1);
      0, 0, 1];

% TODO: Construct the full Jacobian G
G = [Gx, zeros(3, size(sigma,1) - 3);
     zeros(size(sigma,1)-3,3), eye(size(sigma)-[3,3])];

% Motion noise
motionNoise = 0.1;
R3 = [motionNoise, 0, 0; 
     0, motionNoise, 0; 
     0, 0, motionNoise/10];
R = zeros(size(sigma,1));
R(1:3,1:3) = R3;

% TODO: Compute the predicted sigma after incorporating the motion

sigma2 = G * sigma * transpose(G) + R;

end
