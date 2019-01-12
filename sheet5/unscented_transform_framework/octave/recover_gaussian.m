function [mu, sigma] = recover_gaussian(sigma_points, w_m, w_c)
% This function computes the recovered Gaussian distribution (mu and sigma)
% given the sigma points (size: nx2n+1) and their weights w_m and w_c:
% w_m = [w_m_0, ..., w_m_2n], w_c = [w_c_0, ..., w_c_2n].
% The weight vectors are each 1x2n+1 in size,
% where n is the dimensionality of the distribution.

% Try to vectorize your operations as much as possible

% TODO: compute mu
mu = sum(repmat(w_m, 2, 1) .* sigma_points, 2);

% TODO: compute sigma
diff = sigma_points - mu;
sigma = repmat(w_c, 2, 1) .* diff * diff';
end
