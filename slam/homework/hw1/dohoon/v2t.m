function T = v2t(vec)
  x = vec(1);
  y = vec(2);
  theta = vec(3);
  R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
  P = [x ; y];
  T = [[R P];[0 0 1]];
end