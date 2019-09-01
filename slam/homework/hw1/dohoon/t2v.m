function vec = t2v(T)
  R = T(1:2, 1:2);
  theta = -j * log(R(2,2) + j*R(2,1));
  x = T(1,3);
  y = T(2,3);
  vec = [x, y, theta];
end