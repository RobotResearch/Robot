function p = t2v(M)
  p = zeros(3,1);
  p(1:2,1) = M(1:2,3);
  p(3,1) = atan2(M(2,1), M(1,1));
end 