function M = v2t(p)
  M = eye(3);
  M(1:2,1:2) = [cos(p(3)), -sin(p(3));
                sin(p(3)), cos(p(3))];
  M(1:2,3) = p(1:2,1);
end