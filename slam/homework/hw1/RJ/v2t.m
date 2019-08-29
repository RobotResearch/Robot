function M = v2t(pos)
  M = [cos(pos(3)), -sin(pos(3)), pos(1);
       sin(pos(3)), cos(pos(3)), pos(2);
       0,0,1];
end