function ex3_b()
  p1 = [1; 2; 0.1];
  p2 = [3; 5; -0.5];
  
  M01 = v2t(p1); % 1 -> 0
  M02 = v2t(p2); % 2 -> 0
  
  M12 = inv(M01) * M02;
  
  t12 = t2v(M12);
  disp('rotation')
  t12(3) % rotat
  disp('displacement in p1 frame')
  
  [ t12(1) ; t12(2) ]
  dis = [2; 3];
  th = -0.1;
  rot = [cos(th), -sin(th);sin(th), cos(th)];
  rot * dis
end