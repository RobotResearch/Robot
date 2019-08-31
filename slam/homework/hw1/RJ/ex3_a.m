function ex3_a()
  p1 = [1; 2; 0.1];
  p2 = [3; 5; -0.5];
  
  M1 = v2t(p1);
  M2 = v2t(p2);
  
  a1 = t2v(M1);
  a2 = t2v(M2);
  
  p1 - a1
  p2 - a2 
  
end