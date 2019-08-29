function pos = t2v( M )
    pos = zeros(3,1);
    pos(1) = M(1,3);
    pos(2) = M(2,3);
    pos(3) = atan2(M(2,1), M(1,1));
end