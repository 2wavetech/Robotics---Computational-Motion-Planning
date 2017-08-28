function DrawTriangle(P1, P2)
    P1 = [P1; P1(1,:)]
    line(P1(:,1), P1(:,2));
    P2 = [P2; P2(1,:)]
    line(P2(:,1), P2(:,2));
end
