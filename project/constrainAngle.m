function th = constrainAngle(ph)
    th = ph;
    while th > pi
        th = th - (2*pi);
    end
    while th < -pi
        th = th + (2*pi);
    end
end