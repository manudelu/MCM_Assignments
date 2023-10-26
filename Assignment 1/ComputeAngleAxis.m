function R = ComputeAngleAxis(theta,v)

    %Calculate the Skew Matrix
    v_skew=[0 -v(3) v(2); 
            v(3) 0 -v(1); 
            -v(2) v(1) 0];

    %Implement here the Rodrigues formula
    R = eye(3) + v_skew*sin(theta) + (v_skew)^2*(1-cos(theta));
    
end
