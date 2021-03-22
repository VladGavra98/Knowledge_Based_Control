function U = mf(x,c,Xc,m)

    U = zeros(c,1);   % partition matrix
    D = zeros(c,1);   % distance matrix

    for v=1:1:c
        D(v,1) = sqrt((x - Xc(:,v))' * (x - Xc(:,v)));
    end

    for v =1:1:c
        U(v,1) = 1/( sum((D(v,1) ./ D(:,1)) .^(m)));
    end
    

end