function x = correctSize(x,T,extra_features,noise_value)

xsize = size(x);
m = xsize(1);

x = num2cell(x,[2 3]);

for i=1:1:m
    if size(extra_features)== [1,2]
       x{i} = reshape(x{i},[extra_features(1)*extra_features(2),T]);
    else
       x{i} = reshape(x{i},[extra_features,T]);    
    end
    
    x{i} = x{i} + noise_value .* randn(extra_features,T);
end
end