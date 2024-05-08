function [maxvalue,x,y]=max2d(X)
    maxvalue = X(1,1);
    x=1;
    y=1;
    for i = 1:size(X,1)
        for j = 1:size(X,2)
            if X(i,j)>maxvalue
                x=j;
                y=i;
                maxvalue=X(i,j);
            end
        end
    end
end