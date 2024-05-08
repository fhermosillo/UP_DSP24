function Y=maxpooling(X,k)
    for y = 1:k:size(X,1)
        if (y + k - 1) > size(X,1)
            y2=size(X,1);
        else
            y2=y+k-1;
        end
        for x = 1:k:size(X,2)
            if (x + k - 1) > size(X,2)
                Xk=X(y:y2,x:size(X,2));
                [xmax,ix,iy]=max2d(Xk);
                X(y:y2,x:size(X,2))=0;
                X(y+iy-1,x+ix-1)=xmax;
            else
                Xk=X(y:y2,x:(x+k-1));
                [xmax,ix,iy]=max2d(Xk);
                X(y:y2,x:(x+k-1))=0;
                X(y+iy-1,x+ix-1)=xmax;
            end
        end
    end
    Y=X;
end