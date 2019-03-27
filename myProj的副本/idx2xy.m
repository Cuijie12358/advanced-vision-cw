function [x, y] = idx2xy(idx, ifVertical)
    if (nargin<2)
        ifVertical = true;
    end
    width  = 640;
    height = 480;
    
    idx=ceil(idx);
    
    if ifVertical
        x = mod(idx, width);
        x(x==0)=width;
        y = (idx-x)./width+1;
    else
        y = mod(idx, height);
        y(y==0)=height;
        x = (idx-y)./height+1;
    end
end