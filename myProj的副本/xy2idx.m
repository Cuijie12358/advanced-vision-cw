function idx = xy2idx(x,y, ifVertical)
    if (nargin<3)
        ifVertical = true;
    end
    width  = 640;
    height = 480;
    
    x = ceil(x);
    y = ceil(y);
    
    if ifVertical
        idx = (y-1) * width  + x;
    else
        idx = (x-1) * height + y;
    end
end