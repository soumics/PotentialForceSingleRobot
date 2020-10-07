%%% Function to find the angle in the right coordinate
%%% atan2() can be used instead

function c=angcorr(x)

p=floor(x/90);
q=mod(p,4);
y=mod(x,90);
if(q==0)
    c=y;
end
if(q==1)
    c=90+y;
end
if(q==2)
    c=-(180-y);
end
if(q==3)
    c=-(90-y);
end
