%%%% Function to find the minimum change of angle %%%%%%%%%%%
function X=ch_of_ang(a1,a2)
if a1<0
    a1=360+a1;
end
if a2<0
    a2=360+a2;
end
a=a1-a2;
if abs(a)>180
    if a>0
        a=360-a;
        X=-a;  
    else
        a=-(360-abs(a));
        X=-a;  
    end
else
    X=a;
end