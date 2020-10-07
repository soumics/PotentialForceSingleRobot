close all
clear all
clc;


%%%%%%%%%% WORKSPACE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
x=[0,50,50,0,0];
y=[0,0,50,50,0];
plot(x,y,'k')
hold on
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%% robot as a rectangular box %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
initx=0;
inity=0;
width=4; % width of the rectangle (/robot)
height=2; % height of the rectangle (/robot)
x1=initx;
x2=initx+width;
x3=x2;
x4=x1;
x5=x1;
y1=0;
y2=0;
y3=inity+height;
y4=y3;
y5=y1;

x0=[x1,x2,x3,x4,x5]; % x's of the vertex (x_i,y_i) of the rectangle (/robot)
y0=[y1,y2,y3,y4,y5]; % y's of the vertex (x_i,y_i) of the rectangle (/robot)
plot(x0,y0)

cgx=initx+(width/2); % x of centroid of the rectangle (/robot)
cgy=inity+(height/2); % y of centroid of the rectangle (/robot)

% angles of the vertices from the centroid of the rectangle (/robot)
newang=0;
r=sqrt(power((x3-cgx),2)+(power((y3-cgy),2)));
theta1=(atan((y3-cgy)/(x3-cgx)))*(180/pi);
theta2=180-theta1;
theta3=180+theta1;
theta4=360-theta1;


robo=[cgx cgy x1 y1 x2 y2 x3 y3 x4 y4];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%% goal position %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
goalx=25;
goaly=30;
plot(goalx,goaly,'k*');

goalang=(atan2((goaly-cgy),(goalx-cgx)))*(180/pi); % goal angle w.r.t robot centroid (/c.g)
if(goalang<0)
    goalang=360+goalang;
end

dist=(sqrt(power((goalx-robo(1)),2)+(power((goaly-robo(2)),2)))); % distance to goal from c.g

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%% control input/ iniitial velocity and angular velocity %%%%%%%%%%%%

vmax=.5;
v=vmax;
w=.30;
delt=100;
delv=v/delt;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%% position of the obstacles %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

obs1x= [15:25];
obs1y= repmat(20,1,length(obs1x));
plot(obs1x,obs1y,'r*');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

resang=0;


%%%%%%%% robot faces toward goal from initial heading %%%%%%%%%%%%%%%%%%%%%
deltheta=0;

T=table;

  while(resang<(goalang-.1))
     
          goalang=(atan2((goaly-robo(2)),(goalx-robo(1))))*(180/pi);
          chang=goalang-resang;
          w=chang/delt;
          deltheta=(w*delt)/2;
          resang=resang+deltheta;
          
          robo(3)=robo(1)+(r*cos(deg2rad(theta3+resang)));
          robo(4)=robo(2)+(r*sin(deg2rad(theta3+resang)));
          robo(5)=robo(1)+(r*cos(deg2rad(theta4+resang)));
          robo(6)=robo(2)+(r*sin(deg2rad(theta4+resang)));
          robo(7)=robo(1)+(r*cos(deg2rad(theta1+resang)));
          robo(8)=robo(2)+(r*sin(deg2rad(theta1+resang)));
          robo(9)=robo(1)+(r*cos(deg2rad(theta2+resang)));
          robo(10)=robo(2)+(r*sin(deg2rad(theta2+resang)));
          xr=[robo(3),robo(5),robo(7),robo(9),robo(3)];
          yr=[robo(4),robo(6),robo(8),robo(10),robo(4)];
          
          plot(xr,yr,'b');
              
          pause(0.1);
       
  end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%% robot movement due to resultant force %%%%%%%%%%%%%%%%%%%%%%%%%%

%parameters initialized
k=0;
count=0;

rho0=2*r; %safety radius
kattr=1; % attractive force proportionality constant
krep=1000; % repulsive force proportionality constant



w=.25;

resultantang=0;
i=0;
deltheta=0;
chang=0;
in=0;
h=0;
chkang=0;


while(dist>1)
    % should be inside the loop because of instanteneous calculation
    fattr=[0;0]; % attractive force
    frepnet=[0;0]; % net repulsive force
    frep=[0;0]; % single repulsive force
    fres=[0;0]; % resultant force

    %%%%% calculation of  attractive force(magnitude+angle) %%%%%%%%%%%%%%%
    dist=sqrt(power((goalx-robo(1)),2)+(power((goaly-robo(2)),2)));
   
    fattr=-(kattr*[robo(1)-goalx;robo(2)-goaly]);
    fattrmag=sqrt(fattr(1)^2+fattr(2)^2);
    attrang=(atan2((goaly-robo(2)),(goalx-robo(1))))*(180/pi);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %%%%% Calculation of the net repulsive force from all obstacles %%%%%%%
    for cnt=1:1:length(obs1x)
        
        obsdist=sqrt(power((obs1x(cnt)-robo(1)),2)+(power((obs1y(cnt)-robo(2)),2)));
        if(obsdist<=rho0)
            %calculate repulsive force(magnitude+angle)
            frep=krep*((1/obsdist)-(1/rho0))*(1/(obsdist^2))*...
                ([robo(1)-obs1x(cnt);robo(2)-obs1y(cnt)]/norm([robo(1)-obs1x(cnt);robo(2)-obs1y(cnt)])); 
        end
        
        frepnet=frepnet+frep;
    end
        
    frepnetmag=sqrt(frepnet(1)^2+frepnet(2)^2);
    netrepang=(atan2(frepnet(2),frepnet(1)))*(180/pi);
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
    
  %%%%%% resulatnt force=attactive force+repulsive force %%%%%%%%%%%%%%%%%%
    fres=fattr+frepnet;
    
    fresmag=sqrt(power(fres(1),2)+power(fres(2),2));
    resultantang=(atan2(fres(2),fres(1)))*(180/pi);
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  
  %%%%%%% magnitude and angle calculation for control input %%%%%%%%%%%%%%%
    chang=ch_of_ang(resultantang,resang);
    
    w=chang/delt;
    deltheta=(w*delt)/2;
    resangtemp=resang+deltheta;
    resangtemp=angcorr(resangtemp);      
    
     
    %%%%%%%% angle adjustment using atan2(...) %%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if(abs(ch_of_ang(attrang,resangtemp))>90)
        
        netrepang=(atan2(frepnet(2),frepnet(1)))*(180/pi);
        if netrepang<0
            netrepang=360-abs(netrepang);
        end
        attrang=netrepang+145;
        attrang=angcorr(attrang);

        fattr(1)=sqrt(fattr(1)^2+fattr(2)^2)*cos(deg2rad(attrang));
        fattr(2)=sqrt(fattr(1)^2+fattr(2)^2)*sin(deg2rad(attrang));
        fattrmag=sqrt(fattr(1)^2+fattr(2)^2);
        fres=fattr+frepnet;
        
        in=in+1;
        
        fresmag=sqrt(power(fres(1),2)+power(fres(2),2));
        resultantang=(atan2(fres(2),fres(1)))*(180/pi);
        chang=ch_of_ang(resultantang,resang);
    
        w=chang/delt;
        deltheta=(w*delt)/2;
        resang=resang+deltheta;
        resang=angcorr(resang);
    else
        resang=resangtemp;
    end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%% velocity control block %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   
    if(sqrt(frepnet(1)^2+frepnet(2)^2)>0)
        costheta=(frepnet(1)*(v*cos(deg2rad(resultantang))))/(sqrt(frepnet(1)^2+frepnet(2)^2)*v);
      
        v=vmax*(1-abs(costheta));
        delv=v/delt;
    else
        v=vmax;
        delv=v/delt;
    end
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

  
        
 %%%%%%%%% state propagation of robot using the control input %%%%%%%%%%%%%   
    
    robo(1)=robo(1)+(delv*cos(deg2rad(resang))*delt);
    robo(2)=robo(2)+(delv*sin(deg2rad(resang))*delt);
    
    robo(3)=robo(1)+(r*cos(deg2rad(theta3+resang)));
    robo(4)=robo(2)+(r*sin(deg2rad(theta3+resang)));
    robo(5)=robo(1)+(r*cos(deg2rad(theta4+resang)));
    robo(6)=robo(2)+(r*sin(deg2rad(theta4+resang)));
    robo(7)=robo(1)+(r*cos(deg2rad(theta1+resang)));
    robo(8)=robo(2)+(r*sin(deg2rad(theta1+resang)));
    robo(9)=robo(1)+(r*cos(deg2rad(theta2+resang)));
    robo(10)=robo(2)+(r*sin(deg2rad(theta2+resang)));
    xr=[robo(3),robo(5),robo(7),robo(9),robo(3)];
    yr=[robo(4),robo(6),robo(8),robo(10),robo(4)];
    
    plot(xr,yr,'b');
    pause(0.1);
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%% saving required data %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

T_new = table(attrang,frepnetmag,netrepang,resultantang,chang,chkang,resang);
%save('./testdata.txt','attrang',...
    %'frepnetmag','netrepang','resultantang','chang','chkang','resang','-ASCII','-append')
T=[T;T_new];


  h=h+1;
end
T.Properties.VariableNames = { 'attrang' 'frepnetmag' 'netrepang' 'resultantang' 'chang' 'chkang' 'resang'};
writetable(T, './testdata.txt')
