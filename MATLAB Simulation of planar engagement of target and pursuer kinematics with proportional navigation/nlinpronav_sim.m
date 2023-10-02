function dy=nlinpronav_sim (t, y,HE_rad, Np, aT, VM, PN_type)
%Define pointers to state variables
%----------------------
%Pointers to states
sel_beta = 1;
sel_RT1 =2
sel_RT2 =3;
sel_RM1 =4;
sel_RM2 = 5;
sel_VT1 =6;
sel_VT2 = 7
sel_VM1 =8;
sel_VM2 =9;
%Preallocate left hand side vector
%---------------------------
dy=[0; 0; 0; 0; 0; 0; 0; 0; 0];
%Preliminary terms to compute right hand side of governing equations
%target velocity magnitude
VT= sqrt(y(sel_VT1)^2 + y(sel_VT2)^2);

%relative positions and velocitiesRTM1=y(sel_RT1)-y(sel_RM1)
RTM2=y(sel_RT2)-y(sel_RM2)
RTM1=y(sel_RT1)-y(sel_RM1)
VTM1=y(sel_VT1)-y(sel_VM1)
VTM2= y(sel_VT2)-y(sel_VM2);

%relative distance
RTM=sqrt(RTM1^2 + RTM2^2)

%line of sight angle and time derivative 
lambda= atan2(RTM2, RTM1);
lambda_dot=(RTM1*VTM2 - RTM2*VTM1)/RTM^2;

%closing velocity

VC= -(RTM1*VTM1 + RTM2*VTM2)/RTM

%  DE RHS computations y = [beta, RTx, RTZ, RMX, RMZ, VTX, VTz, VMx, VMz]

dy(1) = aT/VT;
dy(2) = VT*cos(y(sel_beta));
dy(3) = VT*sin(y(sel_beta));
dy(4) = y(sel_VM1);
dy(5) = y(sel_VM2)
dy(6) = aT*sin(y(sel_beta));
dy(7) = aT*cos(y(sel_beta));

%compute LHS of pursuer acceleration equation depending on PN tyle

if strcmp(PN_type,'True')
       nc = Np*VC*lambda_dot;
       dy(8)= -nc*sin(lambda) ;
       dy(9) = nc*cos(lambda);
elseif strcmp(PN_type, 'Pure')
       Heading_pursuer = atan2(y(sel_VM2), y(sel_VM1)) ;
       nc = Np*VM*lambda_dot;
       dy(8) = -nc*sin(Heading_pursuer);
       dy(9) = nc*cos(Heading_pursuer);
else
     disp('Error: PN_type must be string with name ''Pure'' or ''True'' ')
     return
end
end
