//definim constants

l1=0.62
l2=0.57
g1=0.1
g2=0.2
g3=0.3
x=0.9
y=-0.2
gamma=0
v=-0.1
n_iter=100
//definim constants de massa i g
m_l1=4
m_l2=3
m_EE=2
m_prod=1
g=9.81

//definim funció cinematica directe

function [x,y,gamma] = jointposition2endeffectorPose(theta1, theta2, theta3)


x=l1*cos(theta1)+l2*cos(theta1+theta2)+(g1+g3)*cos(theta1+theta2+theta3)+g2*sin(theta1+theta2+theta3)
y=l1*sin(theta1)+l2*sin(theta1+theta2)+(g1+g3)*sin(theta1+theta2+theta3)-g2*cos(theta1+theta2+theta3)
gamma=theta1+theta2+theta3

endfunction

//definim funció cinemtaica inversa

function [theta1, theta2, theta3] = inverse(x, y, gamma)

d=2*l1*l2
f=(x-(g1+g3)*cos(gamma)-g2*sin(gamma))^2 + (y-(g1+g3)*sin(gamma)+g2*cos(gamma))^2- (l1)^2- (l2)^2
a=f/d
theta2=acos(a)
if theta2>%pi then theta2=2*%pi-theta2 end

E=x-(g1+g3)*cos(gamma)-g2*sin(gamma)
F=y-(g1+g3)*sin(gamma)+g2*cos(gamma)
A=l1+l2*cos(theta2)
B=l2*sin(theta2)

matA=[E;F]
matB=[A, -B ; B, A]
matC=inv(matB)*matA

U=matC(1)
D=matC(2)
theta1=atan(D, U)

if theta1 > %pi then theta1 =2*%pi-theta1 end

theta3=(gamma-theta2-theta1)

endfunction


// Definim funció per trobar pose de les joints

function [pose_J1, pose_J2, pose_J3] = jointpose (theta1, theta2, theta3)

pose_J1=[0 ; 0]
pose_J2=[l1*cos(theta1) ; l1*sin(theta1) ]
pose_J3=[l1*cos(theta1)+l2*cos(theta1+theta2) ; l1*sin(theta1)+l2*sin(theta1+theta2)]

endfunction

//Definim funció per trobar jacobià, apartir de la pose de les joints i calcular velocitat joints

function [w1, w2, w3]=jointpose2angularvel(pose_J1, pose_J2, pose_J3)
J = [0, pose_J2(2), pose_J3(2); 0 , (-1)*pose_J2(1), (-1)*pose_J3(1); 1,1,1]

T=[0 ; v ; gamma]
W = inv(J) * T //W serà una matriu 3x1

w1 = W(1)
w2 = W(2)
w3 = W(3)

endfunction

//funció per representar el moviemnt del robot

function []= plot3Rmov (pose_J1, pose_J2, pose_J3)
P_g1x = pose_J3(1) + g1*(cos(theta3 + theta2 + theta1));
P_g1y = pose_J3(2) + g1*(sin(theta3 + theta2 + theta1));
P_g2x = P_g1x;
P_g2y = P_g1y - g2;
P_g3x = P_g2x + g3;
P_g3y = P_g2y;

subplot(221)
title("Motion 3R arm", "fontsize",5)

a=get("current_axes") 
a.data_bounds = [-0.2,-1;1,0] //limit eixos
a.axes_visible="on";
xlabel("m")
ylabel("m")
//Borrar anterior
a = gca();
delete(a.children);

//Dibuixar segments entre joints per representar links
xsegs([pose_J1(1),pose_J2(1)],[pose_J1(2),pose_J2(2)],1:1);
xsegs([pose_J2(1),pose_J3(1)],[pose_J2(2),pose_J3(2)],1:1);
xsegs([pose_J3(1),P_g1x],[pose_J3(2),P_g1y],1:1);
xsegs([P_g1x,P_g2x],[P_g1y,P_g2y],1:1);
xsegs([P_g2x,P_g3x],[P_g2y,P_g3y],1:1);

s = a.children
s.thickness=3
endfunction

//definim funció per representar les velocitats angulars de cada joint respecte el temps

function[] = plotW(w1,w2,w3,t)

subplot(222)
title("Angular speed - Time", "fontsize",5)
a=get("current_axes") //eixos
a.data_bounds = [0,-0.25;10,0.3]
xlabel("seconds")
ylabel("rad/s")

plot2d(t,w1, 0)
e = gce();
point = e.children(1);
point.mark_mode="on";
point.mark_size =1;
point.mark_foreground=3;

plot2d(t,w2, 0);
e = gce();
point = e.children(1);
point.mark_mode="on";
point.mark_size =1;
point.mark_foreground=2;

plot2d(t,w3, 0);
e = gce();
point = e.children(1);
point.mark_mode="on";
point.mark_size =1;
point.mark_foreground=4;

hl=legend(['w1';'w2';'w3'])

endfunction

//definim funció pel calcul del parell total
function [thau]=torque(theta1, theta2, gamma, pose_J1, pose_J2, pose_J3)

//calculem posició centre de masses
pose_cm1=[pose_J1(1)+(l1/2)*cos(theta1);pose_J2(2)+(l1/2)*sin(theta1)]
pose_cm2=[pose_J2(1)+(l2/2)*cos(theta1+theta2);pose_J2(2)+(l2/2)*sin(theta1+theta2)]
pose_cm3=[pose_J3(1)+(g1+g3/2)*sin(gamma); pose_J3(2)-g2*cos(gamma)]

//calcul del parell(sum F·d)
thau=m_l1*g*pose_cm1(1)+m_l2*g*pose_cm2(1)+m_EE*g*pose_cm3(1)

endfunction

//funcio per representar el torque respecte el temps
function[]=plottorque(thau,t)

subplot(223)
title("Torque - Time", "fontsize",5)
a=get("current_axes") //eixos
a.data_bounds = [0,0;10,50]
xlabel("seconds")
ylabel("N·m")

plot2d(t,thau, 0)
e = gce();
point = e.children(1);
point.mark_mode="on";
point.mark_size =0.9;
point.mark_foreground=5;

endfunction



figure()
clf()

//establim bucle pel moviment de baixada

for t=0:(5/n_iter):5

// partint de la posició iniocial fem calcul cinemàtica inversa
[theta1,theta2,theta3] = inverse(x,y,gamma);

// calculem POSE joints
[pose_J1, pose_J2, pose_J3] = jointpose (theta1, theta2, theta3);

// Representem el robot fent servir la POSE de les joints
 plot3Rmov (pose_J1, pose_J2, pose_J3);

//Cálculem velocitats angulars de cada joint
[w1, w2, w3]=jointpose2angularvel(pose_J1, pose_J2, pose_J3);

//Representem la velocitat angular de cada joint respecte el temps
plotW(w1,w2,w3,t);

//A cada segon que passa afeguim un kg de pes al end-effector
if (t==1|t==2|t==3|t==4|t==5) then
    m_EE=m_EE+m_prod;
end

//calcul del parell a J1
[thau]=torque(theta1, theta2, gamma, pose_J1, pose_J2, pose_J3)

//Representem torque respecte temps
plottorque(thau,t)


y= y-(0.5/n_iter)

end
//decarraguem el end-effector

m_EE=2

//definim bucle pel moviment de pujada
for t=5:(5/n_iter):10

// partint de la posició final fem calcul cinemàtica inversa
[theta1,theta2,theta3] = inverse(x,y,gamma);

// calculem POSE joints
[pose_J1, pose_J2, pose_J3] = jointpose (theta1, theta2, theta3);

// Representem el robot fent servir la POSE de les joints
 plot3Rmov (pose_J1, pose_J2, pose_J3);

//Cálculem velocitats angulars de cada joint
[w1, w2, w3]=jointpose2angularvel(pose_J1, pose_J2, pose_J3);

//Representem la velocitat angular de cada joint respecte el temps
plotW(w1,w2,w3,t);

//calcul del parell a J1
[thau]=torque(theta1, theta2, gamma, pose_J1, pose_J2, pose_J3)

//Representem torque respecte temps
plottorque(thau,t)



y= y+(0.5/n_iter)

end



