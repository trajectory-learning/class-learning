function [pravaAccuCost] = cost_trajektorija_igra(Q)

global nbStanja;
global start;
% global goal
% global WW;
% global TauN;
% global nbTau;
global TauAttract;
global TauReject;
global oriVariability;
global Tori;
global posW;
global nbIter;
global States reward;
global reward1 reward2 reward3 reward4 reward5 reward6 reward7 reward8;
global reward1_raw reward2_raw reward3_raw reward4_raw reward5_raw reward6_raw reward7_raw reward8_raw;
global pocetna;

pravaAccuCost=0;


nbIter=nbIter+1

Q = reshape(Q, 3, nbStanja);

%% C1 - equal frame spacing

mDistance=[];
for i=1:nbStanja-1
    
    %kriterij medjusobne udaljenosti
    mDistance(i) = sqrt((Q(1,i)-Q(1,i+1))^2 + (Q(2,i)-Q(2,i+1))^2);
    if mDistance(i)<5
       mDistanceCost=100; 
       
    elseif mDistance(i)>50
       mDistanceCost=100; 
       
    else
        mDistanceCost=0;
    end
    
    %mDistanceCostFull=mDistanceCostFull+mDistanceCost;
end

mDistanceCostFull=std(mDistance);

C1=std(mDistance);


%% C3 - Trajectory length

DistanceTraveled=abs(sum(mDistance)-pocetna);


%% C8 – Maximum angle change
%       kriterij za orijentaciju - promjena ne smije biti pre velika izmedu dvije tocke
AngleCostFull=0;
AngleCost=0;
angle_array=[];
for i=1:nbStanja-1
    
    
    theta=Q(3,i)*pi/180;
    T1 = [cos(theta) sin(theta)
        -sin(theta) cos(theta)];
    
    
    theta=Q(3,i+1)*pi/180;
    T2 = [cos(theta) sin(theta)
        -sin(theta) cos(theta)];


    
    AngleDist = norm(T1-T2);
 
    
    angle_array(i) = AngleDist;    
        
end

AngleCostFull=sum(angle_array);


%% C2 – Maximum angle between frame vectors

angle=[];
for i=1:nbStanja-2
    
    a1= [Q(1:2,i);0];
    b1= [Q(1:2,i+1);0];
    c1= [Q(1:2,i+2);0];

    a=b1-a1;
    b=c1-b1;
    
%      a=abs(b1-a1);
%      b=abs(c1-b1);
    
    angle(i) = atan2(norm(cross(a,b)), dot(a,b));
end
angleCostSum=sum(angle);

angleCostMax=max(angle);

 %% kriterij tocke koja zavrsava u goalu
%  goalDistance=0;
%  goalDistance=sqrt((goal(1)-Q(1,nbStanja))^2 + (goal(2)-Q(2,nbStanja))^2);
 
 %% kriterij tocke koja pocinje u startu
 
 startDistance=0;
 startDistance=sqrt((start(1)-Q(1,1))^2 + (start(2)-Q(2,1))^2);
 
 %% C4 – Task parameter cost
 %       kriterij za prilazaka TauN
 TauNCostA=[];
 TauNVector=[];
 numMinDist=[];
 for j=1:size(TauAttract,2)
    for i=1:nbStanja
        %TauNVector(i) = sqrt((Q(1,i)-TauAttract(1,j))^2 + (Q(2,i)-TauAttract(2,j))^2);
        TauNVector(i) = sqrt((Q(1,i)-TauAttract{j}(1,3))^2 + (Q(2,i)-TauAttract{j}(2,3))^2);
    end
    [TauNCostA(1,j), numMinDist(1,j)] = min(TauNVector);
 end
 
 %% C5 – Orientation cost
 %       kriterij za orijentaciju attractor pointsa
 
 for i=1:size(numMinDist,2)
    
    %orijentacija generirane tocke
    theta = Q(3,numMinDist(i))*pi/180;
    T1 = [cos(theta) sin(theta) 
        -sin(theta) cos(theta)];
    
    
    %orijentacija tocke interesa u globalnom k.s.
    %theta = TauAttract(3,i);%*pi/180;
%     T2 = [cos(theta) sin(theta) 
%         -sin(theta) cos(theta)];

    T2=TauAttract{i}(1:2,1:2);
    
    T2 = T2*Tori{i};
    
    AttOriDist(i,1) = norm(T1-T2);
    
    
 end
 
   %% C6 – Collision cost
 TauNCostR=[];
 TauNVector=[];
 Wrej=[];
 for j=1:size(TauReject,2)
    for i=1:nbStanja
        %TauNVector(i) = sqrt((Q(1,i)-TauReject(1,j))^2 + (Q(2,i)-TauReject(2,j))^2);
        TauNVector(i) = sqrt((Q(1,i)-TauReject{j}(1,3))^2 + (Q(2,i)-TauReject{j}(2,3))^2);
    end
    TauNCostR(j) = abs(min(TauNVector));
    
    xxF=[];
    iN=[];
    
    x_start=20;
    x_step=20/200;

    for i=1:200
        xF = 0.98^i;
        xxF(i)=xF;
          
%         x_start=x_start - x_step;
%         xxF(i)=x_start;
    
        iN(i)=i;
    end
    
    if(TauNCostR(j)<1)
            Wrej(j)=0.9;
   
    elseif(TauNCostR(j)>200)
            Wrej(j)=0.00001;
    else
            Wrej(j) = interp1(iN,xxF,TauNCostR(j));
    end
            
 end
 
 
%% kriterij za jerk
  
%{
  Qdx=abs(diff(Q(1,:)));
  Qdy=abs(diff(Q(2,:)));
  
%   Qddx=abs(diff(Qdx));
%   Qddy=abs(diff(Qdy));
%   
%   Qdddx=abs(diff(Qddx));
%   Qdddy=abs(diff(Qddy));
  
  Jerk=max(Qdx)+max(Qdy);
  
   %Jerk=sum(Qdx)+sum(Qdy);
  
%   if Jerk<0.01
%       Jerk=10^8
%   end
  
  %}

%% smoothness
%{
Smth1=std(diff(Q(1,:)))/abs(mean(diff(Q(1,:))));
  
Smth2=std(diff(Q(2,:)))/abs(mean(diff(Q(2,:))));
  
Smth=Smth1+Smth2;
%}

%% ukupni cost
 
             
%cost za Attractor points           
taskParCost = posW*TauNCostA';

%cost za koliziju 
motionColCost = sum(Wrej);

%cost za varijabilnost orijentacije
orientationCost = oriVariability*AttOriDist;


%pravaAccuCost = abs(shapeCost + taskParCost + motionColCost + orientationCost);

%pravaAccuCost = abs(shapeCost + taskParCost + motionColCost); %standardna

pravaAccuCost =  abs((mDistanceCostFull - 0)/(17.5288-0)+...
                    2*(DistanceTraveled - 0)/(66.4948-0)+...
                    (startDistance - 0)/(14.2217-0)+...
                    20*(taskParCost - 0)/(900-0)+...
                    10*(motionColCost - 0)/(1-0))+...
                    5*(angleCostMax - 0)/(2.5-0)+...
                    (orientationCost - 0)/(4-0)+...
                    2*(AngleCostFull-0)/(4-0);
                
reward1_raw(nbIter,1) = mDistanceCostFull;
reward2_raw(nbIter,1) = DistanceTraveled;
reward3_raw(nbIter,1) = startDistance;
reward4_raw(nbIter,1) = taskParCost;
reward5_raw(nbIter,1) = motionColCost;
reward6_raw(nbIter,1) = angleCostMax;
reward7_raw(nbIter,1) = orientationCost;
reward8_raw(nbIter,1) = AngleCostFull;
                
                
%pravaAccuCost = taskParCost;

if (isnan(pravaAccuCost)==1)
    pravaAccuCost = 100000;
end

States{nbIter,1} = Q;
reward(nbIter,1) = pravaAccuCost;



reward1(nbIter,1) = (mDistanceCostFull - 0)/(250-0);
reward2(nbIter,1) = (DistanceTraveled - 0)/(320-0);
reward3(nbIter,1) = (startDistance - 0)/(150-0);
reward4(nbIter,1) = (taskParCost - 0)/(900-0);
reward5(nbIter,1) = (motionColCost - 0)/(1-0);
reward6(nbIter,1) = (angleCostMax - 0)/(2.5-0);
reward7(nbIter,1) = (orientationCost - 0)/(4-0);
reward8(nbIter,1) = (AngleCostFull-0)/(4-0);


end



   