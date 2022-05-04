function [NRB,Tslot,num]=RB_definition(FR,SCS,BW)

% returns the number of RBs per slot (NRB) and the time duration of a slot 
% based on the numerology (Tslot)

BWA1=[5, 10, 15, 20, 25, 30, 40, 50, 60, 70, 80, 90, 100];
BWA2=[50, 100, 200, 400];
SCSA1=[15, 30, 60];
SCSA2=[60, 120];
NRB1=[25 52 79 106 133 160 216 270 NaN NaN NaN NaN NaN;
      11 24 38 51 65 78 106 133 162 189 217 245 273;
      NaN 11 18 24 31 38 51 65 79 93 107 121 135];
NRB2=[66 132 264 NaN
      32 66 132 264];

Tslot1=[1 0.5 0.25];
Tslot2=[0.25 0.125];

num1=[0 1 2];
num2=[2 3];

if FR==1
    i=find(SCSA1==SCS);
    j=find(BWA1==BW);
    if isempty(i)
        disp('Non valid SCS value.');
        exit;
    end
    if isempty(j)
        disp('Non valid BW value.');
        exit;
    end
    num=num1(i);
    Tslot=Tslot1(i);
    NRB=NRB1(i,j);
    if isnan(NRB)
        disp('Non valid combination BW-SCS.');
        exit;
    end
elseif FR==2
    i=find(SCSA2==SCS);
    j=find(BWA2==BW);
    if isempty(i)
        disp('Non valid SCS value.');
        exit;
    end
    if isempty(j)
        disp('Non valid BW value.');
        exit;
    end
    num=num2(i);
    Tslot=Tslot2(i);
    NRB=NRB2(i,j);
    if isnan(NRB)
        disp('Non valid combination BW-SCS.');
        exit;
    end
else
    disp('Non valid FR value.');
    exit;
end
