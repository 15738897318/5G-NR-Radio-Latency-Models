function [latency, RButilization_avg, TRx, Tini, Tfinal]=delay_analysis_rep(FR,BW,SCS, N, Tp, nRBperUE,link_direction, cap, seed, N_MC, n_rep, BLER, traffic, MCS_table, v,data,escenario,density,fidUE,ferror)
%FR
%FR1: 410 MHz- 7125 MHz
%FR2: 24250 MHz- 52600 MHz

%BW
%FR1 (en MHz): 5, 10, 15, 20, 25, 30, 40, 50, 60, 70, 80, 90, 100
%FR2 (en MHz): 50, 100, 200, 400

%SCS
%FR1 (en kHz): 15, 30, 60
%FR2 (en kHz): 60, 120

% N is the number of users
% Tp is the avg time between packet arrivals
% direction indicates if DL or UL comm
%   direction=1 --> DL
%   direction=2 --> UL
% cap is the UE processing capability 1 or 2
% N_MC is the number of vehicles that have to receive the message
% from UE1. If unicast transmissions are performed in DL, N_MC
% unicast transmissions have to be performed


if traffic==0 %aperiodic
    iter=20;
    tsim=Tp*iter*4;
    scheduling='GB'; %Grant-based (Dynamic)
    limInf=1;
    Tini=8*Tp;
    frame_l=8*Tp;    
else          %periodic
    iter=4;
    tsim=Tp*iter;
    scheduling='GF'; %Grant-free (SPS in DL or Configured Grant in UL)
    limInf=0;
    Tini=0;
    frame_l=2*Tp; % ms %frames duration is equal to 10ms (frame_l=10).
            %to solve possible conflicting situation, we consider the frame
            %duration equal to the time period. This means that we consider
            %frame_l/10 frames jointly
end

if SCS==59
    SCS=60;
    CP='ECP';
else
    CP='NCP';
end

[NRB,Tslot,num]=RB_definition(FR,SCS,BW);
slots_per_frame=frame_l/Tslot;

[Tproc1,Tproc2]=processingTimeParameters(num,cap);

if link_direction==1
    link='DL';
    % Tproc,2 is defined in Section 6.4 of TS38.214: d2,1= d2,2= d2,3=0
    TprocTx=Tproc2/2;
    TprocRx=Tproc1/2;
    TprocTxHARQ=Tproc1/2;
    TprocRxHARQ=Tproc2/2;
elseif link_direction==2
    link='UL';
    TprocTx=Tproc2/2;
    TprocRx=Tproc1/2;
    TprocTxHARQ=Tproc1/2;
    TprocRxHARQ=Tproc2/2;
else
    disp('please, indicate a good link DL (1) or UL (2)');
    return;
end

if data>0
    [UEdistToBS,N,Rcell]=set_user_location(escenario,N,density);
    CQI_UEs=calculate_CQI(UEdistToBS,Rcell);
    [tbs_UEs, RBs_UEs]=calculate_nRBs_perUE(data,N,CQI_UEs,MCS_table,v,link_direction,SCS,BW,CP);
    x=find(RBs_UEs>NRB);
    if ~isempty(x)
        fprintf(ferror,'UEs demands a higher number of RBs than available per slot\n');
        fprintf(ferror,'%d\t',x);
        fprintf(ferror,'\n');
    end
    fprintf(fidUE,'%d\t',UEdistToBS);
	fprintf(fidUE,'\n');
    fprintf(fidUE,'%d\t',CQI_UEs);
	fprintf(fidUE,'\n');
    fprintf(fidUE,'%d\t',tbs_UEs);
	fprintf(fidUE,'\n');
    fprintf(fidUE,'%d\t',RBs_UEs);
	fprintf(fidUE,'\n');
else
    RBs_UEs=nRBperUE*ones(N,1);
end

Ta=packet_gen(N,Tp,iter,seed,Tslot,traffic,density);
TaOriginal=Ta;
switch scheduling
    case 'GF'
        Ta=Ta+TprocTx;
    case 'GB'
        if link_direction==2 %'UL'
            Ta=SR_Grant_process(Ta,Tslot,TprocTx,TprocRx,TprocTxHARQ,TprocRxHARQ);
        else %'DL'
            Ta=Ta+TprocTx;
            Ta=DL_Scheduling_process(Ta,Tslot,TprocTx,TprocRx,TprocTxHARQ,TprocRxHARQ);
%             incluir el tiempo entre que se envía el PDCCH con la
%             información de scheduling y el tiempo para enviar el paquete
%             de datos
        end
end

current_time_frame=Tslot*[0:slots_per_frame-1];
current_frame=zeros(NRB,slots_per_frame);
past_frame=zeros(NRB,slots_per_frame);
next_frame=zeros(NRB,slots_per_frame);

flagControl=1;
if flagControl
    if link_direction==1
    %     vamos a reservar RBs para SS/PBCH --> 1 block transmitted each 20
    %     slots. Each block is composed of 240 subcarriers x 4 OFDM symbols
    %     Nos ponemos en un peor caso, y reservamos el slot completo para 240
    %     subcarriers
        i_SSB=1;
        if NRB>=20 
            while i_SSB<=slots_per_frame
                current_frame(1:20,i_SSB)=1;
                i_SSB=i_SSB+20;
            end
            i_SSB=i_SSB-slots_per_frame;
            while i_SSB<=slots_per_frame
                next_frame(1:20,i_SSB)=1;
                i_SSB=i_SSB+20;
            end
        else
            disp('No hay RBs suficientes para el SS/PCBH block');
        end
    elseif link_direction==2
        i_RACH=1;
        if NRB>=12
            slotsperms=1/Tslot; %number of slots per subframe of duration 1ms
            while i_RACH+slotsperms-1<=slots_per_frame
                current_frame(1:12,i_RACH:i_RACH+slotsperms-1)=1;
                i_RACH=i_RACH+5*slotsperms;
            end
            i_RACH=i_RACH-slots_per_frame;
            while i_RACH<=slots_per_frame
                next_frame(1:12,i_RACH:i_RACH+slotsperms-1)=1;
                i_RACH=i_RACH+5*slotsperms;
            end
        else
            disp('No hay RBs suficientes para el PRACH block');
        end
    end
end


ind=ones(1,N);
latency=-1000*ones(N,iter);
Tb=Ta';

RButilization_avg=0;
nframes=0;
currentTime=0;
flag=0;
while 1
    i=iter*((1:N)-1)+ind(1:N);
    nextPacketperUser=Tb(i);
        
    [nextT,user]=min(nextPacketperUser);

    for nUERx=1:N_MC
        if 1    %todos los RBs a un UE deben asignarse en el mismo slot
            flag=0;
            while 1
                m=find(current_time_frame>=nextT);
                [n,p]=find(current_frame(:,m)==0);
                if isempty(m) || isempty(n)
                    past_frame=current_frame;
                    current_frame=next_frame;
                    next_frame=zeros(NRB,slots_per_frame);
                    current_time_frame=current_time_frame+frame_l;
                    if traffic==0 && nframes<limInf
                        nframes=nframes+1;
                    else
                        nframes=nframes+1;
                        RButilization_avg=RButilization_avg+sum(sum(past_frame))/(slots_per_frame*NRB);
                    end
                    if flagControl
                        if link_direction==1
                        %     vamos a reservar RBs para SS/PBCH --> 1 block transmitted each 20
                        %     slots. Each block is composed of 240 subcarriers x 4 OFDM symbols
                            i_SSB=i_SSB-slots_per_frame;
                            while i_SSB<=slots_per_frame
                                next_frame(1:20,i_SSB)=1;
                                i_SSB=i_SSB+20;
                            end
                        elseif link_direction==2
                            i_RACH=i_RACH-slots_per_frame;
                            while i_RACH<=slots_per_frame
                                next_frame(1:12,i_RACH:i_RACH+slotsperms-1)=1;
                                i_RACH=i_RACH+5*slotsperms;
                            end
                        end
                    end
                else
                    for h=1:length(m)
                        if m(h)<=slots_per_frame-(n_rep-1)
                            cons=0;
                            while cons<n_rep
                            	p=find(current_frame(:,m(h)+cons)==0);
                                if length(p)>=RBs_UEs(user)
                                    cons=cons+1;
                                else
                                    break;
                                end
                            end
                            if cons<n_rep
                                continue;
                            end
                            currentTime=current_time_frame(m(h));
                            if ind(user)<iter && TaOriginal(user,ind(user)+1)<=currentTime
                                latency(user,ind(user))=100000;
                                ind(user)=ind(user)+1;
                                if ind(user)>iter
                                    return;
                                end
                                flag=2;
                                break;
                            end
                            for cons=0:n_rep-1
                                p=find(current_frame(:,m(h)+cons)==0);
                                for k=1:RBs_UEs(user)
                                    current_frame(p(k),m(h)+cons)=1;
                                end
                            end
                            perr=rand(1,n_rep);
                            [x,y]=find(perr>BLER);
                            if isempty(x)
                                tend=200000;
                            else
                                tend=current_time_frame(m(h))+Tslot*n_rep; 
                            end
                            flag=1;
                            break;
                        else
                            cons=0;
                            while cons<n_rep
                                if m(h)+cons<=slots_per_frame
                                    p=find(current_frame(:,m(h)+cons)==0);
                                    if length(p)>=RBs_UEs(user)
                                        cons=cons+1;
                                    else
                                        break;
                                    end
                                else
                                    p=find(next_frame(:,m(h)+cons-slots_per_frame)==0);
                                    if length(p)>=RBs_UEs(user)
                                        cons=cons+1;
                                    else
                                        break;
                                    end
                                end
                            end
                            if cons<n_rep
                                continue;
                            end
                            currentTime=current_time_frame(m(h));
                            if ind(user)<iter && TaOriginal(user,ind(user)+1)<=currentTime
                                latency(user,ind(user))=100000;
                                ind(user)=ind(user)+1;
                                if ind(user)>iter
                                    return;
                                end
                                flag=2;
                                break;
                            end
                            for cons=0:n_rep-1
                                if m(h)+cons<=slots_per_frame
                                    p=find(current_frame(:,m(h)+cons)==0);
                                    for k=1:RBs_UEs(user)
                                        current_frame(p(k),m(h)+cons)=1;
                                    end
                                else
                                    p=find(next_frame(:,m(h)+cons-slots_per_frame)==0);
                                    for k=1:RBs_UEs(user)
                                        next_frame(p(k),m(h)+cons-slots_per_frame)=1;
                                    end
                                end
                            end
                            perr=rand(1,n_rep);
                            [x,y]=find(perr>BLER);
                            if isempty(x)
                                tend=200000;
                            else
                                tend=current_time_frame(m(h))+Tslot*n_rep; 
                            end
                            flag=1;
                            break;
                        end
                    end
                    if flag==2
                        break;
                    end
                    if flag==0
                         for h=1:slots_per_frame
                            currentTime=current_time_frame(end)+Tslot+Tslot*(h-1);
                            if ind(user)<iter && TaOriginal(user,ind(user)+1)<=currentTime
                                latency(user,ind(user))=100000;
                                ind(user)=ind(user)+1;
                                if ind(user)>iter
                                    return;
                                end
                                flag=2;
                                break;
                            end
                            cons=0;
                            while cons<n_rep
                                p=find(next_frame(:,h+cons)==0);
                                if length(p)>=RBs_UEs(user)
                                    cons=cons+1;
                                else
                                    break;
                                end
                            end
                            if cons<n_rep
                                continue;
                            end
                            for cons=0:n_rep-1                                
                                p=find(next_frame(:,h+cons)==0);
                                for k=1:RBs_UEs(user)
                                    next_frame(p(k),h+cons)=1;
                                end
                            end
                            perr=rand(1,n_rep);
                            [x,y]=find(perr>BLER);
                            if isempty(x)
                                tend=200000;
                            else
                                tend=current_time_frame(end)+Tslot+Tslot*(h+n_rep-1);
                            end
                            flag=1;
                            break;
                         end
                    end
                    if flag==2
                        break;
                    end
                    if flag==0
                        disp('error: no debería estar aquí');
                    end
                    break;
                end
            end
%         else    %un UE puede recibir RBs en 2 slots consecutivos
%             if ind(user)<iter && Tb(ind(user)+1,user)<=currentTime
%                 currentTime=tend-Tslot;
%                 latency(user,ind(user))=100000;
%                 ind(user)=ind(user)+1;
%                 if ind(user)>iter
%                     break;
%                 end
%                 continue;
%             end        
%             while 1
%                 m=find(current_time_frame>=nextT);
%                 [n,p]=find(current_frame(:,m)==0);
%                 if isempty(m) || isempty(n)
%                     past_frame=current_frame;
%                     current_frame=next_frame;
%                     next_frame=zeros(NRB,slots_per_frame);
%                     current_time_frame=current_time_frame+frame_l;
%                     nframes=nframes+1;
%                     RButilization_avg=RButilization_avg+sum(sum(past_frame))/(slots_per_frame*NRB);
%                     if flagControl
%                         if link_direction==1 && flagControl
%                         %     vamos a reservar RBs para SS/PBCH --> 1 block transmitted each 20
%                         %     slots. Each block is composed of 240 subcarriers x 4 OFDM symbols
%                             i_SSB=i_SSB-slots_per_frame;
%                             while i_SSB<=slots_per_frame
%                                 next_frame(1:20,i_SSB)=1;
%                                 i_SSB=i_SSB+20;
%                             end
%                         elseif link_direction==2
%                             i_RACH=i_RACH-slots_per_frame;
%                             while i_RACH<=slots_per_frame
%                                 next_frame(1:12,i_RACH:i_RACH+slotsperms-1)=1;
%                                 i_RACH=i_RACH+5*slotsperms;
%                             end
%                         end
%                     end
%                 else
%                     break;
%                 end
%             end
%             m=m(1);
%             n=find(current_frame(:,m(1))==0);
%             i=0;
%             while isempty(n)
%                 i=i+1;
%                 if m+i<=slots_per_frame
%                     n=find(current_frame(:,m+i)==0);
%                 else
%                     n=find(next_frame(:,m+i-slots_per_frame)==0);
%                 end
%             end
%             if m+i<=slots_per_frame
%                 current_frame(n(1),m+i)=1;
%                 count=1;
%                 j=0;
%                 flag=1;
%                 while count<RBs_UEs(user)
%                     j=j+1;
%                     if flag==1
%                         if n(1)+j<=NRB
%                             if current_frame(n(1)+j,m+i)==0
%                                 current_frame(n(1)+j,m+i)=1;
%                                 count=count+1;
%                             end
%                         else
%                             flag=0;
%                             i=i+1;
%                             j=0;
%                         end
%                     else
%                         if m+i<=slots_per_frame
%                             if current_frame(j,m+i)==0
%                                 current_frame(j,m+i)=1;
%                                 count=count+1;
%                             end
%                         else
%                             if next_frame(j,m+i-slots_per_frame)==0
%                                 next_frame(j,m+i-slots_per_frame)=1;
%                                 count=count+1;
%                             end
%                         end
%                     end
%                 end
%                 if m+i<=slots_per_frame
%                     tend=current_time_frame(m+i)+Tslot; %hay que añadir tiempos de procesado en el receptor
%                 else
%                     tend=current_time_frame(end)+Tslot+Tslot*(m+i-slots_per_frame);
%                 end
%             else
%                 j=0;
%                 count=0;
%                 flag=1;
%                 while count<RBs_UEs(user)
%                     if flag==1
%                         if n(1)+j<=NRB
%                             if next_frame(n(1)+j,m+i-slots_per_frame)==0
%                                 next_frame(n(1)+j,m+i-slots_per_frame)=1;
%                                 count=count+1;
%                             end
%                             j=j+1;
%                         else
%                             flag=0;
%                             i=i+1;
%                             j=0;
%                         end
%                     else
%                         if m+i-slots_per_frame<=slots_per_frame
%                             if next_frame(1+j,m+i-slots_per_frame)==0
%                                 next_frame(1+j,m+i-slots_per_frame)=1;
%                                 count=count+1;
%                             end
%                         else
%                             disp('error: no deberíamos llegar hasta aquí');
%                         end
%                     end
%                 end
%                 tend=current_time_frame(end)+Tslot+Tslot*(m+i-slots_per_frame); %hay que añadir tiempos de procesado en el receptor
%             end
%             currentTime=tend-Tslot;
        end
        if flag==2
            break;
        end
    end
    if flag==2
        continue;
    end
    if tend==200000
        latency(user,ind(user))=200000;
    else
        latency(user,ind(user))=TprocTx+(tend-nextT)+TprocRx;
    end
    ind(user)=ind(user)+1;
    if ind(user)>iter
        break;
    end
end
if traffic==0
	nframes=nframes-limInf; 
end
RButilization_avg=RButilization_avg/nframes*100;

TRx=TaOriginal+latency;
    
[x,y]=find(latency==200000);
for i=1:length(x)
    TRx(x(i),y(i))=200000;
end
[x,y]=find(latency==100000);
for i=1:length(x)
    TRx(x(i),y(i))=100000;
end
[x,y]=find(latency==-11);
for i=1:length(x)
    TRx(x(i),y(i))=-11;
end
[x,y]=find(TaOriginal<limInf*Tp);
for i=1:length(x)
    TRx(x(i),y(i))=-11;
end
for i=1:length(x)
    latency(x(i),y(i))=-11;
end
for i=1:length(x)
    retx(x(i),y(i))=-11;
end
[x,y]=size(TRx);
tt=reshape(TRx,1,x*y);
[x,y]=find(tt<100000);
Tfinal=max(tt(y));
