function [latency, RButilization_avg, retx, TRx, Tini, Tfinal]=delay_analysis(FR,BW,SCS, N, Tp, nRBperUE,link_direction, cap, seed, N_MC, BLER, maxN_retx, traffic, MCS_table,v,data,escenario,density,fidUE,flagDiscardPkts,RBallocation,segmentationFactor,PDCCH_config,PUCCH_config,flag_control,minislot_config,ferror)
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
	deadtime=Tp;
    iter=30;
    tsim=Tp*iter*4;
    scheduling='GB'; %Grant-based (Dynamic)
    limInf=1;
    Tini=limInf*Tp;
    frame_l=8*Tp;    
else          %periodic
    if RBallocation==1
        segmentationFactor=4;
        deadtime=4*Tp;
        tsim=1000; %10 segundos 
        iter=tsim*segmentationFactor/Tp; %10 segundos entre el periodo
        limInf=0;
        Tini=0;
        frame_l=200; %200 ms
        if frame_l<10*Tp
            frame_l=frame_l*2;
        end
    else
    	deadtime=Tp;
        iter=15;
        tsim=Tp*iter;
        limInf=0;
        Tini=0;
        frame_l=2*Tp; % ms %frames duration is equal to 10ms (frame_l=10). 
                    %to solve possible conflicting situation, we consider the frame
                    %duration equal to the time period. This means that we consider
                    %frame_l/10 frames jointly
    end
	scheduling='GF'; %Grant-free (SPS in DL or Configured Grant in UL)
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
    disp('please, indicate a good link option DL (1) or UL (2)');
    return;
end

switch minislot_config
    case 0
        switch CP
            case 'NCP'
                N_OS=14;
            case 'ECP'
                N_OS=12;
        end
    case 1
        switch CP
            case 'NCP'
                N_OS=7;
            case 'ECP'
                N_OS=6;
        end
    case 2
        N_OS=4;
    case 3
        N_OS=2;
end

switch CP
    case 'NCP'
        symbPerSlot=14;
    case 'ECP'
        symbPerSlot=12;
end

if data>0
    [UEdistToBS,N,Rcell]=set_user_location(escenario,N,density);
    CQI_UEs=calculate_CQI(UEdistToBS,Rcell);
    [tbs_UEs, RBs_UEs]=calculate_nRBs_perUE(data,N,CQI_UEs,MCS_table,v,link_direction,SCS,BW,CP,PDCCH_config,PUCCH_config,N_OS);
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

ind=ones(1,N);
latency=-11*ones(N,iter);

Ta=packet_gen(N,Tp,iter,seed,Tslot,traffic,density,segmentationFactor);
Ta=[Ta tsim*2*ones(N,1)];
TaOriginal=Ta;
switch scheduling
    case 'GF'
        Ta=Ta+TprocTx;
    case 'GB'
        if link_direction==2 %'UL'
            [Ta,latency]=SR_Grant_process(Ta,Tslot,TprocTx,TprocRx,TprocTxHARQ,TprocRxHARQ,latency,tsim,PDCCH_config,PUCCH_config,flag_control);
        else %'DL'
            [Ta,latency]=DL_Scheduling_process(Ta,Tslot,TprocTx,TprocRx,TprocTxHARQ,TprocRxHARQ,latency,tsim,PDCCH_config,flag_control);
%             incluir el tiempo entre que se envía el PDCCH con la
%             información de scheduling y el momento en el que se envía el
%             paquete de datos
        end
end
Tb=Ta';

% current_time_frame=Tslot*[0:slots_per_frame-1];
% current_frame=zeros(NRB,slots_per_frame);
% past_frame=zeros(NRB,slots_per_frame);
% next_frame=zeros(NRB,slots_per_frame);
set_frames_variables;

flagControl=1;
if flagControl
    if link_direction==1
    %     vamos a reservar RBs para SS/PBCH --> 1 block transmitted each 20
    %     slots. Each block is composed of 240 subcarriers x 4 OFDM symbols
    %     Nos ponemos en un peor caso, y reservamos el slot completo para 240
    %     subcarriers
        if minislot_config==0
            N_OS=1;
            symbPerSlot=1;
            factor=0;
        else
            factor=4-1;
        end
	symbols_per_frame=slots_per_frame*symbPerSlot;
        i_SSB=1;
        if NRB>=20
            while i_SSB<=symbols_per_frame
                current_frame(1:20,i_SSB:i_SSB+factor)=1;
                i_SSB=i_SSB+20*symbPerSlot;
            end
            i_SSB=i_SSB-symbols_per_frame;
            while i_SSB<=symbols_per_frame
                next_frame(1:20,i_SSB:i_SSB+factor)=1;
                i_SSB=i_SSB+20*symbPerSlot;
            end
        else
            disp('No hay RBs suficientes para el SS/PCBH block');
        end
    elseif link_direction==2
        if minislot_config==0
            N_OS=1;
            symbPerSlot=1;
            factor=0;
        else
            factor=6-1;
        end
	symbols_per_frame=slots_per_frame*symbPerSlot;
        i_RACH=1;
        if NRB>=12
            slotsperms=1/Tslot; %number of slots per subframe of duration 1ms
            while i_RACH+slotsperms-1<=slots_per_frame
                iaux=0;
                while iaux<slotsperms
                    i_RACHaux=(i_RACH-1+iaux)*symbPerSlot+1;
                    current_frame(1:12,i_RACHaux:i_RACHaux+factor)=1;
                    iaux=iaux+1;
                end
                %current_frame(1:12,i_RACH:i_RACH+slotsperms-1)=1;
                i_RACH=i_RACH+5*slotsperms;
            end
            i_RACH=i_RACH-slots_per_frame;
            while i_RACH<=slots_per_frame
                iaux=0;
                while iaux<slotsperms
                    i_RACHaux=(i_RACH-1+iaux)*symbPerSlot+1;
                    next_frame(1:12,i_RACHaux:i_RACHaux+factor)=1;
                    iaux=iaux+1;
                end
%                 next_frame(1:12,i_RACH:i_RACH+slotsperms-1)=1;
                i_RACH=i_RACH+5*slotsperms;
            end
        else
            disp('No hay RBs suficientes para el PRACH block');
        end
    end
end


retx=zeros(N,iter,N_MC);
retx_time=100000*ones(N,iter,N_MC);
fretx_GB=zeros(N,iter);
t_ini=-1000*ones(N,iter);

RButilization_avg=0;
nframes=0;
currentTime=0;
flag=0;
while 1
%     si queremos dejar que un paquete pueda experimentar una latencia
%     mayor que Tp, lo que podemos hacer es:
%     - para paquetes para los cuales se han transmitido todas sus copias
%       correctamente, Tb(ind(user),user)=tsim*4;
%     - de esta manera, las siguientes 2 instrucciones se sustituirían por
      [nextPacketperUser,iTb]=min(Tb,[],1);
      [nextT,user]=min(nextPacketperUser);
      if nextT>=tsim
          break;
      end
      if flagDiscardPkts==1 && TaOriginal(user,ind(user)+1)>=tsim
          break;
      end
      ind(user)=iTb(user);
      if ind(user)>=iter
          break;
      end

%     i=iter*((1:N)-1)+ind(1:N);
%     nextPacketperUser=Tb(i);
%     [nextT,user]=min(nextPacketperUser);

    retx_index=1;
    if latency(user,ind(user))>=300000
        Tb(ind(user),user)=4*tsim;
        continue;
    end

    for nUERx=1:N_MC
        if 1    %todos los RBs a un UE deben asignarse en el mismo slot
            fretx=0;
            flag=0;
            while 1
                m=find(current_time_frame>=nextT);
                [n,p]=find(current_frame(:,m)==0);
                if isempty(m) || isempty(n)
                    if strcmp(scheduling,'GF')
                        [retx,retx_time,current_frame,latency,flag,fretx]=schedule_retxs(retx,retx_time,current_frame,current_time_frame,next_frame,frame_l,TaOriginal',latency,t_ini,RBs_UEs,Tslot,iter,TprocRx,TprocTx,BLER,N_MC,maxN_retx,Tp,flagDiscardPkts,deadtime,scheduling,N_OS,symbPerSlot);
                    end
                    past_frame=current_frame;
                    current_frame=next_frame;
                    next_frame=zeros(NRB,symbols_per_frame);%recordar que symbols_per_frame es igual a symbols_per_frame cuando consideramos full-slot tx
                    current_time_frame=current_time_frame+frame_l;
                    if traffic==0 && nframes<limInf
                        nframes=nframes+1;
                    else
                        nframes=nframes+1;
                        RButilization_avg=RButilization_avg+sum(sum(past_frame))/(slots_per_frame*symbPerSlot*NRB);
                    end
                    if flagControl
                        if link_direction==1
                        %     vamos a reservar RBs para SS/PBCH --> 1 block transmitted each 20
                        %     slots. Each block is composed of 240 subcarriers x 4 OFDM symbols
                            i_SSB=i_SSB-symbols_per_frame;
                            while i_SSB<=symbols_per_frame
                                next_frame(1:20,i_SSB:i_SSB+factor)=1;
                                i_SSB=i_SSB+20*symbPerSlot;
                            end
                        elseif link_direction==2
                            i_RACH=i_RACH-slots_per_frame;
                            while i_RACH<=slots_per_frame
                                iaux=0;
                                while iaux<slotsperms
                                    i_RACHaux=(i_RACH-1+iaux)*symbPerSlot+1;
                                    next_frame(1:12,i_RACHaux:i_RACHaux+factor)=1;
                                    iaux=iaux+1;
                                end
                                %next_frame(1:12,i_RACH:i_RACH+slotsperms-1)=1;
                                i_RACH=i_RACH+5*slotsperms;
                            end
                        end
                    end
                else
                    if strcmp(scheduling,'GB') && nUERx==1 && fretx_GB(user,ind(user))==1
                        fretx_GB(user,ind(user))=0;
                        [retx(user,ind(user),:),retx_time(user,ind(user),:),current_frame,latency(user,ind(user)),flag,fretx]=schedule_retxs(retx(user,ind(user),:),retx_time(user,ind(user),:),current_frame,current_time_frame,next_frame,frame_l,TaOriginal(user,ind(user):end)',latency(user,ind(user)),t_ini(user,ind(user)),RBs_UEs(user),Tslot,iter,TprocRx,TprocTx,BLER,N_MC,maxN_retx,Tp,flagDiscardPkts,deadtime,scheduling,N_OS,symbPerSlot);
                        if flag==2 || sum(retx_time(user,ind(user),:))==N_MC*100000
                            Tb(ind(user),user)=4*tsim;
                        else
                        	Tb(ind(user),user)=retx_time(user,ind(user),1);
                            fretx_GB(user,ind(user))=1;
                        end
                        flag=2;
                        break;
                    else
                        if flagDiscardPkts==1
                            deadtime=TaOriginal(user,ind(user)+1)-TaOriginal(user,ind(user));
                        end
                        [current_frame,tend,flag,fretx]=schedule_tx(nextT,ind(user),user,current_frame,current_time_frame,TaOriginal',RBs_UEs(user),Tslot/symbPerSlot,iter,BLER,Tp,deadtime,N_OS);
                        if flag==2
                            latency(user,ind(user))=100000;
%                             ind(user)=ind(user)+1;
%                             if ind(user)>iter
%                                 return;
%                             end
                            Tb(ind(user),user)=4*tsim;
                            break;
                        end
                    
                        if fretx==1
                            retx(user,ind(user),retx_index)=retx(user,ind(user),retx_index)+1;
                            if retx(user,ind(user),retx_index)<=maxN_retx
                                auxtime=tend+TprocRx+TprocTxHARQ;
                                if link_direction==1
                                    %el NACK se envía en PUCCH that we
                                    %establish that is transmitted in the last
                                    %symbol of every slot
                                    ACK_time=ceil(auxtime/Tslot)*Tslot+TprocRxHARQ;
                                    %a continuaciÃ³n sumamos el tiempo
                                    %de scheduling de la retransmisiÃ³n
                                    %que estÃ¡ basado en dynamic sch
                                    [retx_time(user,ind(user),retx_index),latency(user,ind(user))]=DL_Scheduling_process(ACK_time,Tslot,TprocTx,TprocRx,TprocTxHARQ,TprocRxHARQ,latency(user,ind(user)),tsim,PDCCH_config,flag_control);
                                else
                                    %el NACK se envía en PDCCH that we
                                    %establish that is transmitted in the first
                                    %symbol of every slot
                                    auxtime2=mod(auxtime,Tslot); %Tslot-auxtime2 is the allignment time
                                    ACK_time=auxtime+(Tslot-auxtime2)+Tslot/14+TprocRxHARQ; %(Tslot-auxtime2) es el frame alignment hasta el siguiente slot
                                    %a continuaciÃ³n sumamos el tiempo
                                    %de scheduling de la retransmisiÃ³n
                                    %que estÃ¡ basado en dynamic sch
                                    [retx_time(user,ind(user),retx_index),latency(user,ind(user))]=SR_Grant_process(ACK_time,Tslot,TprocTx,TprocRx,TprocTxHARQ,TprocRxHARQ,latency(user,ind(user)),tsim,PDCCH_config,PUCCH_config,flag_control);
                                end
                                t_ini(user,ind(user))=TaOriginal(user,ind(user));%nextT-TprocTx;
                                if strcmp(scheduling,'GB')
                                    Tb(ind(user),user)=retx_time(user,ind(user),retx_index);
                                    fretx_GB(user,ind(user))=1;
                                end
                                retx_index=retx_index+1;
                            end
                        end
                        if flag==0
                            if flagDiscardPkts==1
                                deadtime=TaOriginal(user,ind(user)+1)-TaOriginal(user,ind(user));
                            end
                            [next_frame,tend,flag,fretx]=schedule_tx(nextT,ind(user),user,next_frame,current_time_frame+frame_l,TaOriginal',RBs_UEs(user),Tslot/symbPerSlot,iter,BLER,Tp,deadtime,N_OS);
                            if flag==2
                                latency(user,ind(user))=100000;
    %                             ind(user)=ind(user)+1;
    %                             if ind(user)>iter
    %                                 return;
    %                             end
                                Tb(ind(user),user)=4*tsim;
                                break;
                            end
                            if fretx==1
                                retx(user,ind(user),retx_index)=retx(user,ind(user),retx_index)+1;
                                if retx(user,ind(user),retx_index)<=maxN_retx
                                    auxtime=tend+TprocRx+TprocTxHARQ;
                                    if link_direction==1
                                        %el NACK se envía en PUCCH that we
                                        %establish that is transmitted in the last
                                        %symbol of every slot
                                        ACK_time=ceil(auxtime/Tslot)*Tslot+TprocRxHARQ;
                                        %a continuaciÃ³n sumamos el tiempo
                                        %de scheduling de la retransmisiÃ³n
                                        %que estÃ¡ basado en dynamic sch
                                        [retx_time(user,ind(user),retx_index),latency(user,ind(user))]=DL_Scheduling_process(ACK_time,Tslot,TprocTx,TprocRx,TprocTxHARQ,TprocRxHARQ,latency(user,ind(user)),tsim,PDCCH_config,flag_control);
                                    else
                                        %el NACK se envía en PDCCH that we
                                        %establish that is transmitted in the first
                                        %symbol of every slot
                                        auxtime2=mod(auxtime,Tslot);
                                        ACK_time=auxtime+(Tslot-auxtime2)+Tslot/14+TprocRxHARQ; %(Tslot-auxtime2) es el frame alignment hasta el siguiente slot
                                        %a continuaciÃ³n sumamos el tiempo
                                        %de scheduling de la retransmisiÃ³n
                                        %que estÃ¡ basado en dynamic sch
                                        [retx_time(user,ind(user),retx_index),latency(user,ind(user))]=SR_Grant_process(ACK_time,Tslot,TprocTx,TprocRx,TprocTxHARQ,TprocRxHARQ,latency(user,ind(user)),tsim,PDCCH_config,PUCCH_config,flag_control);
                                    end
                                    t_ini(user,ind(user))=TaOriginal(user,ind(user));%nextT-TprocTx;
                                    if strcmp(scheduling,'GB')
                                        Tb(ind(user),user)=retx_time(user,ind(user),retx_index);
                                        fretx_GB(user,ind(user))=1;
                                    end
                                    retx_index=retx_index+1;
                                end
                            end
                        end
                        if flag==0
                            disp('error: no debería estar aquí');
                        end
                    end
                    break;
                end
            end
        end
        if flag==2
            break;
        end
    end
    if flag==2
        continue;
    end
    aux=0;
    if fretx_GB(user,ind(user))==0
        retx_index=1;
        while retx_index<=N_MC && retx(user,ind(user),retx_index)<=maxN_retx
            retx_index=retx_index+1;
        end
        if retx_index<=N_MC
            latency(user,ind(user))=200000;
        else
    %         latency(user,ind(user))=TprocTx+(tend-nextT)+TprocRx;
            latency(user,ind(user))=tend+TprocRx-TaOriginal(user,ind(user));
        end
        Tb(ind(user),user)=4*tsim;
    end
end

if traffic==0
	nframes=nframes-limInf; 
end
RButilization_avg=RButilization_avg/nframes*100;

TaOriginal=TaOriginal(:,1:end-1);
TRx=TaOriginal+latency;
    
[x,y]=find(latency==300000);
for i=1:length(x)
    TRx(x(i),y(i))=300000;
end

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
    latency(x(i),y(i))=-11;
    retx(x(i),y(i))=-11;
end
[x,y]=size(TRx);
tt=reshape(TRx,1,x*y);
[x,y]=find(tt<100000);
Tfinal=max(tt(y));
