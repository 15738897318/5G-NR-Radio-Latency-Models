function [latency, RButilization_avg, retx, TRx, Tini, Tfinal]=delay_analysis_periodic_rep(FR,BW,SCS, N, Tp, nRBperUE,link_direction, cap, seed, N_MC, n_rep, BLER, maxN_retx, traffic, MCS_table,v,data,escenario,density,fidUE,flagDiscardPkts,RBallocation,segmentationFactor,PDCCH_config,PUCCH_config,flag_control,minislot_config,ferror)
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
%         segmentationFactor=4;
        deadtime=segmentationFactor*Tp;
        limInf=1;
        Tini=0;
        frame_l=max(3*Tp,deadtime); %200 ms
        while frame_l<3*Tp || frame_l<deadtime
            frame_l=frame_l*2;
        end
        tsim=2*frame_l+2*deadtime; %1 segundos 
        iter=tsim*segmentationFactor/Tp; %10 segundos entre el periodo
    else
    	deadtime=Tp;
        iter=4;
        tsim=Tp*iter;
        limInf=1;
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
    disp('please, indicate a good link DL (1) or UL (2)');
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
elseif RBallocation==1
    
else
    RBs_UEs=nRBperUE*ones(N,1);
end

ind=ones(1,N);
latency=-11*ones(N,iter);

Ta=packet_gen(N,Tp,tsim/Tp,seed,Tslot,traffic,density,segmentationFactor);
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

aux_frame=[];%zeros(NRB,slots_per_frame);
aux_nextframe=[];%zeros(NRB,slots_per_frame);


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
    aux_frame=current_frame;
    aux_nextframe=next_frame;
end

Tsymbol=Tslot/symbPerSlot;
retx=zeros(N,iter,N_MC);
retx_time=100000*ones(N,iter,N_MC);
fretx_GB=zeros(N,iter);
t_ini=-1000*ones(N,iter);

toSchedule=zeros(N*segmentationFactor*N_MC,4); % en la primera columna guardo el usuario que debe 
                        % recibir recursos y en la segunda, guardo el slot
                        % en la siguiente trama en la que debe ser
                        % allocated, en la tercera columna guardo el
                        % indice al paquete para el cual se piden recursos,
                        % y en la cuarta columna guardo el numero de la
                        % transmisions en DL correspondiente en el caso de
                        % multiples unicast tx en DL.
indtoSchedule=1;
toScheduleNextFrame=zeros(N*segmentationFactor*N_MC,4); 
indtoScheduleNextFrame=1;

SegmentAllocated=-1*ones(N,segmentationFactor,N_MC);
iSegment=zeros(N,N_MC);
aux_ind=zeros(N,1);
ind_original=ones(N,1);
% ind_MC=ones(N,N_MC);

RButilization_avg=0;
nframes=0;
currentTime=0;
flag=0;
flagPeriodicAllocation=0;
N_MCminV=ones(N,1);
N_MCmax=N_MC;
niter=0;

flagUser=zeros(N,1);

while 1
    niter=niter+1;
%     disp(niter);
    if flagPeriodicAllocation==0
        m=find(ind>=iter);
        if length(m)==length(ind)
            break;
        end
        m=find(flagUser==1);
        if length(m)==N
            flagPeriodicAllocation=1;
            m=[];
            N_MCmin=1;
            N_MCmax=1;
        else
            x=find(flagUser==0);
            ind(x)=ind_original(x);
            i=((iter)+1)*((1:N)-1)+ind(1:N);
            nextPacketperUser=Tb(i);
            x=find(flagUser==1);
            nextPacketperUser(x)=tsim*100;

            [nextT,user]=min(nextPacketperUser);
    %         disp(user)
            retx_index=1;

            m=find(current_time_frame>=nextT);
            N_MCmin=N_MCminV(user);
            N_MCmax=N_MC;
            if nframes==0 && iSegment(user,N_MCmin)>=0
                aux_ind(user)=ind(user)+1;
                ind_original(user)=ind(user);
            end
        end
    else
        [mSch,nSch]=size(toSchedule);
        if indtoSchedule>mSch
            flagPeriodicAllocation=0;
            continue;
        end
        user=toSchedule(indtoSchedule,1);
        if user==0 
            flagPeriodicAllocation=0;
            continue;
        end
        next_slot=toSchedule(indtoSchedule,2);
        ind(user)=toSchedule(indtoSchedule,3);
        N_MCmin=toSchedule(indtoSchedule,4);
        N_MCmax=toSchedule(indtoSchedule,4);
        indtoSchedule=indtoSchedule+1;
        if ind(user)>=iter
            break;
        end
        
        retx_index=1;
        nextT=Tb(ind(user),user);
        m=next_slot;
    end
%     if seed==31
%         disp([user ind(user)]);
%     end

    for nUERx=N_MCmin:N_MCmax

        if nframes>0
            aux_frame=current_frame;
            aux_nextframe=next_frame;
        end

        if flagPeriodicAllocation==0
            if nUERx>1
                ind(user)=ind_original(user);
            end
%             if nframes==0 && iSegment(user,nUERx)>=0
            if iSegment(user,nUERx)>=0
                iSegment(user,nUERx)=iSegment(user,nUERx)+1;
                [n,p]=find(aux_frame(:,m)==0);
            else
                [n,p]=find(current_frame(:,m)==0);
            end
        else
            [n,p]=find(current_frame(:,m)==0);
        end

        if isempty(m) || isempty(n)
            if flagPeriodicAllocation==0            
                if flagDiscardPkts==1
                    deadtime=TaOriginal(user,ind(user)+1)-TaOriginal(user,ind(user));
                end
                if ind(user)<iter && current_time_frame(end)-TaOriginal(user,ind(user))>=deadtime
                    flag=2;
                    while ind(user)<=iter
                        latency(user,ind(user))=100000;
                        ind(user)=ind(user)+segmentationFactor;
                    end
                    SegmentAllocated(user,iSegment(user,nUERx),nUERx)=0;
                    nUERx=nUERx+1;
                    while nUERx<=N_MCmax
                        iSegment(user,nUERx)=iSegment(user,nUERx)+1;
                        SegmentAllocated(user,iSegment(user,nUERx),nUERx)=0;
                        nUERx=nUERx+1;
                    end
                    if flagPeriodicAllocation==0
                        flagUser(user)=1;
                    end
                    break;
                end
                if nframes==0 && iSegment(user,nUERx)>=0
                    iSegment(user,nUERx)=iSegment(user,nUERx)-1;
                end
            end
            if strcmp(scheduling,'GF')
                [retx,retx_time,current_frame,latency,flag,fretx]=schedule_retxs(retx,retx_time,current_frame,current_time_frame,next_frame,frame_l,TaOriginal',latency,t_ini,RBs_UEs,Tslot,iter,TprocRx,TprocTx,BLER,N_MC,maxN_retx,Tp,flagDiscardPkts,deadtime,scheduling,N_OS,symbPerSlot);
            end
            past_frame=current_frame;
            current_frame=next_frame;
            next_frame=zeros(NRB,symbols_per_frame);%recordar que symbols_per_frame es igual a slots_per_frame cuando consideramos full-slot tx
            current_time_frame=current_time_frame+frame_l;
            toSchedule=toScheduleNextFrame;
            toScheduleNextFrame=zeros(N*segmentationFactor*N_MC,4);
            indtoSchedule=1;
            indtoScheduleNextFrame=1;
            flagPeriodicAllocation=1;
        
            if traffic==0 && nframes<limInf
                nframes=nframes+1;
            elseif traffic==1 && nframes<limInf
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
            break;
        else
    %         [current_frame,tend,flag,fretx]=schedule_tx_periodicTx(nextT,ind(user),user,current_frame,current_time_frame,TaOriginal',RBs_UEs(user),Tslot,iter,BLER,Tp,flagDiscardPkts,latency,TprocRx);
    
            fretx=0;
            tend=0;
            flag=0;
        
            N_MCminV(user)=nUERx+1;
            if flagPeriodicAllocation==0 && nUERx==N_MCmax
                flagUser(user)=1;
            end

            if flagDiscardPkts==1
                deadtime=TaOriginal(user,ind(user)+1)-TaOriginal(user,ind(user));
            end
            
            for h=1:length(m)
                currentTime=current_time_frame(m(h));
                if ind(user)<iter && currentTime-TaOriginal(user,ind(user))>=deadtime
                    flag=2;
                    SegmentAllocated(user,iSegment(user,nUERx),nUERx)=0;
                    while ind(user)<=iter && TaOriginal(user,ind(user))<=current_time_frame(end)
                        latency(user,ind(user))=100000;
                        ind(user)=ind(user)+segmentationFactor;
                    end
                    break;
                end 

                cons=0;
                while cons<n_rep
                    p=find(aux_frame(:,m(h)+cons)==0);
                    if length(p)>=RBs_UEs(user)
                        i=0;
                        while m(h)+cons+i*Tp/Tsymbol<=symbols_per_frame+Tp/Tsymbol+cons
                            if m(h)+cons+i*Tp/Tsymbol<=symbols_per_frame
                                flagMS=1;
                                for k=0:N_OS-1
                                    p=find(aux_frame(:,m(h)+cons+k+i*Tp/Tsymbol)==0);
                                    if length(p)<RBs_UEs(user)
                                        flagMS=0;
                                        break;
                                    end
                                end
                            else
                                flagMS=1;
                                for k=0:N_OS-1
                                    p=find(aux_nextframe(:,m(h)+cons+k+i*Tp/Tsymbol-symbols_per_frame)==0);
                                    if length(p)<RBs_UEs(user)
                                        flagMS=0;
                                        break;
                                    end
                                end
                            end

                            if flagMS==1
                                i=i+1;
                            else
                                break;
                            end
                        end
                        if m(h)+cons+i*Tp/Tsymbol>symbols_per_frame
                            cons=cons+1;
                        else
                            break;
                        end
                    else
                        break;
                    end
                end
                if cons<n_rep
                    continue;
                else
                    flag=1;
                    if nframes==0 && iSegment(user,nUERx)>=0
%                         disp([user,iSegment(user,nUERx), ind(user)]);
                        SegmentAllocated(user,iSegment(user,nUERx),nUERx)=1;
                    end
                end
                ind_rep=ind(user);
                for cons=0:n_rep-1
                    i=0;
                    ind(user)=ind_rep;
                    while m(h)+cons+i*Tp/Tsymbol<=symbols_per_frame+Tp/Tsymbol+cons && ind(user)<=iter
                        if m(h)+cons+i*Tp/Tsymbol<=symbols_per_frame
                            for k=0:N_OS-1
                                p=find(aux_frame(:,m(h)+cons+k+i*Tp/Tsymbol)==0);
                                aux_frame(p(1:RBs_UEs(user)),m(h)+cons+k+i*Tp/Tsymbol)=user;
                            end
                            if cons==n_rep-1
                                tend=current_time_frame(m(h)+cons+i*Tp/Tsymbol)+N_OS*Tsymbol; %tend es igual al tiempo en que comienza el siguiente símbolo o slot según el caso
                            end
                        else
                            for k=0:N_OS-1
                                p=find(aux_nextframe(:,m(h)+cons+k+i*Tp/Tsymbol-symbols_per_frame)==0);
                                aux_nextframe(p(1:RBs_UEs(user)),m(h)+cons+k+i*Tp/Tsymbol-symbols_per_frame)=user;
                            end
                            if cons==n_rep-1
                                tend=current_time_frame(m(h)+cons+i*Tp/Tsymbol-symbols_per_frame)+symbols_per_frame*Tsymbol+N_OS*Tsymbol; %tend es igual al tiempo en que comienza el siguiente símbolo o slot según el caso
                            end
                        end
                        if cons==n_rep-1
                            perr=rand(1,n_rep);
                            [x,y]=find(perr>BLER);
                            if isempty(x)   %todos llegan con error
                                latency(user,ind(user))=200000;
                            else
                                % solo guardo la latencia de la Ãºltima
                                % transmisiÃ³n unicast (por eso la
                                % sobreescribo en cada transmisiÃ³n
                                latency(user,ind(user))=tend+TprocRx-TaOriginal(user,ind(user));
                            end
                        end                       
                        i=i+1;
                        ind(user)=ind(user)+segmentationFactor;
                    end
                end
                if cons==n_rep-1
                    if indtoScheduleNextFrame>N*segmentationFactor*N_MC
                        disp('error: iSchedule no puede ser mayor que N');
                    end
                    toScheduleNextFrame(indtoScheduleNextFrame,:)=[user m(h)+i*Tp/Tsymbol-symbols_per_frame ind(user) nUERx];
                    indtoScheduleNextFrame=indtoScheduleNextFrame+1;
                    break;
                end
            end        
        
            if flag==0
                ind(user)=ind(user)+1;
            elseif fretx==1
                disp('aquí no debería llegar nunca');
            end
        
            if iSegment(user,nUERx)==segmentationFactor
                if sum(SegmentAllocated(user,:,nUERx))==segmentationFactor
                    current_frame=aux_frame;
                    next_frame=aux_nextframe;
                    iSegment(user,nUERx)=-1;
                else
                    if flag==2
                        aux_frame=current_frame;
                        aux_nextframe=next_frame;
                        iSegment(user,nUERx)=-1;
                        latency(user,:)=100000;
                        retx(user,:,:)=0;
                        retx_time(user,:,:)=100000;
                        x=find(toScheduleNextFrame(:,1)==user & toScheduleNextFrame(:,1)==nUERx);
                        for ii=1:length(x)
                            toScheduleNextFrame=[toScheduleNextFrame(1:x(end-(ii-1))-1,:);toScheduleNextFrame(x(end-(ii-1))+1:end,:)];
                            toScheduleNextFrame=[toScheduleNextFrame;0 0 0 0];
                            indtoScheduleNextFrame=indtoScheduleNextFrame-1;
                        end
                        nUERx=nUERx+1;
                        while nUERx<=N_MCmax
                            SegmentAllocated(user,iSegment(user,nUERx)+1,nUERx)=0;
                            iSegment(user,nUERx)=-1;
                            nUERx=nUERx+1;
                        end
                        if flagPeriodicAllocation==0
                            flagUser(user)=1;
                        end
                        ind(user)=iter;
                    end
                end
            elseif iSegment(user,nUERx)==-1
                current_frame=aux_frame;
                next_frame=aux_nextframe;
            else
                ind(user)=aux_ind(user);
            end        
            if ind(user)>iter
                ind(user)=iter+1;
            end
            if flag==2
                break;
            end
        end 
    end
end

if traffic==0
	nframes=nframes-limInf; 
end
if traffic==1
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
[x,y]=find(TaOriginal<=2*deadtime);
for i=1:length(x)
    TRx(x(i),y(i))=-11;
	latency(x(i),y(i))=-11;
    retx(x(i),y(i))=-11;
end
[x,y]=find(TaOriginal>(tsim-2*deadtime));
for i=1:length(x)
    TRx(x(i),y(i))=-11;
	latency(x(i),y(i))=-11;
    retx(x(i),y(i))=-11;
end

[x,y]=size(TRx);
tt=reshape(TRx,1,x*y);
[x,y]=find(tt<100000);
Tfinal=max(tt(y));
