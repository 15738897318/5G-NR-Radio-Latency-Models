function radio_latency(BW,SCS,traffic,Tp,nRBperUE,data,link_direction,Nmin,Nmax,N_MC,n_rep,maxN_retx,MCS_table,v,escenario,density,flagDiscardPkts,RBallocation,segmentationFactor,PDCCH_config,PUCCH_config,flag_control,minislot_config)

% FR=1;
% BW=10;
% SCS=60;
% traffic=0 --> aperiodic; traffic=1 --> periodic
% Tp=5; %(ms) (avg) transmission period
% nRBperUE indicates a constant demand of RBs for all UEs
%   if nRBperUE >0 --> data must be equal to 0
% data indicates the amount of data (in bytes) to be transported in a packet
%   if data is >0 --> nRBperUE must be equal to 0
%   the nRB demanded by a UE is calculated based on the CQI experienced
% link_direction=1;
%   direction indicates if DL or UL comm
%       direction=1 --> DL
%      direction=2 --> UL
% cap=2;
% Nmax=200;
% N_MC indicates the number of unicast tx performed in DL
%     N_MC =1 if broadcast mode in DL
%     N_MC >1 if multiple unicast tx in DL
% n_rep is the number of copies sent in consecutive slots
%     k_rep is used
% maxN_retx is the maximum number of retx based on HARQ per packet
%     if maxN_retx>0 --> n_rep=1
%     if n_rep>1 --> maxN_retx=0
% MCS_table indicates the MCS table to use 
%     MCS_table==1 or MCS_table==2 are used to achieve a BLER=0.1
%     MCS_table==3 are used to achieve a BLER=0.00001
% v is the number of tx MIMO layers

if escenario==0 && Nmin==0
    disp('In scenario 0, N must be higher than 0.');
    return;
elseif escenario>0 && Nmin>0
    disp('N should be equal to 0, N value will not be considered.');
    return;
end

if escenario==21 && density==602
    density=60.2;
end
    
FR=1;
cap=2; %UE capability 

step=50;

n=Nmin:step:Nmax;

latency_vector_avg=zeros(1,length(n));
latency_vector_max=zeros(1,length(n));
RButilization_avg_vector=zeros(1,length(n));


if traffic==0
    carp='aperiodic';
    seedMax=300; %10000
    rng('shuffle');
elseif traffic==1
    carp='periodic';
    seedMax=1000; %10000
    rng('shuffle');
else
    disp('traffic not valid');
end

if link_direction==1
    link='DL';
elseif link_direction==2
    link='UL';
end

if MCS_table==1 || MCS_table==2
    BLER=0.1;
elseif MCS_table==3
    BLER=0.00001;
end

if n_rep>1 && maxN_retx>0
    disp('repetitions and retx cannot be configured at the same time');
    return;
end   

if N_MC<1
    disp('error: N_MC must equal to or higher than 1.');
    return;
%elseif N_MC>1 && n_rep>1
%    disp('error: Multiple unicast tx cannot be executed with krepetitions.');
%    return;
end    

switch escenario
    case 0
        dir='circular';
    case 11
        if flag_control==0
            dir='highway1732';
        else
            dir='highway1732_control';
        end
    case 13
        if flag_control==0
            dir='highway1732_3lanesxdir';
        else
            dir='highway1732_3lanesxdir_control';
        end
    case 12
        dir='highway500';
    case 21
        dir='urban500';
end

if link_direction==2
    dir=sprintf('%s/ULUnicast',dir);
    N_MC=1;
    if maxN_retx>0
        coletilla2=sprintf('_retx%d',maxN_retx);
        dir=sprintf('%s/output_HARQ',dir);
    elseif n_rep>1
        coletilla2=sprintf('_rep%d',n_rep);
        dir=sprintf('%s/output_rep',dir);
    else
        coletilla2=sprintf('_rep%d',n_rep);
        dir=sprintf('%s/output',dir);
    end
else
    if N_MC==1
        dir=sprintf('%s/DLBroadcast',dir);
        if maxN_retx>0
            coletilla2=sprintf('_retx%d',maxN_retx);
            dir=sprintf('%s/output_HARQ',dir);
        else
            coletilla2=sprintf('_rep%d',n_rep);
            if n_rep>1
                dir=sprintf('%s/output_rep',dir);
            else
                dir=sprintf('%s/output',dir);
            end
        end
    else
        dir=sprintf('%s/multipleDLTx',dir);
        coletilla2=sprintf('_retx%d',maxN_retx);
        if maxN_retx>0
            dir=sprintf('%s/output_HARQ',dir);
        else
            dir=sprintf('%s/output',dir);
        end
    end
end

if nRBperUE>0
    if data>0
        disp('error: nRBperUE and data cannot be higher than zero in the same simulation run.');
        return;
    end
    coletilla3=sprintf('_nRBperUE%d',nRBperUE);
    dir=sprintf('%s/%s_Tp%d_nRB%d',dir,carp,Tp,nRBperUE);
else
    coletilla3=sprintf('_pkt%d',data);
    dir=sprintf('%s/%s_Tp%d_pkt%d',dir,carp,Tp,data);
end

i=0;
data=data*8;    %data in bits
for N=Nmin:step:Nmax
    i=i+1;
    
%     coletilla=sprintf('SA_Tp%d_SCS%d_BW%d_%s_UEcap%d_N%d_nRBperUE%d_MC%d',Tp,SCS,BW,link,cap,N,nRBperUE,N_MC);
    if escenario==0
        coletilla=sprintf('Tp%d_SCS%d_BW%d_%s_N%d_nDLTx%d_MCSTable%d_layers%d%s%s',Tp,SCS,BW,link,N,N_MC,MCS_table,v,coletilla2,coletilla3);
    else
        coletilla=sprintf('Tp%d_SCS%d_BW%d_%s_density%d_nDLTx%d_MCSTable%d_layers%d%s%s',Tp,SCS,BW,link,density,N_MC,MCS_table,v,coletilla2,coletilla3);
    end
    if flag_control==1
        coletilla=sprintf('%s_U%dD%d',coletilla,PUCCH_config,PDCCH_config);
    end
    if minislot_config>0
        coletilla=sprintf('%s_MS%d',coletilla,minislot_config);
    end

    dir_actual=cd();
    cd(dir);
    fid=fopen(sprintf('latency_matrix_%s.txt',coletilla),'a+');
    fid3=fopen(sprintf('UE_data_%s.txt',coletilla),'a+');
    ferror=fopen(sprintf('error_%s.txt',coletilla),'a+');
    if n_rep==1
        fid2=fopen(sprintf('retx_matrix_%s.txt',coletilla),'a+');
    end
    if traffic==0
        fid4=fopen(sprintf('tiempo_rx_pkts_%s.txt',coletilla),'a+');
        fid5=fopen(sprintf('tiempo_sim_%s.txt',coletilla),'a+');
    end
    cd(dir_actual);

    for seed=1:seedMax
        if RBallocation==1
            if n_rep>1
                [latency, RButilization_avg, retx, TRx, Tini, Tfinal]=delay_analysis_periodic_rep(FR,BW,SCS, N, Tp, nRBperUE, link_direction, cap, seed, N_MC, n_rep, BLER, maxN_retx, traffic, MCS_table, v,data,escenario,density,fid3,flagDiscardPkts,RBallocation,segmentationFactor,PDCCH_config,PUCCH_config,flag_control,minislot_config,ferror);
            else
                [latency, RButilization_avg, retx, TRx, Tini, Tfinal]=delay_analysis_periodic(FR,BW,SCS, N, Tp, nRBperUE, link_direction, cap, seed, N_MC, BLER, maxN_retx, traffic, MCS_table, v,data,escenario,density,fid3,flagDiscardPkts,RBallocation,segmentationFactor,PDCCH_config,PUCCH_config,flag_control,minislot_config,ferror);
            end
        else
            if n_rep>1
                [latency, RButilization_avg, TRx, Tini, Tfinal]=delay_analysis_rep(FR,BW,SCS, N, Tp, nRBperUE,link_direction, cap, seed, N_MC, n_rep, BLER, traffic, MCS_table, v,data,escenario,density,fid3,ferror);
            else
                [latency, RButilization_avg, retx, TRx, Tini, Tfinal]=delay_analysis(FR,BW,SCS, N, Tp, nRBperUE, link_direction, cap, seed, N_MC, BLER, maxN_retx, traffic, MCS_table, v,data,escenario,density,fid3,flagDiscardPkts,RBallocation,segmentationFactor,PDCCH_config,PUCCH_config,flag_control,minislot_config,ferror);
            end
        end
        
        latency=latency(:,1:end-1);
        [msize,nsize]=size(latency);
        for j=1:nsize
            [x,y]=find(latency(:,j)==-11);
            if length(x)==length(latency(:,j))
                continue;
            end
            fprintf(fid,'%.3f\t',latency(:,j));
            fprintf(fid,'\n');
        end
        if traffic==0
            for j=1:nsize
                [x,y]=find(TRx(:,j)==-11);
                if length(x)==length(TRx(:,j))
                    continue;
                end
                fprintf(fid4,'%.2f\t',TRx(:,j));
                fprintf(fid4,'\n');
            end
            fprintf(fid5,'%.3f\t%.3f\n',Tini,Tfinal);
        end

        if n_rep==1
            retx=retx(:,:,1);
            retx=retx(:,1:end-1);
            for j=1:nsize
                [x,y]=find(retx(:,j)==-11);
                if length(x)==length(retx(:,j))
                    continue;
                end
                fprintf(fid2,'%d\t',retx(:,j));
                fprintf(fid2,'\n');
            end
        end
        
        [x,y]=size(latency);
        latency=reshape(latency,1,x*y);
        x=find(latency>0);
        latency=latency(x);
        x=find(latency<100000);
        latency_vector_avg(i)=latency_vector_avg(i)+mean(latency(x));
%	disp([latency_vector_max(i),max(latency(x))]);
        latency_vector_max(i)=max(latency_vector_max(i),max(latency(x)));
        abortedPackets(i)=(length(latency)-length(x))/length(latency)*100;
        RButilization_avg_vector(i)=RButilization_avg;
    end
    fclose(fid);
    if n_rep==1
	fclose(fid2);
    end
    fclose(fid3);
    latency_vector_avg(i)=latency_vector_avg(i)/seedMax;
    if traffic==0
        fclose(fid4);
        fclose(fid5);
    end
end

if n_rep>1
    cd(dir);
else
    cd(dir);
end
% coletilla=sprintf('SA_Tp%d_SCS%d_BW%d_%s_UEcap%d_N%d-%d_nRBperUE%d_MC%d',Tp,SCS,BW,link,cap,Nmin,Nmax,nRBperUE,N_MC);
if escenario==0
    coletilla=sprintf('Tp%d_SCS%d_BW%d_%s_N%d-%d_nDLTx%d_MCSTable%d_layers%d%s%s',Tp,SCS,BW,link,Nmin,Nmax,N_MC,MCS_table,v,coletilla2,coletilla3);
else
    coletilla=sprintf('Tp%d_SCS%d_BW%d_%s_density%d_nDLTx%d_MCSTable%d_layers%d%s%s',Tp,SCS,BW,link,density,N_MC,MCS_table,v,coletilla2,coletilla3);
end
if flag_control==1
    coletilla=sprintf('%s_U%dD%d',coletilla,PUCCH_config,PDCCH_config);
end
if minislot_config>0
    coletilla=sprintf('%s_MS%d',coletilla,minislot_config);
end

% h=figure;
% hold on;
% plot(Nmin:Nmax,latency_vector_avg,'b');
% plot(Nmin:Nmax,latency_vector_max,'r');
% xlabel('Number of UE');
% ylabel('Latency (ms)');
% legend('avg','max');
% hgsave(h,sprintf('latency_%s.fig',coletilla));
fid=fopen(sprintf('latency_%s.txt',coletilla),'a+');
for i=1:length(n)
        fprintf(fid,'%d\t%.4f\t%.4f\t%.4f\t%.4f\n',n(i),latency_vector_avg(i),latency_vector_max(i),abortedPackets(i),RButilization_avg_vector(i));
end
fclose(fid);

% h=figure;
% plot(Nmin:Nmax,RButilization_avg_vector);
% xlabel('Number of UE');
% ylabel('% of RB used');
% hgsave(h,sprintf('RButilization_%s.fig',coletilla));
% fid=fopen(sprintf('RButilization_%s.txt',coletilla),'a+');
% for i=1:Nmax-Nmin+1
% 	fprintf(fid,'%d\t%.4f\n',i+Nmin-1,RButilization_avg_vector(i));
% end
% fclose(fid);
cd(dir_actual);
