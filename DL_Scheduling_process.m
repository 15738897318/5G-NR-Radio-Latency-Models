function [Taux,latency]=DL_Scheduling_process(Ta,Tslot,TprocTx,TprocRx,TprocTxHARQ,TprocRxHARQ,latency,tsim,PDCCH_config,flag_control)
% this function accounts for the delay added by sending the scheduling 
% information in DL in the PDCCH
% we consider that:
% 1. the scheduling information is included in the PDCCH and is transmitted 
% in the first symbol of every slot

switch PDCCH_config
    case 1
        NofUEsperSlot=2;%Number of DCI messages with format 1_0 that can be transmitted per slot
    case 2
        NofUEsperSlot=4; %Number of DCI messages with format 1_0 that can be transmitted per slot
    case 3
        NofUEsperSlot=12; %Number of DCI messages with format 1_0 that can be transmitted per slot
    case 4
        NofUEsperSlot=18; %Number of DCI messages with format 1_0 that can be transmitted per slot
    case 5
        NofUEsperSlot=24; %Number of DCI messages with format 1_0 that can be transmitted per slot
    case 9
    	NofUEsperSlot=500; %unlimited capacity
end


Tsymbol=Tslot/14;

if flag_control==0
    [x,y]=size(Ta);
    for i=1:x
        for j=1:y
            Taux(i,j)=Ta(i,j)+TprocTxHARQ;
            Taux(i,j)=ceil(Taux(i,j)/Tslot)*Tslot+Tsymbol;
            % el gNB ha tenido en cuenta que al menos tiene que esperar un
            % tiempo TprocRxHARQ antes de asignarle recursos, por lo que:
            Taux(i,j)=Taux(i,j)+TprocRxHARQ+TprocTx;
            % el siguiente Talligment ya se calcula en base al recurso que se
            % le asigne en el scheduling
        end
    end
else

    imax=floor(tsim/Tslot);
    queue=zeros(1,imax);
    Tb=Ta';
    Taux=Ta;
    while 1
        [nextPacketperUser,iTb]=min(Tb,[],1);
        [nextT,user]=min(nextPacketperUser);
        if nextT>=tsim
          return;
        end
        ind_user=iTb(user);

        Taux(user,ind_user)=Taux(user,ind_user)+TprocTxHARQ;
        ind_ini=ceil(Taux(user,ind_user)/Tslot);
        i=0;
        while ind_ini+i<=imax && queue(ind_ini+i)>=NofUEsperSlot
            i=i+1;
        end

        Taux(user,ind_user)=(ind_ini+i)*Tslot;
        if Taux(user,ind_user)>=Taux(user,ind_user+1) || (ind_ini+i)>imax
            latency(user,ind_user)=300000;
        else    
            queue(ind_ini+i)=queue(ind_ini+i)+1;
        end

        Taux(user,ind_user)=Taux(user,ind_user)+Tsymbol; %el tiempo de transmisión
        %     Taux(user,ind_user)=ceil(Taux(user,ind_user)/Tslot)*Tslot+Tsymbol;
        % el gNB ha tenido en cuenta que al menos tiene que esperar un
        % tiempo TprocRxHARQ antes de asignarle recursos, por lo que:
        Taux(user,ind_user)=Taux(user,ind_user)+TprocRxHARQ+TprocTx;
        % el siguiente Talligment ya se calcula en base al recurso que se
        % le asigne en el scheduling
        Tb(ind_user,user)=tsim*2;
    end
end

