function Ta=packet_gen_video(N,nu,iter,seed,Tslot,traffic,density,segmentationFactor)

flag=0;
Ta=zeros(N,iter);
%arrival time of packets
TiniFixed=rand()*Tslot;
for n=1:N
    % arrival time of packets 
    if traffic==0
%         aperiodic
        % based on a random distribution
        Ta(n,:)=cumsum(nu+randomGen_Exponencial( nu, 1, iter )); % en ms
    elseif traffic==1
        % periodic arrival
        if 0
            % all users start at the same time instant
            Tini=TiniFixed;
        elseif 0
            % first packet for user n arrives at a random time instant between
            % 0 and nu
            Tini=rand()*nu;
        else
%             este caso lo hacemos aparte
            flag=1;
            break;
        end
        Ta(n,:)=cumsum([Tini nu*ones(1,iter-1)]); % en ms
    else
        disp('traffic not valid.');
        return;
    end
end

if flag==1
    % calculamos la llegada media de vehículos por unidad de tiempo
    speed=AvgSpeedAsAFunctionOfDensity(density);
    % tasa media de llegada de vehiculos
    tasa=density*speed/3600/1000; %veh per millisecond
    Tini=cumsum(randomGen_Exponencial( 1/tasa, N, 1 ));
    aux=floor(Tini/nu);
    Tini=Tini-nu.*aux;
    Ta=cumsum([Tini nu*ones(N,iter-1)],2);
end

if segmentationFactor>1
    [x,y]=size(Ta);

    % segmentationFactor=4;
    aux=[];
    for i=1:y
        for j=1:segmentationFactor
            aux=[aux Ta(:,i)];
        end
    end
    Ta=aux;
end

