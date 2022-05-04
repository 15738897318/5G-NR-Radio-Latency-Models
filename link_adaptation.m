function MCS=link_adaptation(CQI, MCS_table)
% devuelve el índice al vector que contiene los iMCS en la tabla de MCS
% table 2
if MCS_table==2
    ind_MCS=[1 2 4 6 8 10 12 14 16 18 20 22 24 26 28];
elseif MCS_table==3
    ind_MCS=[1 3 5 7 9 11 13 15 17 19 21 23 25 27 29];
end

MCS=ind_MCS(CQI);
