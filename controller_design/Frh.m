function hull_resistance = Frh(v_ah)
    load('..\boat_constants.mat')
    hull_resistance = k_rh*v_ah^2;

end