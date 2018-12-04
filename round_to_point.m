function rounded = round_to_point(coords, X, Y)   
    [~, x_ind] = min(abs(coords(1)-X(1,:)));
    [~, y_ind] = min(abs(coords(2)-Y(:,1)));
    rounded = [x_ind y_ind];
end