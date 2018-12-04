function cost = f_n(curr_path, dest)
    latest = curr_path(end,:);
    if length(curr_path) > 2
        gn = sum(sqrt(sum((curr_path(2:end,:)-curr_path(1:end-1,:)).^2, 2))); %euclidean travelled so far
    else
        gn = 0;
    end
    hn = sum(sqrt(sum((dest-latest).^2))); %euc dist to dest
    if hn == 1
        cost = -1; %reached goal!
    else
        cost = gn+hn;
    end
end