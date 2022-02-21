function result = out_of_range(point,search_range)
    result =0;
    if point(1)>search_range(1) || point(1)<0 || ...
       point(2)>search_range(2) || point(2)<0 || ...
       point(3)>search_range(3) || point(3)<0
       result = 1;
    end
end
