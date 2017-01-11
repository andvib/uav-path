function chi_c = lineOfSight(pos, path)
    R = 200;
    cont = 1;
    
    for i = (length(path(:,1)):-1:1)
        radius = (path(i,1)-pos(1))^2 + (path(i,2)-pos(2))^2;
        if (radius > (R - 10)^2) && (radius < (R + 10)^2)
            cont = 0;
            break;
        end
        if(~cont)
            break;
        end
    end
    
    chi_c = atan2(path(i,2)-pos(2), path(i,1)-pos(1));