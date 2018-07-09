function y = color(u)
switch u
    case 'orange'
        y = [1 0.6 0];
    case 'grey'
        y = [0.8 0.8 0.8];
    case 'blue_light'
        y = [0 0.75 1];
    otherwise
        y = [0 0 0];

end