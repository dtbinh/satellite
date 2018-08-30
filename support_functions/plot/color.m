function y = color(u)
switch u
    case 'red'
        y = [1.0 0.0 0.0];
    case 'orange'
        y = [1.0 0.6 0.0];
    case 'grey'
        y = [0.8 0.8 0.8];
    case 'lblue'
        y = [0.0 0.75 1.0];
    case 'teal'
        y = [81, 193, 204]/255;
    otherwise
        y = [0.0 0.0 0.0];

end