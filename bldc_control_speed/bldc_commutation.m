function out = bldc_commutation(in)

h1 = in(1);
h2 = in(2);
h3 = in(3);

gate1 = double((h1)&&(~h2))-double((~h1)&&(h2));
gate2 = double((h2)&&(~h3))-double((~h2)&&(h3)); 
gate3 = double((h3)&&(~h1))-double((~h3)&&(h1)); 

out = [gate1;gate2;gate3];

end