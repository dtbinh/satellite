v = [2;4;6]
max = [4.5;4.5;4.5]
v_unit = v./norm(v)

for i=1:1:3

    if ((v(i) > max(i)) || (v(i) < -max(i)) )

      tmp = abs(max(i)/v(i));
      for k=1:1:3

        v(k) = v(k) * tmp;
      end
    end
end
v
v_unit = v./norm(v)

