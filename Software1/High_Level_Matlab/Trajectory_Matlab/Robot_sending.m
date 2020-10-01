

Old_range_min=[180 -26.6 -79.5 131.048 95.972 -180];
Old_range_max=[-180 173.3 130.5 -172.841 -116.755 180];
New_range_min=[20000 23997 0 4000 0 0];
New_range_max=[0 0 7000 0 2600 7676];
Start = 255;
left_8 = [0 0 0 0 0 0];
right_8 = [0 0 0 0 0 0];
start_speed = [1000 1000 1000 1000 1000 1000]; %use low speeds. 

J_angles_in_deg = rad2deg(J_angles);
%J_speed_2 = J_speed * 100; %speeds are floats so we multiply those flaot values by 10000 for easier data transfer
J_speed_2 = J_speed * 100; %speeds are floats so we multiply those flaot values by 10000 for easier data transfer
J_speed_2 = abs(J_speed_2); %make all speeds that are negative postitive , direction is dealt with different way
%devide speeds by gearbox ratios so we can tell the speed directly to steppers

for i = 1:6
J_speed_2(:,i) = round(J_speed_2(:,i) * J_gearbox(i));
end

%TODO
%vrijednosti impulsa koje robot radi , tehnièki mogu i njih slati
J_pulse_widht = zeros(m,6);
for j = 1:m
for i = 1:6
J_pulse_widht(j,i) = (pi*J_step_angle(i)*J_microstep(i))/(360*J_speed_2(j,i)/100)*1000000;
end
end

delete(instrfind);
MySerial=serial('COM12','BaudRate', 9600);
fopen(MySerial);

Continue = 1;
while (Continue == 1)
  AvailableData = MySerial.BytesAvailable;
  if(AvailableData > 0)
      out = fscanf(MySerial);
      % Pretvori char array u iskoristivi string ,ako se ovdje pojavi broj , pretvara ga u string
      [m_,n_] = size(out);
      string = out(1:(n_-2));
      string = convertCharsToStrings(string);
      
     % Pretvori char array u iskoristivi int, ako se ovdje pojavi string output je ništa
      int_ = str2num(out);
      
      if(string == "start")
         fprintf('Received start  \n')
         Continue = 0; 
      end
  end
end
Continue = 1;
%pause(0.1);



J_robot_limit = [70 70 70 260 70 300];
prevspeed = 0;
%remove zero speed
for i = 1:6
for j = 1:m

if(J_speed_2(j,i) <= J_robot_limit(i));
   J_speed_2(j,i) = prevspeed;
end
prevspeed = J_speed_2(j,i);
    end
end


fwrite(MySerial,Start,'char'); 
fwrite(MySerial,Start,'char'); 
binVal = decimalToBinaryVector(m,16,'MSBFirst');
left_8(i) = bi2de(binVal(1:8),'left-msb');
right_8(i) = bi2de(binVal(9:16),'left-msb');
fwrite(MySerial, left_8(i), 'char'); 
fwrite(MySerial, right_8(i), 'char'); 

fprintf('Printed setup  \n');



Continue = 1;
while (Continue == 1)
  AvailableData = MySerial.BytesAvailable;
  if(AvailableData > 0)
      out = fscanf(MySerial)
      % Pretvori char array u iskoristivi string ,ako se ovdje pojavi broj , pretvara ga u string
      [m_,n_] = size(out)
      string = out(1:(n_-2))
      string = convertCharsToStrings(string)
      
     % Pretvori char array u iskoristivi int, ako se ovdje pojavi string output je ništa
      int_ = str2num(out)
      
      if(string == "done")
         fprintf('Received done  \n')
         Continue = 0 
      end
  end
end
Continue = 1;



j = 1;
while(Continue == 1)
    
time1 = tic();

fwrite(MySerial,Start,'char'); 
fwrite(MySerial,Start,'char'); 
for i = 1:6    
  J_angles_out(j,i) = (((J_angles_in_deg(j,i)-Old_range_min(i))*(New_range_max(i)-New_range_min(i)))/(Old_range_max(i)-Old_range_min(i)))+New_range_min(i);
  J_angles_out(j,i) = round(J_angles_out(j,i));
  binVal = decimalToBinaryVector(J_angles_out(j,i),16,'MSBFirst');
  left_8(i) = bi2de(binVal(1:8),'left-msb');
  fwrite(MySerial, left_8(i), 'char'); 
  right_8(i) = bi2de(binVal(9:16),'left-msb');
  fwrite(MySerial, right_8(i), 'char'); 
end

for i = 1:6
binVal = decimalToBinaryVector(J_speed_2(j,i),16,'MSBFirst');
left_8(i) = bi2de(binVal(1:8),'left-msb');
fwrite(MySerial, left_8(i), 'char'); 
right_8(i) = bi2de(binVal(9:16),'left-msb');
fwrite(MySerial, right_8(i), 'char');       
end

%{
binVal = decimalToBinaryVector(j,16,'MSBFirst');
left_8(i) = bi2de(binVal(1:8),'left-msb');
fwrite(MySerial, left_8(i), 'char'); 
right_8(i) = bi2de(binVal(9:16),'left-msb');
fwrite(MySerial, right_8(i), 'char'); 
%}

time2 = toc(time1);

j = j + 1;
if(j == (m + 1))
   Continue = 0; 
end


end
%}