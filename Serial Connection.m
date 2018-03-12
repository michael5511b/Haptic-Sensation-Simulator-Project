clear, clc

s = serial('COM3','BaudRate',9600,'DataBits',8,'Parity','none');
fopen(s);

numtosend = input('90');  % collect numerical input from user
fprintf(s,num2str(numtosend))  % send to serial port, after converting number to string

fclose(s);
delete(s);
clear s