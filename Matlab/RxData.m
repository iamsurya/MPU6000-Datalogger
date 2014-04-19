clear
clc
oldserial = instrfind('Port', 'COM4')
delete(oldserial)

s = serial('COM4');
instrhwinfo('serial')


set(s,'BaudRate',4800,'InputBufferSize',512)
fopen(s);


% fprintf(s,'*IDN?')  %Send something

% out = fscanf(s) % receive something

