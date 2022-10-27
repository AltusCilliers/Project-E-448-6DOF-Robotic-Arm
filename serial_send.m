function serial_send(data,s)
    disp(data)
    %s = serialport('COM5',115200)
    configureTerminator(s,'CR') ;
    fopen(s);
    pause(0.5)

    writeline(s,data);

    %fclose(s);
    %delete(s);
    %clear s;
    
end