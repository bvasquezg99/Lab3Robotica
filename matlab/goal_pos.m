function goal_pos(service,id,value,time,movePX_RTB)
    if value <= 1023 & value >= 0     
        if movePX_RTB
            msg_svc = rosmessage(service);
            msg_svc.AddrName = "Goal_Position";
            msg_svc.Id = id;
            msg_svc.Value = value;
            call(service,msg_svc);
            pause(time);
        else
            disp("Comando no habilitado")
        end
    else
        disp("Fuera de rangos articulares");
    end
end