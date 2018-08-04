print("ESP8266 Client 2")
wifi.sta.disconnect()
wifi.setmode(wifi.STATION) 
wifi.sta.config("test","12345678") -- connecting to server
wifi.sta.connect() 
print("Looking for a connection")

tmr.alarm(1, 2000, 1, function() -- execute every 2 sec
     if(wifi.sta.getip()~=nil) then --looks for connection
          tmr.stop(1)
          print("Connected!")
          print("Client IP Address:",wifi.sta.getip())
          cl=net.createConnection(net.TCP, 0)
          cl:connect(80,"192.168.4.1")
          uart.on("data", 1,
            function(data)
                cl:send(data)
                print(data) 
            end, 0)
      else
         print("Connecting...")
      end
end)
