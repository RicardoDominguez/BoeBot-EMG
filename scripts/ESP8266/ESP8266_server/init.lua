print("ESP8266 Server3")
wifi.setmode(wifi.STATIONAP);
wifi.ap.config({ssid="test",pwd="12345678"});
print("Server IP Address:",wifi.ap.getip())

sv = net.createServer(net.TCP) 
sv:listen(80, function(conn)
    conn:on("receive", function(conn, receivedData) 
        --print("Received Data: " .. receivedData)
        uart.write(0, receivedData)         
    end) 
    conn:on("sent", function(conn) 
      collectgarbage()
    end)
end)