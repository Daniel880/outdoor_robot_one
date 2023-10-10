sudo slcan_attach -f -s6 -o /dev/ttyACM0  
sudo slcand -S 1000000 ttyACM0 can0  
sudo ip link set can0 txqueuelen 1000
sudo ifconfig can0 up  
