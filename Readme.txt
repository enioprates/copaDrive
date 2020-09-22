Testar NMEA com as OBUs

Open new terminal
ssh user@<IP_OBU>
Password: user

cd /mnt/ubi
./MessageBroker

Open new terminal
ssh user@<IP_OBU>
Password: user

cd /mnt/ubi/exampleETSI

//Configure the GPS IP and Port
vi obu.conf
/GPSD
Shift + i

Cohda_GPSD_HostName            = <IP>
Cohda_GPSD_Port 	       = <Defined_Port>

Esc
:wq (write and quit)

//Run ETSI
./rc.exampleETSI start obu


Check on the cout context: ETSIInfoVstate

Received Message ETSIInfoVstate [ID;Speed;Latitude;Longitude;Heading]



