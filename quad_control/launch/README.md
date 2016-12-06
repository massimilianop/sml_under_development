# Different Launch Files


arguments to be set "file_name" and "playing_back"

```
roslaunch quad_control firefly_set_up.launch
```


## iris1_mavros_vbad.launch

```
roslaunch quad_control iris1_mavros_vbad.launch
```

1. runs cycle ...; gui ; simulators; all in namespace Iris1/


# TODO: comment better
# for connecting mavros you may need to change some permissions

see permissions:
➜ ls -l /dev/ttyUSB0
crw-rw---- 1 root dialout 188, 0 jul 18 12:58 /dev/ttyUSB0
➜ sudo chmod 666 /dev/ttyUSB0
[sudo] password for pedrootao: 
➜ ls -l /dev/ttyUSB0         
crw-rw-rw- 1 root dialout 188, 0 jul 18 12:58 /


➜ whoami
pedrootao
➜ groups
pedrootao adm cdrom sudo dip plugdev lpadmin sambashare
➜ sudo gpasswd -a pedrootao dialout
Adding user pedrootao to group dialout

after reboot
➜ groups                           
pedrootao adm dialout cdrom sudo dip plugdev lpadmin sambashare
