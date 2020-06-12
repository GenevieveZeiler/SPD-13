# SetUp for Raspberry pi Zero W with Open CV

$ sudo raspi-config   
#Interfacing Options => Camera: YES => FINISH  

$ sudo apt update 
$ sudo apt upgrade  
$ sudo apt-get purge wolfram-engine 
$ sudo apt-get remove --purge python-minecraftpi  
$ sudo rm -rf ~/python_games  
$ sudo apt-get purge libreoffice* 
$ sudo apt-get clean  
$ sudo apt-get autoremove 
$ sudo reboot 

$ echo 'deb [trusted=yes] http://dl.bintray.com/yoursunny/PiZero stretch-backports main' |\
sudo tee  /etc/apt/sources.list.d/bintray-yoursunny-PiZero.list   
$ sudo apt update    
$ sudo apt install python3-opencv   
$ python3 -c 'import cv2; print(cv2. __ version __ )'    <= __ version __ without spaces

$ sudo echo "sudo modprobe bcm2835-v4l2" >> ~/.profile
