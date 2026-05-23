# install the xpadneo driver for the bluetooth xbox controller

cd $HOME

USER_TO_ADD=${logname}

# get repository
git clone https://github.com/atar-axis/xpadneo.git

# install dependencies
sudo apt-get install -y dkms linux-headers-$(uname -r)

# run install script
sudo ./install.sh

# add to "input" group
sudo usermod -aG input $USER_TO_ADD

#FIXME idk if these commands will actually work
sudo echo "options hid_xpadneo disable_shift_mode=1" > /etc/modprobe.d/99-xpadneo-quirks.conf
sudo echo "options hid_xpadneo disable_mouse=1" > /etc/modprobe.d/99-xpadneo-quirks.conf

#TODO: might need to edit the dkms configuration file:
# sudo nano /etc/dkms/framework.conf
# and uncomment the lines that say
# mok_signing_key=...
# mok_certificate=...

echo NOTE: you will need a GUI for the next reboot to edit the UEFI settings or something
echo if you don\'t see the MOK import screen, then run the following command
echo "# sudo mokutil --import /var/lib/shim-signed/mok/MOK.der"

echo then reboot