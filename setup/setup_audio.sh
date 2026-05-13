systemctl --user enable pulseaudio
systemctl --user start pulseaudio
pulseaudio --start
systemctl restart bluetooth
