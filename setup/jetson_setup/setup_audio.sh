# Enable and start pulseaudio for the current user
systemctl --user enable pulseaudio
systemctl --user start pulseaudio

# Restart bluetooth service (requires sudo)
sudo systemctl restart bluetooth
