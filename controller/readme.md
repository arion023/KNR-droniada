# Quick start
source myenv/bin/activate
# pobranie libek
pip install -r REQUIREMENTS.txt
# aktywacja protokołów
sudo raspi-config --> Interface Options --> I2C,SPI,Serial Port, Remote GPIO na YES
# aktywacja deamona od kamery
sudo pigpiod
