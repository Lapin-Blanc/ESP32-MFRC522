## ESP32-MFRC522
Copy mfrc522 directory to esp root

**Hardware settings**  
Using hardware SPI(2) :

SDA = select_pin = 5

RST = reset_pin = 22

MOSI = 23  

MISO = 19  

GND = any ground  

3.3V = any 3V3

**Code example**
```python
from machine import SPI
from mfrc522 import SimpleMFRC522

spi = SPI(2, baudrate=1000000, polarity=0, phase=0)

select_pin = 5
reset_pin = 22
reader = SimpleMFRC522(spi, select_pin, reset_pin)

while True:
    print("\nHold a tag near the reader")
    id, text = reader.read()
    print("ID: %s\nText: %s" % (id,text))
```
