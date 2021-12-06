## ESP32-MFRC522

**Hardware settings**
Using hardware SPI :

SDA = select_pin = 4
RST = reset_pin = 5
MOSI = 23
MISO = 19
GND = any ground
3.3V = any 3V3

**Code example**

```python
from machine import SPI
from mfrc522 import SimpleMFRC522

spi = SPI(2, baudrate=1000000, polarity=0, phase=0)

select_pin = 4
reset_pin = 5
reader = SimpleMFRC522(spi, select_pin, reset_pin)

while True:
    print("\nHold a tag near the reader")
    id, text = reader.read()
    print("ID: %s\nText: %s" % (id,text))
```
