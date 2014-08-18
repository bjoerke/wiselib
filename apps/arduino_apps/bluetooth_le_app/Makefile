WISELIB_PATH=/home/wiselib/wiselib

WISELIB_STABLE=$(WISELIB_PATH)/wiselib.stable
WISELIB_TESTING=$(WISELIB_PATH)/wiselib.testing

ARDUINO_PATH=/home/wiselib/Arduino
VARIANTS=standard

SPI_PATH=$(ARDUINO_PATH)/libraries/SPI
PINS_PATH=$(ARDUINO_PATH)/hardware/arduino/variants/$(VARIANTS)
WIRING_PATH=$(ARDUINO_PATH)/hardware/arduino/cores/arduino
ETHERNET_PATH=$(ARDUINO_PATH)/libraries/Ethernet
SD_PATH=$(ARDUINO_PATH)/libraries/SD
SOFTWARE_SERIAL_PATH=$(ARDUINO_PATH)/libraries/SoftwareSerial

ARDUINO_HEADERS=-I$(SPI_PATH) -I$(PINS_PATH) -I$(WIRING_PATH) -I$(ETHERNET_PATH) \
	-I$(ETHERNET_PATH)/utility -I$(SD_PATH) -I$(SD_PATH)/utility -I$(SOFTWARE_SERIAL_PATH)

CXX=avr-g++
INCLUDE=$(ARDUINO_HEADERS) -I$(WISELIB_STABLE) -I$(WISELIB_TESTING)
LIBS=-L$(ARDUINO_PATH) -Wl,--start-group -lsoftwareserial -larduino -Wl,--end-group

MCU=-mmcu=atmega328p
CPU_SPEED=-DF_CPU=16000000UL
CFLAGS=$(MCU) $(CPU_SPEED) -DARDUINO=1 -DARDUINO_WIRING_DIGITAL -DARDUINO_LITE -DARDUINO_STRING -DARDUINO_USE_BLE_RADIO -Os -w \
	-funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums -fno-exceptions -fno-use-cxa-atexit

PORT=/dev/ttyACM0

default: build-libs build upload

build-libs:
	make -C $(ARDUINO_PATH)

build: example.hex

example.hex: example.elf
	avr-objcopy -O ihex $< $@

example.elf: example_app.cpp
	$(CXX) $(INCLUDE) $^ $(CFLAGS) -o $@ $(LIBS)
	avr-strip $@

upload:
	avrdude -V -F -p m328p -c arduino -b 115200 -Uflash:w:example.hex -P$(PORT)

clean:
	@echo -n Cleaning ...
	$(shell rm example.elf 2> /dev/null)
	$(shell rm example.hex 2> /dev/null)
	$(shell rm *.o 2> /dev/null)
	@echo " done"

%.o: %.cpp
	$(CXX) $< $(CFLAGS) $(INCLUDE) -c -o $@
