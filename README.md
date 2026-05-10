<IMG BORDER=0 SRC="https://raw.githubusercontent.com/jafrado/2004_i2c_oled/master/doc/s-l1600.jpg">

# 2004_i2c_oled
Display Driver for the Wide.HK 20x4 SSD1311 Character mode OLED display

This display library implements output and positioning along with
custom character sets for the Wide.HK 20x4 OLED Display which can be
purchased online at the <A HREF="http://www.wide.hk/index.php?route=product/product&path=24&product_id=55"> Wide.HK Store</a>

To use with a Raspberry-Pi (I used a Zero); connect the
SCL/SDA/3V3/GND pins (I used I2C1) and copy the source code to your system. 
<IMG BORDER=0 SRC="https://raw.githubusercontent.com/jafrado/2004_i2c_oled/master/doc/raspberry-pi-pinout.png">

<IMG BORDER=0 SRC="https://github.com/jafrado/2004_i2c_oled/blob/master/doc/s-l1600-2.jpg">

# Building
Issue the "make" command and build the application. 
Run the oledtest as below:

 $./oledtest /dev/i2c-1

The application will run a demo showing features of the display and cycling through all of the CGROM
contents.

# Arduino
An Arduino port of the same demo lives in the <code>2024_oled/</code> folder and has been verified on an
<A HREF="https://store.arduino.cc/products/uno-r4-minima">Arduino Uno R4 Minima</a> (Renesas RA4M1).
It should also build unchanged on classic Uno / Nano / Mega / ESP32 / RP2040 - only the SDA/SCL header pins differ.

Wiring (Uno R4 Minima):
<UL>
<LI>SDA -> dedicated SDA pin (also exposed on A4)</LI>
<LI>SCL -> dedicated SCL pin (also exposed on A5)</LI>
<LI>VCC -> 3V3 or 5V per the Wide.HK module's onboard jumper</LI>
<LI>GND -> GND</LI>
</UL>

Sketch layout:
<UL>
<LI><code>2024_oled/2024_oled.ino</code> - <code>setup()</code> / <code>loop()</code> demo, calls only the public <code>lcd_*</code> API.</LI>
<LI><code>2024_oled/ssd13xx_20x4_oled.c</code> and <code>ssd13xx_20x4_oled.h</code> - copies of the common driver. The Arduino IDE only compiles sources inside the sketch folder, so these are duplicated rather than referenced from the repo root.</LI>
<LI><code>2024_oled/ssd13xx_io_arduino.cpp</code> - Arduino-specific glue: <code>ssd13xx_write()</code> over the <code>Wire</code> library and <code>ms_delay()</code> mapped to <code>delay()</code>. Wrapped in <code>extern "C"</code> so the C driver links cleanly into the C++ sketch.</LI>
</UL>

The common header <code>ssd13xx_20x4_oled.h</code> carries <code>extern "C"</code> guards so the same driver
source builds verbatim under Linux (this repo's <code>Makefile</code>), NuttX (<code>lcd_2004_nuttx.c</code>),
and Arduino - only the platform glue file differs.

To build: open <code>2024_oled/2024_oled.ino</code> in the Arduino IDE, select Tools -> Board -> Arduino Uno R4 Minima
(or your target), and upload. No external libraries are required - only the built-in <code>Wire</code>.
The OLED slave address (0x3c) is set in <code>ssd13xx_io_arduino.cpp</code>.

I2C clock defaults to 100 kHz to match the Linux / NuttX configuration. Uncomment
<code>Wire.setClock(400000);</code> in <code>setup()</code> for 400 kHz Fast Mode - the SSD1311 supports it.

The display is low power, small, and lightweight and may be used in a
wide variety of embedded applications where a high visibility text
display is required that will operate in a wide variety of lighting 
conditions.


