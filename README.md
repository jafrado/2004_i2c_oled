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

The display is low power, small, and lightweight and may be used in a
wide variety of embedded applications where a high visibility text
display is required that will operate in a wide variety of lighting 
conditions.


