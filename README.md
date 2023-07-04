# 40MHz-GPS-FT8-WSPR-MEPT

Note: Although the FT8 mode is included in this sketch, it is not optimal. FT8 uses
gausian pulse width shaping which will be included in a forthcoming version of the sketch.


This project was intended to be used as a very low power experimental license-free transmitter on the 40 MHz
ISM band as defined by ITU Radio Regulations. The project may also be used as an exciter to a power amplifier
by individuals holding experimental licenses for the 40 MHz band. It uses the very popular Arduino Nano and Si5351A clock generator board to generate FT8 or WSPR signals.  Frequency and time accuracy are maintained by using an inexpensive (NEO-6M) GPS receiver board. A 60 minute schedule sets operation on the 40.62 or 40.68 Mhz band and the mode of operation.  Other frequencies below 40 MHz may also be used. 
