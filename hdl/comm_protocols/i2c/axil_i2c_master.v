/*  I2C Notes:
    Speeds:
        standard = 100kbps
        fast = 400kbps
        high speed = 3.4 Mbps
        ultra fast = 5 Mbps

    I2C Message:
        [ START | ADDRESS | READ/WRITE | ACK/NACK | DATAFRAME1 | ACK/NACK | DATAFRAME2 | ACK/NACK | STOP ]

        START      = SDA switches from hight to low before the SCL line switches from high to low
        ADDRESS    = 7 or 10 bit sequence unique to each slave (slave id)
        READ/WRITE = Single bit that indicates if master is reading(1) from or writing(0) to the slave
        ACK/NACK   = receiving device will notify sender that a frame has been received (ACK=SDA pulled low)
        STOP       = SDA switches from low to high after SCL switches from low to high
        DATAFRAME  = 8 bits long send MSB first

    - use AXI4-lite interface for output of master module
        - This should be an AXI-Lite slave
    - should debounce SDA/SCL see https://support.xilinx.com/s/article/61861?language=en_US
    - I2C usually externally pulled high using pullup resistors
    - SDA must be pulled up to VCC with a 5.6-kilo-ohm resistor and SCL also needs to be pulled up with the same 
        resistor only if there are multiple masters in the system or when the slave will perform clock stretching 
        as a flow control measure to synchronise with the master.
    - The master device controls the SCL, which dictates the timings of all transfers on the I2C bus.
    - SDA is an open-drain bidirectional line. therefore in the entity deceleration, SDA signal is defined as 
       in/out so that it is capable of both input and output operations.
    
    - need glitch filtering
*/