#GPS + Sigfox tests

##Technology

* [Akeru](http://akeru.cc) Arduino + Sigfox modem board
* [Adafruit GPS](https://www.adafruit.com/products/746) 
* 

##What

### Step 1 : GPS Tracker

Periodically fetch the current position of the tracker through GPS  
Send it via Sigfox
Data is then forwarded to a server 

_Current status_ : (Almost) work. Main issue is the GPS, as we're using the naked GPS chip, without antenna. Which only works when facing the sky, and in good conditions.

###Step 2 : Treasure Hunt

Same as step 1 + use of the ACK transmission to forward data to the device through Sigfox

_Current status_ : Looking for info about the way to handle the Sigfox ACK. Doesn't seem covered by the Akeru lib, but still waiting for confirmation.

