***************************************************************
* Arduino PID Library - Version 1.2.1
* by Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
*
* This Library is licensed under the MIT License
***************************************************************

 - For an ultra-detailed explanation of why the code is the way it is, please visit: 
   http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/

 - For function documentation see:  http://playground.arduino.cc/Code/PIDLibrary/ (Click "Libraries" on the left panel. The link to the documentation is listed as "PIDLibrary - Provides basic feedback control".)

 - NM changes:
 - doubles replaced with floats.
 - runAlways() function added to force compute() to calculate whenever called. Needed to work with external system tick.
 - #defines for mode, direction and action replaced with enums to avoid name clashes (e.g. use PID::AUTOMATIC).
 - various text format tidying
 - (note: the examples have not been updated to work with the various changes)