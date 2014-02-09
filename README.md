Adafruit_AHRS
=============

AHRS (Altitude and Heading Reference System) example for Adafruit's 9DOF and 10DOF breakouts.

This code is based on the GPL licensed AHRS code from Firetail UAV Systems, available at: http://www.camelsoftware.com/firetail/blog/uncategorized/quaternion-based-ahrs-using-altimu-10-arduino/, which was itself based on the work of David Grayson at http://blog.davidegrayson.com/2012/11/orientation-sensing-with-raspberry-pi.html

Using This Code
===============
This sample code can produce Euler angle, North-East-Down (NED) direction cosine matrix, or Quaternion output using the following commands:

- “output_euler\n” : Switches the output to euler angles (heading, pitch & roll in degrees). This is the default output.
- “output_mat\n” : Switches output to a north-east-down (NED) direction cosine matrix.
- “output_quat\n” : Switches output to quaternion.
