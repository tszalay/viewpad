viewpad
=======

Rudimentary stereo gaze tracking software, written in 2010-2012.
Relies on two infrared-modified Logitech C600 cameras running at 1600x1200x30fps, along with two bright IR LED emitters.
It uses only dark-pupil method along with reflection tracking, none of this bright-pupil nonsense, and OpenCV's camera calibration routines to do 3D position tracking.
Eye localization is done using Haar classifiers, pupil detection using a variant of Starburst/RANSAC algorithm, and bright dot reflection tracking using a modified version of OpenCV's SURF/star detection code (which was an instructive pain to decipher).
As it stands, the code is decent, and not great, at the full gaze tracking loop, but all of the feature detection is fairly robust, and with some minor tweaks and bug fixes, should result in solid performance. In addition, there is Mathematica code output by the program that displays the detected calibration data after a calibration run, in order to verify the results.

This code is completely unsupported, and I likely will never return to it again, since the motivation for writing it was to implement some new ideas I had, which subsequently got patented by somebody else. So, good luck!

-Tamas
