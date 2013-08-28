text_locator
============

text_locator is a ROS package that can detect and recognize text in images from the MS kinect / Asus Xtion sensors. Also, it can give you spatial information about the text, ie. where the text is in the 3D space, with respect to the sensor.

usage
-----
install by
	 

Launch the node by running

	roslaunch text_locator text_locator.launch

Note that before using, you need to install and set up Tesseract OCR, which should be available from your package manager.

output
------
The output of the node is advertised on text_locator_topic. The message contains the following information:

	Header
	Coordinates of the upper left corner of detected text region in the 3D space
	Coordinates of the lower right corner of detected text region in the 3D space
	Recognized text


settings
--------
The launchfile contains some parameters you may want to change:

enable_pcl - 
Enables the spatial information. If disabled, you will only get the recognized text. True by default.

enable_param_debug - 
If enabled, the node will write an image with the detection result to your home directory. You can use it to fine-tune the parameters of the detector or just observe the results. Currently, changing the detection parameters is only available from the sources. False by default. Note that normally the node does not write any images to your drive.

lang - 
Language for tesseract to expect. This is always a three-character code.Tesseract supports a number of [languages](https://code.google.com/p/tesseract-ocr/downloads/list). Set to "eng" by default.

page_mode - 
Tesseract page mode. Tells tesseract whether to expect a character, a word, a whole line, etc. Defined [here](https://code.google.com/p/tesseract-ocr/source/browse/trunk/ccstruct/publictypes.h#151). Set to "PSM_SINGLE_LINE" by default.

general information
-------------------
When used out of the box, the detection algorithm uses its default parameters and the recognition works best for words consisting of large characters (about 4 cm in height) which are positioned horizontally, facing the sensor and up to about 3 m from it (when using VGA resolution). Keep in mind that the depth sensor has limited range and the spatial information will contain NaNs if the object is too close. Use standard fonts. The text published in the message is the raw output from Tesseract OCR and it may look like a completely random character sequence. There is no preprocessing of the image.

acknowledgements
----------------
The code builds upon:

[CCV](http://libccv.org/) computer vision library for text detection.

[Tesseract OCR](https://code.google.com/p/tesseract-ocr/) for text recognition.

