# UWB-Indoor-Localization
## Indoor localization using ESP32_UWB (DW1000) tags and anchors

Example Arduino code using ESP32_UWB modules to estimate the 2D or 3D position of a tag, in the presence of 3, 4 or more anchors at known locations. With careful anchor calibration and anchor position determination, +/- 10 cm accuracy in tag position can be obtained.

The code makes extensive use of the Arduino DW1000 libary by Thomas Trojer (https://github.com/thotro/arduino-dw1000), source code copied here for convenience, with
minor changes required to eliminate compilation errors using the ESP32_Arduino IDE.

The code collects distances from the tag to all anchors and solves the linear least squares problem of computing the tag location from known distances and anchor locations.
For the method, see this short technical paper: https://www.th-luebeck.de/fileadmin/media_cosa/Dateien/Veroeffentlichungen/Sammlung/TR-2-2015-least-sqaures-with-ToA.pdf

An advantage of this particular approach is that the normal "A" matrix depends only on the anchor configuration, and thus matrix inversion needs to happen only once.

### HARDWARE REQUIRED: four (2D), five (3D) or more ESP32_UWB modules from makerfabs:
https://www.makerfabs.com/esp32-uwb-ultra-wideband.html


## Setup and Installation Notes:  

1. Each anchor must be individually calibrated!  This is done quite simply by trial and error adjustment of the "antenna delay" parameter, which is unique to each anchor, in order that a known distance is accurately reported. Only one module of any pair need be calibrated. If one module has the library default antenna delay of 16384, in my experience the antenna delay of other module will be in the range of 16550 to 16650 (units are 15 picoseconds).
For general information on the DW1000 module and antenna delay calibration, see these materials:

DW1000 User manual https://www.decawave.com/sites/default/files/resources/dw1000_user_manual_2.11.pdf

DW1000 Calibration: https://www.decawave.com/wp-content/uploads/2018/10/APS014_Antennna-Delay-Calibration_V1.2.pdf

I chose a uniquely identified tag, later used to report positions, as the common factor, with its anchor delay set at the Arduino DW1000 library default=16384 (which is too small) and calibrated each individual anchor at a distance of 7.19 meters. Somewhere in the Decawave documentation, it is recommended that the tag be assigned antenna delay = 0 and calibrate the anchors accordingly, but I have not experimented with that option.

The plot below shows the result for calibration of one anchor/tag pair and demonstrates that +/- 10 cm accuracy is achieved over a range of 1 to 8 m. I have not experimented with larger distances.

![Anchor_2_calibration](https://user-images.githubusercontent.com/5509037/151675622-8fdc3bac-088d-49b5-a4bf-96fc753d4aa2.PNG)

2. All anchors must be uniquely identifed using the lowest bits of the assigned device addresses and their 2D or 3D locations must be accurately measured. See the anchor code for examples of addressing schemes. The measurement origin is arbitrary, but that choice will become the origin of the subsequent tag coordinates.  Measurement units are arbitrary (feet, meters, etc.), but the examples assume meters.
 
4. In order to compute its position, the tag must recognize the broadcast anchor IDs and know the positions of the anchors. See the tag code for examples.

4. Anchor Placement: Anchors may be arbitrarily placed, but should be well distributed over the covered area. The corners of a large room are convenient. Coordinates must be carefully measured on a square grid. The tag coordinates will be reported in the same measurement units as the anchor coordinates, relative to the same origin. 
 
5. **Note on Z coordinates**:  In the 2D examples, the anchor Z coordinates are ignored, but the anchors and tag DO have Z coordinates, and the reported distances will include those values. This introduces some error into the position calculation if the anchors are not located in the same Z plane as the tag. In the 3D example, Z coordinates tend to be poorly determined if the tags are not widely spaced in Z. That means having at least one tag on the floor and 
at least one at a height of several meters above the floor.

6. The tag determines its position from the anchor distances by a linear least squares algorithm. A very crude estimate of the error is also output, which is simply the root mean square of the difference between the calculated distances and the reported distances. I have not made an effort to determine if there is a useful relationship between this value and the actual position error. 

6. Currently, localization examples are hard wired to recognize a certain number and configuration of anchors. It is straightforward to generalize the method
to work with a variable number of anchors and to choose the appropriate method for location determination. I'll add to this as the project progresses and welcome input from others.

7. There is a lot to be done to improve accuracy, robustness, and flexibility of anchor selection. Hopefully this will get other contributors started.

8. For experimentation with the trilateration algorithm, I've uploaded simple C test code. I use it with the Code::Blocks IDE. It allows studies of sensitivity to noise in the measurements, the effect of anchor placement, etc.
