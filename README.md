# UWB-Indoor-Localization

## Indoor localization using ESP32_UWB (DW1000) tags and anchors

WARNING 7/10/2023: There appears to be a problem with Thomas Trojer's DW1000 library that limits the number of anchors to four.

UPDATE 5/3/2022: First comprehensive range tests done (line of sight, antennas parallel), and there is a clear winner! Choose the initialization option below for range up to 33 m. All other options lead to 5-10 m at best. Linking with the newly added "high power" library provides > 50 m range under ideal conditions, but battery life will be shortened.
```
//tag:
DW1000Ranging.startAsTag(tag_addr, DW1000.MODE_LONGDATA_RANGE_LOWPOWER, false); 
// anchor:
DW1000Ranging.startAsAnchor(anchor_addr, DW1000.MODE_LONGDATA_RANGE_LOWPOWER, false);
```

BACKGROUND:

A robot should know where it is and which direction it is headed, without human input. Companies like Pozyx offer complete systems to do this, which work well and are even Arduino-compatible, but they are prohibitively expensive for hobbyists.

Several years ago, Decawave announced inexpensive, ultra wide band 3-6 GHz transceiver modules capable of indoor ranging with +/- 10 cm accuracy. Although a DW1000 Arduino library has been developed, to date, I’ve seen only one ranging example suitable for DIY robot applications, and none that are capable of true indoor localization, i.e. accurately and unambiguously determining the (x, y) or (x, y, z) position of a tag.

This repository provides working example code for using Makerfabs ESP32_UWB modules to estimate the 2D or 3D position of a tag, in the presence of 3, 4 or more anchors at known locations. With careful anchor calibration and anchor position determination, +/- 10 cm accuracy in tag position can be obtained.

The code makes extensive use of the DW1000 library by Thomas Trojer (https://github.com/thotro/arduino-dw1000), source code copied here for convenience, with
minor changes required to eliminate compilation errors using the ESP32_Arduino IDE. I've added code that is required for the tag autocalibrate procedure, so please use the library version in this repository.

The tag code collects distances to all anchors and solves the linear least squares problem of computing the tag location from known distances and anchor locations. For the method, see this short technical paper: https://www.th-luebeck.de/fileadmin/media_cosa/Dateien/Veroeffentlichungen/Sammlung/TR-2-2015-least-sqaures-with-ToA.pdf

Avantages of this particular approach (one of many) include its marvelous simplicity, linearity, high speed, accuracy, and that the normal "A" matrix depends only on the anchor configuration, and thus potentially unstable matrix inversion needs to happen only once.

### HARDWARE REQUIRED: four (2D), five (3D) or more ESP32_UWB modules from makerfabs:
https://www.makerfabs.com/esp32-uwb-ultra-wideband.html


## Setup and Installation Notes:  

1. Each anchor must be individually calibrated!  This is done quite simply by adjustment of the "antenna delay" parameter, which is unique to each anchor, in order that a known distance is accurately reported. Only one module of any pair need be calibrated. If one module has the library default antenna delay of 16384, in my experience the antenna delay of other module will be in the range of 16550 to 16650 (units are 15 picoseconds).

For general information on the DW1000 module and antenna delay calibration, see these materials:

DW1000 User manual https://www.decawave.com/sites/default/files/resources/dw1000_user_manual_2.11.pdf

DW1000 Calibration: https://www.decawave.com/wp-content/uploads/2018/10/APS014_Antennna-Delay-Calibration_V1.2.pdf

I chose a uniquely identified module, later used as the roving tag to report positions, as the common factor. Its antenna delay is set at the Arduino DW1000 library default=16384 (which is too small) and calibrated each individual anchor at a distance of 7.19 meters. Somewhere in the Decawave documentation, it is recommended that the tag be assigned antenna delay = 0 and calibrate the anchors accordingly, but I have not experimented with that option.

To perform anchor calibration:

1) Download the most recent modified version of the DW1000 library from this Github site and replace any old versions in the Arduino library folder

2) Set up a tag using the ESP32_UWB_setup_tag.ino Arduino code. The antenna delay parameter should be set to the library default.

3) Power up the tag and set it 7-8 m away from the anchor

3) Measure the distance from the tag to the anchor, and enter this distance (variable: this_anchor_target_dist) into the ESP32_anchor_autocalibrate.ino Arduino code

4) compile/link/run the autocalibrate anchor code. It does a binary search to find the optimal antenna delay, so that the measured and Time of Flight distances match.

5) Enter the reported anchor antenna delay to the ESP32_UWB_setup_anchor code, specific for that anchor, and run that code to set up the anchor. Don't forget to set each anchor to have a unique anchor MAC address. My arbitrary naming convention is for anchors 1, 2, 3, ... to have the lowest order byte of the MAC address set to 81, 82, 83, ...  and the ranging code running on the tag CURRENTLY REQUIRES them to be named accordingly.

The plot below shows the result for calibration of one anchor/tag pair and demonstrates that +/- 10 cm accuracy is achieved over a range of 1 to 8 m. I have not experimented with larger distances.

![Anchor_2_calibration](https://user-images.githubusercontent.com/5509037/151675622-8fdc3bac-088d-49b5-a4bf-96fc753d4aa2.PNG)

2. All anchors must be uniquely identifed using the lowest bits of the assigned device addresses and their 2D or 3D locations must be accurately measured. See the anchor code for examples of addressing schemes. The measurement origin is arbitrary, but that choice will become the origin of the subsequent tag coordinates.  Measurement units are arbitrary (feet, meters, etc.), but the examples assume meters.
 
4. In order to compute its position, the tag must recognize the broadcast anchor IDs and know the positions of the anchors. See the tag code for examples.

4. Anchor Placement: Anchors may be arbitrarily placed, but should be well distributed over the covered area. The corners of a large room are convenient. Coordinates must be carefully measured on a square grid. The tag coordinates will be reported in the same measurement units as the anchor coordinates, relative to the same origin. 
 
5. **Note on Z coordinates**:  In the 2D examples, the anchor Z coordinates are ignored, but the anchors and tag DO have Z coordinates, and the reported distances will include those values. This introduces some error into the position calculation if the anchors are not located in *roughly* the same Z plane (some variation in Z is required!) as the tag. In the 3D example, Z coordinates tend to be poorly determined if the tags are not widely spaced in Z. That means having at least one tag on the floor and at least one at a height of several meters above the floor.

6. The tag determines its position from the anchor distances by a linear least squares algorithm. A very crude estimate of the error is also output, which is simply the root mean square difference between the calculated anchor-tag distances and the reported anchor-tag distances. See below for experiments to determine whether there is a useful relationship between this value and the actual position error. It is of course possible to estimate coordinate uncertainties from the least squares normal matrix, but that feature remains to be added to the code.

6. Currently, localization examples are hard coded to recognize a certain number and configuration of anchors. It is straightforward to generalize the method
to work with a variable number of anchors and to choose the appropriate method for location determination. I'll add to this as the project progresses and welcome input from others.

7. There is a lot to be done to improve accuracy, robustness, and flexibility of anchor selection. Hopefully this will get other contributors started.

8. For experimentation with the trilateration algorithm, I've uploaded simple C test code. I use it with the Code::Blocks IDE. It allows studies of sensitivity to noise in the measurements, the effect of anchor placement, etc.

## Test cases

I've posted C code that can be used to simulate and conduct various tests of the basic localization algorithm in either 2D or 3D.

## 3D tests

Initial tests with the 3D, 4 anchor (not overdetermined) case indicate that the returned position is very sensitive to noise in the distance measurements as well as the details of anchor placement.  Averaging of several position measurements and/or distance measurements is clearly required. Increasing the number of anchors, so that the least squares problem is overdetermined, will certainly help.  

As stated above, the localization procedure publishes a crude estimate of the error in position, which is in actuality the RMS deviation of calculated versus measured tag-anchor distances.

The plot below is from a run in which 1000 random positions were generated within the box defined by the anchors, noisy anchor distances were generated, and were used to make 1000 position estimates, each an average of 10 independent position calculations from 10 different sets of noisy distance data. Noise from the radio module distance estimates was simulated by adding +/- 10 cm (zero mean Gaussian distribution) to the calculated distances.  

The plot suggests that a reasonable estimate the true coordinate error is obtained by multiplying the reported distance error by a factor 3.

![error_analysis](https://user-images.githubusercontent.com/5509037/152608604-b8ad3b52-d900-421b-a4cb-7c2bc54b87d2.PNG)

On the other hand, averaging multiple measurements looks to provide a much better estimate of the coordinate error. The plot below shows the result of a 1000 point simulation, where each point was obtained as an average of 10 individual position estimates, each based on noisy distance data as above. The standard deviation of the average, divided by 2, appears to be a reasonable estimate of the true coordinate error. The standard deviation of the average is in this particular case dominated by errors in the Z coordinates, largely due to too little range in the anchor Z coordinates.

![position_averaging](https://user-images.githubusercontent.com/5509037/152617681-fb00e5b7-99f0-4a1d-b684-ae8821c010f5.PNG)

## 2D tests

The 2D_4A (4 anchor) calculation, which is overdetermined, is MUCH more accurate than the 3D, 4 anchor case!  Simulations using the "2D_4A_noise_tests" code shows that with proper anchor placement and 10-fold position averaging, the 2D (x,y) position error is strictly comparable to the assumed noise in the distance measurements (+/- 10 cm). 

Plot for 1000 simulations:  
![2D_obs_vs_calc](https://user-images.githubusercontent.com/5509037/152667662-15a8966c-72c4-439f-ab07-ac3df539ddba.PNG)

Source code 2D_4A_noise_tests.c

EXCEL file of those results here:

[tl2D_1.xlsx](https://github.com/jremington/UWB-Indoor-Localization_Arduino/files/8009421/tl2D_1.xlsx)


## Ideas for future development

1. Fix the DW10000 library. Range errors are common (e.g. negative distances), etc.
2. Experiment with higher power transmit options, for larger distance ranging
3. Have the anchor transmit its coordinates along with a range report?
4. Implement use of bluetooth or WiFi to obtain tag coordinates, update tag and anchor parameters, etc.
5. Reduce tag/anchor power consumption using sleep modes
6. Your input to this list...
