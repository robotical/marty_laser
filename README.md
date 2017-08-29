# marty_laser
This class controls the operation of our DIY laser range finder, which uses visual data from a
camera to estimate distances to objects.

In this setup, the camera and the laser are mounted in parallel.

This concept works on a simple principle: the closer the dot is to the bottom of the image, the closer the object is, and vice-versa. The movement of this dot can be tracked and matched to a known distance.

This principle can be extended to line lasers, where a horizontally positioned laser can be used to collect 2D data.

> CAUTION: this is bleeding edge and highly experimental. Still a WIP.
>
> Currently only performs range estimation based on a "dot" laser (think laser
pointers). "Line" laser range estimation is still in-dev.

## Installation

To install, simply clone this repo into your `catkin` workspace and perform a `catkin_make`. For example, cloning onto your Marty:

    cd ~/marty_ws/src
    git clone https://github.com/robotical/marty_laser.git
    cd .. && catkin_make

## Running

To execute the node, simple `roslaunch` the launch file:

    roslaunch marty_laser laser_node.launch

By default, the node is set to not show the result of image processing, if you wish to see this set the `laser_video_out` param in `laser_params.yaml` to True.

Once executed, this node will publish range estimation over the `/marty/laser/distance` topic.

## Configuration

See the provided configuration file, `laser_params.yaml` for details on configuration.

> Note: currently, the node is calibrated for a **red** laser on a white background. You will most likely have to alter both the lower and upper thresholding values to suit your environment. In addition to this, you will most likely have to alter the range estimation function defined in `calc_distance()`, as minute differences in your laser/camera setup can cause varying results. These are fortunately relatively easy to obtain experimentally.

## Accuracy

The accuracy of your obtained results are *hugely* dependent on a number of factors:

  1. the suitability of your colour thresholding parameters
  2. the accuracy of your range estimation function
  3. the design of your laser and camera mount

For 1, this rangefinder will not be able to provide you with a range
estimation if it cannot detect the laser to begin with. There are default
thresholding parameters provided in `laser_params.yaml`
(HSV values for a red laser) but you will very likely have to tweak these
for your own operating environment. To aid with this, you can enable
`laser_video_out` and `threshold_config`, which will bring up a video feed
with configuration trackbars. You can adjust the settings until you find
one that is suitable and then replace the values in `laser_params.yaml`.
Note: the HSV colour space has been inverted (to simplify the detection
of red).

For 2, creating a good function is dependent on good experimental practices.
The provided function (in `calc_distance()`) was obtained by plotting the
position of the centroid of the laser dot against known distances.

For 3, having a loose mount where the laser and/or the camera can move
relative to each other is a bad idea. The method implemented in this
library uses movement in the y axis, so for that reason the laser and
camera are mounted inline vertically. For best results, ensure that
your laser and camera are *secure* and will not move relative to each
other.
