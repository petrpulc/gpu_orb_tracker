# Hierarchical Motion Tracking for UHD video processing

This repository complements a research of Petr Pulc, Martin Holeňa et al. in extraction of human-comprehensible information from complex data.

In this particular case, we take a challenge of processing large amounts of multimedia content with high-resolution video and/or high frame rates.

First issue we came across, and that we tackle in this particular repository, is to enable real-time extraction of motion vectors from such large amounts of data with a low amount of false motion vectors. As a result, no additional motion vector filtering should be needed.

To this end, we propose a hierarchical approach that estimates a global homography on points of interest (features) in the top layer of Gaussian pyramid and uses this information during extraction of motion vectors in lower levels as an initial estimate of feature position in the incoming frame. Estimation given in lower levels is then based on exact matches from upper layers.

Please go to [wiki](https://github.com/petrpulc/gpu_orb_tracker/wiki) for more technical information.

## Reference

If you find herein presented approach helpful and use the algorithm in your own research, we kindly ask you for a reference to a following article:

> Towards Real-time Motion Estimation in High-Definition Video Based on Points of InterestProceedings of the 2017 Federated Conference on Computer Science and Information Systems

> DOI: [10.15439/2017f417](http://dx.doi.org/10.15439/2017F417)

If you are interested how to apply our approach in your workflow, please contact us at: petrpulc@gmail.com

## Compilation & execution

Prerequisites:

-  OpenCV, developed against v. 3.4.1
    - CUDA support required for GPU acceleration
    - If CUDA is not available, merge the `cpu` branch to remove CUDA dependency

Otherwise, standard CMake procedure:

```bash
mkdir build
cd build
cmake ..
make
```

Binary (`gpu_orb_tracker`) is called with two arguments:
- Video source path
- Number of interest points detected by ORB (try 500 or more on decent hardware)

## Usage

For a basic visualisation of the motion history, checkout the `viz` branch.

For more practical use, please note, that the most interesting data are hidden in the `Motion` object in the `layers` container, where the first `OCTAVE` layers (defined in `settings.h`) are objects of class `PointLayer`. These objects then contain a `std::list` of objects from class `Feature`. And these individual features contain in `history` a `std::map` of frame number and the position of the feature in `cv::Point2f`.

In case of any troubles, please don't hesitate to contact us.
