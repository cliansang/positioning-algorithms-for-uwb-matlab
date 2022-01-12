# Positioning Algorithms for UWB Localization in Matlab

The Matlab scripts and its corresponding experimental data for five positioning algorithms regarding UWB localization system in positioning and navigation purposes are provided in this repository. The five algorithms are Extended Kalman Filter (EKF), Unscented Kalman Filter (UKF), Taylor Series-based location estimation technique, Trilaateration, and Multilateration methods. The UWB system under investigation is assumed as the state-space model. A constant velocity motion model is applied in the state model of the UWB-based positioning and navigation system. In contrast to typical simulation approach for comparative analysis of location estimation algorithms in literature, we use the experimental data directly achieved from the UWB hardware module manufacutured by Qorvo (formely Decawave) namely TREK1000/EVK1000 development boards. The documentation of the evalauted algorithms and a few results regarding their comparative analyses can be found in our paper entitle [A Comparative Study of UWB-based True-Range Positioning Algorithms using Experimental Data](https://ieeexplore.ieee.org/document/8970249). 

The code snippets regarding the point clould transformation and registration of the Vicon camera system used in this repo were written by [Timo](https://github.com/tik0) and are fully credited to him.


## Dependencies
The [Control System Toolbox](https://www.mathworks.com/products/control.html) from mathworks was used for the implementation of UKF and EKF in this repo.   

## Publications 
If you found this work useful in the academic context, please consider to cite the following paper:

    @INPROCEEDINGS{Sang2019Comparative,
        author={Sang, Cung Lian and Adams, Michael and Hesse, Marc and Hörmann, Timm and Korthals, Timo and Rückert, Ulrich},
        booktitle={2019 16th Workshop on Positioning, Navigation and Communications (WPNC)}, 
        title={A Comparative Study of UWB-based True-Range Positioning Algorithms using Experimental Data}, 
        year={2019},
        volume={},
        number={},
        pages={1-6},
        doi={10.1109/WPNC47567.2019.8970249}
    }
