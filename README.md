# Positioning Algorithms for UWB Localization in Matlab

The Matlab scripts and its corresponding experimental data for five positioning algorithms regarding UWB localization system are provided in this repository. The five algorithms are Extended Kalman Filter (EKF), Unscented Kalman Filter (UKF), Taylor Series-based location estimation technique, Trilateration, and Multilateration methods. The UWB system under investigation is assumed as the state-space model. A constant velocity (CV) motion model and constant acceleration (CA) motion model are usable as the state model for the implementation of the mentioned UWB positioning and navigation system in this repo. In contrast to typical simulation approach for comparative analysis in literature, we use the experimental data directly achieved from the UWB hardware module manufacutured by Qorvo (Decawave) called [TREK1000/EVK1000](https://www.decawave.com/product/evk1000-evaluation-kit/) evaluation boards. 

The detailed description of the evaluated five algorithms, their implementation processes, and comparative results were addressed in [Chapter 4 of my Dissertation](https://doi.org/10.4119/unibi/2967742), which is self-contained and independently readable. However, the mentioned chapter is recommended to read in conjunction with Chapter 2. A short version of the documentation is also available on our IEEE workshop paper, of which the accepted version is publicly provided in [here](https://pub.uni-bielefeld.de/download/2937505/2966816/KS_IEEE_WPNC_2019_paper_accepted_version.pdf).

The code snippets regarding the point clould transformation and registration of the Vicon camera system used in this repo were written by [Timo](https://github.com/tik0).


## Dependencies
The [Control System Toolbox](https://www.mathworks.com/products/control.html) from mathworks was used for the implementation of UKF and EKF in this repo.   

## Publications 
If you find this work useful in the academic context, please consider to cite one of the following papers:

    @INPROCEEDINGS{Sang2019Comparative,
        author={Sang, Cung Lian and Adams, Michael and Hesse, Marc and Hörmann, Timm and Korthals, Timo and Rückert, Ulrich},
        booktitle={2019 16th Workshop on Positioning, Navigation and Communications (WPNC)}, 
        title={A Comparative Study of UWB-based True-Range Positioning Algorithms using Experimental Data}, 
        year={2019},        
        pages={1-6},
        doi={10.1109/WPNC47567.2019.8970249}
    }

    @INPROCEEDINGS{Sang2019Bidirectional,
        author={Sang, Cung Lian and Adams, Michael and Korthals, Timo and Hörmann, Timm and Hesse, Marc and Rückert, Ulrich},
        booktitle={2019 International Conference on Indoor Positioning and Indoor Navigation (IPIN)}, 
        title={A Bidirectional Object Tracking and Navigation System using a True-Range Multilateration Method}, 
        year={2019},        
        pages={1-8},
        doi={10.1109/IPIN.2019.8911811}
    }

    @ARTICLE{Sang2023Bidirectional,
        author={Sang, Cung Lian and Adams, Michael and Hesse, Marc and Rückert, Ulrich},
        journal={IEEE Journal of Indoor and Seamless Positioning and Navigation}, 
        title={Bidirectional UWB Localization: A Review on an Elastic Positioning Scheme for GNSS-deprived Zones}, 
        year={2023},
        volume={1},
        pages={161-179},
        doi={10.1109/JISPIN.2023.3337055}
    } 
