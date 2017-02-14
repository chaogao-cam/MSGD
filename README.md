# MSGD
**Authors:** [Chao Gao]<cg500 at cam dot ac dot uk> (University of Cambridge)

**Current version:** 1.0.0 

MSGD is a scalable back-end for indoor magnetic field-based graphslam. Please refert to the related publication below for details.


###Related Publication:
Chao Gao and Robert Harle. **Msgd: Scalable back-end for indoor magnetic field-based graphslam**. In *Robotics and Automation (ICRA), 2017 IEEE International Conference on*. IEEE, 2017.

###License
MSGD is released under a [GPLv3 license](https://github.com/chaogao-cam/MSGD/COPYING.txt). For all the third-party code/libraries included/extended/linked by MSGD, please see [Dependencies.md](https://github.com/chaogao-cam/MSGD/Dependencies.md).

###Prerequisites
MSGD has been tested in **OS X El Capitan** and **macOS Sierra**. 

###Datasets and Groundtruth
All datasets and groundtruth files are in the folder *datasets* (g2o format). For WGB2a-1, WGB2a-2, WGB2a-3 and WGB2a-4, the Bat Ultrasonic Location System (http://www.cl.cam.ac.uk/research/dtg/attarchive/bat/) was used to provide high-accuracy groundtruth. For other datasets, the groundtruths were obtained by manual labelling etc., so the accuracy is relatively low.

###Build and Run
```
git clone https://github.com/chaogao-cam/MSGD.git MSGD
cd MSGD
make all
./msgd -in ./datasets/input_g2o_file_name.g2o -out result_folder_path
```
