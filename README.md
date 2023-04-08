# Specularity Removal for Detecting a Flexible Manipulator for Surgical Operations with an Endoscopic Camera
Master thesis for completeing M.Sc. in Mechatronics from University of Siegen, Siegen, Germany.

**Copyright:**
* Copyright and all rights therein are retained by authors  or by other copyright holders. All persons copying this  information are expected to adhere to the terms and constraints invoked by each author's copyright. The code may be used for non-commercial purposes only.

Published reports of research using this code (or a modified version) should cite the article that describes the algorithm: 
* Shuqair, M. Specularity Removal for Detecting a Flexible Manipulator for Surgical Operations with an Endoscopic Camera. Master Thesis, University of Siegen. 2014.
* Miyazaki, D., Tan, R. T., Hara K. and Ikeuchi K. Polarization-based Inverse Rendering from a Single View. Proceedings of International Conference on Computer Vision, pp. 982-987. 2003.
* Shen, H. L., Cai, Q. Y. Simple and efficient method for specularity removal in an image. Applied Optics, Vol. 48, Issue 14, pp. 2711-2719. 2009.

**TO DO:**
* For the standalone code, change the path of the desired image to be processed in the code from /home/mustafa/workspace/BeginnerProject/Images to the path of your image.
* For the ROS Package, copy and paste the "tubedetect" folder into the package path depending on the installation configuration of your ROS. Run the command rosmake on the tubedetect package to make sure it works well. Run the tubetect package normally after that using the command rosrun. 
* I have compiled the code in Debian Linux 7.4 using Eclipse and ROS without any problems.

## Source Images
The images were obtained from the following internet sources:
* Computational Vision Laboratory, School of Computer Science, Simon Fraser University. Accessed: 21/3/2014. http://www.cs.sfu.ca/~colour/data/colour_constancy_test_images/index.html.
* Tan, R. T. Singapore Institute of Management. Accessed: 21/3/2014.
http://php-robbytan.rhcloud.com/code.html
* Yang, Q., Yang, T. Department of Computer Science, City University of Hong Kong. Accessed: 21/3/2014.
http://www.cs.cityu.edu.hk/~qiyang/

