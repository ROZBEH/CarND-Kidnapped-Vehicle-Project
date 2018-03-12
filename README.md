# Kidnapped Vehicle Project

[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)



<p align="center">
<img src="https://j.gifs.com/L8DDWg.gif" width = "600" />
</p>


Overview
---


In this project we are going to use Particle Filter for the localization of self driving car.


Pipeline
---


*The overall pipeline along with the results will be described here!*

<br>

I. First thing first, initialization of the particles happens. 100 was choosen as the number of particles. Once the number of particles has been choosen we loop through each particl and set the weights for all of them equal to one. After that, we set the x and y location of each particle equal to the sensor data that were return from the simulation. Once the initialization is made, next round we will ignore initialization and go to the prediction stage. Which will be described in section 4.


</br>


II. Once the particle is initialized, the weights of each particle gets updated. Depending on house close each particle is to the actual landmark observation that was made by the car, that particle gets the higher weight. Using weights of these particles we update the particle filter in such a way that particles with higher chance of being picked up as the actual location of the vehicle. Particles with lower weight have a slight chance of being picked up and they basically become discarded.


</br>

III. One of the steps that is necessary during weight update is data association. We want to know which of the observations is closest to the our predicted landmarks.


</br>


IV. During the prediction stage, we estimate the location (x, y, theta) of the vehicle based on the current measurements (velocity and yaw rate!)


</br>


V. Final video of the result is provided below. Please click on the following image to view the full video on YouTube. 
</br>

[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/jh6S3Ta3iMs/0.jpg)](https://www.youtube.com/watch?v=jh6S3Ta3iMs)

</br>
<br></br>