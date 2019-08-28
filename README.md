

##Kalman Filter and Gaussian distribution
In Kalman filters the distribution is given by what we called a Gaussian and a Gaussian is a continuous function over the space of locations in the area underneath sums up to one.A Gaussian is characterized by two parameters:


    •	The mean often abbreviated with the Greek letter Mu 
    •	The width of the Gaussian often called Variance.
    
<img src="./img/1.jpg" alt="Gaussian"  style="display: block; margin: 0 auto" />

The formula of the Gaussian is presented below:

<img src="./img/2.jpg" alt="The formula of the Gaussian "  style="display: block; margin: 0 auto" />


To track objects, the Kalman Filter represents our distributions by Gaussians and iterates on two main cycles. The key concepts from these cycles referenced in the below:

The first cycle is the Measurement Update.

        Requires a product
        Uses Bayes rule.
        
The second cycle is the Motion Update (Prediction).

        Involves a convolution
        Uses total probability.

### The first cycle is the Measurement Update:

I am going to talk about the measurement cycle using Gaussians: Suppose we are localizing another vehicle which has a distribution (called prior) that looks like as follow (black line) now we get a measurement that tell us something about localization of the vehicle (blue line) .

<p align="center">
<img src="./img/3.jpg" alt="The first cycle is the Measurement Update " />
<p align="center">

By multiplying two Gaussians (prior and Measurement), the new mean and covariance of the subsequent Gaussian (measurement update) has a smaller covariance then the measurement and prior distribution (Notice: wide covariance means we are uncertain about localization and smaller covariance means we are more certain about localization)

<p align="center">
<img src="./img/4.JPG" alt="multiplying two Gaussians " />
<p align="center">


### The second cycle is the Motion Update (Prediction):
Suppose the robot lives in the life like below (blue line) and want to move to the right side with a certain distance, the motion(green line) has itself its own set of uncertainty r2 (because the motion tends to lost information ), which adds to the uncertainty of the current uncertainty σ2 and leads to a new Gaussian with high uncertainty  σ2 prime (red line):
<p align="center">
<img src="./img/5.jpg" alt="The second cycle is the Motion Update (Prediction) " />
<p align="center">

This leads to increased uncertainty over the initial uncertainty and the maths for this is really easy:
<p align="center">
<img src="./img/6.jpg" alt="uncertainty over the initial uncertainty  " />
<p align="center">

## That was a full kalman filter for 1D case but in the reality, we have many dimensions:

Suppose we have 2D state (x and y position) in our case, you might have a car that uses a Radar to detect the location of other vehicles over time (the sensor itself only sees positions and it never sees the actual velocity), what the 2D kalman filter affords you is something amazing.

<p align="center">
<img src="./img/7.JPG" alt=" 2D state " />
<p align="center">
    
    
At time t=0 you observe the object of interest to be at the coordinate t=0, one time step later you see over here (t=1) and so on. A Kalman filter allows you to figure out what the velocity of the object is and uses the velocity estimate to make a good prediction about the future location of an object (the velocity is inferred from seeing multiple positions).

## To explain a kalman filter for many dimension I have to explain high dimensional Gaussian, which often called Multivariate Gaussian:

The mean is now a vector for each of the dimensions:
<p align="center">
<img src="./img/8.JPG" alt=" mean vector" />
<p align="center">
    
The variance is replace by what called a covariance and it is a matrix with D rows and D columns (if dimensionality of the estimate is D) 

<p align="center">
<img src="./img/9.JPG" alt=" covariance matrix" />
<p align="center">
    
The formula is: 

<p align="center">
<img src="./img/10.JPG" alt=" formula " />
<p align="center">
    
For two-dimensional space, a two dimensional Gaussian is defined over the space (it is possible to draw the contour lines of a Gaussian), the mean of the Gaussian is x and y pair and the covariance defined over the spread of the Gaussian. When the Gaussian is tilted as showed the uncertainty of x and y is correlated, which mean if I get information about x (point A) that make me believe that y probably sits at coordinate B.
<p align="center">
<img src="./img/11.JPG" alt=" correlation " />
<p align="center">
## Another Example that can explain a Kalman filter is presented below:
    
Suppose we have two dimensions (one for the location, which is observable from the sensor and denoted by x, and one for the velocity, which is not observable from the sensor and denoted by x dot) 

        •	you have a correlation Gaussian called prior (we know our locations is correlated to the velocity, much faster I move, the further is the location ) 
        •	you got a new measurement about the location (measurement) but you know nothing about the velocity)
By multiplying the measurement and prior Gaussians, you get a Gaussian (black line) that sits on the middle and has a good estimate what your velocity is and where your location is.
<p align="center">
<img src="./img/12.JPG" alt=" Example  " />
<p align="center">

# Big Lesson:
## The Variables of a Kalman Filter 
	Often called State because they reflect the state of the physical world like position and velocity.
	They separate into two subsets:
        o	Observable (like the position)
        o	Hidden which can never directly observed (in our example velocity) and because these two thing interact (observable variables give us information about hidden information) we can estimate or inference what these hidden variables are.



