

## Kalman Filter and Gaussian distribution
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
Suppose the robot lives in the life like below (blue line) and want to move to the right side with a certain motion, the motion(green line) has itself its own set of uncertainty r2 (because the motion tends to lost information ), which adds to the uncertainty of the current uncertainty σ2 and leads to a new Gaussian with high uncertainty  σ2 prime (red line):
<p align="center">
<img src="./img/5.jpg" alt="The second cycle is the Motion Update (Prediction) " />
<p align="center">

This leads to increased uncertainty over the initial uncertainty and the maths for this is really easy:
<p align="center">
<img src="./img/6.JPG" alt="uncertainty over the initial uncertainty  " />
<p align="center">

#### That was a full kalman filter for 1D case but in the reality, we have many dimensions:

Suppose we have 2D state (x and y position) in our case, you might have a car that uses a Radar to detect the location of other vehicles over time (the sensor itself only sees positions and it never sees the actual velocity), what the 2D kalman filter affords you is something amazing.

<p align="center">
<img src="./img/7.JPG" alt=" 2D state " />
<p align="center">
    
    
At time t=0 you observe the object of interest to be at the coordinate t=0, one time step later you see over here (t=1) and so on. A Kalman filter allows you to figure out what the velocity of the object is and uses the velocity estimate to make a good prediction about the future location of an object (the velocity is inferred from seeing multiple positions).

#####  To explain a kalman filter for many dimension I have to explain high dimensional Gaussian, which often called Multivariate Gaussian:

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

        •	you have a correlation Gaussian called prior
		(we know our locations is correlated to the velocity, much faster I move, the further is the location ) 
        •	you got a new measurement about the location but you know nothing about the velocity

By multiplying the measurement and prior Gaussians, you get a Gaussian (black line) that sits on the middle and has a good estimate what your velocity is and where your location is.
<p align="center">
<img src="./img/12.JPG" alt=" Example  " />
<p align="center">

### Big Lesson:
#### The Variables of a Kalman Filter 
	Often called State because they reflect the state of the physical world like position and velocity.
	They separate into two subsets:
        o	Observable (like the position)
        o	Hidden which can never directly observed (in our example velocity) and because these two thing interact 
	 	(observable variables give us information about hidden information) we can estimate or inference what these hidden variables are.
<p align="center">
<img src="./img/13.JPG" alt=" Variables of a Kalman Filter   " />
<p align="center">
	
## Extended Kalman Filter:
##### It is extended in the sense that it will be capable of handling more complex motion model and measurement models and consists of an endless loop of prediction and update state.

Imagine you are in a car equipped with sensors on the outside. The car sensors can detect objects moving around: for example, the sensors might detect a pedestrian. Let's step through the Kalman Filter algorithm using the pedestrian example. We have two sensors(a Lidar and a Radar) and the information provided be these two sensors is used to estimate the state of a moving pedestrian and this state is presented by a 4D state  vector( a x,y position and a x,y velocity).

<p align="center">
<img src="./img/14.JPG" alt="state  vector " />
<p align="center">
	
*	x is the mean state vector. For an extended Kalman filter, the mean state vector contains information about the object's position and velocity that you are tracking. It is called the "mean" state vector because position and velocity are represented by a Gaussian distribution with mean x.

*	P is the state covariance matrix, which contains information about the uncertainty of the object's position and velocity. 

Based on the below diagram, the filter (first measurement) will receive initial measurements of the pedestrian’s position relative to the car. These measurements will come from a radar or lidar sensor and the filter will initialize the pedestrian’s position based on the first measurement (initialize state and covariance matrices). At this point, the car will receive another sensor measurement after a time period Δt and then we perform two steps, state prediction and measurement update (Each time we receive a new measurement from a given sensor, the estimation function is trigged).
	
<p align="center">
<img src="./img/15.JPG" alt="process chain" />
<p align="center">
	
## In the prediction state (for Radar and Laser Measurements):
<p align="right">
<img src="./img/16.JPG" alt="In the prediction state (for Radar and Laser Measurements)" />
	
*	We predict the pedestrian’s state by taking to account the elapsed time between the current and the pervious observations, because in the reality the time elapsed between two consecutive observations might vary and is not constant.

	1.	We can use the timestamp values to compute the elapsed time 
		 between two consecutive observations and additionally
		 we divide the result by 10^6 to transform it from microseconds to seconds
		 <p align="right">
		<img src="./img/17.JPG" alt="timestamp" />
		<p align="right">
	2.	Linear Motion Model, we can predict the next state with help of the old positions plus 			displacement times Δt and plus noise, which has the mean zero.
		<p align="right">
		<img src="./img/18.JPG" alt="Linear Motion Model" />
		<p align="right">
	3.	F: is a transition matrix that, when multiplied with x, predicts where the object will be 		after time Δt.
		<p align="right">
		<img src="./img/19.JPG" alt="F: is a transition matrix " />
		<p align="right">
	4.	ν: We assume the object travels at a constant velocity, but in reality, the object might accelerate or decelerate. The notation ν∼N(0,Q) defines the process noise as a Gaussian distribution with mean zero and covariance Q. (The mean = 0 is saying that the mean noise is zero and the uncertainty shows up in the Q matrix as acceleration noise).
	5.	Q: Motion noise and process noise refer to the same case: uncertainty in the object's position when predicting location. The model assumes velocity is constant between time intervals, but in reality, we know that an object's velocity can change due to acceleration. The model includes this uncertainty via the process noise, which explained in the section (ν). The process noise depends on two things: the elapsed time and the uncertainty of acceleration. We can model the process noise by considering both of these factors.
			
		First I am going to show how the acceleration is expressed by the kinematic equation then I use that information to drive the process covariance Q (ν∼N(0,Q)).

		Say we have two consecutive observation of the same pedestrian with initial and final velocities then we can drive from the kinematic formula the change in the velocity, in other word including acceleration:		
	<p align="right">
	<img src="./img/20.JPG" alt="acceleration " />
	<p align="right">
		
	Since the acceleration is unknown, we can add it to the last component (stochastic part) and acceleration is a random vector with zero mean and standard deviation sigma ax and sigma ay.
	
	<p align="right">
	<img src="./img/21.JPG" alt="stochastic part " />
	<p align="right">
		
	The below presented vector can be decomposed into two components:

		A four by two matrix G  which doesn’t contain random components
		A two by one matrix contains random acceleration components
			
	<p align="right">
	<img src="./img/22.JPG" alt="matrix G " />
	<p align="right">
	
	Based on our noise vector we can define now the new covariance matrix Q. The covariance matrix is defined as the expectation value of the noise vector ν times the noise vector νT:
	<p align="right">
	<img src="./img/23.JPG" alt="new covariance matrix Q " />
	<p align="right">
	
	As G does not contain random variables, we can put it outside the expectation calculation.
	<p align="right">
	<img src="./img/24.JPG" alt="G does not contain random variables " />
	<p align="right">
		
	This leaves us with three statistical moments: 
		
	<p align="right">
	<img src="./img/25.JPG" alt="three statistical moments " />
	<p align="right">
	
	After combining everything in one matrix, we obtain our 4 by 4 Q matrix:
	<p align="right">
	<img src="./img/26.JPG" alt="combining everything in one matrix " />
	<p align="right">
		
	5.	B: is a matrix called the control input matrix and u is the control vector. We will assume that there is no way to measure or know the exact acceleration of a tracked object. For example, if we were in an autonomous vehicle tracking a bicycle, pedestrian or another car, we would not be able to model the internal forces of the other object; hence, we do not know for certain what the other object's acceleration is. Instead, we will set Bu=0 and represent acceleration as a random noise with mean ν.
		
	6.	x′: The x′=Fx+Bu+ν equation does these prediction calculations for us but then Bu was crossed out leaving x′=Fx+ν. The noise mean = 0 is saying that the mean noise is zero. The equation then becomes x′=F∗x.o	x′: The x′=Fx+Bu+ν equation does these prediction calculations for us but then Bu was crossed out leaving x′=Fx+ν. The noise mean = 0 is saying that the mean noise is zero. The equation then becomes x′=F∗x.
	<p align="right">
	<img src="./img/27.JPG" alt="prediction calculations  " />
	<p align="right">
		
	7.	P′=FPFT +Q represents this increase in uncertainty.
	
<p align="right">

## In The measurement update

Uses new observations to correct our belief about the state of the predictions and depends on sensor type (Lidar and Radar). If a laser sensor generates the current measurement, we just apply a standard kalman filter to update the pedestrian state and why radar measurements involves a non-linear measurement function when we receive a radar measurement, we use the extended kalman filter to measurement update.

<p align="center">
<img src="./img/28.JPG" alt="In The measurement update " />
<p align="center">
	
Notice: the state of the pedestrian’s position and velocity is updated asynchronously each time the measurement received regardless of the source sensor.
	
## 	Update Step for Laser Measurement:

1.	H (state transition matrix) we use the measurement function to map the state vector into the measurement space of the sensor. To give a concrete example, lidar only measures an object's position (px, py) but the extended Kalman filter models an object's position and velocity. So multiplying by the measurement function H matrix will drop the velocity information from the state vector x.

<p align="right">
<img src="./img/29.JPG" alt="o	H (state transition matrix " />
<p align="right">

2.	w: represents sensor measurement noise. Measurement noise refers to uncertainty in sensor measurements. The notation ω∼N(0,R) defines the measurement noise as a Gaussian distribution with mean zero and covariance R. Measurement noise comes from uncertainty in sensor measurements.

3.	z=H∗x+w for the update step (Measurement Function which is a linear function). We use the measurement function to map the state vector into the measurement space of the sensor. To give a concrete example, LIDAR only measures an object's position but the extended Kalman filter models an object's position and velocity. So multiplying by the measurement function H matrix will drop the velocity information from the state vector x. Then the lidar measurement position and our belief about the object's position can be compared. 

<p align="right">
<img src="./img/30.JPG" alt="Measurement Function  " />
<p align="right">


	
4.	R, which represents the uncertainty in our sensor measurements. The dimensions of the R matrix is square and each side of its matrix is the same length as the number of measurements parameters z. The matrix R represents the uncertainty in the position measurements we receive from the laser sensor and generally, the parameters for the random noise measurement matrix will be provided by the sensor manufacturer.

<p align="right">
<img src="./img/31.JPG" alt="o	R, which represents the uncertainty in our sensor measurements " />
<p align="right">


5.	y=z−Hx′: Now we get some sensor information (z) that tells where the object is relative to the car. First we compare where we think we are with what the sensor data tells us y=z−Hx′.

6.	The K matrix, often called the Kalman filter gain, combines the uncertainty of where we think we are P′ with the uncertainty of our sensor measurement R. If our sensor measurements are very uncertain (R is high relative to P'), then the Kalman filter will give more weight to where we think we are: x′. If where we think is uncertain (P' is high relative to R), the Kalman filter will put more weight on the sensor measurement: z.

7.	I is identity matrix.

8.	The Kalman Filter update Formula for Laser Measurements:

<p align="right">
<img src="./img/32.JPG" alt="The Kalman Filter update Formula for Laser Measurements: " />
<p align="right">
	
##	 Update Step for Radar Measurements:

Radar sees the world differently, as you can see in the photo our vehicle is at A coordinate and the pedestrian at B coordinate. The x-axis is always points in the vehicle’s direction of movement and the y-axis is always points to the left. Instead of 2D, the Radar can directly measure:


<p align="right">
<img src="./img/33.JPG" alt="Radar measurement" />
<p align="right">
	
<p align="right">
<img src="./img/34.JPG" alt="Radar measurement" />
<p align="right">
	
Using Doppler effects the Radar can directly measure the radial velocity of a moving object and the radial velocity is the component (ρ˙) of the velocity moving towards or away from the sensor. 

1.	z: Measurement vector is 3-measurement vector component:
<p align="right">
<img src="./img/35.JPG" alt="z: Measurement vector " />
<p align="right">

	
2.	R: Radar measurement Covariance becomes a 3by 3 diagonal matrix (considering that the tree measurement vector component are not cross-correlated).

<p align="right">
<img src="./img/36.JPG" alt="z: R: Radar measurement Covariance  " />
<p align="right">
	
3.	y=z−h(x′): Our state (x′) from the prediction step still has four parameter as the same and as before said the measurement vector has 3 parameters, in order to calculate y=z−h(x′) we need a measurement function that maps the predicted state(x′) into measurement space:

<p align="right">
<img src="./img/37.JPG" alt="measurement function   " />
<p align="right">
	
The h is nonlinear function that specifies how the predicted position and speed get mapped to the 	polar coordinates of range, bearing and range rate and h function is presented below:
	<p align="right">
	<img src="./img/38.JPG" alt="h is nonlinear function   " />
	<p align="right">

Notice: One other important point when calculating y with radar sensor data: the second 	value in the polar coordinate vector is the angle ϕ. You will need to make sure to normalize ϕ in 	the y vector so that its angle is between –π and π.

4.	Hj : After applying a nonlinear measurement function, we have 3-measurement vector component (z) which cannot be used for the Kalman Filter equations to update the predicted state (with new measurements) because we are not working with the Gaussian distribution after applying a nonlinear measurement.
To understand the problem I used a Gaussian (1) from 10,000 random values in a normal distribution with a mean of 0 then applied (2) a nonlinear function (arctan) to transform each value as you can see in 3 the resulting distribution is not any more a Gaussian distribution.

<p align="right">
<img src="./img/39.JPG" alt="a nonlinear function (arctan) to transform each value  " />
<p align="right">

To solve this problem, we have to approximate (linearize) our measurement function to a linear function, which is the key idea of the extended Kalman filter. I repeated the test by using a linear approximation. Notice how the blue graph, the output, remains a Gaussian after applying a first order Taylor expansion.

<p align="right">
<img src="./img/40.JPG" alt="after applying a first order Taylor expansion  " />
<p align="right">

The extended kalman filter uses a method called First Order Tylor Expansion:

<p align="right">
<img src="./img/41.JPG" alt="First Order Tylor Expansion: " />
<p align="right">

What we do is we first evaluate the nonlinear function h at the mean location (μ), which is the best estimate of our predicted distribution then we extrapolate a line with slope around μ and this slope is given by the first derivative of h.

<p align="right">
<img src="./img/42.JPG" alt=" function h : " />
<p align="right">

Now that you have seen how to do a Taylor series expansion with a one-dimensional equation, we will need to look at the Taylor series expansion for multi-dimensional equations. We will need to use a multi-dimensional Taylor series expansion to make a linear approximation of the h function. Here is a general formula for the multi-dimensional Taylor series expansion:

<p align="right">
<img src="./img/43.JPG" alt="multi-dimensional Taylor series expansion  " />
<p align="right">

Where Df(a) is called the Jacobian matrix and D2f(a) is called the Hessian matrix. They represent first order and second order derivatives of multi-dimensional equations. A full Taylor series expansion would include higher order terms as well for the third order derivatives, fourth order derivatives, and so on. Notice the similarities between the multi-dimensional Taylor series expansion and the one-dimensional Taylor series expansion:

<p align="right">
<img src="./img/44.JPG" alt="the similarities between the multi-dimensional Taylor series expansion and the one-dimensional Taylor series expansion " />
<p align="right">

To derive a linear approximation for the h function, we will only keep the expansion up to the Jacobian matrix Df(a). We will ignore the Hessian matrix (https://www.khanacademy.org/math/multivariable-calculus/applications-of-multivariable-derivatives/quadratic-approximations/a/the-hessian) D2f(a) and other higher order terms. Assuming (x−a) is small, (x−a)2 or the multi-dimensional equivalent (x−a)T (x−a) will be even smaller; the extended Kalman filter we'll be using assumes that higher order terms beyond the Jacobian are negligible.

The derivative of h(x) with the respect to x is called Jacobian matrix and is going to be a matrix containing all the partial derivatives, we know, the measurement function describes tree component (Range, Bearing, Range Rate) and my state is a vector with four components (px,py,vx,vy), in that case the Jacobian matric is going to be a matrix with 3 rows and 4 columns.
	

<p align="right">
<img src="./img/45.JPG" alt="The derivative of h(x) " />
<p align="right">

After calculating all the partial derivatives, our resulted Jacobian, Hj is:

<p align="right">
<img src="./img/46.JPG" alt="After calculating all the partial derivatives " />
<p align="right">
	
Notice: because the linearization points change, we have to recompute the Jacobian matrix at every point in the time.


5.	For radar measurement update, Hj is used to calculate S, K and P.

<p align="right">
<img src="./img/47.JPG" alt="For radar measurement update, Hj is used to calculate S, K and P." />
<p align="right">
	
## Workflow of the extended kalman filter works:

Here the pedestrian’s state at time K is a distribution with mean (X) and covariance (P) and at time k+1 we just received the laser measurement (Cartesian coordinate System) and the first thing to do is to make a prediction about where we think the pedestrian would be at time k+1.

The second thing is to do called measurement update where we combine the pedestrian predicted state with the new laser measurement and what we now have is a more accurate beliefs about the pedestrian’s position at time k+1, this is what we called posterior.

Now we received the radar measurement (Polar coordinate system) at time k+2 and we make again a prediction and then combine the prediction with the radar measurement (the prediction for the radar is the same as in the laser case but what change is the measurement update step because the radar sees the world differently).

<p align="center">
<img src="./img/48.JPG" alt="Workflow of the extended kalman filter works." />
<p align="center">

Notice: Because of two different worlds (Cartesian and Polar) we have to use two different measurement steps. If both the radar and laser measurements arrive at the same time, it uses one of the sensors then another sensor to prediction and measurement step.

## Evaluating Kalman Filter Performanc:

Once we have implemented our tracking Kalman filter algorithm, we might want to check its performance in terms how far the estimated result is from the true result, there are many evaluation metrics, but the most common one is what called Root Mean Squared Error.
In the context of the tracking it is an accuracy metric used to measure deviation of the estimated state of the true state.
At a given processing step, we need two values:

*	The estimated values which is a vector with position and velocity components
*	The real values that are usually known as ground truth.


The difference between ground truths and estimated values called residual, this residual are squared and averaged, and finally the square root gives us the error metric. The lower is the error the higher is the estimation accuracy.


<p align="center">
<img src="./img/49.JPG" alt="Root Mean Squared Error" />
<p align="center">


## Implementation of the Extended Kalman Filter for Sensor Fusion:

Now that we have learned how the extended Kalman filter works, in this section we are going to implement the extended Kalman filter in C++. We are providing simulated lidar and radar measurements detecting a bicycle that travels around your vehicle. You will use a Kalman filter, lidar measurements and radar measurements to track the bicycle's position and velocity like below:

<p align="center">
<img src="./img/50.JPG" alt="Implementation of the Extended Kalman Filter for Sensor Fusion" />
<p align="center">

### 1.	Explanation of the Data File

Lidar measurements are red circles, radar measurements are blue circles with an arrow pointing in the direction of the observed angle, and estimation markers are green triangles. The simulator will be using this data file, and feed main.cpp values from it one line at a time.
	
<p align="right">
<img src="./img/51.JPG" alt="Data File" />
<p align="right">
	
*	Each row represents a sensor measurement where the first column tells you if the measurement comes from radar (R) or lidar (L).

*	For a row containing radar data, the columns are: sensor_type, rho_measured, phi_measured, rhodot_measured, timestamp, x_groundtruth, y_groundtruth, vx_groundtruth, vy_groundtruth, yaw_groundtruth, yawrate_groundtruth.

*	For a row containing lidar data, the columns are: sensor_type, x_measured, y_measured, timestamp, x_groundtruth, y_groundtruth, vx_groundtruth, vy_groundtruth, yaw_groundtruth, yawrate_groundtruth.

*	Whereas radar has three measurements (rho, phi, rhodot), lidar has two measurements (x, y).

*	You will use the measurement values and timestamp in your Kalman filter algorithm. Ground truth, which represents the actual path the bicycle took, is for calculating root mean squared error.


### 2.	Reading in the Data

*	It provided code that will read in and parse the data files for you. This code is in the main.cpp file. The main.cpp file creates instances of a MeasurementPackage.

*	If you look inside 'main.cpp', you will see code like:

<p align="right">
<img src="./img/52.JPG" alt="'main.cpp'" />
<p align="right">
	
*	The ground truth [px, py,vx,vy] for each line in the data file gets pushed onto ground_truth so RMSE can be calculated later from tools.cpp.

### 3.	File Structure

To review what we learned in the extended Kalman filter lectures, let's discuss the three main steps for programming a Kalman filter:

*	initializing Kalman filter variables
*	predicting where our object is going to be after a time step Δt
*	updating where our object is based on sensor measurements

Then the prediction and update steps repeat themselves in a loop. To measure how well our Kalman filter performs, we will then calculate root mean squared error comparing the Kalman filter results with the provided ground truth. These three steps (initialize, predict, update) plus calculating RMSE encapsulate the entire extended Kalman filter project.

Files in the Github src Folder, the files you need to work with are in the src folder of the github repository:
*	main.cpp - communicates with the Simulator receiving data measurements, calls a function to run the Kalman filter, calls a function to calculate RMSE
*	FusionEKF.cpp - initializes the filter, calls the predict function, calls the update function
*	kalman_filter.cpp- defines the predict function, the update function for lidar, and the update function for radar
*	tools.cpp- function to calculate RMSE and the Jacobian matrix

### 4.	How the Files Relate to Each Other:
Here is a brief overview of what happens when you run the code files:

*	Main.cpp reads in the data and sends a sensor measurement to FusionEKF.cpp

*	FusionEKF.cpp takes the sensor data, initializes variables, and updates variables. The Kalman filter equations are not in this file. FusionEKF.cpp has a variable called ekf_, which is an instance of a KalmanFilter class. The ekf_ will hold the matrix and vector values. You will also use the ekf_ instance to call the predict and update equations. 

*	The KalmanFilter class is defined in kalman_filter.cpp (contains functions for the prediction and update steps) and kalman_filter.h.


### 5.	Main.cpp

We already discussed how main.cpp reads in the sensor data. Recall that main.cpp reads in the sensor data line by line from the client and stores the data into a measurement object that it passes to the Kalman filter for processing. Also a ground truth list and an estimation list are used for tracking RMSE.

*	main.cpp is made up of several functions within main(), these all handle the uWebsocketIO communication between the simulator and itself and all the main code loops in h.onMessage(), to have access to initial variables that we created at the beginning of main(), we pass pointers as arguments into the header of h.onMessage(). 

<p align="right">
<img src="./img/53.JPG" alt="'main.cpp'" />
<p align="right">

*	The rest of the arguments in h.onMessage are used to set up the server.
*	The below code is:

	*	creating an instance of the FusionEKF class
	*	Receiving the measurement data calling the ProcessMeasurement() function. ProcessMeasurement() is responsible for the initialization of the Kalman filter as well as calling the prediction and update steps of the Kalman filter. The ProcessMeasurement() function is built in FusionEKF.cpp


<p align="right">
<img src="./img/54.JPG" alt="'FusionEKF.cpp" />
<p align="right">

*	Finally, The rest of main.cpp will output the following results to the simulator:
	*	estimation position
	*	Calculated RMSE (RMSE function is implemented in the tools.cpp file.)
	
### 6.	FusionEKF.cpp
Every time main.cpp calls fusionEKF.ProcessMeasurement(measurement_pack_list[k]),the code in FusionEKF.cpp will run. - If this is the first measurement, the Kalman filter will try to initialize the object's location with the sensor measurement.

*	Variables and matrices (x, F, H_laser, H_jacobian, P, etc.) will be Initialized.
*	The Kalman filter position vector with the first sensor measurements will be initialized.
*	The F and Q matrices prior to the prediction step based on the elapsed time between measurements will be calculated.
*	The update step for either the lidar or the radar sensor measurement will be called. 

### 7.	Important Dependencies

*	cmake >= 3.5 
	*	All OSes: click here for installation instructions (https://cmake.org/install/)
*	make >= 4.1 
	*	Linux: make is installed by default on most Linux distros
	*	Mac: install Xcode command line tools to get make (https://developer.apple.com/xcode/features/)
	*	Windows: Click here for installation instructions (http://gnuwin32.sourceforge.net/packages/make.htm)
*	gcc/g++ >= 5.4 
	*	Linux: gcc / g++ is installed by default on most Linux distros
	*	Mac: same deal as make - install Xcode command line tools (https://developer.apple.com/xcode/features/)
	*	Windows: recommend using MinGW (http://www.mingw.org/)



### 8.	Build Instructions
*	First, clone this repo.
*	This project can be used with a Simulator, which can be downloaded here(https://github.com/udacity/self-driving-car-sim/releases)
*	In order to use the simulator, we need to install uWebSocketIO so that it can communicate with the C++ program--The simulator is the client, and the C++ program is the web server. To install it download uWebSocketIO(https://github.com/uNetworking/uWebSockets) then:
	*	chmod +x install-mac.sh
	*	./install-mac.sh

*	Once the install for uWebSocketIO is complete, the main program can be built--with the completed code--and run by doing the following from the project top directory.
	*	Make the build directory: mkdir build && cd build
	*	Compile: cmake .. && make
	*	Run it: ./ExtendedKF path/to/input.txt
*	e.g: ./ExtendedKF ../data/sample-laser-radar-measurement-data-1.txt
*	This repository includes two files that can be used to set up and install uWebSocketIO for either Linux or Mac systems. For windows you can use either Docker, VMware, or even Windows 10 Bash on Ubuntu to install uWebSocketIO.

	

##	Result

<p align="center">
<img src="./img/44.gif" alt="'main.cpp'" />
<p align="center">

@Amir Ziaee
Deep learing -Machine Learnig - Web communication information systems-Computer Vision- Self Driving Car
