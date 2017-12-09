#**Behavioral Cloning** 

---

**Behavioral Cloning Project**

The goals / steps of this project are the following:

* Use the simulator to collect data of good driving behavior
* Build, a convolution neural network in Keras that predicts steering angles from images
* Train and validate the model with a training and validation set
* Test that the model successfully drives around track one without leaving the road
* Summarize the results with a written report


[//]: # (Image References)

[image1]: ./examples/center.jpg "Center line driving forward"
[image2]: ./examples/r1.jpg "Recovering from left edge"
[image3]: ./examples/r2.jpg "Recovering from left edge"
[image4]: ./examples/r3.jpg "Recovering from left edge"
[image5]: ./examples/r4.jpg "Recovering from left edge"
[image6]: ./examples/r5.jpg "Recovering from left edge"
[image7]: ./examples/r6.jpg "Recovering from left edge"


## Rubric Points
###Here I will consider the [rubric points](https://review.udacity.com/#!/rubrics/432/view) individually and describe how I addressed each point in my implementation.  


---
###Files Submitted & Code Quality

####1. Submission includes all required files and can be used to run the simulator in autonomous mode

My project includes the following files:

* model.py containing the script to create and train the model
* drive.py for driving the car in autonomous mode
* model.h5 containing a trained convolution neural network 
* writeup_report.md summarizing the results

####2. Submission includes functional code
Using the Udacity provided simulator and my drive.py file, the car can be driven autonomously around the track by executing: 
```sh
python drive.py model.h5
```

####3. Submission code is usable and readable

The model.py file contains the code for training and saving the convolution neural network. The file shows the pipeline I used for training and validating the model, and it contains comments to explain how the code works.


###Model Architecture and Training Strategy

####1. An appropriate model architecture has been employed

My model consists of a convolution neural network with 5x5 and 3x3 filter sizes and depth 2 (model.py lines 78-98). 

The model includes RELU activation layers to introduce nonlinearity (code lines 88, 89), and the data is normalized in the model using a Keras lambda layer (code line 81). 

####2. Attempts to reduce overfitting in the model

The model contains dropout layer in order to reduce overfitting (model.py lines 90). 

The model was trained and validated on different data sets to ensure that the model was not overfitting (code line 106-115). The model was tested by running it through the simulator and ensuring that the vehicle could stay on the track.

####3. Model parameter tuning

The model used an adam optimizer, so the learning rate was not tuned manually (model.py line 96).

####4. Appropriate training data

Training data was chosen to keep the vehicle driving on the road. I collected the data using center, left and right camera when driving both forward and backward the track. For details about how I created the training data, see the next section.


###Model Architecture and Training Strategy

####1. Solution Design Approach

The overall strategy for deriving a model architecture was to start with a model similar to NVIDIA
model but simplified due to the fact that the simulator environment is less complicated than a real
road for which NVIDIA model was build. So my initial model had a few convolutional layers followed by flatenning layers and fully connected ones. Gradually I was fine tuning parameters such as size of the kernels in convolutional layers and size of the strides based on how well the car drives in autonomous mode. During that process I was also reducing number of convolutional and dense layers if it wasn't worsening the autonomous drive. To address the overfitting I included also a dropout layer and I split the data in training and validation set. At the end of the fine tuning process, the vehicle is able to drive autonomously around the track without leaving the road.

####2. Final Model Architecture

The final model architecture (model.py lines 77-98) consisted of a convolution neural network with the following layers and layer output sizes :


    Layer (type)                     Output Shape       
    ====================================================
    lambda_1 (Lambda)                (None, 80, 160, 1) 
    ____________________________________________________
    cropping2d_1 (Cropping2D)        (None, 45, 160, 1) 
    ____________________________________________________
    convolution2d_1 (Convolution2D)  (None, 11, 39, 2)  
    ____________________________________________________
    convolution2d_2 (Convolution2D)  (None, 3, 13, 2)   
    ____________________________________________________
    dropout_1 (Dropout)              (None, 3, 13, 2)   
    ____________________________________________________
    flatten_1 (Flatten)              (None, 78)         
    ____________________________________________________
    dense_1 (Dense)                  (None, 1)          
    ====================================================
    Total params: 169
    Trainable params: 169
    Non-trainable params: 0
    
The first layer, lambda layer, is doing normalization.
The second layer, cropping layer, removes top 25 and bottom 10 rows of pixels since
there is nothing important there.

####3. Creation of the Training Set & Training Process

To capture good driving behavior, I first recorded 5 laps of center line driving forward the track:

![alt text][image1]


I then recorded 5 laps of center line driving backward the track.


I then recorded the vehicle recovering from the left side and right sides of the road back to center so that the vehicle would learn to drive to the road center once it starts drifting off the road center. Following images show what a recovery looks like. Starting from left edge the car drives toward the center :

![alt text][image2]
![alt text][image3]
![alt text][image4]
![alt text][image5]
![alt text][image6]
![alt text][image7]

Then I repeated this recovery collecting process driving backwards the track.


After the collection process, I had 47,667 number of images. I then preprocessed this data by 
resizing them by a factor of 0.5. Then I converted the image to HSV format and to reduce the size of input data to the model I also decided to keep only the S channel of the image. Then to double the number of data I augmented it by flipping each of them and its corresponding steering. Then the steering was appropriately corrected for left and right images (model.py line 45).


I finally randomly shuffled the data set and put 10% of the data into a validation set. 

I used this training data for training the model. The validation set helped determine if the model was over or under fitting. The ideal number of epochs was 3 (for more epcochs loss values was not decreasing significantly). I used an adam optimizer so that manually training the learning rate wasn't necessary.
