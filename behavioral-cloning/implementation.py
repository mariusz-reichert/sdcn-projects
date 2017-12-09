from keras.models import Sequential
from keras.layers.core import Dense, Flatten, Dropout
from keras.layers.convolutional import Convolution2D
from keras.layers import Cropping2D, Lambda
from keras.preprocessing.image import load_img, img_to_array 
from sklearn.utils import shuffle
from sklearn.model_selection import train_test_split
import numpy as np
import csv
from keras import backend as K
from enum import IntEnum


HEIGHT = 80
WIDTH = 160
CORRECTION = 0.3
EPOCHS = 3
DATA_PATH = '../resources/data/'
IMGS_PATH = DATA_PATH + 'IMG/'
BATCH_SIZE = 32


class DataIdx(IntEnum):
    CenterImg = 0  
    LeftImg = 1
    RightImg = 2
    Steering = 3


def load_samples(path):
    samples = []

    with open(path+'driving_log.csv') as csvfile:
        reader = csv.reader(csvfile)
        for line in reader:
            samples.append(line)    

    return shuffle(samples)


def filename(path):
    return path.split('/')[-1]


def preprocess_data(sample, dataIdx, correction, images, steerings, size=(HEIGHT,WIDTH)):
    # resize the image, convert to HSV and extract only S channel
    _, img, _, = load_img(IMGS_PATH+filename(sample[dataIdx]), target_size=size).convert('HSV').split()
    
    # add image and its flipped version
    images.append(img_to_array(img))
    images.append(img_to_array(np.fliplr(img)))

    # add steering and its corrected value corresponding to flipped image
    steering = float(sample[DataIdx.Steering])
    steerings.append(steering)
    steerings.append(-(steering+correction))


def generator(samples, batch_size):
    num_of_samples = len(samples)
    while True:
        shuffle(samples)
        for offset in range(0, num_of_samples, batch_size):
            batch_samples = samples[offset:offset+batch_size]

            images = []
            steerings = []

            for sample in batch_samples:
                preprocess_data(sample, DataIdx.CenterImg, 0.0, images, steerings)
                preprocess_data(sample, DataIdx.LeftImg, CORRECTION, images, steerings)
                preprocess_data(sample, DataIdx.RightImg, -CORRECTION, images, steerings)

            yield shuffle(np.asarray(images), np.asarray(steerings))


def build_model():
    model = Sequential()

    # normalize the image
    model.add(Lambda(lambda x: x/127.5 - 1.,
        input_shape=(HEIGHT, WIDTH, 1),
        output_shape=(HEIGHT, WIDTH, 1)))

    # chop off 25 pixels from the top and 10 pixels from the bottom of the image
    model.add(Cropping2D(cropping=((25,10), (0,0))))

    model.add(Convolution2D(2, 5, 5, subsample=(4,4), activation='relu', border_mode='valid'))
    model.add(Convolution2D(2, 3, 3, subsample=(3,3), activation='relu', border_mode='valid'))
    model.add(Dropout(0.2))
    model.add(Flatten())
    model.add(Dense(1))

    model.summary()

    model.compile(optimizer='adam', loss='mse')

    return model


def main():
    # force tensor flow ordering of dimensions even with Theano backend (which by default uses 'th')
    K.set_image_dim_ordering('tf')

    # load, split just images paths, images will be loaded on the fly
    samples = load_samples(DATA_PATH)
    train_samples, validation_samples = train_test_split(samples, train_size=0.9, random_state=0)
    
    train_generator = generator(train_samples, BATCH_SIZE)
    validation_generator = generator(validation_samples, BATCH_SIZE)
   
    model = build_model()
    model.compile(loss='mse', optimizer='adam')
    
    model.fit_generator(train_generator, 
        samples_per_epoch=len(train_samples)*6, 
        validation_data=validation_generator, 
        nb_val_samples=len(validation_samples), 
        nb_epoch=EPOCHS)
    model.save('model.h5')

if __name__ == '__main__':
    main()

