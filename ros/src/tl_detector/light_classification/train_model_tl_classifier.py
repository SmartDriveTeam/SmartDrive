import csv
import cv2
import numpy as np
from glob import glob
import os
from keras.models import Sequential
from keras.layers import Flatten, Lambda,  Dense, Dropout, Activation, ELU, BatchNormalization
from keras.layers import Dropout, Conv2D, Convolution2D, MaxPooling2D, Cropping2D
from keras.callbacks import TensorBoard
from keras.models import load_model

# Traffic Light    RED    YELLOW    GREEN
tl_categories = [[1,0,0], [0,1,0], [0,0,1]]

images = []
labels = []

# Training Data
for i in range(3):
    paths = glob(os.path.join('./image/train/' + str(i) + '/'+'*.jpg')) # train data path
    for path in paths:
        img = cv2.imread(path)
        images.append(img)
        labels.append(tl_categories[i])

images = np.array(images)
labels = np.array(labels)

print("NUMBER OF Train Image: ", len(images))
print("Image shape:", images.shape)
print("Label shape:", labels.shape)

# Traffic Light Classifier Model
# Modify LeNet-5
model = Sequential()
model.add( Lambda(lambda x: x/255.0 - 0.5, input_shape=(32,32,3)))

model.add(Conv2D(32, kernel_size=(5, 5), strides=(1, 1), activation='relu', input_shape=(32,32,3)))
#model.add(BatchNormalization())
model.add(MaxPooling2D(pool_size=(2, 2), strides=(2, 2)))

model.add(Conv2D(64, (5, 5), activation='relu'))
#model.add(BatchNormalization())
model.add(MaxPooling2D(pool_size=(2, 2)))

model.add(Flatten())
model.add(Dense(120))
model.add(Activation("relu"))
#model.add(BatchNormalization())
#model.add(Dropout(0.5))

model.add(Dense(60))
model.add(Activation("relu"))
#model.add(BatchNormalization())
#model.add(Dropout(0.5))

model.add(Dense(3))
model.add(Activation("softmax")) #softmax classifier

model.compile(loss='categorical_crossentropy', optimizer='adam', metrics=['accuracy'])

# Train Validation 20%, shuffle
model.fit(images, labels, epochs=100, validation_split=0.2, shuffle=True)

# Train Model Save
model.save('./train_log/model_smartdrive_04262110.h5')
