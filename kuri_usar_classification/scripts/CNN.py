
"""
Created on Thu July 11 2017
@author: Reem Ashour 
"""
#KERAS
from keras.models import Sequential
from keras.layers.core import Dense, Dropout, Activation, Flatten
from keras.layers.convolutional import Convolution2D, MaxPooling2D
from keras.optimizers import SGD,RMSprop,adam
from keras.utils import np_utils

from keras import backend as K
from keras.utils.layer_utils import convert_all_kernels_in_model
from keras.utils import np_utils
from keras.optimizers import SGD

#sklearn
from sklearn.utils import shuffle
from sklearn.cross_validation import train_test_split
from sklearn.metrics import confusion_matrix, precision_score, recall_score, f1_score


# Import libraries
import os,cv2
import sys
import numpy as np
import matplotlib.pyplot as plt
import matplotlib
from PIL import Image

K.set_image_dim_ordering('tf')

#important Params 
img_rows, img_cols = 200, 200  # input image dimensions #460, 480
img_channels = 1 # number of channels (Gray Scale)
num_epoch=5      # Training epoch
batch_size = 100  # batch_size to train
nb_pool = 2       # size of pooling area for max pooling
stride = 2        # ?????? 
num_classes = 2	  # Define the number of classes
nb_conv = 3	  # convolution kernel size


# Define data path
# get the current directory 
PATH_1 = os.getcwd()
data_path = PATH_1 + '/dataSet3'
# get the list of all the fiels and directories inside the path 
data_list = os.listdir(data_path) 
num_samples=len(data_list)
print num_samples 

# use the following code to generate new dataSet with resized Images 
'''
PATH_2=os.getcwd()
data_path_2 = PATH_2 + '/dataSet3_resized'
# Done once 
index = 0 
for file in data_list:
    im = Image.open(data_path + '/' + file)  
    img = im.resize((img_rows,img_cols))
    gray = img.convert('L')
    #need to do some more processing here  
    if index<=1235: 	        
    	gray.save(data_path_2 +'/' +  "Forward_" + str(index) , "JPEG")
    else: 
	gray.save(data_path_2 +'/' +  "Not_" + str(index) , "JPEG")
    index+=1
'''

# Resize the list of images 
img_data_list=[]
for img in data_list:
	input_img=cv2.imread(data_path +'/'+ img )
	input_img=cv2.cvtColor(input_img, cv2.COLOR_BGR2GRAY)
	input_img_resize=cv2.resize(input_img,(img_rows,img_cols))
	img_data_list.append(input_img_resize)
img_data = np.array(img_data_list)
img_data = img_data.astype('float32') #changed the type of data 
img_data /= 255 # normalization 

print (img_data.shape)

img_data= np.expand_dims(img_data, axis=4) 
print (img_data.shape)


# Assigning Labels to the Images 
num_of_samples = img_data.shape[0]
labels = np.ones((num_of_samples,),dtype='int64')

# Two Class Problem 
labels[0:1234]=0 # Forward 
labels[1235:]=1  # Not forward 
names = ['Forward','NOT_F']


# Three Class Problem 
#labels[0:1234]=0     # Forward 
#labels[1235:1537]=1  # LEFT 
#labels[1537:]=1      # Right 


# convert class labels to on-hot encoding
Y = np_utils.to_categorical(labels, num_classes)

#Shuffle the dataset
x,y = shuffle(img_data,Y, random_state=2)

# Split the dataset
X_train, X_test, y_train, y_test = train_test_split(x, y, test_size=0.2, random_state=2)
input_shape=img_data[0].shape


print "############################# Start the Model ###########################" 
# Defining the model			
model = Sequential()
model.add(Convolution2D(32, nb_conv,nb_conv,border_mode='same',input_shape=input_shape,name='conv1'))
model.add(Activation('relu'))
model.add(Convolution2D(32, nb_conv, nb_conv))
model.add(Activation('relu'))
model.add(MaxPooling2D(pool_size=(2, 2)))
model.add(Dropout(0.5))

model.add(Convolution2D(64, nb_conv, nb_conv))
model.add(Activation('relu'))
#model.add(Convolution2D(64, 3, 3))
#model.add(Activation('relu'))
model.add(MaxPooling2D(pool_size=(2, 2)))
model.add(Dropout(0.5))

model.add(Flatten())
model.add(Dense(64))
model.add(Activation('relu'))
model.add(Dropout(0.5))
model.add(Dense(num_classes))
model.add(Activation('softmax'))


print "############################# Compile the Model ###########################" 
#sgd = SGD(lr=0.01, decay=1e-6, momentum=0.9, nesterov=True)
#model.compile(loss='categorical_crossentropy', optimizer=sgd,metrics=["accuracy"])
model.compile(loss='categorical_crossentropy', optimizer='rmsprop',metrics=["accuracy"])
#model.compile(loss='categorical_crossentropy', optimizer='adadelta', metrics=['accuracy'])
# Viewing model_configuration

model.summary()
model.get_config()
model.layers[0].get_config()
model.layers[0].input_shape			
model.layers[0].output_shape			
model.layers[0].get_weights()
np.shape(model.layers[0].get_weights()[0])
model.layers[0].trainable

print "############################# Training  the Model ###########################" 
#%%
# Training
#print "X_Train" , X_train
#print "Y_Train" , y_train 
history = model.fit(X_train, y_train, batch_size=10, epochs=5, verbose=1,validation_split=0.2)



# list all data in history
print(history.history.keys())
# visualizing losses and accuracy
xc=range(num_epoch)
# summarize history for accuracy
plt.figure(1,figsize=(7,5))
plt.plot(history.history['acc'])
plt.plot(history.history['val_acc'])
plt.title('model accuracy')
plt.ylabel('accuracy')
plt.xlabel('num of Epochs')
plt.grid(True)
plt.legend(['train', 'test'], loc='upper left')
plt.show()

# summarize history for loss
plt.figure(2,figsize=(7,5))
plt.plot(history.history['loss'])
plt.plot(history.history['val_loss'])
plt.title('model loss')
plt.ylabel('loss')
plt.xlabel('num of Epochs')
plt.grid(True)
plt.legend(['train', 'test'], loc='upper left')
plt.show() 

print "############################# evaluating the Model ###########################" 
# Evaluating the model
score = model.evaluate(X_test, y_test, verbose=0)
print('Test Loss:', score[0])
print('Test accuracy:', score[1])

print "############################# Predict ###########################" 
print "1" 
print (model.predict_classes(X_test[1:5]))
print "2"
print (y_test[1:5])

test_image = X_test[0:1]
print "3" , (test_image.shape)
print "4" , (model.predict(test_image))
print "5" , (model.predict_classes(test_image))
print "6" , (y_test[0:1])


print "############################# Save the Model ###########################" 

model.save('model.hdf5')
loaded_model=load_model('model.hdf5')

