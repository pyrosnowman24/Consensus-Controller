import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import os
import scipy.stats
import math

def resize_array(array,size):
    diff = size - len(array)
    if diff > 0:
        padding = np.zeros(diff)
        padding[:] = np.nan
        return np.append(array,padding)
    else :return array
# Test 1
# test1_cloud_model = pd.read_csv("~/catkin_ws/src/sensor_dist_ros/results/test1/cloud_model_test1.csv",header = None)
# test1_cloud_position = pd.read_csv("~/catkin_ws/src/sensor_dist_ros/results/test1/cloud_pos_test1.csv",header = None)
# test1_end_position = pd.read_csv("~/catkin_ws/src/sensor_dist_ros/results/test1/end_test1.csv",header = None)

test1_cloud_model = pd.read_csv("~/catkin_ws/src/sensor_dist_ros/results/test8/cloud_modeling_runtime.csv",header = None)
test1_cloud_position = pd.read_csv("~/catkin_ws/src/sensor_dist_ros/results/test8/cloud_pos_runtime.csv",header = None)
test1_end_position = pd.read_csv("~/catkin_ws/src/sensor_dist_ros/results/test6/end_runtime.csv",header = None)

test1_cloud_model_ = test1_cloud_model.iloc[:].values
test1_cloud_position_ = test1_cloud_position.iloc[:].values
test1_end_position_ = test1_end_position.iloc[:].values

# Test 2
test2_cloud_model = pd.read_csv("~/catkin_ws/src/sensor_dist_ros/results/test2/cloud_modeling_runtime.csv",header = None)
test2_cloud_position = pd.read_csv("~/catkin_ws/src/sensor_dist_ros/results/test2/cloud_pos_runtime.csv",header = None)
test2_end_position = pd.read_csv("~/catkin_ws/src/sensor_dist_ros/results/test2/end_runtime.csv",header = None)
test2_edge_model = pd.read_csv("~/catkin_ws/src/sensor_dist_ros/results/test2/edge_runtime.csv",header = None)

test2_cloud_model_ = test2_cloud_model.iloc[:].values
test2_cloud_position_ = test2_cloud_position.iloc[:].values
test2_end_position_ = test2_end_position.iloc[:].values
test2_edge_model_ = test2_edge_model.iloc[:].values

# Test 3
test3_cloud_model = pd.read_csv("~/catkin_ws/src/sensor_dist_ros/results/test7/cloud_modeling_runtime.csv",header = None)
test3_cloud_position = pd.read_csv("~/catkin_ws/src/sensor_dist_ros/results/test7/cloud_pos_runtime.csv",header = None)
test3_end_position = pd.read_csv("~/catkin_ws/src/sensor_dist_ros/results/test7/speed_pos_runtime.csv",header = None)
test3_edge_model = pd.read_csv("~/catkin_ws/src/sensor_dist_ros/results/test7/speed_modeling_runtime.csv",header = None)

test3_cloud_model_ = test3_cloud_model.iloc[:].values
test3_cloud_position_ = test3_cloud_position.iloc[:].values
test3_end_position_ = test3_end_position.iloc[:].values
test3_edge_model_ = test3_edge_model.iloc[:].values

# Used to size up arrays to longest if you want to use all data
# max1 = max(test1_cloud_model_.shape[0],test2_cloud_model_.shape[0],test3_cloud_model_.shape[0])
# resized_test1_cloud_model_ = resize_array(test1_cloud_model_,max1)
# resized_test1_cloud_model_ = np.reshape(resized_test1_cloud_model_,(len(resized_test1_cloud_model_),1))
# resized_test2_cloud_model_ = resize_array(test2_cloud_model_,max1)
# resized_test2_cloud_model_ = np.reshape(resized_test2_cloud_model_,(len(resized_test2_cloud_model_),1))
# resized_test3_cloud_model_ = resize_array(test3_cloud_model_,max1)
# resized_test3_cloud_model_ = np.reshape(resized_test3_cloud_model_,(len(resized_test3_cloud_model_),1))
#
# max2 = max(test1_end_position_.shape[0],test2_end_position_.shape[0],test3_end_position_.shape[0])
# resized_test1_end_position_ = resize_array(test1_end_position_,max2)
# resized_test1_end_position_ = np.reshape(resized_test1_end_position_,(len(resized_test1_end_position_),1))
# resized_test2_end_position_ = resize_array(test2_end_position_,max2)
# resized_test2_end_position_ = np.reshape(resized_test2_end_position_,(len(resized_test2_end_position_),1))
# resized_test3_end_position_ = resize_array(test3_end_position_,max2)
# resized_test3_end_position_ = np.reshape(resized_test3_end_position_,(len(resized_test3_end_position_),1))
#
# max3 = max(test2_edge_model_.shape[0],test3_edge_model_.shape[0])
# resized_test2_edge_model_ = resize_array(test2_edge_model_,max3)
# resized_test2_edge_model_ = np.reshape(resized_test2_edge_model_,(len(resized_test2_edge_model_),1))
# resized_test3_edge_model_ = resize_array(test3_edge_model_,max3)
# resized_test3_edge_model_ = np.reshape(resized_test3_edge_model_,(len(resized_test3_edge_model_),1))
#
# cloud_model_concat = np.concatenate((resized_test1_cloud_model_,resized_test2_cloud_model_,resized_test3_cloud_model_),axis=1)
# end_position_concat = np.concatenate((resized_test1_end_position_,resized_test2_end_position_,resized_test3_end_position_),axis=1)
# edge_model_concat = np.concatenate((resized_test2_edge_model_,resized_test3_edge_model_),axis=1)

# Use smallest size to determine number of elements
# min1 = min(test1_cloud_model_.shape[0],test2_cloud_model_.shape[0],test3_cloud_model_.shape[0])
# cloud_model_concat = np.concatenate((test1_cloud_model_[:min1],test2_cloud_model_[:min1],test3_cloud_model_[:min1]),axis=1)
# min2 = min(test1_end_position_.shape[0],test2_end_position_.shape[0],test3_end_position_.shape[0])
# end_position_concat = np.concatenate((test1_end_position_[:min2],test2_end_position_[:min2],test3_end_position_[:min2]),axis=1)
# min3 = min(test2_edge_model_.shape[0],test3_edge_model_.shape[0])
# edge_model_concat = np.concatenate((test2_edge_model_[:min3],test3_edge_model_[:min3]),axis=1)

cloud_model_concat = [test2_cloud_model_.flatten(),test3_cloud_model_.flatten()]
end_position_concat = [test2_end_position_,test3_end_position_]
edge_model_concat = [test1_cloud_model_,test2_edge_model_,test3_edge_model_]


# Plots Cloud_Positions
fig,ax = plt.subplots(3,2)
ax[0,0].plot(test1_cloud_position_,label='Control')
ax[1,1].plot(test2_cloud_position_[:22],label='Lambda',color="#ff7f0e")
ax[2,1].plot(test3_cloud_position_[:22],label='Kappa',color="#2ca02c")

ax[0,1].axis('off')
ax[1,0].plot(test2_end_position_,label='Lambda',color="#ff7f0e")
ax[2,0].plot(test3_end_position_,label='Kappa',color="#2ca02c")
# ax[0].set_title("Runtime for each Iteration for Batch Layer Desired Position Algorithm")
ax[2,1].set_xlabel("Iteration")
ax[2,0].set_xlabel("Iteration")
ax[1,0].set_ylabel("Runtime (s)")
ax[1,1].set_title("Runtimes for Batch Desired Position")
ax[0,0].set_title("Runtimes for Speed Desired Position")
fig.legend(["Control","Lambda","Kappa"],loc = 1,bbox_to_anchor=(-0.15, -0.15, 1, 1))
plt.show()

# Plots Histograms
fig,ax = plt.subplots(2)
plot_labels = ["Control","Lambda","Kappa"]
# ax[0].hist(cloud_model_concat,100,density=True,histtype='bar',label=plot_labels)
# ax[1].hist(end_position_concat,100,density=True,histtype='bar',label=plot_labels)
plot_labels2 = ["Lambda","Kappa"]
# colors2 = ["#ff7f0e","#2ca02c"]
# ax[2].hist(edge_model_concat, 200, density=True, histtype='bar',color = colors2, label=plot_labels2)

ax[0].boxplot(cloud_model_concat,labels=plot_labels2)
ax[1].boxplot(edge_model_concat,labels=plot_labels)

ax[0].set_title("Runtimes for Batch Modeling")
ax[1].set_title("Runtimes for Speed Modeling")
ax[0].set_ylabel("Runtime (s)")
ax[1].set_ylabel("Runtime (s)")
plt.show()
