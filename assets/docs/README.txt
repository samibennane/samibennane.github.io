RNN vs CNN for Image Classification
📌 Project Overview

This project explores and compares Recurrent Neural Networks (RNNs) and Convolutional Neural Networks (CNNs) for image classification tasks.
Although CNNs are the standard approach for image processing, this work investigates whether RNNs can also be applied to image data and how their performance compares in practice.

The goal is to highlight the strengths and limitations of each architecture when dealing with visual information.

🎯 Objectives

Implement an RNN-based model and a CNN-based model for image classification

Train both models on the same dataset under comparable conditions

Evaluate and compare:

Classification accuracy

Training and validation behavior

Computational efficiency

Understand why CNNs are generally better suited for image tasks

🧠 Models Used
1. Recurrent Neural Network (RNN)

Images are reshaped into sequences (e.g., rows or columns of pixels)

The RNN processes the image sequentially

This approach allows testing RNNs on image data, although they are not designed for spatial feature extraction

Limitations:

Loss of spatial locality

Long training times

Difficulty capturing complex visual patterns

2. Convolutional Neural Network (CNN)

Uses convolutional layers to automatically extract spatial features

Preserves local patterns such as edges, textures, and shapes

Pooling layers reduce dimensionality and improve generalization

Advantages:

Strong inductive bias for images

Better accuracy and faster convergence

More robust to variations in position and scale