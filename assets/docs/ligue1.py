import numpy as np
import pandas as pd
from sklearn.datasets import make_regression
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from sklearn.preprocessing import OneHotEncoder, LabelEncoder
from sklearn.model_selection import train_test_split
from sklearn.linear_model import LogisticRegression
from sklearn.metrics import classification_report, accuracy_score
from sklearn.metrics import ConfusionMatrixDisplay



league_data = pd.read_csv("l1c.csv")

#print(league_data.head())

features = ["Home Team","Away Team", "Home Team Goals","Away Team Goals"]
target = "Winner"

league_data = league_data[features + [target]].dropna()

X = league_data[features]
y = league_data[target]


#print (league_data.dtypes)

X_cat = X[["Home Team", "Away Team"]] #text values
X_num = X[["Home Team Goals", "Away Team Goals"]] #numeric values

X_cat_oh = pd.get_dummies(X_cat, prefix=["Home", "Away"]) #Text values must be converted into numeric values for matrix calculcations
X_final = pd.concat([X_cat_oh, X_num], axis=1)

print("X_cat_oh shape :", X_cat_oh.shape)
print("X_final shape  :", X_final.shape)
print("Columns overview :", X_final.columns[:10])

label_encoder = LabelEncoder()
y_encoded = label_encoder.fit_transform(y)
print(label_encoder.classes_)

X_train,X_test,y_train,y_test = train_test_split(X_final,y_encoded,test_size=0.2,random_state=1)

print("Size of the training set :", X_train.shape)
print("Size of the test set :", X_test.shape)

best_accuracy = 0
best_C = None
best_model = None


for C in [0.01, 0.1, 1, 10, 100]: #finding the best C = 1/λ in the L2 penalty term among a set of values
    model = LogisticRegression(C=C, penalty="l2", solver="lbfgs", max_iter=1000)
    model.fit(X_train, y_train)
    acc = model.score(X_test,y_test)
    print(f"C={C:<5} → Accuracy: {model.score(X_test, y_test):.4f}")

    if acc > best_accuracy :
        best_accuracy = acc
        best_C = C
        best_model = model
        
y_pred = best_model.predict(X_test)

print("Accuracy :", accuracy_score(y_test, y_pred))
print("\nClassification report :\n", classification_report(y_test, y_pred))

