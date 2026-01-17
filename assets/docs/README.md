# French Ligue 1 Match Predictions

**Predicting football match outcomes in the French Ligue 1 using multinomial logistic regression and data preprocessing with Python (pandas, NumPy, scikit-learn).**


## Project Overview

This project was designed with a mathematical and conceptual objective rather than a performance-driven one. Instead of using a standard academic dataset such as Iris, I chose a real-world dataset based on French football match predictions in order to verify that the same fundamental learning principles apply in a less idealized setting.

The dataset is intentionally limited in size and in the number of features, which allows a clear analysis of:

loss minimization,

decision boundaries,

generalization,

and model behavior.

The goal of this project is to demonstrate that machine learning relies on the same mathematical foundations regardless of the application domain, and that these foundations remain observable even on imperfect real-world data.

The **mathematical formulation**, including the derivation of the gradient, cost function, and softmax activation, is fully detailed in the accompanying report:

📄 **french_league_predictions_sami_bennane.pdf**


## Technologies Used

| Category | Tools / Libraries |
|-----------|------------------|
| Programming | Python 3.12 |
| Data Processing | pandas, NumPy |
| Machine Learning | scikit-learn |
| Visualization | matplotlib |
| Documentation | LaTeX |
| Version Control | Git & GitHub |

## How to Run

```bash
git clone https://github.com/samibennane/french_league_matchs_predictions.git
cd french_league_matchs_predictions
pip install pandas numpy scikit-learn matplotlib
python ligue1.py
