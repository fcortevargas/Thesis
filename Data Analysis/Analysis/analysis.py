# In[1]:

import pandas as pd
import numpy as np
import seaborn as sns
import matplotlib.pyplot as plt
import scipy.stats as stats
from itertools import combinations
from sklearn.preprocessing import StandardScaler, PolynomialFeatures
from sklearn.linear_model import LinearRegression
from sklearn.ensemble import RandomForestRegressor
from sklearn.mixture import GaussianMixture
from sklearn.feature_selection import f_regression
from sklearn.model_selection import KFold, GridSearchCV
from sklearn.metrics import mean_squared_error
from sklearn.metrics import r2_score
from statsmodels.stats.multitest import multipletests

# In[2]:


custom_params = {"axes.spines.right": False, "axes.spines.top": False}
sns.set_theme(style="ticks", rc=custom_params)


# ## Modelling

# ### Hypotheses Testing

# In[3]:


def spearman_test(df, independent_vars, dependent_vars, alpha=0.05):
    data = []
    p_values_non_zero = []
    p_values_positive = []
    p_values_negative = []

    for independent_var in independent_vars:
        for dependent_var in dependent_vars:
            # Calculate Spearman correlation for two-sided test
            rho, p_non_zero = stats.spearmanr(df[independent_var], df[dependent_var], alternative='two-sided')

            # Calculate Spearman correlation for one-sided tests
            _, p_negative = stats.spearmanr(df[independent_var], df[dependent_var], alternative='less')
            _, p_positive = stats.spearmanr(df[independent_var], df[dependent_var], alternative='greater')

            # Collect p-values for later adjustment
            p_values_non_zero.append(p_non_zero)
            p_values_positive.append(p_positive)
            p_values_negative.append(p_negative)

            # Store the initial results
            row = {'independent_variable': independent_var, 'dependent_variable': dependent_var, 'correlation': rho,
                   'p_non_zero': p_non_zero, 'is_non_zero': p_non_zero < alpha,
                   'p_positive': p_positive, 'is_positive': p_positive < alpha / 2,
                   'p_negative': p_negative, 'is_negative': p_negative < alpha / 2}

            data.append(row)

    # Convert the list of dictionaries to a DataFrame
    spearman_test_results_df = pd.DataFrame(data)

    spearman_test_results_df.set_index(['independent_variable', 'dependent_variable'], inplace=True)

    return spearman_test_results_df


# In[4]:


def get_correlation_annots(spearman_test_results_df):
    correlations = spearman_test_results_df.correlation.to_numpy()
    annot = correlations.astype(str)
    indexed_spearman_test_results_df = spearman_test_results_df.reset_index()
    significance_mask = indexed_spearman_test_results_df[indexed_spearman_test_results_df['is_non_zero'] & (
            indexed_spearman_test_results_df['is_positive'] | indexed_spearman_test_results_df[
        'is_negative'])].index

    # Apply asterisks to significant correlations
    for i in range(len(correlations)):
        if i in significance_mask:
            annot[i] = f'{correlations[i]:.3f}*'
        else:
            annot[i] = f'{correlations[i]:.3f}'

    annot = annot.reshape(9, 9).T

    return annot


# In[5]:


def t_test(df, dependent_vars, alpha=0.05):
    data = []
    p_values_unequal = []
    p_values_positive = []
    p_values_negative = []

    for score_type in ['mse_scores', 'r2_scores']:
        for dependent_var in dependent_vars:
            # Get the score and baseline score
            score = df.loc[score_type, dependent_var].scores
            baseline_score = df.loc[score_type, dependent_var].baseline_scores

            # Calculate one-sample t-test for two-sided hypothesis
            t_stat, p_unequal = stats.ttest_rel(score, baseline_score)

            # Calculate one-sample t-test for one-sided hypotheses
            t_stat, p_positive = stats.ttest_rel(score, baseline_score, alternative='greater')
            t_stat, p_negative = stats.ttest_rel(score, baseline_score, alternative='less')

            # Collect p-values for later adjustment
            p_values_unequal.append(p_unequal)
            p_values_positive.append(p_positive)
            p_values_negative.append(p_negative)

            # Store the initial results
            row = {'score_type': score_type, 'dependent_variable': dependent_var,
                   'baseline_scores': baseline_score, 'scores': score, 't_statistic': t_stat,
                   'p_unequal': p_unequal, 'is_unequal_significant': p_unequal < alpha,
                   'p_positive': p_positive, 'is_positive_significant': p_positive < alpha / 2,
                   'p_negative': p_negative, 'is_negative_significant': p_negative < alpha / 2}

            data.append(row)

    # Convert the list of dictionaries to a DataFrame
    t_test_results_df = pd.DataFrame(data)

    t_test_results_df.set_index(['score_type', 'dependent_variable'], inplace=True)

    return t_test_results_df


# In[6]:


def bonferrroni_correction(df, alpha=0.05):
    # Apply multiple testing correction
    statistics = df['statistic'].to_numpy()
    p_values = df['p_value'].to_numpy()
    test_results = multipletests(p_values, alpha=alpha, method='bonferroni')
    data = {}
    data['statistic'] = statistics
    data['p_value'] = p_values
    data['p_value_corrected'] = test_results[1]
    data['reject_h0'] = test_results[0]
    test_results_df = pd.DataFrame(data)
    test_results_df.index = df.index
    return test_results_df


# ### Model Training

# In[7]:


def backward_selection(X, y, significance_level=0.05):
    included = list(X.columns)
    while True:
        changed = False
        model = LinearRegression().fit(X[included], y)
        p_values = pd.Series(f_regression(X[included], y)[1], index=included)
        worst_pval = p_values.max()
        if worst_pval > significance_level:
            worst_feature = p_values.idxmax()
            included.remove(worst_feature)
            changed = True
        if not changed:
            break
    return included


# In[8]:


def k_fold_training_linear_model(X, y, n_splits=5, significance_level=0.05, random_state=42):
    kf = KFold(n_splits=n_splits, shuffle=True, random_state=random_state)
    train_mse_scores = []
    test_mse_scores = []
    train_r2_scores = []
    test_r2_scores = []

    best_model = None
    best_selected_features = None
    best_score = float('inf')
    best_train_index = None
    best_test_index = None

    for train_index, test_index in kf.split(X):
        X_train, X_test = X.iloc[train_index], X.iloc[test_index]
        y_train, y_test = y.iloc[train_index], y.iloc[test_index]

        selected_features = backward_selection(X_train, y_train, significance_level)
        model = LinearRegression().fit(X_train[selected_features], y_train)

        y_train_pred = model.predict(X_train[selected_features])
        y_test_pred = model.predict(X_test[selected_features])

        train_mse = mean_squared_error(y_train, y_train_pred)
        test_mse = mean_squared_error(y_test, y_test_pred)
        train_r2 = r2_score(y_train, y_train_pred)
        test_r2 = r2_score(y_test, y_test_pred)

        train_mse_scores.append(train_mse)
        test_mse_scores.append(test_mse)
        train_r2_scores.append(train_r2)
        test_r2_scores.append(test_r2)

        # Update the best model based on the performance on the train set
        if train_mse < best_score:
            best_score = train_mse
            best_model = model
            best_selected_features = selected_features
            best_train_index = train_index
            best_test_index = test_index

    return best_model, best_selected_features, best_train_index, best_test_index, np.array(train_mse_scores), np.array(
        test_mse_scores), np.array(train_r2_scores), np.array(test_r2_scores)


# In[9]:


def k_fold_training_random_forest_model(X, y, param_grid, n_splits=5, random_state=42):
    kf = KFold(n_splits=n_splits, shuffle=True, random_state=random_state)
    train_mse_scores = []
    test_mse_scores = []
    train_r2_scores = []
    test_r2_scores = []

    best_model = None
    best_score = float('inf')
    best_train_index = None
    best_test_index = None

    for train_index, test_index in kf.split(X):
        X_train, X_test = X[train_index], X[test_index]
        y_train, y_test = y.iloc[train_index], y.iloc[test_index]

        # Use GridSearchCV for hyperparameter tuning
        model = RandomForestRegressor(random_state=random_state)
        grid_search = GridSearchCV(estimator=model, param_grid=param_grid, cv=5, scoring='neg_mean_squared_error',
                                   n_jobs=-1)
        grid_search.fit(X_train, y_train)

        best_fold_model = grid_search.best_estimator_

        y_train_pred = best_fold_model.predict(X_train)
        y_test_pred = best_fold_model.predict(X_test)

        train_mse = mean_squared_error(y_train, y_train_pred)
        test_mse = mean_squared_error(y_test, y_test_pred)
        train_r2 = r2_score(y_train, y_train_pred)
        test_r2 = r2_score(y_test, y_test_pred)

        train_mse_scores.append(train_mse)
        test_mse_scores.append(test_mse)
        train_r2_scores.append(train_r2)
        test_r2_scores.append(test_r2)

        # Update the best model based on the performance on the train set
        if train_mse < best_score:
            best_score = train_mse
            best_model = best_fold_model
            best_train_index = train_index
            best_test_index = test_index

    return best_model, best_train_index, best_test_index, np.array(train_mse_scores), np.array(
        test_mse_scores), np.array(train_r2_scores), np.array(test_r2_scores)


# In[10]:


def print_scores(train_mse_scores, test_mse_scores, train_r2_scores, test_r2_scores):
    print(f"\nAverage Training Mean Squared Error: {train_mse_scores.mean().round(3)}")
    print(f"Average Testing Mean Squared Error: {test_mse_scores.mean().round(3)}")

    print(f"\nAverage Training R² Score: {train_r2_scores.mean().round(3)}")
    print(f"Average Testing R² Score: {test_r2_scores.mean().round(3)}")

    best_model_index = np.argmin(train_mse_scores)

    print(f"\n\nTraining Mean Squared Error of the Best Model: {train_mse_scores[best_model_index].round(3)}")
    print(f"Testing Mean Squared Error of the Best Model: {test_mse_scores[best_model_index].round(3)}")

    print(f"\nTraining R² Score of the Best Model: {train_r2_scores[best_model_index].round(3)}")
    print(f"Testing R² Score of the Best Model: {test_r2_scores[best_model_index].round(3)}")


# In[11]:


def plot_performance(dependent_var, model, model_name, X_train, y_train, X_test, y_test):
    # Define the range of the dependent variable
    if '_intensity' in dependent_var:
        var_range = [0, 5]
    else:
        var_range = [1, 9]

    # Predict on training and test data
    y_train_pred = model.predict(X_train)
    y_test_pred = model.predict(X_test)

    # Plot actual vs predicted values
    plt.figure(figsize=(12, 4))

    # Plot train data
    plt.subplot(1, 2, 1)
    plt.scatter(y_train, y_train_pred, color='royalblue', alpha=0.5, label='Train data')
    plt.plot(var_range, var_range, 'k--', lw=2)
    plt.xlabel(f'Actual {titles[dependent_var]}')
    plt.xlim(var_range[0] - 0.5, var_range[1] + 0.5)
    plt.ylabel(f'Predicted {titles[dependent_var]}')
    plt.ylim(var_range[0] - 0.5, var_range[1] + 0.5)
    plt.text(var_range[0], var_range[1] - 0.5, "MSE = {:.3f}".format(mean_squared_error(y_train, y_train_pred)))
    plt.text(var_range[0], var_range[1] - 1, "R² = {:.3f}".format(r2_score(y_train, y_train_pred)))
    plt.legend(loc="lower right")

    # Plot test data
    plt.subplot(1, 2, 2)
    plt.scatter(y_test, y_test_pred, color='lightskyblue', alpha=0.5, label='Test data')
    plt.plot(var_range, var_range, 'k--', lw=2)
    plt.xlabel(f'Actual {titles[dependent_var]}')
    plt.xlim(var_range[0] - 0.5, var_range[1] + 0.5)
    plt.ylabel(f'Predicted {titles[dependent_var]}')
    plt.ylim(var_range[0] - 0.5, var_range[1] + 0.5)
    plt.text(var_range[0], var_range[1] - 0.5, "MSE = {:.3f}".format(mean_squared_error(y_test, y_test_pred)))
    plt.text(var_range[0], var_range[1] - 1, "R² = {:.3f}".format(r2_score(y_test, y_test_pred)))
    plt.legend(loc="lower right")

    plt.savefig(f'../Data/Figures/{model_name}_performance.pdf', bbox_inches='tight')

    plt.show()


# In[12]:


def plot_importances(rf_importances_df, plot_name):
    rf_importances_df.index = rf_importances_df.index.map(lambda feature: titles.get(feature, feature))
    plt.figure(figsize=(6, 4))
    sns.barplot(x='Importance', y='Feature',
                data=rf_importances_df.sort_values(by='Importance', ascending=False).head(10), color='lightskyblue')
    plt.xlabel('Importance')
    plt.ylabel('')
    plt.savefig(f'../Data/Figures/{plot_name}.pdf', bbox_inches='tight')
    plt.show()


# ## Data Preparation

# In[13]:


file_path = '../Data/Processed/rating_numeric.csv'

# In[14]:


# Load data
data = pd.read_csv(file_path, header=0, index_col=[0, 1])

# Convert start_time and end_time to datetime format
data['start_time'] = pd.to_datetime(data['start_time'])
data['end_time'] = pd.to_datetime(data['end_time'])

# Drop the 'appraisal' column as it is non-numeric
data = data.drop(columns=['appraisal'])

# In[15]:


# Display basic information about the dataset
data.info()

# In[16]:


data.describe(include=np.number)

# In[17]:


data

# In[18]:


# Define the emotion intensities columns
intensity_columns = [
    'joy_intensity', 'sadness_intensity', 'anger_intensity',
    'fear_intensity', 'disgust_intensity', 'surprise_intensity'
]

# Define the SAM columns
sam_columns = ['pleasure', 'arousal', 'dominance']

# Define the dependent variables
dependent_vars = intensity_columns + sam_columns

# Define the index of each dependent variables in the y dataset
dependent_var_index = {dependent_var: i for i, dependent_var in enumerate(dependent_vars)}

# Define the independent variables continuous independent variables
independent_vars = [
    'wander_speed', 'wander_roundness', 'wander_cycle_rate',
    'blink_temperature', 'blink_slope', 'blink_cycle_rate',
    'beep_pitch', 'beep_slope', 'beep_cycle_rate'
]

# Specify which of the independent variables are continuous
independent_continuous_vars = [
    'wander_speed', 'wander_roundness', 'wander_cycle_rate',
    'blink_temperature', 'blink_cycle_rate',
    'beep_pitch', 'beep_cycle_rate'
]

# In[19]:


titles = {'wander_speed': 'Wander Speed', 'wander_roundness': 'Wander Roundness',
          'wander_cycle_rate': 'Wander Cycle Rate',
          'blink_temperature': 'Blink Temperature', 'blink_slope': 'Blink Slope',
          'blink_cycle_rate': 'Blink Cycle Rate',
          'beep_pitch': 'Beep Pitch', 'beep_slope': 'Beep Slope', 'beep_cycle_rate': 'Beep Cycle Rate',
          'joy_intensity': 'Joy Intensity', 'sadness_intensity': 'Sadness Intensity',
          'anger_intensity': 'Anger Intensity',
          'fear_intensity': 'Fear Intensity', 'disgust_intensity': 'Disgust Intensity',
          'surprise_intensity': 'Surprise Intensity',
          'pleasure': 'Pleasure', 'arousal': 'Arousal', 'dominance': 'Dominance',
          'cluster_prob_0': 'Membership Probability of Cluster 1',
          'cluster_prob_1': 'Membership Probability of Cluster 2',
          'cluster_prob_2': 'Membership Probability of Cluster 3',
          'cluster_prob_3': 'Membership Probability of Cluster 4',
          'cluster_prob_4': 'Membership Probability of Cluster 5',
          'cluster_prob_5': 'Membership Probability of Cluster 6',
          'cluster_prob_6': 'Membership Probability of Cluster 7',
          'cluster_prob_7': 'Membership Probability of Cluster 8',
          'cluster_prob_8': 'Membership Probability of Cluster 9'}

# ## Exploratory Data Analysis

# In[20]:


fig, axes = plt.subplots(3, 3, figsize=(12, 6))

for ax, independent_var in zip(axes.flatten(), independent_vars):
    # Create the histplot in the specified subplot
    sns.histplot(data=data, x=independent_var, ax=ax, color='lightskyblue')
    ax.set_title(titles[independent_var])
    ax.set_xlabel('')
    ax.set_ylabel('Count')

plt.tight_layout(rect=[0, 0, 1, 0.96])
plt.savefig('../Data/Figures/independent_vars_hists.pdf', bbox_inches='tight')
plt.show()

# In[21]:


fig, axes = plt.subplots(2, 3, figsize=(12, 6))

for ax, dependent_var in zip(axes.flatten(), intensity_columns):
    # Create the catplot in the specified subplot
    sns.countplot(data=data, x=dependent_var, ax=ax, color='lightskyblue')
    ax.set_title(titles[dependent_var])
    ax.set_xlabel('')
    ax.set_xlim(-0.5, 5.5)
    ax.set_xticks(range(6))
    ax.set_xticklabels(['N/A', 'Very Low', 'Low', 'Average', 'High', 'Very High'])
    ax.set_ylabel('Count')

plt.tight_layout(rect=[0, 0, 1, 0.96])
plt.savefig('../Data/Figures/intensity_vars_hists.pdf', bbox_inches='tight')
plt.show()

# Next, we create a grid of count plots to visualize the distributions of the Self-Assessment Manikin columns:

# In[22]:


fig, axes = plt.subplots(1, 3, figsize=(12, 3))

for ax, dependent_var in zip(axes.flatten(), sam_columns):
    # Create the catplot in the specified subplot
    sns.countplot(data=data, x=dependent_var, ax=ax, color='lightskyblue')
    ax.set_title(titles[dependent_var])
    ax.set_xlabel('')
    ax.set_xticks(range(9))
    ax.set_xticklabels(range(1, 10))
    ax.set_ylabel('Count')

plt.tight_layout(rect=[0, 0, 1, 0.96])
plt.savefig('../Data/Figures/sam_vars_hists.pdf', bbox_inches='tight')
plt.show()

# #### Correlation Matrix of the Independent Variables

# In[23]:


corr_matrix = data[independent_vars + dependent_vars].corr(method='spearman')

# In[24]:


spearman_test_results_df = spearman_test(data, independent_vars, independent_vars)
annot = get_correlation_annots(spearman_test_results_df)

plt.figure(figsize=(12, 6))
ticklabels = [titles[var] for var in independent_vars]
sns.heatmap(corr_matrix.iloc[:9, :9], mask=np.triu(np.ones_like(corr_matrix.iloc[:9, :9], dtype=bool), k=1),
            annot=annot, xticklabels=ticklabels, yticklabels=ticklabels, cmap='coolwarm', fmt='', center=0, vmin=-1,
            vmax=1, cbar=False)
plt.savefig('../Data/Figures/correlation_independent_vars.pdf', bbox_inches='tight')
plt.show()

# #### Correlation Matrix of the Dependent Variables

# In[25]:


spearman_test_results_df = spearman_test(data, dependent_vars, dependent_vars)
annot = get_correlation_annots(spearman_test_results_df)

plt.figure(figsize=(12, 6))
ticklabels = [titles[var] for var in dependent_vars]
sns.heatmap(corr_matrix.iloc[9:, 9:], mask=np.triu(np.ones_like(corr_matrix.iloc[9:, 9:], dtype=bool), k=1),
            annot=annot, xticklabels=ticklabels, yticklabels=ticklabels, cmap='coolwarm', fmt='', center=0, vmin=-1,
            vmax=1, cbar=False)
plt.savefig('../Data/Figures/correlation_dependent_vars.pdf', bbox_inches='tight')
plt.show()

# #### Correlation Matrix of the Independent and the Dependent Variables

# In[26]:


spearman_test_results_df = spearman_test(data, independent_vars, dependent_vars)
annot = get_correlation_annots(spearman_test_results_df)

plt.figure(figsize=(12, 6))
xticklabels = [titles[var] for var in independent_vars]
yticklabels = [titles[var] for var in dependent_vars]
sns.heatmap(corr_matrix.iloc[9:, :9], annot=annot, xticklabels=xticklabels, yticklabels=yticklabels, cmap='coolwarm',
            fmt='', center=0, vmin=-1, vmax=1, cbar=False)
plt.savefig('../Data/Figures/correlation_independent_dependent_vars.pdf', bbox_inches='tight')
plt.show()


# ## Outlier Removal

# In[27]:


# Function to detect and remove outliers using IQR
def remove_outliers(df, columns):
    for col in columns:
        Q1 = df[col].quantile(0.25)
        Q3 = df[col].quantile(0.75)
        IQR = Q3 - Q1
        lower_bound = Q1 - 1.5 * IQR
        upper_bound = Q3 + 1.5 * IQR
        df = df[(df[col] >= lower_bound) & (df[col] <= upper_bound)]
    return df


# Remove outliers from the dependent variables
cleaned_data = remove_outliers(data, dependent_vars)

# In[28]:


cleaned_data

# In[29]:


cleaned_data.groupby("video_id").transform("size").mean()

# ## Exploratory Data Analysis Post Outlier Removal

# In[30]:


ffig, axes = plt.subplots(3, 3, figsize=(12, 6))

for ax, independent_var in zip(axes.flatten(), independent_vars):
    # Create the histplot in the specified subplot
    sns.histplot(data=cleaned_data, x=independent_var, ax=ax, color='lightskyblue')
    ax.set_title(titles[independent_var])
    ax.set_xlabel('')
    ax.set_ylabel('Count')

plt.tight_layout(rect=[0, 0, 1, 0.96])
plt.savefig('../Data/Figures/independent_vars_hists_no_outliers.pdf', bbox_inches='tight')
plt.show()

# ### Visualize Distributions of Dependent Variables
#
# Next, we create a grid of count plots to visualize the distributions of the emotion intensity variables. This helps us understand the distribution of the intensities for each emotion:

# In[31]:


fig, axes = plt.subplots(2, 3, figsize=(12, 6))

for ax, dependent_var in zip(axes.flatten(), intensity_columns):
    # Create the catplot in the specified subplot
    sns.countplot(data=cleaned_data, x=dependent_var, ax=ax, color='lightskyblue')
    ax.set_title(titles[dependent_var])
    ax.set_xlabel('')
    ax.set_xlim(-0.5, 5.5)
    ax.set_xticks(range(6))
    ax.set_xticklabels(['N/A', 'Very Low', 'Low', 'Average', 'High', 'Very High'])
    ax.set_ylabel('Count')

plt.tight_layout(rect=[0, 0, 1, 0.96])
plt.savefig('../Data/Figures/intensity_vars_hists_no_outliers.pdf', bbox_inches='tight')
plt.show()

# In[32]:


fig, axes = plt.subplots(1, 3, figsize=(12, 3))

for ax, dependent_var in zip(axes.flatten(), sam_columns):
    # Create the catplot in the specified subplot
    sns.countplot(data=cleaned_data, x=dependent_var, ax=ax, color='lightskyblue')
    ax.set_title(titles[dependent_var])
    ax.set_xlabel('')
    ax.set_xticks(range(9))
    ax.set_xticklabels(range(1, 10))
    ax.set_ylabel('Count')

plt.tight_layout(rect=[0, 0, 1, 0.96])
plt.savefig('../Data/Figures/sam_vars_hists_no_outliers.pdf', bbox_inches='tight')
plt.show()

# #### Correlation Matrix of the Independent Variables Post Outlier Removal

# In[33]:


corr_matrix = cleaned_data[independent_vars + dependent_vars].corr(method='spearman')

# In[34]:


spearman_test_results_df = spearman_test(cleaned_data, independent_vars, independent_vars)
annot = get_correlation_annots(spearman_test_results_df)

plt.figure(figsize=(12, 6))
ticklabels = [titles[var] for var in independent_vars]
sns.heatmap(corr_matrix.iloc[:9, :9], mask=np.triu(np.ones_like(corr_matrix.iloc[:9, :9], dtype=bool), k=1),
            annot=annot, xticklabels=ticklabels, yticklabels=ticklabels, cmap='coolwarm', fmt='', center=0, vmin=-1,
            vmax=1, cbar=False)
plt.savefig('../Data/Figures/correlation_independent_vars_no_outliers.pdf', bbox_inches='tight')
plt.show()

# #### Correlation Matrix of the Dependent Variables Post Outlier Removal

# In[35]:


spearman_test_results_df = spearman_test(cleaned_data, dependent_vars, dependent_vars)
annot = get_correlation_annots(spearman_test_results_df)

plt.figure(figsize=(12, 6))
ticklabels = [titles[var] for var in dependent_vars]
sns.heatmap(corr_matrix.iloc[9:, 9:], mask=np.triu(np.ones_like(corr_matrix.iloc[9:, 9:], dtype=bool), k=1),
            annot=annot, xticklabels=ticklabels, yticklabels=ticklabels, cmap='coolwarm', fmt='', center=0, vmin=-1,
            vmax=1, cbar=False)
plt.savefig('../Data/Figures/correlation_dependent_vars_no_outliers.pdf', bbox_inches='tight')
plt.show()

# #### Correlation Matrix of the Independent and the Dependent Variables Post Outlier Removal

# In[36]:


spearman_test_results_df = spearman_test(cleaned_data, independent_vars, dependent_vars)
annot = get_correlation_annots(spearman_test_results_df)

plt.figure(figsize=(12, 6))
xticklabels = [titles[var] for var in independent_vars]
yticklabels = [titles[var] for var in dependent_vars]
sns.heatmap(corr_matrix.iloc[9:, :9], annot=annot, xticklabels=xticklabels, yticklabels=yticklabels, cmap='coolwarm',
            fmt='', center=0, vmin=-1, vmax=1, cbar=False)
plt.savefig('../Data/Figures/correlation_independent_dependent_vars_no_outliers.pdf', bbox_inches='tight')
plt.show()

# ## Reducing Noise in the Dependent Variables by Aggregation

# In[37]:


# Aggregate the dependent variables by video_id (mean, median, and mode)
dependent_aggregated_mean = cleaned_data.groupby('video_id')[dependent_vars].mean().add_suffix('_mean')
dependent_aggregated_median = cleaned_data.groupby('video_id')[dependent_vars].median().add_suffix('_median')
dependent_aggregated_mode = cleaned_data.groupby('video_id')[dependent_vars].agg(lambda x: x.mode().iloc[0]).add_suffix(
    '_mode')

# Add the titles of the aggregated variables
for dependent_var in dependent_vars:
    for aggregation in ['mean', 'median', 'mode']:
        dependent_aggregated_var = dependent_var + '_' + aggregation
        titles[dependent_aggregated_var] = aggregation.title() + ' ' + titles[dependent_var]

# Merge the aggregated dependent variables with the original independent variables
aggregated_dependent_vars = pd.concat(
    [dependent_aggregated_mean, dependent_aggregated_median, dependent_aggregated_mode], axis=1)
cleaned_data = cleaned_data.merge(aggregated_dependent_vars, on='video_id')
cleaned_aggregated_data = cleaned_data.reset_index().drop_duplicates('video_id').set_index('video_id')
cleaned_aggregated_data = cleaned_aggregated_data.drop(['start_time', 'end_time'], axis=1)

# In[38]:


cleaned_aggregated_data.sort_index()

# In[39]:


statistics = cleaned_aggregated_data[dependent_vars].describe().loc[['mean', 'std'], :].round(3)

# In[40]:


statistics

# In[41]:


statistics_aggregated = dependent_aggregated_mean.describe().loc[['mean', 'std'], :].round(3)
statistics_aggregated.columns = dependent_vars
statistics_aggregated.index = ['mean_agg', 'std_agg']

# In[42]:


statistics_aggregated

# In[43]:


pd.concat([statistics, statistics_aggregated]).T.to_csv('../Data/Tables/dependent_aggregated_vars_statistics.csv')

# In[44]:


# Select mean because it's the one that reduces noise the most
target_aggregation = 'mean'

# Select target aggregated variable
mask = [target_aggregation in column for column in cleaned_aggregated_data.columns]
dependent_aggregated_vars = list(cleaned_aggregated_data.columns[mask])

# ## Exploratory Data Analysis Post Aggregation

# In[45]:


plt.figure(figsize=(12, 6))
g = sns.pairplot(cleaned_aggregated_data[dependent_aggregated_vars], diag_kind="hist",
                 plot_kws={'color': 'lightskyblue'}, diag_kws={'color': 'lightskyblue'})
for ax in g.axes.flatten():
    try:
        xlabel = ax.get_xlabel()
        ylabel = ax.get_ylabel()
        if xlabel in titles:
            ax.set_xlabel(titles[xlabel])
        if ylabel in titles:
            ax.set_ylabel(titles[ylabel])
    except:
        continue
plt.show()

# In[46]:


ffig, axes = plt.subplots(3, 3, figsize=(12, 6))

for ax, independent_var in zip(axes.flatten(), independent_vars):
    # Create the histplot in the specified subplot
    sns.histplot(data=cleaned_aggregated_data, x=independent_var, ax=ax, color='lightskyblue')
    ax.set_title(titles[independent_var])
    ax.set_xlabel('')
    ax.set_ylabel('Count')

plt.tight_layout(rect=[0, 0, 1, 0.96])
plt.savefig('../Data/Figures/independent_vars_hists_aggregated.pdf', bbox_inches='tight')
plt.show()

# ### Visualize Distributions of Dependent Variables Post Aggregation

# In[47]:


fig, axes = plt.subplots(2, 3, figsize=(12, 6))

intensity_columns_aggregated = [column + '_' + target_aggregation for column in intensity_columns]

for ax, dependent_var in zip(axes.flatten(), intensity_columns_aggregated):
    # Create the catplot in the specified subplot
    sns.histplot(data=cleaned_aggregated_data, x=dependent_var, ax=ax, color='lightskyblue')
    ax.set_title(titles[dependent_var])
    ax.set_xlabel('')
    ax.set_xlim(-0.5, 5.5)
    ax.set_xticks(range(6))
    ax.set_xticklabels(['N/A', 'Very Low', 'Low', 'Average', 'High', 'Very High'])
    ax.set_ylabel('Count')

plt.tight_layout(rect=[0, 0, 1, 0.96])
plt.savefig('../Data/Figures/intensity_vars_hists_aggregated.pdf', bbox_inches='tight')
plt.show()

# In[48]:


fig, axes = plt.subplots(1, 3, figsize=(16, 4))

sam_columns_aggregated = [column + '_' + target_aggregation for column in sam_columns]

for ax, dependent_var in zip(axes.flatten(), sam_columns_aggregated):
    # Create the catplot in the specified subplot
    sns.histplot(data=cleaned_aggregated_data, x=dependent_var, ax=ax, color='lightskyblue')
    ax.set_title(titles[dependent_var])
    ax.set_xlabel('')
    ax.set_xticks(range(9))
    ax.set_xticklabels(range(1, 10))
    ax.set_ylabel('Count')

plt.tight_layout(rect=[0, 0, 1, 0.96])
plt.savefig('../Data/Figures/sam_vars_hists_aggregated.pdf', bbox_inches='tight')
plt.show()

# #### Correlation Matrix of the Independent Variables Post Aggregation

# In[49]:


corr_matrix = cleaned_aggregated_data[independent_vars + dependent_aggregated_vars].corr(method='spearman')

# In[50]:


spearman_test_results_df = spearman_test(cleaned_aggregated_data, independent_vars, independent_vars)
annot = get_correlation_annots(spearman_test_results_df)

plt.figure(figsize=(12, 6))
ticklabels = [titles[var] for var in independent_vars]
sns.heatmap(corr_matrix.iloc[:9, :9], mask=np.triu(np.ones_like(corr_matrix.iloc[:9, :9], dtype=bool), k=1),
            annot=annot, xticklabels=ticklabels, yticklabels=ticklabels, cmap='coolwarm', fmt='', center=0, vmin=-1,
            vmax=1, cbar=False)
plt.savefig('../Data/Figures/correlation_independent_vars_aggregated.pdf', bbox_inches='tight')
plt.show()

# #### Correlation Matrix of the Dependent Variables Post Aggregation

# In[51]:


spearman_test_results_df = spearman_test(cleaned_aggregated_data, dependent_aggregated_vars, dependent_aggregated_vars)
annot = get_correlation_annots(spearman_test_results_df)

plt.figure(figsize=(12, 6))
ticklabels = [titles[var] for var in dependent_vars]
sns.heatmap(corr_matrix.iloc[9:, 9:], mask=np.triu(np.ones_like(corr_matrix.iloc[9:, 9:], dtype=bool), k=1),
            annot=annot, xticklabels=ticklabels, yticklabels=ticklabels, cmap='coolwarm', fmt='', center=0, vmin=-1,
            vmax=1, cbar=False)
plt.savefig('../Data/Figures/correlation_dependent_vars_aggregated.pdf', bbox_inches='tight')
plt.show()

# #### Correlation Matrix of the Independent and the Dependent Variables Aggregation

# In[52]:


spearman_test_results_df = spearman_test(cleaned_data, independent_vars, dependent_aggregated_vars)
annot = get_correlation_annots(spearman_test_results_df)

plt.figure(figsize=(12, 6))
xticklabels = [titles[var] for var in independent_vars]
yticklabels = [titles[var] for var in dependent_vars]
sns.heatmap(corr_matrix.iloc[9:, :9], annot=annot, xticklabels=xticklabels, yticklabels=yticklabels, cmap='coolwarm',
            fmt='', center=0, vmin=-1, vmax=1, cbar=False)
plt.savefig('../Data/Figures/correlation_independent_dependent_vars_aggregated.pdf', bbox_inches='tight')
plt.show()

# In[53]:


# Set up the matplotlib figure
plt.figure(figsize=(16, 4))

# Scatter plot for Wander Speed vs Arousal
plt.subplot(1, 3, 1)
sns.scatterplot(data=cleaned_aggregated_data, x='wander_speed', y='_'.join(('arousal', target_aggregation)),
                color='lightskyblue')
plt.xlabel('Wander Speed')
plt.ylabel('Mean Arousal')
plt.ylim(1, 9)

# Scatter plot for Blink Temperature vs Anger Intensity
plt.subplot(1, 3, 2)
sns.scatterplot(data=cleaned_aggregated_data, x='blink_temperature',
                y='_'.join(('anger_intensity', target_aggregation)), color='lightskyblue')

plt.xlabel('Blink Temperature')
plt.ylabel('Mean Anger Intensity')
plt.ylim(0, 5)
plt.yticks(range(0, 6), ['N/A', 'Very Low', 'Low', 'Average', 'High', 'Very High'])

# Scatter plot for Beep Pitch vs Surprise Intensity
plt.subplot(1, 3, 3)
sns.scatterplot(data=cleaned_aggregated_data, x='beep_pitch', y='_'.join(('surprise_intensity', target_aggregation)),
                color='lightskyblue')
plt.xlabel('Beep Pitch')
plt.ylabel('Mean Surprise Intensity')
plt.ylim(0, 5)
plt.yticks(range(0, 6), ['N/A', 'Very Low', 'Low', 'Average', 'High', 'Very High'])

# Display the plots
plt.tight_layout()
plt.show()

# ## Hypotheses Testing RQ1, RQ2 and RQ3

# In[54]:


selected = []

# In[55]:


rq1_positive_tests_indices = [['wander_speed', 'arousal_mean'], ['wander_roundness', 'pleasure_mean'],
                              ['wander_cycle_rate', 'arousal_mean']]
rq1_negative_tests_indices = [['wander_cycle_rate', 'pleasure_mean']]

# In[56]:


selected.append(spearman_test_results_df.loc[rq1_positive_tests_indices][['correlation', 'p_positive']].rename(
    columns={'correlation': 'statistic', 'p_positive': 'p_value'}))

# In[57]:


selected.append(spearman_test_results_df.loc[rq1_negative_tests_indices][['correlation', 'p_negative']].rename(
    columns={'correlation': 'statistic', 'p_negative': 'p_value'}))

# In[58]:


rq2_positive_tests_indices = [['blink_temperature', 'arousal_mean'], ['blink_cycle_rate', 'arousal_mean']]
rq2_negative_tests_indices = [['blink_temperature', 'pleasure_mean']]

# In[59]:


selected.append(spearman_test_results_df.loc[rq2_positive_tests_indices][['correlation', 'p_positive']].rename(
    columns={'correlation': 'statistic', 'p_positive': 'p_value'}))

# In[60]:


selected.append(spearman_test_results_df.loc[rq2_negative_tests_indices][['correlation', 'p_negative']].rename(
    columns={'correlation': 'statistic', 'p_negative': 'p_value'}))

# In[61]:


rq3_positive_tests_indices = [['beep_pitch', 'arousal_mean'], ['beep_cycle_rate', 'arousal_mean'],
                              ['beep_slope', 'anger_intensity_mean']]
rq3_negative_tests_indices = [['beep_slope', 'sadness_intensity_mean']]

# In[62]:


selected.append(spearman_test_results_df.loc[rq3_positive_tests_indices][['correlation', 'p_positive']].rename(
    columns={'correlation': 'statistic', 'p_positive': 'p_value'}))

# In[63]:


selected.append(spearman_test_results_df.loc[rq3_negative_tests_indices][['correlation', 'p_negative']].rename(
    columns={'correlation': 'statistic', 'p_negative': 'p_value'}))

# In[64]:


rq1_rq2_rq3_df = pd.concat(selected, axis=0)

# In[65]:


rq1_rq2_rq3_df = bonferrroni_correction(rq1_rq2_rq3_df)

# In[66]:


rq1_rq2_rq3_df.reset_index(inplace=True)
rq1_rq2_rq3_df.index = pd.Index(['rq1.1', 'rq1.2', 'rq1.3.1', 'rq1.3.2',
                                 'rq2.1.1', 'rq2.1.2', 'rq2.2',
                                 'rq3.1.1', 'rq3.1.2', 'rq3.2', 'rq3.3'])
rq1_rq2_rq3_df['reject_h0'] = rq1_rq2_rq3_df['reject_h0'].astype("string")
rq1_rq2_rq3_df.drop(columns=['p_value'], inplace=True)
rq1_rq2_rq3_df.loc[rq1_rq2_rq3_df['reject_h0'] == 'True', 'reject_h0'] = 'Reject $H_0$'
rq1_rq2_rq3_df.loc[rq1_rq2_rq3_df['reject_h0'] == 'False', 'reject_h0'] = 'Fail to reject $H_0$'
rq1_rq2_rq3_df.independent_variable = rq1_rq2_rq3_df.independent_variable.map(
    lambda feature: titles.get(feature, feature))
rq1_rq2_rq3_df.dependent_variable = rq1_rq2_rq3_df.dependent_variable.map(lambda feature: titles.get(feature, feature))
rq1_rq2_rq3_df.columns = ['Independent Variable', 'Dependent Variable', 'Spearman $\rho$', '$p$-value', 'Decision']
rq1_rq2_rq3_df = rq1_rq2_rq3_df.round(3)

# In[67]:


rq1_rq2_rq3_df

# In[68]:


rq1_rq2_rq3_df.to_csv('../Data/Tables/rq1_rq2_rq3.csv')

# ## Model Training

# In[69]:


param_grid = {
    'n_estimators': [100, 200, 500],
    'max_depth': [10, 20, 30, None],
    'min_samples_split': [2, 5, 10, 20],
    'min_samples_leaf': [1, 2, 4, 10],
    'max_features': [1, 'sqrt', 'log2']
}

# In[70]:


scores = []

# ### Gaussian Mixture Model Clustering

# In[71]:


# Fit a Gaussian Mixture Model
# Standardize the data
scaler = StandardScaler()
data_scaled = scaler.fit_transform(cleaned_aggregated_data[dependent_aggregated_vars])

gmm = GaussianMixture(n_components=9, random_state=42)
gmm.fit(data_scaled)
probabilities = gmm.predict_proba(data_scaled)

# Add cluster probabilities as new columns
probabilities_df = pd.DataFrame(probabilities, columns=[f'cluster_prob_{i}' for i in range(probabilities.shape[1])],
                                index=cleaned_aggregated_data.index)

cleaned_aggregated_data = pd.concat([cleaned_aggregated_data, probabilities_df], axis=1)

cluster_cols = list(probabilities_df.columns)

cleaned_aggregated_data['cluster_id'] = np.argmax(cleaned_aggregated_data[cluster_cols], axis=1)

cluster_cols_with_id = cluster_cols + ['cluster_id']

# ### Data Preparation for Regression

# In[72]:


# Select the independent and dependent variables
X = cleaned_aggregated_data[independent_vars + cluster_cols]

# Include the interaction effect features
poly = PolynomialFeatures(degree=2, interaction_only=True, include_bias=False)
X_poly = poly.fit_transform(X)

# Get the feature names
feature_names = poly.get_feature_names_out(X.columns)

# In[73]:


keys = np.array(list(titles.keys()))
mask = ['wander' in key or 'blink' in key or 'beep' in key or 'cluster' in key for key in titles.keys()]

# Get all combinations of 2 variables
variable_combinations = list(combinations(keys[mask], 2))

# Create new titles for interaction terms
for var1, var2 in variable_combinations:
    if f"{var1} {var2}" not in titles and f"{var2} {var1}" not in titles:
        new_title = '{} × {}'.format(titles[var1], titles[var2])
        titles['{} {}'.format(var1, var2)] = new_title

# In[74]:


# Standardize the features
scaler = StandardScaler()
X_scaled = scaler.fit_transform(X_poly)

# Convert X_scaled to a DataFrame with the correct column names
X_scaled_df = pd.DataFrame(X_scaled, columns=feature_names)

# #### Selecting the Mean Joy Intensity as Target Variable

# In[75]:


# Select the target dependent variable
target_dependent_var = 'joy_intensity'
y = cleaned_aggregated_data[target_dependent_var + '_' + target_aggregation]

# #### Linear Regression Model of the Mean Joy Intensity

# In[76]:


lin_gmm_model_joy, selected_features, train_index, test_index, baseline_train_mse_scores, baseline_test_mse_scores, baseline_train_r2_scores, baseline_test_r2_scores = k_fold_training_linear_model(
    X_scaled_df, y)

# In[77]:


# Split the data
X_train, X_test, y_train, y_test = X_scaled_df.iloc[train_index], X_scaled_df.iloc[test_index], y.iloc[train_index], \
    y.iloc[test_index]

# In[78]:


lin_gmm_model_joy_df = pd.DataFrame(
    {'Feature': lin_gmm_model_joy.feature_names_in_, 'Coefficient': lin_gmm_model_joy.coef_})
lin_gmm_model_joy_df['absolute_coefficient'] = abs(lin_gmm_model_joy_df['Coefficient'])
lin_gmm_model_joy_df.set_index('Feature', inplace=True)
lin_gmm_model_joy_df.index = lin_gmm_model_joy_df.index.map(lambda feature: titles.get(feature, feature))
lin_gmm_model_joy_df.sort_values(by='absolute_coefficient', ascending=False, inplace=True)
lin_gmm_model_joy_df[['Coefficient']].head(10).round(3).to_csv('../Data/Tables/lin_gmm_model_joy_coefficients.csv')

# In[79]:


lin_gmm_model_joy_df

# In[80]:


print_scores(baseline_train_mse_scores, baseline_test_mse_scores, baseline_train_r2_scores, baseline_test_r2_scores)

# In[81]:


plot_performance(target_dependent_var + '_' + target_aggregation, lin_gmm_model_joy, 'lin_gmm_model_joy',
                 X_train[selected_features], y_train, X_test[selected_features], y_test)

# #### Random Forest Regression Model of the Mean Joy Intensity

# In[82]:


rf_gmm_model_joy, train_index, test_index, train_mse_scores, test_mse_scores, train_r2_scores, test_r2_scores = k_fold_training_random_forest_model(
    X_scaled_df.to_numpy(), y, param_grid)

# In[83]:


# Split the data
X_train, X_test, y_train, y_test = X_scaled_df.iloc[train_index], X_scaled_df.iloc[test_index], y.iloc[train_index], \
    y.iloc[test_index]

# In[84]:


rf_gmm_model_joy.get_params()

# In[85]:


rf_gmm_model_joy_df = pd.DataFrame({'Feature': feature_names, 'Importance': rf_gmm_model_joy.feature_importances_})
rf_gmm_model_joy_df.set_index('Feature', inplace=True)

# In[86]:


plot_importances(rf_gmm_model_joy_df, 'rf_gmm_model_joy_importances')

# In[87]:


print_scores(train_mse_scores, test_mse_scores, train_r2_scores, test_r2_scores)

# In[88]:


plot_performance(target_dependent_var + '_' + target_aggregation, rf_gmm_model_joy, 'rf_gmm_model_joy',
                 X_train.to_numpy(), y_train, X_test.to_numpy(), y_test)

# In[89]:


scores.append({'score_type': 'mse_scores', 'dependent_variable': target_dependent_var,
               'baseline_scores': baseline_test_mse_scores, 'scores': test_mse_scores})
scores.append(
    {'score_type': 'r2_scores', 'dependent_variable': target_dependent_var, 'baseline_scores': baseline_test_r2_scores,
     'scores': test_r2_scores})

# #### Selecting the Mean Sadness Intensity as Target Variable

# In[90]:


# Select the target dependent variable
target_dependent_var = 'sadness_intensity'
y = cleaned_aggregated_data[target_dependent_var + '_' + target_aggregation]

# #### Linear Regression Model of the Mean Sadness Intensity

# In[91]:


lin_gmm_model_sadness, selected_features, train_index, test_index, baseline_train_mse_scores, baseline_test_mse_scores, baseline_train_r2_scores, baseline_test_r2_scores = k_fold_training_linear_model(
    X_scaled_df, y)

# In[92]:


# Split the data
X_train, X_test, y_train, y_test = X_scaled_df.iloc[train_index], X_scaled_df.iloc[test_index], y.iloc[train_index], \
    y.iloc[test_index]

# In[93]:


lin_gmm_model_sadness_df = pd.DataFrame(
    {'Feature': lin_gmm_model_sadness.feature_names_in_, 'Coefficient': lin_gmm_model_sadness.coef_})
lin_gmm_model_sadness_df['absolute_coefficient'] = abs(lin_gmm_model_sadness_df['Coefficient'])
lin_gmm_model_sadness_df.set_index('Feature', inplace=True)
lin_gmm_model_sadness_df.index = lin_gmm_model_sadness_df.index.map(lambda feature: titles.get(feature, feature))
lin_gmm_model_sadness_df.sort_values(by='absolute_coefficient', ascending=False, inplace=True)
lin_gmm_model_sadness_df[['Coefficient']].head(10).round(3).to_csv(
    '../Data/Tables/lin_gmm_model_sadness_coefficients.csv')

# In[94]:


lin_gmm_model_sadness_df

# In[95]:


print_scores(baseline_train_mse_scores, baseline_test_mse_scores, baseline_train_r2_scores, baseline_test_r2_scores)

# In[96]:


plot_performance(target_dependent_var + '_' + target_aggregation, lin_gmm_model_sadness, 'lin_gmm_model_sadness',
                 X_train[selected_features], y_train, X_test[selected_features], y_test)

# #### Random Forest Regression Model of the Mean Sadness Intensity

# In[97]:


rf_gmm_model_sadness, train_index, test_index, train_mse_scores, test_mse_scores, train_r2_scores, test_r2_scores = k_fold_training_random_forest_model(
    X_scaled_df.to_numpy(), y, param_grid)

# In[98]:


# Split the data
X_train, X_test, y_train, y_test = X_scaled_df.iloc[train_index], X_scaled_df.iloc[test_index], y.iloc[train_index], \
    y.iloc[test_index]

# In[99]:


rf_gmm_model_sadness.get_params()

# In[100]:


rf_gmm_model_sadness_df = pd.DataFrame(
    {'Feature': feature_names, 'Importance': rf_gmm_model_sadness.feature_importances_})
rf_gmm_model_sadness_df.set_index('Feature', inplace=True)

# In[101]:


plot_importances(rf_gmm_model_sadness_df, 'rf_gmm_model_sadness_importances')

# In[102]:


print_scores(train_mse_scores, test_mse_scores, train_r2_scores, test_r2_scores)

# In[103]:


plot_performance(target_dependent_var + '_' + target_aggregation, rf_gmm_model_sadness, 'rf_gmm_model_sadness',
                 X_train.to_numpy(), y_train, X_test.to_numpy(), y_test)

# In[104]:


scores.append({'score_type': 'mse_scores', 'dependent_variable': target_dependent_var,
               'baseline_scores': baseline_test_mse_scores, 'scores': test_mse_scores})

scores.append(
    {'score_type': 'r2_scores', 'dependent_variable': target_dependent_var, 'baseline_scores': baseline_test_r2_scores,
     'scores': test_r2_scores})

# #### Selecting the Mean Fear Intensity as Target Variable

# In[105]:


# Select the target dependent variable
target_dependent_var = 'fear_intensity'
y = cleaned_aggregated_data[target_dependent_var + '_' + target_aggregation]

# #### Linear Regression Model of the Mean Fear Intensity

# In[106]:


lin_gmm_model_fear, selected_features, train_index, test_index, baseline_train_mse_scores, baseline_test_mse_scores, baseline_train_r2_scores, baseline_test_r2_scores = k_fold_training_linear_model(
    X_scaled_df, y)

# In[107]:


# Split the data
X_train, X_test, y_train, y_test = X_scaled_df.iloc[train_index], X_scaled_df.iloc[test_index], y.iloc[train_index], \
    y.iloc[test_index]

# In[108]:


lin_gmm_model_fear_df = pd.DataFrame(
    {'Feature': lin_gmm_model_fear.feature_names_in_, 'Coefficient': lin_gmm_model_fear.coef_})
lin_gmm_model_fear_df['absolute_coefficient'] = abs(lin_gmm_model_fear_df['Coefficient'])
lin_gmm_model_fear_df.set_index('Feature', inplace=True)
lin_gmm_model_fear_df.index = lin_gmm_model_fear_df.index.map(lambda feature: titles.get(feature, feature))
lin_gmm_model_fear_df.sort_values(by='absolute_coefficient', ascending=False, inplace=True)
lin_gmm_model_fear_df[['Coefficient']].head(10).round(3).to_csv('../Data/Tables/lin_gmm_model_fear_coefficients.csv')

# In[109]:


lin_gmm_model_fear_df

# In[110]:


print_scores(baseline_train_mse_scores, baseline_test_mse_scores, baseline_train_r2_scores, baseline_test_r2_scores)

# In[111]:


plot_performance(target_dependent_var + '_' + target_aggregation, lin_gmm_model_fear, 'lin_gmm_model_fear',
                 X_train[selected_features], y_train, X_test[selected_features], y_test)

# #### Random Forest Regression Model of the Mean Fear Intensity

# In[112]:


rf_gmm_model_fear, train_index, test_index, train_mse_scores, test_mse_scores, train_r2_scores, test_r2_scores = k_fold_training_random_forest_model(
    X_scaled_df.to_numpy(), y, param_grid)

# In[113]:


# Split the data
X_train, X_test, y_train, y_test = X_scaled_df.iloc[train_index], X_scaled_df.iloc[test_index], y.iloc[train_index], \
    y.iloc[test_index]

# In[114]:


rf_gmm_model_fear.get_params()

# In[115]:


rf_gmm_model_fear_df = pd.DataFrame({'Feature': feature_names, 'Importance': rf_gmm_model_fear.feature_importances_})
rf_gmm_model_fear_df.set_index('Feature', inplace=True)

# In[116]:


plot_importances(rf_gmm_model_fear_df, 'rf_gmm_model_fear_importances')

# In[117]:


print_scores(train_mse_scores, test_mse_scores, train_r2_scores, test_r2_scores)

# In[118]:


plot_performance(target_dependent_var + '_' + target_aggregation, rf_gmm_model_fear, 'rf_gmm_model_fear',
                 X_train.to_numpy(), y_train, X_test.to_numpy(), y_test)

# In[119]:


scores.append({'score_type': 'mse_scores', 'dependent_variable': target_dependent_var,
               'baseline_scores': baseline_test_mse_scores, 'scores': test_mse_scores})
scores.append(
    {'score_type': 'r2_scores', 'dependent_variable': target_dependent_var, 'baseline_scores': baseline_test_r2_scores,
     'scores': test_r2_scores})

# #### Selecting the Mean Anger Intensity as Target Variable

# In[120]:


# Select the target dependent variable
target_dependent_var = 'anger_intensity'
y = cleaned_aggregated_data[target_dependent_var + '_' + target_aggregation]

# #### Linear Regression Model of the Mean Anger Intensity

# In[121]:


lin_gmm_model_anger, selected_features, train_index, test_index, baseline_train_mse_scores, baseline_test_mse_scores, baseline_train_r2_scores, baseline_test_r2_scores = k_fold_training_linear_model(
    X_scaled_df, y)

# In[122]:


# Split the data
X_train, X_test, y_train, y_test = X_scaled_df.iloc[train_index], X_scaled_df.iloc[test_index], y.iloc[train_index], \
    y.iloc[test_index]

# In[123]:


lin_gmm_model_anger_df = pd.DataFrame(
    {'Feature': lin_gmm_model_anger.feature_names_in_, 'Coefficient': lin_gmm_model_anger.coef_})
lin_gmm_model_anger_df['absolute_coefficient'] = abs(lin_gmm_model_anger_df['Coefficient'])
lin_gmm_model_anger_df.set_index('Feature', inplace=True)
lin_gmm_model_anger_df.index = lin_gmm_model_anger_df.index.map(lambda feature: titles.get(feature, feature))
lin_gmm_model_anger_df.sort_values(by='absolute_coefficient', ascending=False, inplace=True)
lin_gmm_model_anger_df[['Coefficient']].head(10).round(3).to_csv('../Data/Tables/lin_gmm_model_anger_coefficients.csv')

# In[124]:


lin_gmm_model_anger_df

# In[125]:


print_scores(baseline_train_mse_scores, baseline_test_mse_scores, baseline_train_r2_scores, baseline_test_r2_scores)

# In[126]:


plot_performance(target_dependent_var + '_' + target_aggregation, lin_gmm_model_anger, 'lin_gmm_model_anger',
                 X_train[selected_features], y_train, X_test[selected_features], y_test)

# #### Random Forest Regression Model of the Mean Anger Intensity

# In[127]:


rf_gmm_model_anger, train_index, test_index, train_mse_scores, test_mse_scores, train_r2_scores, test_r2_scores = k_fold_training_random_forest_model(
    X_scaled_df.to_numpy(), y, param_grid)

# In[128]:


# Split the data
X_train, X_test, y_train, y_test = X_scaled_df.iloc[train_index], X_scaled_df.iloc[test_index], y.iloc[train_index], \
    y.iloc[test_index]

# In[129]:


rf_gmm_model_anger.get_params()

# In[130]:


rf_gmm_model_anger_df = pd.DataFrame({'Feature': feature_names, 'Importance': rf_gmm_model_anger.feature_importances_})
rf_gmm_model_anger_df.set_index('Feature', inplace=True)

# In[131]:


plot_importances(rf_gmm_model_anger_df, 'rf_gmm_model_anger_importances')

# In[132]:


print_scores(train_mse_scores, test_mse_scores, train_r2_scores, test_r2_scores)

# In[133]:


plot_performance(target_dependent_var + '_' + target_aggregation, rf_gmm_model_anger, 'rf_gmm_model_anger',
                 X_train.to_numpy(), y_train, X_test.to_numpy(), y_test)

# In[134]:


scores.append({'score_type': 'mse_scores', 'dependent_variable': target_dependent_var,
               'baseline_scores': baseline_test_mse_scores, 'scores': test_mse_scores})
scores.append(
    {'score_type': 'r2_scores', 'dependent_variable': target_dependent_var, 'baseline_scores': baseline_test_r2_scores,
     'scores': test_r2_scores})

# #### Selecting the Mean Disgust Intensity as Target Variable

# In[135]:


# Select the target dependent variable
target_dependent_var = 'disgust_intensity'
y = cleaned_aggregated_data[target_dependent_var + '_' + target_aggregation]

# #### Linear Regression Model of the Mean Disgust Intensity

# In[136]:


lin_gmm_model_disgust, selected_features, train_index, test_index, baseline_train_mse_scores, baseline_test_mse_scores, baseline_train_r2_scores, baseline_test_r2_scores = k_fold_training_linear_model(
    X_scaled_df, y)

# In[137]:


# Split the data
X_train, X_test, y_train, y_test = X_scaled_df.iloc[train_index], X_scaled_df.iloc[test_index], y.iloc[train_index], \
    y.iloc[test_index]

# In[138]:


lin_gmm_model_disgust_df = pd.DataFrame(
    {'Feature': lin_gmm_model_disgust.feature_names_in_, 'Coefficient': lin_gmm_model_disgust.coef_})
lin_gmm_model_disgust_df['absolute_coefficient'] = abs(lin_gmm_model_disgust_df['Coefficient'])
lin_gmm_model_disgust_df.set_index('Feature', inplace=True)
lin_gmm_model_disgust_df.index = lin_gmm_model_disgust_df.index.map(lambda feature: titles.get(feature, feature))
lin_gmm_model_disgust_df.sort_values(by='absolute_coefficient', ascending=False, inplace=True)
lin_gmm_model_disgust_df[['Coefficient']].head(10).round(3).to_csv(
    '../Data/Tables/lin_gmm_model_disgust_coefficients.csv')

# In[139]:


lin_gmm_model_disgust_df

# In[140]:


print_scores(baseline_train_mse_scores, baseline_test_mse_scores, baseline_train_r2_scores, baseline_test_r2_scores)

# In[141]:


plot_performance(target_dependent_var + '_' + target_aggregation, lin_gmm_model_disgust, 'lin_gmm_model_disgust',
                 X_train[selected_features], y_train, X_test[selected_features], y_test)

# #### Random Forest Regression Model of the Mean Disgust Intensity

# In[142]:


rf_gmm_model_disgust, train_index, test_index, train_mse_scores, test_mse_scores, train_r2_scores, test_r2_scores = k_fold_training_random_forest_model(
    X_scaled_df.to_numpy(), y, param_grid)

# In[143]:


# Split the data
X_train, X_test, y_train, y_test = X_scaled_df.iloc[train_index], X_scaled_df.iloc[test_index], y.iloc[train_index], \
    y.iloc[test_index]

# In[144]:


rf_gmm_model_disgust.get_params()

# In[145]:


rf_gmm_model_disgust_df = pd.DataFrame(
    {'Feature': feature_names, 'Importance': rf_gmm_model_disgust.feature_importances_})
rf_gmm_model_disgust_df.set_index('Feature', inplace=True)

# In[146]:


plot_importances(rf_gmm_model_disgust_df, 'rf_gmm_model_disgust_importances')

# In[147]:


print_scores(train_mse_scores, test_mse_scores, train_r2_scores, test_r2_scores)

# In[148]:


plot_performance(target_dependent_var + '_' + target_aggregation, rf_gmm_model_disgust, 'rf_gmm_model_disgust',
                 X_train.to_numpy(), y_train, X_test.to_numpy(), y_test)

# In[149]:


scores.append({'score_type': 'mse_scores', 'dependent_variable': target_dependent_var,
               'baseline_scores': baseline_test_mse_scores, 'scores': test_mse_scores})
scores.append(
    {'score_type': 'r2_scores', 'dependent_variable': target_dependent_var, 'baseline_scores': baseline_test_r2_scores,
     'scores': test_r2_scores})

# #### Selecting the Mean Surprise Intensity as Target Variable

# In[150]:


# Select the target dependent variable
target_dependent_var = 'surprise_intensity'
y = cleaned_aggregated_data[target_dependent_var + '_' + target_aggregation]

# #### Linear Regression Model of the Mean Surprise Intensity

# In[151]:


lin_gmm_model_surprise, selected_features, train_index, test_index, baseline_train_mse_scores, baseline_test_mse_scores, baseline_train_r2_scores, baseline_test_r2_scores = k_fold_training_linear_model(
    X_scaled_df, y)

# In[152]:


# Split the data
X_train, X_test, y_train, y_test = X_scaled_df.iloc[train_index], X_scaled_df.iloc[test_index], y.iloc[train_index], \
    y.iloc[test_index]

# In[153]:


lin_gmm_model_surprise_df = pd.DataFrame(
    {'Feature': lin_gmm_model_surprise.feature_names_in_, 'Coefficient': lin_gmm_model_surprise.coef_})
lin_gmm_model_surprise_df['absolute_coefficient'] = abs(lin_gmm_model_surprise_df['Coefficient'])
lin_gmm_model_surprise_df.set_index('Feature', inplace=True)
lin_gmm_model_surprise_df.index = lin_gmm_model_surprise_df.index.map(lambda feature: titles.get(feature, feature))
lin_gmm_model_surprise_df.sort_values(by='absolute_coefficient', ascending=False, inplace=True)
lin_gmm_model_surprise_df[['Coefficient']].head(10).round(3).to_csv(
    '../Data/Tables/lin_gmm_model_surprise_coefficients.csv')

# In[154]:


lin_gmm_model_surprise_df

# In[155]:


print_scores(baseline_train_mse_scores, baseline_test_mse_scores, baseline_train_r2_scores, baseline_test_r2_scores)

# In[156]:


plot_performance(target_dependent_var + '_' + target_aggregation, lin_gmm_model_surprise, 'lin_gmm_model_surprise',
                 X_train[selected_features], y_train, X_test[selected_features], y_test)

# #### Random Forest Regression Model of the Mean Surprise Intensity

# In[157]:


rf_gmm_model_surprise, train_index, test_index, train_mse_scores, test_mse_scores, train_r2_scores, test_r2_scores = k_fold_training_random_forest_model(
    X_scaled_df.to_numpy(), y, param_grid)

# In[158]:


# Split the data
X_train, X_test, y_train, y_test = X_scaled_df.iloc[train_index], X_scaled_df.iloc[test_index], y.iloc[train_index], \
    y.iloc[test_index]

# In[159]:


rf_gmm_model_surprise.get_params()

# In[160]:


rf_gmm_model_surprise_df = pd.DataFrame(
    {'Feature': feature_names, 'Importance': rf_gmm_model_surprise.feature_importances_})
rf_gmm_model_surprise_df.set_index('Feature', inplace=True)

# In[161]:


plot_importances(rf_gmm_model_surprise_df, 'rf_gmm_model_surprise_importances')

# In[162]:


print_scores(train_mse_scores, test_mse_scores, train_r2_scores, test_r2_scores)

# In[163]:


plot_performance(target_dependent_var + '_' + target_aggregation, rf_gmm_model_surprise, 'rf_gmm_model_surprise',
                 X_train.to_numpy(), y_train, X_test.to_numpy(), y_test)

# In[164]:


scores.append({'score_type': 'mse_scores', 'dependent_variable': target_dependent_var,
               'baseline_scores': baseline_test_mse_scores, 'scores': test_mse_scores})
scores.append(
    {'score_type': 'r2_scores', 'dependent_variable': target_dependent_var, 'baseline_scores': baseline_test_r2_scores,
     'scores': test_r2_scores})

# #### Selecting the Mean Pleasure as Target Variable

# In[165]:


# Select the target dependent variable
target_dependent_var = 'pleasure'
y = cleaned_aggregated_data[target_dependent_var + '_' + target_aggregation]

# #### Linear Regression Model of the Mean Pleasure

# In[166]:


lin_gmm_model_pleasure, selected_features, train_index, test_index, baseline_train_mse_scores, baseline_test_mse_scores, baseline_train_r2_scores, baseline_test_r2_scores = k_fold_training_linear_model(
    X_scaled_df, y)

# In[167]:


# Split the data
X_train, X_test, y_train, y_test = X_scaled_df.iloc[train_index], X_scaled_df.iloc[test_index], y.iloc[train_index], \
    y.iloc[test_index]

# In[168]:


lin_gmm_model_pleasure_df = pd.DataFrame(
    {'Feature': lin_gmm_model_pleasure.feature_names_in_, 'Coefficient': lin_gmm_model_pleasure.coef_})
lin_gmm_model_pleasure_df['absolute_coefficient'] = abs(lin_gmm_model_pleasure_df['Coefficient'])
lin_gmm_model_pleasure_df.set_index('Feature', inplace=True)
lin_gmm_model_pleasure_df.index = lin_gmm_model_pleasure_df.index.map(lambda feature: titles.get(feature, feature))
lin_gmm_model_pleasure_df.sort_values(by='absolute_coefficient', ascending=False, inplace=True)
lin_gmm_model_pleasure_df[['Coefficient']].head(10).round(3).to_csv(
    '../Data/Tables/lin_gmm_model_pleasure_coefficients.csv')

# In[169]:


lin_gmm_model_pleasure_df

# In[170]:


print_scores(baseline_train_mse_scores, baseline_test_mse_scores, baseline_train_r2_scores, baseline_test_r2_scores)

# In[171]:


plot_performance(target_dependent_var + '_' + target_aggregation, lin_gmm_model_pleasure, 'lin_gmm_model_pleasure',
                 X_train[selected_features], y_train, X_test[selected_features], y_test)

# #### Random Forest Regression Model of the Mean Pleasure

# In[172]:


rf_gmm_model_pleasure, train_index, test_index, train_mse_scores, test_mse_scores, train_r2_scores, test_r2_scores = k_fold_training_random_forest_model(
    X_scaled_df.to_numpy(), y, param_grid)

# In[173]:


# Split the data
X_train, X_test, y_train, y_test = X_scaled_df.iloc[train_index], X_scaled_df.iloc[test_index], y.iloc[train_index], \
    y.iloc[test_index]

# In[174]:


rf_gmm_model_pleasure.get_params()

# In[175]:


rf_gmm_model_pleasure_df = pd.DataFrame(
    {'Feature': feature_names, 'Importance': rf_gmm_model_pleasure.feature_importances_})
rf_gmm_model_pleasure_df.set_index('Feature', inplace=True)

# In[176]:


plot_importances(rf_gmm_model_pleasure_df, 'rf_gmm_model_pleasure_importances')

# In[177]:


print_scores(train_mse_scores, test_mse_scores, train_r2_scores, test_r2_scores)

# In[178]:


plot_performance(target_dependent_var + '_' + target_aggregation, rf_gmm_model_pleasure, 'rf_gmm_model_pleasure',
                 X_train.to_numpy(), y_train, X_test.to_numpy(), y_test)

# In[179]:


scores.append({'score_type': 'mse_scores', 'dependent_variable': target_dependent_var,
               'baseline_scores': baseline_test_mse_scores, 'scores': test_mse_scores})
scores.append(
    {'score_type': 'r2_scores', 'dependent_variable': target_dependent_var, 'baseline_scores': baseline_test_r2_scores,
     'scores': test_r2_scores})

# #### Selecting the Mean Arousal as Target Variable

# In[180]:


# Select the target dependent variable
target_dependent_var = 'arousal'
y = cleaned_aggregated_data[target_dependent_var + '_' + target_aggregation]

# #### Linear Regression Model of the Mean Arousal

# In[181]:


lin_gmm_model_arousal, selected_features, train_index, test_index, baseline_train_mse_scores, baseline_test_mse_scores, baseline_train_r2_scores, baseline_test_r2_scores = k_fold_training_linear_model(
    X_scaled_df, y)

# In[182]:


# Split the data
X_train, X_test, y_train, y_test = X_scaled_df.iloc[train_index], X_scaled_df.iloc[test_index], y.iloc[train_index], \
    y.iloc[test_index]

# In[183]:


lin_gmm_model_arousal_df = pd.DataFrame(
    {'Feature': lin_gmm_model_arousal.feature_names_in_, 'Coefficient': lin_gmm_model_arousal.coef_})
lin_gmm_model_arousal_df['absolute_coefficient'] = abs(lin_gmm_model_arousal_df['Coefficient'])
lin_gmm_model_arousal_df.set_index('Feature', inplace=True)
lin_gmm_model_arousal_df.index = lin_gmm_model_arousal_df.index.map(lambda feature: titles.get(feature, feature))
lin_gmm_model_arousal_df.sort_values(by='absolute_coefficient', ascending=False, inplace=True)
lin_gmm_model_arousal_df[['Coefficient']].head(10).round(3).to_csv(
    '../Data/Tables/lin_gmm_model_arousal_coefficients.csv')

# In[184]:


lin_gmm_model_arousal_df

# In[185]:


print_scores(baseline_train_mse_scores, baseline_test_mse_scores, baseline_train_r2_scores, baseline_test_r2_scores)

# In[186]:


plot_performance(target_dependent_var + '_' + target_aggregation, lin_gmm_model_arousal, 'lin_gmm_model_arousal',
                 X_train[selected_features], y_train, X_test[selected_features], y_test)

# #### Random Forest Regression Model of the Mean Arousal

# In[187]:


rf_gmm_model_arousal, train_index, test_index, train_mse_scores, test_mse_scores, train_r2_scores, test_r2_scores = k_fold_training_random_forest_model(
    X_scaled_df.to_numpy(), y, param_grid)

# In[188]:


# Split the data
X_train, X_test, y_train, y_test = X_scaled_df.iloc[train_index], X_scaled_df.iloc[test_index], y.iloc[train_index], \
    y.iloc[test_index]

# In[189]:


rf_gmm_model_arousal.get_params()

# In[190]:


rf_gmm_model_arousal_df = pd.DataFrame(
    {'Feature': feature_names, 'Importance': rf_gmm_model_arousal.feature_importances_})
rf_gmm_model_arousal_df.set_index('Feature', inplace=True)

# In[191]:


plot_importances(rf_gmm_model_arousal_df, 'rf_gmm_model_arousal_importances')

# In[192]:


print_scores(train_mse_scores, test_mse_scores, train_r2_scores, test_r2_scores)

# In[193]:


plot_performance(target_dependent_var + '_' + target_aggregation, rf_gmm_model_arousal, 'rf_gmm_model_arousal',
                 X_train.to_numpy(), y_train, X_test.to_numpy(), y_test)

# In[194]:


scores.append({'score_type': 'mse_scores', 'dependent_variable': target_dependent_var,
               'baseline_scores': baseline_test_mse_scores, 'scores': test_mse_scores})
scores.append(
    {'score_type': 'r2_scores', 'dependent_variable': target_dependent_var, 'baseline_scores': baseline_test_r2_scores,
     'scores': test_r2_scores})

# #### Selecting the Mean Dominance as Target Variable

# In[195]:


# Select the target dependent variable
target_dependent_var = 'dominance'
y = cleaned_aggregated_data[target_dependent_var + '_' + target_aggregation]

# #### Linear Regression Model of the Mean Dominance

# In[196]:


lin_gmm_model_dominance, selected_features, train_index, test_index, baseline_train_mse_scores, baseline_test_mse_scores, baseline_train_r2_scores, baseline_test_r2_scores = k_fold_training_linear_model(
    X_scaled_df, y)

# In[197]:


# Split the data
X_train, X_test, y_train, y_test = X_scaled_df.iloc[train_index], X_scaled_df.iloc[test_index], y.iloc[train_index], \
    y.iloc[test_index]

# In[198]:


lin_gmm_model_dominance_df = pd.DataFrame(
    {'Feature': lin_gmm_model_dominance.feature_names_in_, 'Coefficient': lin_gmm_model_dominance.coef_})
lin_gmm_model_dominance_df['absolute_coefficient'] = abs(lin_gmm_model_dominance_df['Coefficient'])
lin_gmm_model_dominance_df.set_index('Feature', inplace=True)
lin_gmm_model_dominance_df.index = lin_gmm_model_dominance_df.index.map(lambda feature: titles.get(feature, feature))
lin_gmm_model_dominance_df.sort_values(by='absolute_coefficient', ascending=False, inplace=True)
lin_gmm_model_dominance_df[['Coefficient']].head(10).round(3).to_csv(
    '../Data/Tables/lin_gmm_model_dominance_coefficients.csv')

# In[199]:


lin_gmm_model_dominance_df

# In[200]:


print_scores(baseline_train_mse_scores, baseline_test_mse_scores, baseline_train_r2_scores, baseline_test_r2_scores)

# In[201]:


plot_performance(target_dependent_var + '_' + target_aggregation, lin_gmm_model_dominance, 'lin_gmm_model_dominance',
                 X_train[selected_features], y_train, X_test[selected_features], y_test)

# #### Random Forest Regression Model of the Mean Dominance

# In[202]:


rf_gmm_model_dominance, train_index, test_index, train_mse_scores, test_mse_scores, train_r2_scores, test_r2_scores = k_fold_training_random_forest_model(
    X_scaled_df.to_numpy(), y, param_grid)

# In[203]:


# Split the data
X_train, X_test, y_train, y_test = X_scaled_df.iloc[train_index], X_scaled_df.iloc[test_index], y.iloc[train_index], \
    y.iloc[test_index]

# In[204]:


rf_gmm_model_dominance.get_params()

# In[205]:


rf_gmm_model_dominance_df = pd.DataFrame(
    {'Feature': feature_names, 'Importance': rf_gmm_model_dominance.feature_importances_})
rf_gmm_model_dominance_df.set_index('Feature', inplace=True)

# In[206]:


plot_importances(rf_gmm_model_dominance_df, 'rf_gmm_model_dominance_importances')

# In[207]:


print_scores(train_mse_scores, test_mse_scores, train_r2_scores, test_r2_scores)

# In[208]:


plot_performance(target_dependent_var + '_' + target_aggregation, rf_gmm_model_dominance, 'rf_gmm_model_dominance',
                 X_train.to_numpy(), y_train, X_test.to_numpy(), y_test)

# In[209]:


scores.append({'score_type': 'mse_scores', 'dependent_variable': target_dependent_var,
               'baseline_scores': baseline_test_mse_scores, 'scores': test_mse_scores})
scores.append(
    {'score_type': 'r2_scores', 'dependent_variable': target_dependent_var, 'baseline_scores': baseline_test_r2_scores,
     'scores': test_r2_scores})

# ## Hypotheses Testing RQ4

# In[210]:


scores_df = pd.DataFrame(scores)
scores_df.set_index(['score_type', 'dependent_variable'], inplace=True)

# In[211]:


t_test_results_df = t_test(scores_df, dependent_vars)

# In[212]:


selected = []

# In[213]:


selected.append(
    t_test_results_df.loc['mse_scores', :][['baseline_scores', 'scores', 't_statistic', 'p_negative']].rename(
        columns={'t_statistic': 'statistic', 'p_negative': 'p_value'}))
selected.append(
    t_test_results_df.loc['r2_scores', :][['baseline_scores', 'scores', 't_statistic', 'p_positive']].rename(
        columns={'t_statistic': 'statistic', 'p_positive': 'p_value'}))

# In[214]:


rq4_df = pd.concat(selected, axis=0)
rq4_df.index = t_test_results_df.index
rq4_df = pd.concat([rq4_df[['baseline_scores', 'scores']], bonferrroni_correction(rq4_df)], axis=1)
rq4_df.reset_index(inplace=True)
rq4_df['id'] = pd.Series(
    ['rq4.1.1', 'rq4.1.2', 'rq4.1.3', 'rq4.1.4', 'rq4.1.5', 'rq4.1.6', 'rq4.2.1', 'rq4.2.2', 'rq4.2.3',
     'rq4.1.1', 'rq4.1.2', 'rq4.1.3', 'rq4.1.4', 'rq4.1.5', 'rq4.1.6', 'rq4.2.1', 'rq4.2.2', 'rq4.2.3'])
rq4_df.sort_values(by=['id', 'score_type'], inplace=True)
rq4_df.set_index(['id', 'score_type'], inplace=True)
rq4_df['baseline_scores'] = rq4_df.baseline_scores.apply(np.mean)
rq4_df['scores'] = rq4_df.scores.apply(np.mean)
rq4_df = rq4_df.round(3)
rq4_df['reject_h0'] = rq4_df['reject_h0'].astype("string")
rq4_df.drop(columns=['dependent_variable', 'p_value'], inplace=True)
rq4_df.loc[rq4_df['reject_h0'] == 'True', 'reject_h0'] = 'Reject $H_0$'
rq4_df.loc[rq4_df['reject_h0'] == 'False', 'reject_h0'] = 'Fail to reject $H_0$'
rq4_df.columns = ['Average Score Baseline Model', 'Average Score Optimized Model', '$t$-statistic', '$p$-value',
                  'Decision']

# In[215]:


rq4_df

# In[216]:


rq4_df.to_csv('../Data/Tables/rq4.csv')