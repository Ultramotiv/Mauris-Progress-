import pandas as pd
import numpy as np

def create_baseline_zscore(file_path, threshold=3):
    df = pd.read_csv(file_path)
    
    baseline = pd.DataFrame()
    for col in ['T1', 'T2', 'T3', 'T4', 'T5', 'T6', '1JS', '2JS', '3JS', '4JS', '5JS', '6JS', 'Sx', 'Sy', 'Sz', 'Ax', 'Ay', 'Az']:
        mean = df[col].mean()
        std = df[col].std()
        
        baseline.loc[col, 'Lower'] = mean - threshold * std
        baseline.loc[col, 'Upper'] = mean + threshold * std
    
    print("Baseline Limits using Z-Score Created Successfully")
    return baseline

ed = create_baseline_zscore('robot_data.csv',10)
print(ed)