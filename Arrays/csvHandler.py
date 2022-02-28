


#class to handle csv read/write 


import os
import pandas as pd



#store column in csv
def store(results_to_be_stored):
    df = pd.DataFrame(results_to_be_stored)
    df.to_csv('store_test')
    print("stored")


#returns the column we want as a numpy array 
def read(column_to_be_read):
    path = os.path.expanduser('~') + "/dataset1.csv"
    df = pd.read_csv(path, encoding= 'utf-8') #static
    df.to_numpy()
    #selecting columns
    return df[[column_to_be_read + ""]].to_numpy() #position estimate x

