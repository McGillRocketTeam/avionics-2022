


#class to handle csv read/write 


import pandas as pd


#store columns in csv
def store(results_to_be_stored, column):
    df = pd.DataFrame(results_to_be_stored)
    df.to_csv(str(column) + '.csv')
    print("stored")


#returns the column we want as a numpy array 
def read(column_to_be_read, column):
    # yes, I know, a relative path would have been better 
    path = "C:/Users/Dell/McGillRocketTeam/avionics-2022/Arrays/" + str(column) +".csv"
    df = pd.read_csv(path, encoding= 'utf-8') #static
    df.to_numpy()
    #selecting columns
    return df[str(column_to_be_read)].to_numpy() 

