import pandas

def read(path):
     d = pandas.read_csv(path, index_col=0)
     return d.T
