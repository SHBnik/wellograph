from numpy import genfromtxt
my_data = genfromtxt('/home/shb/Desktop/Gaavkhouni.csv', delimiter=',')
print my_data[32][0]
