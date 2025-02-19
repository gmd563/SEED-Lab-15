# Grace Davis

# imported libraries:
import numpy as np
from collections import Counter

# Exercise 1 _________________________________________________________________________________
with open('datafile.txt', 'r') as f: # opening datafile.txt
    data_list = eval(f.read()) # reading datafile.txt into a list
    
# [1] The Max --------------------------------------------------------------------------------
print("\n [1] Maximum: ", max(data_list)) # printing the largest number with max() function

# [2] The Min --------------------------------------------------------------------------------
print("\n [2] Minimum: ", min(data_list)) # printing the smallest number with min() function

# [3] Index ----------------------------------------------------------------------------------
print("\n [3] Index: ", data_list.index(38)) # printing the index of 38 with index() function

# [4] Repeated Number and Number of Times Repeated -------------------------------------------
num_of_nums = Counter(data_list) # counting the number of occurrences of each number
repeats = num_of_nums.most_common(7) # finding the most repeated numbers with most_common() function
print("\n [4] 7 most repeated numbers and their number of occurrences [(REPEATED NUMBER, NUMBER OF TIMES REPEATED), . . .]: \n", repeats)

# [5] Array ----------------------------------------------------------------------------------
numpy_array = np.array(data_list) # converting data_list into numpy array
sorted_numpy = np.sort(numpy_array) # sorting numpy array with np.sort() function
print("\n [5] Sorted Numpy Array: \n",sorted_numpy) # printing the sorted numpy array

# [6] All even numbers in order --------------------------------------------------------------
bool_numpy = sorted_numpy % 2 == 0 # creating a boolean list. True = even, False = odd

# creating list from sorted_numpy. Only contains numbers where bool_numpy has "True" elements
evens = sorted_numpy[bool_numpy]

evens_no_repeats = list(set(evens)) # list of all even numbers without repeats.
print("\n [6] Evens number in order: \n", evens)
print("\n [extra] Evens number in order, without repeats: \n", evens_no_repeats)
