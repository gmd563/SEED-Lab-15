# Assignment 1 Exercise 1 
# Sophia Mimlitz 

# Import all necessary libraries 
from collections import Counter  
import numpy as np 

# Open and read datafile.txt 
with open('datafile.txt','r') as f:  
nums = eval(f.read()) 

# 1. Maximum using the max() method 
max_value = max(nums)  
print("Maximum: %d\n" % max_value) 

# 2. Minimum using the min() method 
min_value = min(nums)  
print("Minimum: %d\n" % min_value) 

# 3. Index of 38 using the index() method 
index_38 = nums.index(38)  
print("Index of 38: %d\n" % index_38) 

# 4. Most repeated number(s) and number of repetitions 
# Used the Counter library and its methods 
counter = Counter(nums)  
most_repeated = counter.most_common(1)  
print("[(Most repeated number, number of repetitions)]")  
print(most_repeated) 

# 5. A sorted list (as numpy array) 
nums_array = np.array(nums)  
sorted = np.sort(nums_array)  
print("\nSorted list: ")  
print(sorted) 

# 6. All even numbers, in order (also as numpy array) 
evens = sorted[sorted % 2 == 0]  
print("\nAll evens: ") 
print(evens) 
