#Ryan Cockrill
#Assignment 1 part 1
#Does a bunch of random stuff. 6 different things. 

#Imports
from collections import Counter
import numpy as np

#Given Setup
with open("datafile.txt", "r") as f:
    nums = eval(f.read())

#1
max_value = max(nums)
print("Maximum: %d\n" % max_value)

#2
min_value = min(nums)
print("Minimum: %d\n" % min_value)

#3
index_38 = nums.index(38)
print("Index at 38: %d\n" % index_38)

#4
counter = Counter(nums)
#max_count = max(counter.values())
most_repeated = counter.most_common(1)
print("[(Most repeated number, number of repititions)]")
print(most_repeated)

#5
nums_array = np.array(nums)
sorted = np.sort(nums_array)
print("\nSorted list: ")
print(sorted)
      
#6
evens = sorted[sorted % 2 ==0]
print("\nAll evens: ")
print(evens)
