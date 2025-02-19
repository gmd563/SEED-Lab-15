# Grace Davis

# Exercise 2 _________________________________________________________________________________
def state0(char): # defining state0
  if char == "a":
    return stateA
  else:
    return state0
    
def stateA(char): # defining stateA
  if char == "b":
    return stateB
  else:
    return state0(char)
    
def stateB(char): # defining stateB
  if char == "c":
    return stateC
  else:
    return state0(char)

def stateC(char): # defining stateC
  if char == "d":
    print("\n\n abcd is contained in the string")
    return stateD
  else:
    return state0(char)
    
def stateD(char): # defining stateD
  # return state0 to re-initialize the search process for multiple occurrences of “abcd”
  return state0(char)
  state_dict = { #create dictionary
    state0 : "state0",
    stateA : "stateA",
    stateB : "stateB",
    stateC : "stateC",
    stateD : "stateD"
  }
# initialization - start at initial state (pointer to state0 as function)
state = state0
print("\nType your string below:")
string = input("\nInput string: ")

for char in string: # checking every character contained in the string
  new_state = state(char) # variable to contain current state of character
  state = new_state
  print("\n\n Done with state machine\n\n")
  
