# Assignment 1 Exercise 2
# Sophia Mimlitz

# Go through states
# Wait for 'a'
def state0(char):
    if char == 'a':
        return state1
    else:
        return state0

def state1(char):
    if char == 'b':
        return state2
    else:
        return state0(char)

def state2(char):
    if char == 'c':
        return state3
    else:
        return state0(char)

def state3(char):
    if char == 'd':
        print("abcd is contained in the string")
        return state4(char)
    else:
        return state0(char)

def state4(char):
    return state0(char)

# Create a state dictionary
state_dictionary = {
    state0: "wait",
    state1: "a",
    state2: "ab",
    state3: "abc",
    state4: "abcd"
    }

# Initialization
state = state0
string_input = input("Enter a string: ")
contained = 0
for char in string_input:
    new_state = state(char)
    state = new_state
    if state == state4(char):
        contained = 1
if contained == 0:
    print("abcd is not contained in the string")
