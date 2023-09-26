import random

def fakeData():
    # Generate random integers for x, y, and z
    x = random.randint(-10000, 10000)
    y = random.randint(-10000, 10000)
    z = random.randint(-10000, 10000)

# Create the message string
    message = f"N3 U0 {x} {y} {z} 100 100 1"
    return message