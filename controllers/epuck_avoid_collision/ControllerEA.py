
print("Hello")
import csv

def main():
    f = open("EAToController.txt", "w")
    NNValues = [1, 2]
    for i in range(len(NNValues)):
        f.write(str(NNValues[i]) + ", ")
    

main()

