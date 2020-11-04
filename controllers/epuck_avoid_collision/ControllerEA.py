
print("Hello")
import csv

def main():
    f = open("EAToController.txt", "x")
    NNValues = [1, 2]
    for i in range(len(NNValues)):
        f.write(str(NNValues[i]) + ", ")
    print(f.read())
    f.close()

main()

