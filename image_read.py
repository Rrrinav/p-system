from sys import float_repr_style
import cv2

PATH = "./nvim.png"
SIZE = (1000, 900)


image = cv2.imread(PATH)

IMAGE_SIZE = (image.shape[1], image.shape[0])

if image is None:
    print("Error: Image not found or could not be loaded.")
    exit()


image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)


def getColor(line):
    coor = line.strip("\n").split(" ")

    x = float(coor[0])
    y = float(coor[1])

    ratioX = IMAGE_SIZE[0] / SIZE[0]
    ratioY = IMAGE_SIZE[1] / SIZE[1]

    coorX = int(x * ratioX)
    coorY = int(y * ratioY)

    return image_rgb[coorY][coorX]


f = open("./pp_position.txt", "r")

lines = f.readlines()

colors = []

for i in range(len(lines)):
    colors.append(getColor(lines[i]))

f.close()

fw = open("./pp_color.txt", "w")

for i in range(len(colors)):
    fw.write(str(colors[i][0]) + " " + str(colors[i][1]) + " " + str(colors[i][2]) + "\n")
