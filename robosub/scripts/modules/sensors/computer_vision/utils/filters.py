import numpy as np

''' BASIC KERNEL '''

#kernel = np.ones((3, 3), np.float32)
#kernel = np.ones((3, 3), np.float32) / 9

kernel = np.ones((5, 5), np.uint8)
kernel = np.ones((5, 5), np.float32) / 25

#kernel = cv2.flip(kernel_1, -1)

''' VERTICAL 3x3 '''
vert = [
    [1, 0, -1],
    [1, 0, -1],
    [1, 0, -1]
]
''' HORIZ 3x3 '''
horiz = [
    [1, 1, 1],
    [0, 0, 0],
    [-1, -1, -1]
]

kernel_vert = np.array(vert)
kernel_horiz = np.array(horiz)
#kernel_vert = np.array(vert)/9
#kernel_horiz = np.array(horiz)/9

# kernel_vert = cv2.flip(kernel_vert)
# kernel_horiz = cv2.flip(kernel_horiz)


# 5x5 normal
vert = [
    [1, 1, 0, -1, -1],
    [1, 1, 0, -1, -1],
    [1, 1, 0, -1, -1],
    [1, 1, 0, -1, -1],
    [1, 1, 0, -1, -1]
]

horiz = [
    [1, 1, 1, 1, 1],
    [1, 1, 1, 1, 1],
    [0, 0, 0, 0, 0],
    [-1, -1, -1, -1, -1],
    [-1, -1, -1, -1, -1]
]

# kernel_vert = np.array(vert)
# kernel_horiz = np.array(horiz)
kernel_vert = np.array(vert)/25
kernel_horiz = np.array(horiz)/25

# kernel_vert = cv2.flip(kernel_vert)
# kernel_horiz = cv2.flip(kernel_horiz)

# 5x5 thicc

vert = [
    [1, 0, 0, 0, -1],
    [1, 0, 0, 0, -1],
    [1, 0, 0, 0, -1],
    [1, 0, 0, 0, -1],
    [1, 0, 0, 0, -1]
]

horiz = [
    [1, 1, 1, 1, 1],
    [0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0],
    [-1, -1, -1, -1, -1]
]

kernel_vert = np.array(vert)
kernel_horiz = np.array(horiz)
#kernel_vert = np.array(vert)/25
#kernel_horiz = np.array(horiz)/25

# kernel_vert = cv2.flip(kernel_vert)
# kernel_horiz = cv2.flip(kernel_horiz)

# 3x3

sorbel_y = [
    [1, 0, -1],
    [2, 0, -2],
    [1, 0, -1]
]

sorbel_x = [
    [1, 2, 1],
    [0, 0, 0],
    [-1, -2, -1]
]

# kernel_sorbel_y = np.array(sorbel_y)
# kernel_sorbel_x = np.array(sorbel_x)

kernel_sorbel_y = np.array(sorbel_y)/9
kernel_sorbel_x = np.array(sorbel_x)/9

# kernel_sorbel_y = cv2.flip(kernel_sorbel_y)
# kernel_sorbel_x = cv2.flip(kernel_sorbel_x)

# 3x3

diag_pos = [
    [0, 1, 0],
    [1, 0, -1],
    [0, -1, 0]
]

diag_neg = [
    [0, 1, 0],
    [-1, 0, 1],
    [0, -1, 0]
]

kernel_diag_pos = np.array(diag_pos)
kernel_diag_neg = np.array(diag_neg)
#kernel_diag_pos = np.array(diag_pos)/9
#kernel_diag_neg = np.array(diag_neg)/9

# kernel_diag_pos = cv2.flip(kernel_diag_pos)
# kernel_diag_neg = cv2.flip(kernel_diag_neg)

# 5x5

diag_pos = [
    [0, 1, 1, 1, 0],
    [1, 0, 1, 0, -1],
    [1, 1, 0, -1, -1],
    [1, 0, -1, 0, -1],
    [0, -1, -1, -1, 0]
]

diag_neg = [
    [ 0,  1,  1,  1, 0],
    [-1,  0,  1,  0, 1],
    [-1, -1,  0,  1, 1],
    [-1,  0, -1,  0, 1],
    [ 0, -1, -1, -1, 0]
]

kernel_diag_pos = np.array(diag_pos)
kernel_diag_neg = np.array(diag_neg) # best
#kernel_diag_pos = np.array(diag_pos)/25
#kernel_diag_neg = np.array(diag_neg)/25

# kernel_diag_pos = cv2.flip(kernel_diag_pos)
# kernel_diag_neg = cv2.flip(kernel_diag_neg)

def get_filter(img, kernel):
    return cv2.filter2D(img, -1, kernel)
