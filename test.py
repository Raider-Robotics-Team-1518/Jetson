
distance_table = [12.7, 15.2, 18.1, 23.9, 30.0, 35.8, 43.0, 49.7]
pixel_table = [454, 375, 319, 241, 192, 161, 134, 116]

def get_pixels_per_inch(distance):
    index = 0
    for i, dist in enumerate(distance_table):
        if distance < dist:
            index = i
            break
    if index == 0:
        return pixel_table[0]
    ratio = (distance - distance_table[index - 1]) / (distance_table[index] - distance_table[index - 1])
    px_scale = ratio * (pixel_table[index] - pixel_table[index - 1]) + pixel_table[index - 1]
    return px_scale


# for i in range(12, 50):
#     px = get_pixels_per_inch(i)
#     print(i, px)

for val in pixel_table:
    print(val / 12)