import numpy as np
from PIL import Image
from PIL import Image, ImageChops, ImageFilter,ImageEnhance

def get_boarder(image_path):
    image = Image.open(image_path)
    image = image.convert('L')  # Convert to grayscale

    # Convert image to numpy array
    data = np.array(image)
    print(data)

    # Find indices where the pixels are black (value 0)
    black_indices = np.where(data == 0)
    print(black_indices)

    # Convert indices to a numpy array and save as .npy file
    black_indices_array = np.array(list(zip(black_indices[0], black_indices[1])))
    np.save('black_indices.npy', black_indices_array)

    print("Indices of black pixels have been saved to black_indices.npy.")


def modify_image_with_saved_slices(image_path, slices_path):
    # Load the image
    img = Image.open(image_path)
    img_array = np.array(img)

    # Load the slices
    indices= np.load(slices_path)

    # Modify the image using the slices
    # img_array[indices] = 0  # For example, setting the border area to black
    img_array[indices[:, 0], indices[:, 1]] = 100

    # Convert array back to image and save or display
    modified_img = Image.fromarray(img_array)
    modified_img.save('modified_img.png')


def add_neighbors_to_indices_separated(npy_file_path, image_shape):
    '''
    Example usage:
    image_dimensions = (320,240)
    add_neighbors_to_indices_separated('black_indices.npy', image_dimensions)
    '''
    # Load the original indices from the .npy file
    original_indices = np.load(npy_file_path)
    
    # Convert to a set of tuples for easy addition of neighbors
    indices_set = set(tuple(index) for index in original_indices)  # Combine x and y into tuples
    
    # Directions representing the 8 neighbors
    directions = [(di, dj) for di in range(-1, 2) for dj in range(-1, 2) if not (di == 0 and dj == 0)]
    # directions = [(di, dj) for di in range(-2, 3) for dj in range(-2, 3) if not (di == 0 and dj == 0)]
    # directions = [(di, dj) for di in range(-3, 4) for dj in range(-3, 4) if not (di == 0 and dj == 0) and abs(di) < 4 and abs(dj) < 4]
    
    new_indices_set = set(indices_set)  # Start with a copy of the original indices

    for index in indices_set:
        i, j = index
        for di, dj in directions:
            ni, nj = i + di, j + dj
            # Check if the new index is within the image bounds
            if 0 <= ni < image_shape[0] and 0 <= nj < image_shape[1]:
                new_indices_set.add((ni, nj))

    # Convert the set of tuples back to separate arrays for x and y
    expanded_indices_array = np.array(list(new_indices_set))
    
    # Save the expanded indices to a new .npy file
    np.save(npy_file_path.replace('.npy', '_expanded.npy'), expanded_indices_array)


def process_image_rgba(input_array, static_img_path, default_img_path, radius=5):
    # Convert the input RGBA array to an RGB Image
    if input_array.shape[2] == 4:  # Check if the input array has four channels
        input_array = input_array[:, :, :3]  # Drop the alpha channel
    # input_array = input_array[:, :, ::-1]
    input_img = Image.fromarray(input_array, 'RGB')
    input_img.save('input_img.png')

    # Load the static and default images, and convert them to RGB
    static_img = Image.open(static_img_path).convert('RGB')
    default_img = Image.open(default_img_path).convert('RGB')

    # Subtract the static image from the input image
    subtracted_img = ImageChops.subtract(input_img, static_img)
    enhancer = ImageEnhance.Brightness(subtracted_img)
    subtracted_img = enhancer.enhance(1)
    subtracted_img.save('subtracted_img.png')
    # subtracted_img = subtracted_img.filter(ImageFilter.GaussianBlur(radius=radius))
    # Add the result to the default image
    result_img = ImageChops.add(subtracted_img, default_img)
    result_array = np.array(result_img)

    # Add noise to the result to make it look more realistic

    gauss = np.random.normal(0, 7, result_array.shape)
    result_array = np.clip((result_array+gauss), 0, 255).astype(np.uint8)
    result_img = Image.fromarray(result_array)

    return result_img



