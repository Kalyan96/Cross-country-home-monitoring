import cv2

# Check for GPU availability
if cv2.cuda.getCudaEnabledDeviceCount() == 0:
    print("No GPU found. Falling back to CPU.")
else:
    print("Found a GPU at:", cv2.cuda.getDeviceName(0))

    # Read the image with OpenCV
    image = cv2.imread('image.jpg')

    # Upload the image to the GPU
    gpu_image = cv2.cuda_GpuMat()
    gpu_image.upload(image)

    # Convert the image to gray scale
    gpu_gray = cv2.cuda.cvtColor(gpu_image,  cv2.COLOR_BGR2GRAY)

    # Download the result back to the CPU
    gray_image = gpu_gray.download()

    # Show the CPU image
    cv2.imshow('Gray Image', gray_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
