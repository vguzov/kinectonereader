from skimage.io import imread, imsave
folder = './images/'
for i in range(1,301):
    img = imread(folder+str(i)+'.png',plugin = "matplotlib")
    imsave(folder+'f'+str(i)+'.png',img[:424,:512])