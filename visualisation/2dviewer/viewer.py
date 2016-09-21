import numpy as np
from skimage.io import imsave
from plyfile import PlyData, PlyElement
from os.path import isfile
from PIL import Image, ImageDraw

KINECT_SHAPE = (480,640)


def verts_to_map(verts, order,zmin,zmax):
    vmap = np.zeros(KINECT_SHAPE)
    verts = np.vstack([verts[l] for l in ['x','y','z']]).transpose()
    for vert in verts:
        vmap[vert[1]*order[0],vert[0]*order[1]] = (vert[2]-zmin)/(zmax-zmin)
    return vmap
skel_lines=[
    #Torso
    (3,2),
    (2,4),
    (2,8),
    (2,1),
    (1,0),
    (0,12),
    (0,16),
    #Left arm
    (4,5),
    (5,6),
    (6,7),
    #Right arm
    (8,9),
    (9,10),
    (10,11),
    #Left leg
    (12,13),
    (13,14),
    (14,15),
    #Right leg
    (16,17),
    (17,18),
    (18,19)
]
def draw_n_save(filename,img,points):
    radius = 0.008*KINECT_SHAPE[0]
    point_color = (255,0,0)
    line_color = (0,255,0)
    line_width = 1
    img = (img*255).astype(np.uint8).copy()
    pil_img = Image.fromarray(img).convert(mode="RGB")
    pil_draw = ImageDraw.Draw(pil_img)
    for line in skel_lines:
        coords = np.hstack((points[line[0],:],points[line[1],:]))
        pil_draw.line((coords[0],coords[1],coords[2],coords[3]),fill=line_color,width=line_width)
    for j in range(points.shape[0]):
        pt = points[j,:]
        pil_draw.ellipse((pt[0]-radius,pt[1]-radius,pt[0]+radius,pt[1]+radius),fill=point_color)
    pil_img.save(filename)

folder = "/Users/Vladimir/cv/kinectonereader/src/output/"
out_folder = './images/'
start_frame=0
frames = 300
zmax = 5000
zmin = 0
# print("Determining minmax")
# for i in range (1,2):
#     plydata = PlyData.read(folder+"output"+str(i)+".ply")
#     verts = plydata['vertex']
#     zmax = max(verts['z'].max(),zmax)
#     zmin = min(verts['z'].min(),zmin)
# print("Min:",zmin,"max:",zmax)
print("Creating pictures")
plydata = PlyData.read(folder+"background.ply")
vertexorder = [-1 if (plydata['vertex'][l].min()<0) else 1 for l in ['y','x']]
background = verts_to_map(plydata['vertex'],vertexorder,zmin,zmax).copy()
for i in range (1,frames+1):
    print(i,"of",frames)
    plydata = PlyData.read(folder + "output" + str(i+start_frame) + ".ply")
    verts = plydata['vertex']
    vmap = verts_to_map(verts, vertexorder,zmin,zmax)
    if isfile(folder + "output" + str(i+start_frame) + ".txt"):
        points = np.loadtxt(folder + "output" + str(i+start_frame) + ".txt",usecols=(2,3))*2
        draw_n_save(out_folder+str(i+start_frame)+'.png',vmap,points)
    else:
        imsave(out_folder+str(i+start_frame)+'.png',vmap)



