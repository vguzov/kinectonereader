import numpy as np
from skimage.io import imsave
from os.path import isfile
from PIL import Image, ImageDraw
import sys


def background_remove(vmap, background, surface = None):
    vmap[vmap>=surface] = 0
    vmap[np.logical_and(background>0.01,np.abs(vmap-background)<0.03)]=0
    return vmap

def estimate_surface(background):
    dot1 = background[320,230:250]
    dot2 = background[380,230:250]
    k = np.average((dot2-dot1)/(380-320))
    b = np.average(dot1-320*k)
    column = np.zeros((background.shape[0],1))
    for i in range(column.shape[0]):
        column[i,0] = i*k+b
    surface = np.tile(column,background.shape[1])
    
    dot1 = background[40:55,320]
    dot2 = background[40:55,380]
    k = np.average((dot2-dot1)/(380-320))
    b = np.average(dot1-320*k)-0.05
    row = np.zeros((1,background.shape[1]))
    for i in range(background.shape[1]):
        row[0,i] = i*k+b
    surface = np.minimum(surface,np.tile(row,(background.shape[0],1)))
    return surface


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
    
start_frame=0
frames = 300
zmax = 5000
zmin = 0
if len(sys.argv)<3:
    print("Usage: %s input_folder output_folder [frames]" % (sys.argv[0]))
    sys.exit(1)
folder = sys.argv[1] if sys.argv[1][-1]=='/' else sys.argv[1]+'/'
out_folder = sys.argv[2] if sys.argv[2][-1]=='/' else sys.argv[2]+'/'
if len(sys.argv) >= 4:
    frames = int(sys.argv[3])


# print("Determining minmax")
# for i in range (1,2):
#     plydata = PlyData.read(folder+"output"+str(i)+".ply")
#     verts = plydata['vertex']
#     zmax = max(verts['z'].max(),zmax)
#     zmin = min(verts['z'].min(),zmin)
# print("Min:",zmin,"max:",zmax)
print("Creating pictures")
# plydata = PlyData.read(folder+"background.ply")
# background = verts_to_map(plydata['vertex'],zmin,zmax).copy()
vmap = np.load(folder + "background.npy")
background = (vmap-zmin)/(zmax-zmin)
background = background[:,::-1]
surface = estimate_surface(background)
imsave(out_folder+'background.png',background)
for i in range (1,frames+1):
    print(i,"of",frames)
    vmap = np.load(folder + "output" + str(i+start_frame) + ".npy")
    vmap = (vmap-zmin)/(zmax-zmin)
    vmap = vmap[:,::-1]
    vmap = background_remove(vmap,background,surface)
    if isfile(folder + "output" + str(i+start_frame) + ".txt"):
        points = np.loadtxt(folder + "output" + str(i+start_frame) + ".txt",usecols=(2,3))*2
        draw_n_save(out_folder+str(i+start_frame)+'.png',vmap,points)
    else:
        imsave(out_folder+str(i+start_frame)+'.png',vmap)



