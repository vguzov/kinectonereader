import numpy as np
from skimage.io import imsave
from os.path import isfile
from PIL import Image, ImageDraw
import sys
import os

start_frame=0
frames = 100
zmax = 5000
zmin = 0
zthresh = (1000,6000)
KINECT_SHAPE = (424,512)
zmim,zmax=zthresh

def background_remove_oldest(vmap, background, surface = None):
    vmap[vmap>=surface] = 0
    vmap[np.logical_and(background>0.01,np.abs(vmap-background)<0.03)]=0
    return vmap
	
def background_remove_old(vmap, points):
    h,w = vmap.shape
    nblist = [(points[1,1],w-points[1,0]-1)]
    for pt in nblist:
        print(pt)
        vcty = np.arange(-pt[0],h-pt[0])
        vctx = np.arange(-pt[1],w-pt[1])
        dists=vcty[:,np.newaxis]**2+vctx**2+((vmap[pt[1],pt[0]]-vmap)*1000)**2
        dists = np.sqrt(dists)
        cnd = np.argwhere(dists<100)
        nblist += list(cnd)
        break
#        for c in cnd:
#            if not (tuple(c) in nblist):
#                nblist.append(tuple(c))
    newmap = np.zeros((h,w))
    for pt in nblist:
        newmap[pt[0],pt[1]] = vmap[pt[0],pt[1]]
    return newmap
        
def background_remove(vmap, points):
    h,w=vmap.shape
    wfpts = np.zeros((h,w),np.bool)
    nhpts = np.zeros((h,w),np.bool)
    nhpts_old = nhpts.copy()
    nhpts[points[1,1],w-points[1,0]-1]=True
    #while not np.array_equal(nhpts,nhpts_old):
    for i in range(2):
        cnd = np.argwhere(np.logical_xor(nhpts,wfpts))
        nhpts_old = nhpts.copy()
        for pt in cnd:
            vcty = np.arange(-pt[0],h-pt[0])
            vctx = np.arange(-pt[1],w-pt[1])
            dists=vcty[:,np.newaxis]**2+vctx**2+((vmap[pt[0],pt[1]]-vmap)*1000)**2
            nhpts = np.logical_or(nhpts,dists<10**2)
        wfpts = nhpts_old.copy()
    return np.where(nhpts,vmap,np.zeros((h,w)))
            
    
def load_npy(path):
    arr = np.load(path).astype(np.float32)
    arr[arr>zthresh[1]]=zthresh[1]
    arr[arr<zthresh[0]]=zthresh[0]
    return arr

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
    points[:,0] = KINECT_SHAPE[1]-points[:,0]
    radius = 0.008*KINECT_SHAPE[0]
    point_color = (255,0,0)
    line_color = (0,255,0)
    line_width = 1
    img = (img*255).astype(np.uint8).copy()
    pil_img = Image.fromarray(img).convert(mode="RGB")
    pil_draw = ImageDraw.Draw(pil_img)
    for line in skel_lines:
        coords = np.hstack((points[line[0],:],points[line[1],:]))
        if not np.isnan(coords).any():
            pil_draw.line((coords[0],coords[1],coords[2],coords[3]),fill=line_color,width=line_width)
    for j in range(points.shape[0]):
        pt = points[j,:]
        if not np.isnan(pt).any():    
            pil_draw.ellipse((pt[0]-radius,pt[1]-radius,pt[0]+radius,pt[1]+radius),fill=point_color)
    pil_img.save(filename)
    
def txtconvert(text):
    if text==b'-1.#J':
        return np.nan
    else:
        return float(text)

if len(sys.argv)<3:
    print("Usage: %s input_folder output_folder [frames]" % (sys.argv[0]))
    sys.exit(1)
#folder = sys.argv[1] if sys.argv[1][-1]=='/' else sys.argv[1]+'/'
#out_folder = sys.argv[2] if sys.argv[2][-1]=='/' else sys.argv[2]+'/'
folder = sys.argv[1]
out_folder = sys.argv[2]
if len(sys.argv) >= 4:
    frames = int(sys.argv[3])


#print("Determining minmax")
#imarr = np.zeros((frames,KINECT_SHAPE[0],KINECT_SHAPE[1]))
#for i in range(1,frames+1):
#    imarr[i-1,:,:] = np.load(os.path.join(folder,"output" + str(i+start_frame) + ".npy")).astype(np.float32)
#imarr[imarr==-1]=float('nan')
#zthresh = (np.nanpercentile(imarr, 10),np.nanpercentile(imarr, 99.5))
zmin,zmax = zthresh
print("Min:",zmin,"max:",zmax)
print("Creating pictures")
vmap = load_npy(os.path.join(folder, "background.npy"))
background = (vmap-zmin)/(zmax-zmin)
background = background[:,::-1]
#surface = estimate_surface(background)
imsave(os.path.join(out_folder,'background.png'),background)
for i in range (1,frames+1):
    print(i,"of",frames)
    vmap = load_npy(os.path.join(folder,"output" + str(i+start_frame) + ".npy"))
    vmap = (vmap-zmin)/(zmax-zmin)
    vmap = vmap[:,::-1]
    #vmap = background_remove(vmap,background,surface)
    if isfile(os.path.join(folder,"output" + str(i+start_frame) + ".txt")):
        points = np.loadtxt(os.path.join(folder, "output" + str(i+start_frame) + ".txt"),
                            usecols=(2,3),comments=';',converters={2:txtconvert,3:txtconvert})
        vmap = background_remove(vmap,points)
        draw_n_save(os.path.join(out_folder,str(i+start_frame)+'.png'),vmap,points)
    else:
        imsave(os.path.join(out_folder,str(i+start_frame)+'.png'),vmap)



