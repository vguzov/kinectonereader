# -*- coding: utf-8 -*-
from OpenGL.GL import *
from OpenGL.GLUT import *
from plyfile import PlyData, PlyElement
import sys
import numpy as np
import scipy.misc
import transforms as tr
import struct
import pickle
import socket


windowsize = (500, 500)


class RotMtr:
    def __init__(self):
        self.reset()

    def reset(self):
        self.x=0
        self.y=0
        self.z=0

    def get_mtr(self):
        return get_rotation(self.x,self.y,self.z)


class Camera:
    def __init__(self):
        self.view = np.eye(4, dtype=np.float32)
        self.projection = np.eye(4, dtype=np.float32)
        
def normalize(v):
    norm=np.sum(v**2)
    if norm==0: 
       return v
    return v/np.sqrt(norm)

def draw_ply(plypath,pos=(0,0,0),size=0.1,color=(0,1,0,1),rotation=0):
    global program
    plydata = PlyData.read(plypath)
    verts = plydata['vertex']
    #verts = np.vstack((verts['x'],verts['y'],verts['z'])).transpose()
    verts = np.vstack([(verts[l]-verts[l].min())/(verts[l].max()-verts[l].min())*2-1 for l in ['x','y','z']]).transpose().astype(np.float32).copy()
    # verts = verts*size
    
    indices = np.zeros((0),dtype=np.uint32)
    norms = np.zeros((verts.shape[0],3))
    norms[:,1]=1
    for face in plydata['face']['vertex_indices']:
        indices = np.hstack((indices,face.astype(np.uint32)))
        
        norms[face[1],:] = normalize(np.cross(verts[face[0]]-verts[face[1]],verts[face[2]]-verts[face[1]]))
    #indices = np.arange(0,indices.shape[0],dtype = np.uint32)
    vertsbuf = glGenBuffers(1)
    indicesbuf = glGenBuffers(1)
    normsbuf = glGenBuffers(1)
    model = np.eye(4, dtype=np.float32)
    tr.rotate(model, rotation, 0.0, 1.0, 0.0)
    tr.translate(model,pos[0],pos[1],pos[2])
    
    # print(verts)
    # print(indices)
    # print(norms)
    
    offset = ctypes.c_void_p(0)
    glBindBuffer(GL_ARRAY_BUFFER, vertsbuf)
    glBufferData(GL_ARRAY_BUFFER, verts.nbytes, verts, GL_DYNAMIC_DRAW)
    stride = verts.strides[0]
    loc = glGetAttribLocation(program, "vertex")
    glEnableVertexAttribArray(loc)
    glVertexAttribPointer(loc, 3, GL_FLOAT, False, stride, offset)

    glBindBuffer(GL_ARRAY_BUFFER, normsbuf)
    glBufferData(GL_ARRAY_BUFFER, norms.nbytes, norms, GL_DYNAMIC_DRAW)
    stride = norms.strides[0]
    loc = glGetAttribLocation(program, "normal")
    glEnableVertexAttribArray(loc)
    glVertexAttribPointer(loc, 3, GL_FLOAT, False, stride, offset)

    loc = glGetUniformLocation(program, "objectMatrix")
    glUniformMatrix4fv(loc, 1, GL_FALSE, model)

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indicesbuf)
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.nbytes, indices, GL_DYNAMIC_DRAW)
    loc = glGetUniformLocation(program, "vertex_color")
    glUniform4f(loc, color[0], color[1], color[2], color[3])

    glDrawElements(GL_TRIANGLES, indices.shape[0], GL_UNSIGNED_INT, ctypes.c_void_p(0))
    
class Cube:
    def __init__(self, pos, size, color, rotation=0):
        self.verts, self.indices, self.norms = create_cube((0,0,0),size)
        self.vertsbuf = glGenBuffers(1)
        self.indicesbuf = glGenBuffers(1)
        self.normsbuf = glGenBuffers(1)
        self.color = np.array(color, dtype=np.float32)
        self.model = np.eye(4, dtype=np.float32)
        tr.rotate(self.model, rotation, 0.0, 1.0, 0.0)
        tr.translate(self.model,pos[0],pos[1],pos[2])

    def update(self, pos, size, color, rotation=0):
        self.verts, self.indices, self.norms = create_cube((0,0,0),size)
        self.color = np.array(color, dtype=np.float32)
        self.model = np.eye(4, dtype=np.float32)
        tr.rotate(self.model, rotation, 0.0, 1.0, 0.0)
        tr.translate(self.model,pos[0],pos[1],pos[2])
        
    # def __del__(self):
        # glDeleteBuffers(1,self.vertsbuf)
        # glDeleteBuffers(1,self.normsbuf)
        # glDeleteBuffers(1,self.indicesbuf)

    def draw(self):
        offset = ctypes.c_void_p(0)
        glBindBuffer(GL_ARRAY_BUFFER, self.vertsbuf)
        glBufferData(GL_ARRAY_BUFFER, self.verts.nbytes, self.verts, GL_DYNAMIC_DRAW)
        stride = self.verts.strides[0]
        loc = glGetAttribLocation(program, "vertex")
        glEnableVertexAttribArray(loc)
        glVertexAttribPointer(loc, 3, GL_FLOAT, False, stride, offset)

        glBindBuffer(GL_ARRAY_BUFFER, self.normsbuf)
        glBufferData(GL_ARRAY_BUFFER, self.norms.nbytes, self.norms, GL_DYNAMIC_DRAW)
        stride = self.norms.strides[0]
        loc = glGetAttribLocation(program, "normal")
        glEnableVertexAttribArray(loc)
        glVertexAttribPointer(loc, 3, GL_FLOAT, False, stride, offset)

        loc = glGetUniformLocation(program, "objectMatrix")
        glUniformMatrix4fv(loc, 1, GL_FALSE, self.model)

        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, self.indicesbuf)
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, self.indices.nbytes, self.indices, GL_DYNAMIC_DRAW)
        loc = glGetUniformLocation(program, "vertex_color")
        glUniform4f(loc, self.color[0], self.color[1], self.color[2], self.color[3])

        glDrawElements(GL_TRIANGLES, 36, GL_UNSIGNED_INT, ctypes.c_void_p(0))

rotation = RotMtr()


def randfromto(a,b):
    return a+(b-a)*np.random.random()


def generate_cubes(count, posrect, scales):
    cubes = []
    for i in range(count):
        scale = scales[np.random.randint(len(scales))]
        rot = randfromto(0,90)
        curr_rect = (posrect[0]+scale[0]*1.5,posrect[1]+scale[2]*1.5,
                     posrect[2]-scale[0]*1.5,posrect[3]-scale[2]*1.5)
        pos = (randfromto(curr_rect[0],curr_rect[2]),0,randfromto(curr_rect[1],curr_rect[3]))
        color = [np.random.random(), np.random.random(), np.random.random(), 1]
        cubes.append(Cube(pos,scale,color,rot))
    return cubes


def handlekeys(key, x, y):
    global rotation
    val = 5
    if key == GLUT_KEY_UP:
        rotation.x += val
    if key == GLUT_KEY_DOWN:
        rotation.x -= val
    if key == GLUT_KEY_LEFT:
        rotation.y+=val
    if key == GLUT_KEY_RIGHT:
        rotation.y-=val
    if key == 97:
        tr.translate(camera.view,0.3,0,0)
    if key == 106:
        tr.translate(camera.view,-0.3,0,0)
    if key == 115:
        tr.translate(camera.view,0,0,-0.3)
    if key == 119:
        tr.translate(camera.view,0,0,0.3)
    if key == GLUT_KEY_F1:
        img = np.zeros((windowsize[0], windowsize[1], 3),np.uint8)
        glReadPixels(0,0,windowsize[0], windowsize[1], GL_RGB, GL_UNSIGNED_BYTE, img)
        scipy.misc.imsave('picture.png', np.flipud(img))
    if key == GLUT_KEY_F2:
        sys.exit(0)


def create_shader_from_text(shader_type, source):
    shader = glCreateShader(shader_type)
    glShaderSource(shader, source)
    glCompileShader(shader)
    return shader


def create_shader(shader_type, path):
    shader = glCreateShader(shader_type)
    f = open(path, 'r')
    source = f.readlines()
    glShaderSource(shader, source)
    f.close()
    glCompileShader(shader)
    result = glGetShaderiv(shader, GL_COMPILE_STATUS)
    if not(result):
        raise RuntimeError(glGetShaderInfoLog(shader))
    return shader


def create_program(vertex_scr, fragment_scr):
    program = glCreateProgram()
    vertex = create_shader(GL_VERTEX_SHADER, vertex_scr)
    fragment = create_shader(GL_FRAGMENT_SHADER, fragment_scr)
    glAttachShader(program, vertex)
    glAttachShader(program, fragment)
    glLinkProgram(program)
    result = glGetProgramiv(program, GL_LINK_STATUS)
    if not(result):
        raise RuntimeError(glGetProgramInfoLog(program))
    return program


def reshape(width,height):
    global camera, windowsize
    glViewport(0, 0, width, height)
    camera.projection = tr.perspective(50.0, width/float(height), 2.0, 10.0)
    loc = glGetUniformLocation(program, "projectionMatrix")
    glUniformMatrix4fv(loc, 1, GL_FALSE, camera.projection)
    windowsize = (width, height)


def draw():
    global camera, scene_objects, scales, command_socket, command_conn, default_box, frame_counter

    # data = command_conn.recv(1024)
#     if not data:
#         print("Closing connection")
#         command_conn.close()
#         print("Waiting for connection...")
#         command_conn, reply_addr = command_socket.accept()
#         print("Client connected: ", reply_addr)
#         data = command_conn.recv(1024)
#     plypath = pickle.loads(data)
    #plypath = "./output/output88.ply"
    plypath = "bunny2.ply"
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT)
    glEnable(GL_DEPTH_TEST)
    tr.rotate(camera.view,rotation.x,1,0,0)
    tr.rotate(camera.view,rotation.y,0,1,0)
    rotation.reset()
    loc = glGetUniformLocation(program, "modelViewMatrix")
    glUniformMatrix4fv(loc, 1, GL_FALSE, camera.view)
    draw_ply(plypath,size = 1./1000)#1.0/(frame_counter**2))
    print("Frame", frame_counter)
    frame_counter+=1
    # img = None
#     if (img_mode&1) != 0:
#         img = np.zeros((windowsize[0], windowsize[1], 3),np.uint8)
#         glReadPixels(0,0,windowsize[0], windowsize[1], GL_RGB, GL_UNSIGNED_BYTE, img)
#         img = np.flipud(img)
#         packet = pickle.dumps(img)
#     if (img_mode&2) != 0:
#         depth = np.zeros(windowsize, np.float32)
#         glReadPixels(0, 0, windowsize[0], windowsize[1], GL_DEPTH_COMPONENT, GL_FLOAT, depth)
#         depth = np.flipud(depth)
#         if (img_mode&1) != 0:
#             packet = pickle.dumps((img, depth))
#         else:
#             packet = pickle.dumps(depth)
#     length = struct.pack('!I', len(packet))
#     packet = length + packet
#     try:
#         command_conn.send(packet)
#     except BrokenPipeError:
#         print("Closing connection")
#         command_conn.close()
#         print("Waiting for connection...")
#         command_conn, reply_addr = command_socket.accept()
#         print("Client connected: ", reply_addr)

    glutSwapBuffers()




def create_cube(pos, size):
    verts = np.zeros((8,3), np.float32)
    ind = 0
    for xi in range(2):
        for yi in range(2):
            for zi in range(2):
                verts[ind, :] = np.array([pos[0] + xi*size[0], pos[1] + yi*size[1], pos[2] + zi*size[2]])
                ind += 1
    indices = np.array([0,1,2,
                        1,2,3,
                        4,0,5,
                        5,0,1,
                        5,1,7,
                        7,1,3,
                        6,2,4,
                        4,2,0,
                        6,4,7,
                        7,4,5,
                        2,6,3,
                        3,6,7],
                       np.uint32)
    res_verts = verts[indices, :]
    res_indices = np.arange(0,36,dtype=np.uint32)
    res_norms = np.repeat(np.array([[-1,0,0],
                                    [0,-1,0],
                                    [0,0,1],
                                    [0,0,-1],
                                    [1,0,0],
                                    [0,1,0]],np.float32),6,axis=0)
    return (res_verts, res_indices,res_norms)


def get_rotation(tx,ty,tz):
    Rx = np.array([[1,0,0], [0, np.cos(tx), -np.sin(tx)], [0, np.sin(tx), np.cos(tx)]])
    Ry = np.array([[np.cos(ty), 0, -np.sin(ty)], [0, 1, 0], [np.sin(ty), 0, np.cos(ty)]])
    Rz = np.array([[np.cos(tz), -np.sin(tz), 0], [np.sin(tz), np.cos(tz), 0], [0,0,1]])
    mat3 = np.dot(Rx, np.dot(Ry, Rz)).astype(np.float32)
    mat4 = np.eye(4,dtype=np.float32)
    mat4[:3,:3] = mat3
    return mat4

glutInitWindowSize(windowsize[0], windowsize[1])
glutInit()
glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH)

glutCreateWindow('OpenGL renderer')
# glutReshapeWindow(windowsize[0], windowsize[1])
glutDisplayFunc(draw)
glutIdleFunc(draw)
glutReshapeFunc(reshape)
glutSpecialFunc(handlekeys)
glClearColor(0.2, 0.2, 0.2, 1)

program = create_program('shader.vert', 'shader.frag')
glUseProgram(program)

camera = Camera()
#tr.translate(camera.view, 0, -1, -4.5)
tr.translate(camera.view, 0, 0, -2.5)
loc = glGetUniformLocation(program, "projectionMatrix")
glUniformMatrix4fv(loc, 1, GL_FALSE, camera.projection)
loc = glGetUniformLocation(program, "modelViewMatrix")
glUniformMatrix4fv(loc, 1, GL_FALSE, camera.view)
loc = glGetUniformLocation(program, "sunDir")
glUniform3f(loc, -2.0, 2.0, 2.0)
loc = glGetUniformLocation(program, "ambientLight")
glUniform1f(loc,0.3)
loc = glGetUniformLocation(program, "spotLightIntensity")
glUniform1f(loc,0.7)

frame_counter=1
# default_box = Cube((-1,-1,-1),(2,2,2),[0,0,1,1])
default_box = Cube((-2.5,0,-2.5),(5,2.5,10),[0,0,1,1])
scene_objects = [default_box]
scales_temp = [(0.25,0.25,0.25),(0.5,0.25,0.25),(0.25,0.5,0.25)]
scales = []
for i in range(5):
    for s in scales_temp:
        scales.append((s[0]*i,s[1]*i,s[2]*i))
scene_objects = scene_objects + generate_cubes(1,(-2.5,-2.5,2.5,2.5),scales)
# scene_objects.append(Cube((0,0,-1),(0.5,0.5,0.5),[1,0,0,1]))

command_socket = socket.socket()
# command_socket = socket.socket(socket.AF_UNIX,socket.SOCK_DGRAM)
# command_socket.bind("supesock.c")
command_socket.bind(('localhost',32400))
# command_socket.listen(1)
print("Waiting for connection...")
# command_conn, reply_addr = command_socket.accept()
# print("Client connected: ", reply_addr)
glutMainLoop()
#soc = socket.socket(socket.AF_UNIX, fileno='socketfile.soc')
