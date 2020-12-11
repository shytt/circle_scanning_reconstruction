#!/usr/bin/env python

"""
"""

import vtk
import cv2
import math
import argparse

from  tables import *

class Point(object):
    def __init__(self, x, y, z):
        '''Defines x and y variables'''
        self.X = x
        self.Y = y
        self.Z = z

    def __add__(self, o): 
        return Point(self.X+o.X, self.Y+o.Y, self.Z+o.Z)

    def __sub__(self, o):
        return Point(self.X-o.X, self.Y-o.Y, self.Z-o.Z)

    def __mul__(self, i):
        return Point(self.X*i, self.Y*i, self.Z*i)

    def __truediv__(self, i):
        return Point(self.X/i, self.Y/i, self.Z/i)

    def normal(self):
        sqrt = math.sqrt(self.X**2 + self.Y**2 + self.Z**2)
        return Point(self.X/sqrt, self.Y/sqrt, self.Z/sqrt)


class CS():
    def __init__(self):
        args = get_program_parameters()
        self.start_idx = args.startidx
        self.end_idx   = args.endidx
        self.iso       = args.isovalue
        self.camera    = Point(args.camera[0],args.camera[1],args.camera[2])
        self.center    = Point(args.center[0],args.center[1],args.center[2])

        self.gridX, self.gridY = args.resolution

        self.images    = load_image(args.filename,args.startidx,args.endidx)
        self.shape     = self.images[0].shape
        self.m_polys   = vtk.vtkCellArray()
        self.m_points  = vtk.vtkPoints()

    def render(self):
        polyData = vtk.vtkPolyData()
        polyData.SetPoints(self.m_points)
        polyData.SetPolys(self.m_polys)

        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputData(polyData)
        mapper.ScalarVisibilityOff()

        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().SetDiffuseColor([255, 125, 64])

        camera = vtk.vtkCamera()
        camera.SetViewUp(0, 0, -1)
        camera.SetPosition(0, -1, 0)
        camera.SetFocalPoint(0, 0, 0)
        camera.ComputeViewPlaneNormal()
        camera.Azimuth(30.0)
        camera.Elevation(30.0)

        renderer = vtk.vtkRenderer()
        renderer.AddActor(actor)
        renderer.SetActiveCamera(camera)
        renderer.ResetCamera()
        camera.Dolly(1.5)
        renderer.SetBackground(0, 0, 0);

        renWin = vtk.vtkRenderWindow()
        renWin.AddRenderer(renderer)

        iren = vtk.vtkRenderWindowInteractor()
        iren.SetRenderWindow(renWin)

        renWin.SetSize(640, 480)

        renderer.ResetCameraClippingRange()

    
        iren.Initialize()
        iren.Start()

    def marchingCube(self):
        for z in range(self.start_idx,self.end_idx):
            print(z)
            for y in range(self.shape[0]-1):
                for x in range(self.shape[1]-1):
                    value = []
                    points = []
                    ind = [[0,0,0],[1,0,0],[1,0,1],[0,0,1],
                           [0,1,0],[1,1,0],[1,1,1],[0,1,1]]
                    for i in range(8):
                        value.append(self.images[z-self.start_idx+ind[i][2]][y+ind[i][1],x+ind[i][0]])
                        points.append(Point(x+ind[i][0],y+ind[i][1],z+ind[i][2]))
                    for i in range(6):
                        # 6 triangles
                        self.add_faces(value,points,TeraTable[i])




    def add_faces(self,v,p,t):
        tableIndex = 0
        for idx in range(4):
            if v[t[idx]] < self.iso:
                 tableIndex = tableIndex| (1 << idx)
        tf = TeraFace.get(tableIndex)
        for i in range(int(len(tf)/3)):
            p1 = self.add_line(p[t[tf[i][0]]],p[t[tf[i][1]]])
            p2 = self.add_line(p[t[tf[i+1][0]]],p[t[tf[i+1][1]]])
            p3 = self.add_line(p[t[tf[i+2][0]]],p[t[tf[i+2][1]]])

            self.m_polys.InsertNextCell(3,[p1,p2,p3])

    def add_line(self,p1,p2):
        v1 = self.rotate(p1)
        v2 = self.rotate(p2)

        mid = (v1+v2)/2
        pid = self.m_points.InsertNextPoint([mid.X,mid.Y,mid.Z])
        return pid

    def rotate(self,p):
        # rotate around z axis
        arc = math.radians(p.Z/2-60)
        vec = self.camera-self.center
        vrc = Point(vec.X*math.cos(arc)-vec.Y*math.sin(arc),
                   vec.X*math.sin(arc)+vec.Y*math.cos(arc),
                   vec.Z) + self.center
        assert(vrc.Z == 0)
        dirc = (self.center-vrc).normal()


        v = Point(vrc.X,vrc.Y,vrc.Z)
        v = v + dirc * p.Y * self.gridY 
        v = v + Point(0,0,1) * p.X * self.gridX
        return v


def load_image(filename, start_idx,end_idx):
    images = []
    for i in range(start_idx,end_idx+1):
        curFile = "{}_{}.bmp".format(filename, i) 
        img = cv2.imread(curFile)
        img_gray = cv2.cvtColor(img,cv2.COLOR_RGB2GRAY)
        images.append(img_gray)
    return images


def get_program_parameters():
    parser = argparse.ArgumentParser()
    parser.add_argument("-f", "--filename", help="the volume data location. Without the index of image. e.g. */image")
    parser.add_argument("-s", "--startidx", type=int ,default = 0, help="the starting index of the images")
    parser.add_argument("-e", "--endidx", type = int, default = 0, help="the ending index of the images")
    parser.add_argument("-r", "--resolution",type = float, nargs=2, default = [0.1,0.1], help="the resolution of the pixel of the image")
    parser.add_argument("--camera",type = int, nargs=3, default = [0,0,0], help="the initial position of the probe")
    parser.add_argument("--center",type = int, nargs=3, default = [0,0,0], help="the rotation center of the probe")
    parser.add_argument("-v", "--isovalue", type = float, default = 0, help="the isovalue of the surface")
    # iso-surface related, signal source related, resolution, step size
    args = parser.parse_args()
    return args


if __name__ == '__main__':
    cs = CS()
    cs.marchingCube()
    cs.render()
