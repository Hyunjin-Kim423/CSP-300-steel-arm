"""Hardware-free pinhole fixtures. Synthetic values are isolated from production configs."""
import math
def project_point(point, fx, fy, cx, cy):
    x,y,z=point
    if z <= 0: raise ValueError('point is behind camera')
    return (fx*x/z + cx, fy*y/z + cy)
def tag_corners(size_m):
    if size_m <= 0: raise ValueError('synthetic tag size must be positive')
    s=size_m/2; return ((-s,-s,0),(s,-s,0),(s,s,0),(-s,s,0))
def translate(points, xyz): return tuple((p[0]+xyz[0],p[1]+xyz[1],p[2]+xyz[2]) for p in points)
def reprojection_rmse(observed, expected):
    if len(observed) != len(expected) or not observed: raise ValueError('equal non-empty corner lists required')
    return math.sqrt(sum((a[0]-b[0])**2+(a[1]-b[1])**2 for a,b in zip(observed,expected))/len(observed))
