import numpy as np
from numpy.core.numerictypes import maximum_sctype
import cv2
from google.colab.patches import cv2_imshow
import math
from random import random

class Ray():
  def __init__(self, direction, origin):
    self.direction = direction
    self.origin = origin

  def calculate_point(self, t):
    return np.add(np.multiply(t, self.direction), self.origin)


class Sphere():
  def __init__(self, radius, origin, color):
    self.radius = radius
    self.origin = origin
    self.color = color

  def get_ray_intersection(self, ray):
    a = ray.direction[0]**2+ray.direction[1]**2+ray.direction[2]**2
    b = 2*(ray.direction[0]*(ray.origin[0]-self.origin[0])+ray.direction[1]*(ray.origin[1]-self.origin[1])+ray.direction[2]*(ray.origin[2]-self.origin[2]))
    c = (ray.origin[0]-self.origin[0])**2+(ray.origin[1]-self.origin[1])**2+(ray.origin[2]-self.origin[2])**2-self.radius**2

    if (b**2 - 4 * a * c) < 0:
      return False
    else:
      s_1 = (-b + math.sqrt(b**2-4*a*c))/(2*a)
      s_2 = (-b - math.sqrt(b**2-4*a*c))/(2*a)
      return (s_1, s_2)

  def get_normal(self, ray, t):
    point = [ray.direction[i]*t+ray.origin[i] for i in range (3)]

    normal_vector = np.subtract(point, self.origin)
    normal_vector = normal_vector/np.linalg.norm(normal_vector)

    return normal_vector

class Light():
  def __init__(self, origin):
    self.origin = origin

  def get_vector(self, point):
    v = np.subtract(self.origin, point)
    v /= np.linalg.norm(v)

    return v

  def is_obstructed(self, base_obj, point, obj_list, debug=False, img=None):
    ray = Ray(self.get_vector(point), point)

    t_dir = {}

    for obj in object_list:
        intersections = obj.get_ray_intersection(ray)
        if intersections != False:
          t_dir[intersections[0]] = obj
          t_dir[intersections[1]] = obj

    if debug:
      print(t_dir)

    if t_dir != {}:
      return max(t_dir) > 0 and t_dir[max(t_dir)] != base_obj

    return False
  
display_size = (0, 512, 512)
display_dir = (-1, 0, 0)

img = np.zeros((512, 512, 3), np.uint8)

object_list = [Sphere(150, (-150, -150, -150), (1, 210, 250)),
               Sphere(80, (0, 0, 0), (250, 210, 1)),
               Sphere(50, (80, 80, 80), (210, 1, 250)),
               ]

light_list = [Light((300, 300, 300)),
              Light((300, -300, 300)),
              ]

def main():
  for y in range(display_size[1]):
    for z in range(display_size[2]):
      ray_origin = (512, y-256, z-256)
      ray_dir = display_dir

      ray = Ray(ray_dir, ray_origin)
      t_dir = {}


      for obj in object_list:
        intersections = obj.get_ray_intersection(ray)
        if intersections != False and min(intersections)>0:
          t_dir[min(intersections)] = obj


      if t_dir != {}:
        #print(t_dir)
        intersect_obj = t_dir[min(t_dir)]
        intersect_point = ray.calculate_point(min(t_dir))

        normal_obj = intersect_obj.get_normal(ray, min(t_dir))
        rough_normal = [i+random()/10 for i in normal_obj]
        rough_normal /= np.linalg.norm(rough_normal)

        shade_list = []

        for light in light_list:
          if not light.is_obstructed(intersect_obj, intersect_point, object_list):
            shade_list.append(np.dot(normal_obj, light.get_vector(intersect_point)))
        shade = 0

        if len(shade_list) > 0:
          for s in shade_list:
            if s>0:
              shade += s
        if shade > 1:
          shade = 1
        if shade > 0:
          img[y][z] = np.multiply(shade, t_dir[min(t_dir)].color)


main()

#print(img[101][256])
#img[101][256:500] = [0, 0, 255]
cv2_imshow(img)