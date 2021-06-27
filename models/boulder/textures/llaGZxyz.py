import cv2
import navpy
import json
import argparse

parser = argparse.ArgumentParser(
    description='convert lat lon to gazebo XYZ world location')
parser.add_argument('--lat-lon', type=float, nargs='+',
                    help='lat lon location to be converted to Gazebo XYZ')

args = parser.parse_args()
# print(args)
lat_in = args.lat_lon[0]
lon_in = args.lat_lon[1]

with open('terrain_cfg.json') as f:
    data = json.load(f)

# print(data)
center_lat_lon = data['latlon']
offset = data['offset']
terrain_delta = data['terrain_delta']
model_name = data['model_name']
width = data['width']

# print(lat_in, lon_in, offset, center_lat_lon[0], center_lat_lon[1], offset)
ned = navpy.lla2ned(lat_in, lon_in, offset,
                    center_lat_lon[0], center_lat_lon[1], offset)
x = ned[0]
y = ned[1]

elevation_map = cv2.imread(model_name+'_heightmap.png')
height, width, channels = elevation_map.shape
elevation = elevation_map[int(
    height/2)-int(x)][int(width/2) + int(y)]*(terrain_delta/255)
# print(elevation)
print('location is gazebo world XYZ %.4f %.4f %.4f ' %
      (float(x), float(y), float(elevation[0])))
