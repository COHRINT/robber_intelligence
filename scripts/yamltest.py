import os
import yaml
import geometry_msgs.msg as geo_msgs

mapInfo = 'map2.yaml'
print(mapInfo)
with open(mapInfo, 'r') as stream:
    try:
    	yamled = yaml.load(stream)
    except yaml.YAMLError as exc:
        print(exc)
# deletes info
del yamled['info']
# makes list of location names
# gets locations of each object, attaches them to name
objDict = yamled.values()
objLocations = {}
for item in objDict:
	itemName = item['name']
	itemLoc = geo_msgs.Pose(geo_msgs.Point(item['centroid_x'], item['centroid_y'], 0), geo_msgs.Quaternion(0,0,0,1))
	objLocations[itemName] = itemLoc
objList = objLocations.keys()

print (objLocations)