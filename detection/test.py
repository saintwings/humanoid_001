import detect
bbox = detect.BBox(300, 500, 1990, 1200)
bbox = bbox.fit(1920, 1080)
print(bbox)