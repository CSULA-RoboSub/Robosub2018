import BuoyDetector as bd
bd = bd.BuoyDetector()

while True:
    bd.detect()
    print bd.directions
