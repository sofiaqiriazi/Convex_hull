import sys
sys.path.append('..')

from pycompgeom import *
import time

def wait():
	time.sleep(0.1)


def jarvis(points):
	r0 = min(points)
	hull = [r0]
	r = r0
	vhull = []
	while True:
		u = random.choice(points)
		curr_ru = VSegment2.from_endpoints(r,u,color=BLUE); wait()
		for t in points:
			curr_ut = VSegment2.from_endpoints(u,t,color=MAGENTA); wait()
			if cw(r, u, t) or collinear(r, u, t) and between(r, t, u):
				curr_ut = VSegment2.from_endpoints(u,t,color=GREEN); wait()
				u = t
				curr_ru = VSegment2.from_endpoints(r,u,color=BLUE); del curr_ut; wait()
			else:
				curr_ut = VSegment2.from_endpoints(u,t,color=RED); wait() 
		if u == r0: break
		else:
			vhull.append(VSegment2.from_endpoints(hull[-1], u, color=YELLOW))
			r = u
			curr_ru = VSegment2.from_endpoints(r,u,color=BLUE); wait()
			points.remove(r)
			hull.append(r)
	vhull.append(VSegment2.from_endpoints(hull[-1], u, color=YELLOW));
	return hull
#Voithitiki ths DistancePointLine
def lineMagnitude (x1, y1, x2, y2):
    lineMagnitude = math.sqrt(math.pow((x2 - x1), 2)+ math.pow((y2 - y1), 2))
    return lineMagnitude
 
#Calc minimum distance from a point and a line segment (i.e. consecutive vertices in a polyline).
def DistancePointLine (px, py, x1, y1, x2, y2):
    #http://local.wasp.uwa.edu.au/~pbourke/geometry/pointline/source.vba
    LineMag = lineMagnitude(x1, y1, x2, y2)
 
    if LineMag < 0.00000001:
        DistancePointLine = 9999
        return DistancePointLine
 
    u1 = (((px - x1) * (x2 - x1)) + ((py - y1) * (y2 - y1)))
    u = u1 / (LineMag * LineMag)
 
    if (u < 0.00001) or (u > 1):
        #// closest point does not fall within the line segment, take the shorter distance
        #// to an endpoint
        ix = lineMagnitude(px, py, x1, y1)
        iy = lineMagnitude(px, py, x2, y2)
        if ix > iy:
            DistancePointLine = iy
        else:
            DistancePointLine = ix
    else:
        # Intersecting point is on the line, use the formula
        ix = x1 + u * (x2 - x1)
        iy = y1 + u * (y2 - y1)
        DistancePointLine = lineMagnitude(px, py, ix, iy)
    print DistancePointLine
    return DistancePointLine

#Quicksort
def qsortr(base,list):
    return [] if list==[]  else qsortr(base,[x for x in list[1:] if DistancePointLine(base.x,base.y,x.start.x,x.start.y,x.end.x,x.end.y) < DistancePointLine(base.x,base.y,list[0].start.x,list[0].start.y,list[0].end.x,list[0].end.y)]) + [list[0]] + qsortr(base,[x for x in list[1:] if DistancePointLine(base.x,base.y,x.start.x,x.start.y,x.end.x,x.end.y) >= DistancePointLine(base.x,base.y,list[0].start.x,list[0].start.y,list[0].end.x,list[0].end.y)])

def min_point(base, points):
	dist = 0	
	ret = ()
	for i in points:	
		temp = qsortr(i,base)
		if(dist == 0 or DistancePointLine(i.x,i.y,temp[0].start.x,temp[0].start.y,temp[0].end.x,temp[0].end.y) < dist):
			print dist
			dist = DistancePointLine(i.x,i.y,temp[0].start.x,temp[0].start.y,temp[0].end.x,temp[0].end.y)
			ret = (temp[0] ,i)
	print dist	
	return ret
#vasiki
def random_simple_polygon(num):

	points = getVPoints()
	segmentpoints = set()
	segments1 = jarvis(points)
	segments = []
	for i in range(len(segments1)-1):
		segments.append(Segment2(segments1[i],segments1[i+1]))
	segments.append(Segment2(segments1[-1],segments1[0]))	
	print segments
	for i in segments:
		segmentpoints.add(i.start)		
		segmentpoints.add(i.end)

	nsegmentpoints = set(points)
	print nsegmentpoints
	nsegmentpoints = nsegmentpoints.difference(segmentpoints)	
	print nsegmentpoints

	while(nsegmentpoints):
		vhull = []		
		for i in segments:
			vhull.append(VSegment2.from_endpoints(i.start,i.end,color=BLUE)); wait();wait();wait();wait();wait();wait();wait();wait();wait();	
		canditate = min_point(list(segments),list(nsegmentpoints))		
		choice = canditate[0]
		canditate = canditate[1]
		segments.remove(choice)		
		prev = choice.start
		choice = choice.end
		print choice
		
		print canditate
		
		segments.append(Segment2(prev,canditate))
		segments.append(Segment2(canditate,choice))
		nsegmentpoints.remove(canditate)
		segmentpoints.add(canditate)
	vhull = []	
	for i in segments:
		print i.start
		print i.end
		vhull.append(VSegment2.from_endpoints(i.start,i.end,color=BLUE)); wait()
	wait();wait();wait();wait();wait();wait();wait();wait();wait();	
	return segments			

print random_simple_polygon(0)
