import rospy
import tf
from numpy import array
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.srv import GetPlan
from geometry_msgs.msg import PoseStamped, Quaternion
from numpy import floor
from numpy.linalg import norm
from numpy import inf
from tf.transformations import quaternion_from_euler
from math import sqrt
#________________________________________________________________________________
class robot:
	goal = MoveBaseGoal()
	start = PoseStamped()
	end = PoseStamped()
	
	def __init__(self,name):
		self.assigned_point=[]
		self.name=name
		self.global_frame=rospy.get_param('~global_frame','/map')
		self.listener=tf.TransformListener()
		self.listener.waitForTransform(self.global_frame, name+'/base_link', rospy.Time(0),rospy.Duration(10.0))
		cond=0;	
		while cond==0:	
			try:
				(trans,rot) = self.listener.lookupTransform(self.global_frame, self.name+'/base_link', rospy.Time(0))
				cond=1
			except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
				cond==0
		self.position=array([trans[0],trans[1]])		
		self.assigned_point=self.position
		self.client=actionlib.SimpleActionClient(self.name+'/move_base', MoveBaseAction)
		self.client.wait_for_server()
		robot.goal.target_pose.header.frame_id=self.global_frame
		robot.goal.target_pose.header.stamp=rospy.Time.now()
		
		rospy.wait_for_service(self.name+'/move_base_node/NavfnROS/make_plan')
		self.make_plan = rospy.ServiceProxy(self.name+'/move_base_node/NavfnROS/make_plan', GetPlan)
		robot.start.header.frame_id=self.global_frame
		robot.end.header.frame_id=self.global_frame

	def getPosition(self):
		cond=0;	
		while cond==0:	
			try:
				(trans,rot) = self.listener.lookupTransform(self.global_frame, self.name+'/base_link', rospy.Time(0))
				cond=1
			except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
				cond==0
		self.position=array([trans[0],trans[1]])
		return self.position
		
	def sendGoal(self,point, rotation=False):
		robot.goal.target_pose.pose.position.x=point[0]
		robot.goal.target_pose.pose.position.y=point[1]
                if rotation:
                        # robot.goal.target_pose.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, 3.1415926))
                        robot.goal.target_pose.pose.orientation.z = 1.0
                        robot.goal.target_pose.pose.orientation.w = 0.0
                else:
                        robot.goal.target_pose.pose.orientation.w = 1.0
                        robot.goal.target_pose.pose.orientation.z = 0.0
		self.client.send_goal(robot.goal)
		self.assigned_point=array(point)
	
	def cancelGoal(self):
		self.client.cancel_goal()
		self.assigned_point=self.getPosition()
	
	def getState(self):
		return self.client.get_state()
		
	def makePlan(self,start,end):
		robot.start.pose.position.x=start[0]
		robot.start.pose.position.y=start[1]
		robot.end.pose.position.x=end[0]
		robot.end.pose.position.y=end[1]
		start=self.listener.transformPose(self.name+'/map', robot.start)
		end=self.listener.transformPose(self.name+'/map', robot.end)
		plan=self.make_plan(start = start, goal = end, tolerance = 0.0)
		return plan.plan.poses	

        def getPathDistance(self, start, end):
                poses = self.makePlan(start, end)
                if (len(poses) == 0):
                    return 1e10
                distance = 0
                for i in range(len(poses)-1):
                    distance += eulerDistance(poses[i].pose.position, poses[i+1].pose.position)

                return distance


#________________________________________________________________________________
def eulerDistance(start, end):
    return sqrt((start.x-end.x)**2 + (start.y-end.y)**2)

def index_of_point(mapData,Xp):
	resolution=mapData.info.resolution
	Xstartx=mapData.info.origin.position.x
	Xstarty=mapData.info.origin.position.y
	width=mapData.info.width
	Data=mapData.data
	index=int(	(  floor((Xp[1]-Xstarty)/resolution)*width)+( floor((Xp[0]-Xstartx)/resolution) ))
	return index
	
def point_of_index(mapData,i):
	y=mapData.info.origin.position.y+(i/mapData.info.width)*mapData.info.resolution
	x=mapData.info.origin.position.x+(i-(i/mapData.info.width)*(mapData.info.width))*mapData.info.resolution
	return array([x,y])
#________________________________________________________________________________		

def informationGain(mapData,point,r):
	infoGain=0;
	index=index_of_point(mapData,point)
	r_region=int(r/mapData.info.resolution)
	init_index=index-r_region*(mapData.info.width+1)	
	for n in range(0,2*r_region+1):
		start=n*mapData.info.width+init_index
		end=start+2*r_region
		limit=((start/mapData.info.width)+2)*mapData.info.width
		for i in range(start,end+1):
			if (i>=0 and i<limit and i<len(mapData.data)):
				if(mapData.data[i]==-1 and norm(array(point)-point_of_index(mapData,i))<=r):
					infoGain+=1
	return infoGain*(mapData.info.resolution**2)
#________________________________________________________________________________

def discount(mapData,assigned_pt,centroids,infoGain,r):
	index=index_of_point(mapData,assigned_pt)
	r_region=int(r/mapData.info.resolution)
	init_index=index-r_region*(mapData.info.width+1)
        factor = mapData.info.resolution**2
	for n in range(0,2*r_region+1):
		start=n*mapData.info.width+init_index
		end=start+2*r_region
		limit=((start/mapData.info.width)+2)*mapData.info.width	
		for i in range(start,end+1):	
			if (i>=0 and i<limit and i<len(mapData.data)):
				for j in range(0,len(centroids)):
					current_pt=centroids[j]
					if(mapData.data[i]==-1 and norm(point_of_index(mapData,i)-current_pt)<=r and norm(point_of_index(mapData,i)-assigned_pt)<=r):
						infoGain[j]-=1*factor #this should be modified, subtract the area of a cell, not 1
        if infoGain < 0:
            rospy.logwarn("Multi robots discount the same centriod let the infoGain to be negative.")
            return 0
        else:
	    return infoGain
#________________________________________________________________________________

def pathCost(path):
	if (len(path)>0):
		i=len(path)/2
		p1=array([path[i-1].pose.position.x,path[i-1].pose.position.y])
		p2=array([path[i].pose.position.x,path[i].pose.position.y])
		return norm(p1-p2)*(len(path)-1)
	else:
		return inf
#________________________________________________________________________________
		
def unvalid(mapData,pt):
	index=index_of_point(mapData,pt)
	r_region=5
	init_index=index-r_region*(mapData.info.width+1)
	for n in range(0,2*r_region+1):
		start=n*mapData.info.width+init_index
		end=start+2*r_region
		limit=((start/mapData.info.width)+2)*mapData.info.width	
		for i in range(start,end+1):	
			if (i>=0 and i<limit and i<len(mapData.data)):
				if(mapData.data[i]==1):
					return True
	return False
#________________________________________________________________________________
def Nearest(V,x):
 n=inf
 i=0
 for i in range(0,V.shape[0]):
    n1=norm(V[i,:]-x)
    if (n1<n):
	n=n1
        result=i    
 return result

#________________________________________________________________________________ 
def Nearest2(V,x):
 n=inf
 result=0
 for i in range(0,len(V)):
	n1=norm(V[i]-x)
    
	if (n1<n):
		n=n1
 return i
#________________________________________________________________________________
def isOldFrontier(mapData, Xp): #OldFrontier now isnot frontier
    resolution=mapData.info.resolution
    Xstartx=mapData.info.origin.position.x
    Xstarty=mapData.info.origin.position.y

    width=mapData.info.width
    height=mapData.info.height
    data=mapData.data
    # returns if the cell is an old frontier at "Xp" location
    #map data:  100 occupied      -1 unknown       0 free
    index=(int)(floor((Xp[1]-Xstarty)/resolution)*width + floor((Xp[0]-Xstartx)/resolution) )
    if data[index] != -1:
        return True
    # all region belong to uknown isunvalid frontier
    for nbr in nhood8(index, width, height):
        if (data[nbr] == 0):
            return False
    print "unvalid frontier"
    return True

def nhood8(index, width, height):
    # returns if the cell is an old frontier at "Xp" location
    #map data:  100 occupied      -1 unknown       0 free
    out = []
    if (index > width*height-1):
        rospy.logwarn("Evaluating nhood for offmap point")
        return out
    
    if (index % width > 0):
        out.append(index-1)
    
    if (index % width < (width-1)):
        out.append(index+1)

    if (index >= width):
        out.append(index-width)

    if (index < width*(height-1)):
        out.append(index+width)

    if (index % width > 0 and index >= width):
        out.append(index-1-width)

    if (index % width > 0 and index < width*(height-1)):
        out.append(index-1+width)

    if (index % width < (width-1) and index >= width):
        out.append(index+1-width)

    if (index % width < (width-1) and index < width*(height-1)):
        out.append(index+1+width)

    return out

def gridValue(mapData,Xp):
 resolution=mapData.info.resolution
 Xstartx=mapData.info.origin.position.x
 Xstarty=mapData.info.origin.position.y

 width=mapData.info.width
 Data=mapData.data
 # returns grid value at "Xp" location
 #map data:  100 occupied      -1 unknown       0 free
 index=(  floor((Xp[1]-Xstarty)/resolution)*width)+( floor((Xp[0]-Xstartx)/resolution) )
 
 if int(index) < len(Data):
 	return Data[int(index)]
 else:
 	return 100
