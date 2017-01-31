#!/usr/bin/python

#import librarys
import signal
from time import sleep
from Tkinter import *
from PIL import ImageTk, Image
import socket
import threading
import os
import sys
import select
import rospy
from array import *
from std_msgs.msg import Float64, String, Header, Time
from geometry_msgs.msg import PoseWithCovariance, PoseStamped, Pose, Point, Quaternion
from nav_msgs.msg import OccupancyGrid, MapMetaData
from move_base_msgs.msg import MoveBaseActionFeedback

class application(Frame): #application class
    """ GUI """
    def __init__(self, master): #the initialize function
        """ init frame """
        Frame.__init__(self,master,background = 'gray60')
        self.res = 1
        self.xoffset = 0
        self.yoffset = 0
        self.robotposx = 0
        self.robotposy = 0
        self.prevrobotposx = 0
        self.prevrobotposy = 0 
        self.place(x=0,y=0,width=1000,height=800)
        self.create_widgets()
        self.clickboolean = False
        self.clickboolean2 = False
        self.coordinates = []
        self.secondpoint = False
        self.clicks = 0
        self.pclicks = 0
        self.startpointboolean = False
        self.drawstart = False
        points_file = open("points.txt", "r")
        num = 0
        for line in points_file:
            line = line.strip('\n')
            num += 1
            coords = line.split(":")
            if coords[0] == "startpoint ":
                self.xstartpoint = coords[1]
                self.ystartpoint = coords[2]
                self.xstartpointtwo = coords[3]
                self.ystartpointtwo = coords[4]
            else: 
                self.coordinates.append([int(coords[0]),int(coords[1]),int(coords[2]),int(coords[3])])
        points_file.close()
        self.draw()
        rospy.init_node('interface')
        self.interfacecomm = rospy.Publisher('interface_comm', String, queue_size = 1)
        rospy.Subscriber('map', OccupancyGrid, self.replacemap)
        rospy.Subscriber('move_base/feedback', MoveBaseActionFeedback, self.updaterobotpos)
        self.movebase = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size = 1)
        #self.initialpose = rospy.Publisher('initialpose', PoseWithCovariance, queue_size = 1)
        nowtime = rospy.get_rostime()
        self.moveheader = Header(0, nowtime, "map")
        
    def create_widgets(self): #function witch creates the widgets

        # button addobjecct
        self.button_add = Button(self, anchor = "center", font = "calibri 10",bg = 'light gray')
        self.button_add["text"] = "add"
        self.button_add["command"] = self.addobject
        self.button_add.pack()
        self.button_add.place(x=601, y=21,width = 198, height = 48)

        # button deleteobjecct
        self.button_delete = Button(self, anchor = "center", font = "calibri 10",bg = 'light gray')
        self.button_delete["text"] = "delete"
        self.button_delete["command"] = self.deleteobject
        self.button_delete.pack()
        self.button_delete.place(x=601, y=71,width = 198, height = 48)

        # button send
        self.button_send = Button(self, anchor = "center", font = "calibri 10",bg = 'light gray')
        self.button_send["text"] = "send"
        self.button_send["command"] = self.send
        self.button_send.pack()
        self.button_send.place(x=1, y=121,width = 998, height = 48)

        # button addpoint
        self.button_addpoint = Button(self, anchor = "center", font = "calibri 10",bg = 'light gray')
        self.button_addpoint["text"] = "add point"
        self.button_addpoint["command"] = self.addpoint
        self.button_addpoint.pack()
        self.button_addpoint.place(x=1, y=171, width = 198, height = 48)

        # button deletepoint
        self.button_deletepoint = Button(self, anchor = "center", font = "calibri 10",bg = 'light gray')
        self.button_deletepoint["text"] = "delete point"
        self.button_deletepoint["command"] = self.deletepoint
        self.button_deletepoint.pack()
        self.button_deletepoint.place(x=1, y=221, width = 198, height = 48)

        # button startpoint
        self.button_startpoint = Button(self, anchor = "center", font = "calibri 10",bg = 'light gray')
        self.button_startpoint["text"] = "start point"
        self.button_startpoint["command"] = self.startpoint
        self.button_startpoint.pack()
        self.button_startpoint.place(x=401, y=171, width = 198, height = 48)

        # selected object label
        self.label_object = Label(self, anchor = "center", font = "calibri 10",bg = 'light gray')
        self.label_object["text"] = ""
        self.label_object.pack()
        self.label_object.place(x=601, y=1,width = 198, height = 18)

        # object label
        self.label_objectlabel = Label(self, anchor = "center", font = "calibri 10",bg = 'light gray')
        self.label_objectlabel["text"] = "Object"
        self.label_objectlabel.pack()
        self.label_objectlabel.place(x=1, y=1,width = 198, height = 18)

        # pick place label
        self.label_pickplacelabel = Label(self, anchor = "center", font = "calibri 10",bg = 'light gray')
        self.label_pickplacelabel["text"] = "Pick place"
        self.label_pickplacelabel.pack()
        self.label_pickplacelabel.place(x=201, y=1,width = 198, height = 18)
 
        # Drop place label
        self.label_dropplacelabel = Label(self, anchor = "center", font = "calibri 10",bg = 'light gray')
        self.label_dropplacelabel["text"] = "Drop place"
        self.label_dropplacelabel.pack()
        self.label_dropplacelabel.place(x=401, y=1,width = 198, height = 18)

        # Action list label
        self.label_actionlistlabel = Label(self, anchor = "center", font = "calibri 10",bg = 'light gray')
        self.label_actionlistlabel["text"] = "Action list"
        self.label_actionlistlabel.pack()
        self.label_actionlistlabel.place(x=801, y=1,width = 198, height = 18)        

        # object list
        self.listbox_objectlist = Listbox(self, font = "calibri 10",bg = 'white')
        self.listbox_objectlist.pack()
        self.listbox_objectlist.place(x=801, y=21,width = 198, height = 98)

        # listbox object
        self.listbox_objects = Listbox(self, font = "calibri 10",bg = 'white')
        for item in ["Profile 20x20 black", "Profile 20x20 gray", "Profile 40x40 black", "Profile 40x40 gray", "Bolt M20 black", "Nut M20", "Nut M30"]:
            self.listbox_objects.insert(END, item)
        self.listbox_objects.pack()
        self.listbox_objects.place(x=1, y=21, width = 198, height = 98)

        # listbox pickplace
        self.listbox_pickplace = Listbox(self, font = "calibri 10",bg = 'white')
        self.listbox_pickplace.pack()
        self.listbox_pickplace.place(x=201, y=21, width = 198, height = 98)

        # listbox dropplace
        self.listbox_dropplace = Listbox(self, font = "calibri 10",bg = 'white')
        self.listbox_dropplace.pack()
        self.listbox_dropplace.place(x=401, y=21, width = 198, height = 98)

        # listbox points
        self.listbox_points = Listbox(self, font = "calibri 10",bg = 'white')
        self.listbox_points.pack()
        self.listbox_points.place(x = 201, y = 171, width = 198, height = 98)

        # map
        self.mapx = 1
        self.mapy = 271
        self.mapimagewidth = 100
        self.mapimageheight = 100
        self.map = Canvas(self)
        try:
            self.mapimage = ImageTk.PhotoImage(Image.open("map.pgm"))
            self.mapimagewidth, self.mapimageheight = Image.open("map.pgm").size
            self.map.create_image(0,0, anchor='nw', image=self.mapimage)
            self.map.image = self.mapimage
        except:
            print "No image"
        self.map.pack()
        self.map.bind("<Button-1>", self.setpoint)
        self.map.place(x=self.mapx, y=self.mapy,width = self.mapimagewidth, height = self.mapimageheight)

    def updaterobotpos(self, robotposdata):
        self.robotposx = (robotposdata.feedback.base_position.pose.position.x - self.xoffset)/self.res
        self.robotposy = (robotposdata.feedback.base_position.pose.position.y - self.yoffset)/self.res
        if(abs(self.robotposx - self.prevrobotposx) > 1 or abs(self.robotposy - self.prevrobotposy) > 1):
            self.prevrobotposx = self.robotposx
            self.prevrobotposy = self.robotposy
            self.map.create_image(0,0, anchor='nw', image=self.mapimage)
            self.map.image = self.mapimage
            self.draw()
        
    def replacemap(self, mapdata):
        print "Replace map"
        self.res = mapdata.info.resolution
        self.xoffset = mapdata.info.origin.position.x
        self.yoffset = mapdata.info.origin.position.y
        map_file = open("map.pgm", "w")
        map_file.write("P5 \n")
        map_file.write("# CREATOR: Map_generator.cpp 0.025 m/pix \n")
        map_file.write(str(mapdata.info.width) + " " + str(mapdata.info.height)+"\n")
        map_file.write(str(255) + "\n")
        maparray = bytearray()
        for i in range(0, (mapdata.info.width*mapdata.info.height)):
            maparray.append(bytes(0))
        #maparray = array('I')
        #place = (mapdata.info.height * mapdata.info.width)-(mapdata.info.width)
        place = 0
        for number in mapdata.data:
            if number < 0:
                number = 150
            maparray[place] = number
            #if (place % mapdata.info.width) < (mapdata.info.width-1):
            #    place += 1
            #else:
            #    place -= ((mapdata.info.width*2) -1)
            place += 1

        #maparray.tofile(map_file)
        map_file.write(maparray)
        map_file.close()
        
    def send(self):
        message = ""
        i = 0

        conv_coordinates = []
        for coord in self.coordinates: #to convert coordinates
            coordx1 = (coord[0]*self.res)+self.xoffset
            coordy1 = (coord[1]*self.res)+self.yoffset
            coordx2 = (coord[2]*self.res)+self.xoffset
            coordy2 = (coord[3]*self.res)+self.yoffset
            conv_coordinates.append([coordx1,coordy1,coordx2,coordy2])

            
        while i < self.listbox_objectlist.size():
            object1 = self.listbox_objectlist.get(i).split(' : ')[0]
            place1 = self.listbox_objectlist.get(i).split(' : ')[1]
            place2 = self.listbox_objectlist.get(i).split(' : ')[2]
                
            place1 = place1 + str(conv_coordinates[int(place1)-1])
            place2 = place2 + str(conv_coordinates[int(place2)-1])
            message = message + object1 + " : " + place1 + " : " + place2 + " | "
            i += 1
            
        startplace = [self.xstartpoint, self.ystartpoint, self.xstartpointtwo, self.ystartpointtwo]
        message = message + str(startplace)
        self.interfacecomm.publish(message)
        print message
        print conv_coordinates[0]
        #self.moveposition = Pose(Point(conv_coordinates[0][0],conv_coordinates[0][1],0), Quaternion(0,0,(conv_coordinates[0][0]-conv_coordinates[0][2]),(conv_coordinates[0][1]-conv_coordinates[0][3])))
        #self.movebase.publish(PoseStamped(self.moveheader, self.moveposition))

    def selectobject(self):
        self.label_object["text"] = str(self.listbox_objects.get(ACTIVE)) + " : " + str(self.listbox_pickplace.get(ACTIVE)) + " : " + str(self.listbox_dropplace.get(ACTIVE))

    def addobject(self):
        if self.listbox_points.size() > 0:
            self.listbox_objectlist.insert(END, self.label_object["text"])
        
    def deleteobject(self):
        self.listbox_objectlist.delete(ACTIVE)

    def startpoint(self):
        self.secondpoint = False
        self.startpointboolean = not self.startpointboolean
        if self.startpointboolean:
            self.button_startpoint.config(bg = "dark gray")
        else:
            self.button_startpoint.config(bg = "light gray")
        
    def setpoint(self, event):
        if not self.startpointboolean:
            r = 3
            markercolor = "purple"
            if not self.secondpoint:
                self.xpoint = root.winfo_pointerx() - root.winfo_rootx() - (self.mapx+1)
                self.ypoint = root.winfo_pointery() - root.winfo_rooty() - (self.mapy+1)
                self.map.create_image(0,0, anchor='nw', image=self.mapimage)
                self.map.image = self.mapimage
                self.map.create_oval(self.xpoint-r,self.ypoint-r, self.xpoint+r, self.ypoint+r, outline = markercolor)
                self.map.create_line(self.xpoint,self.ypoint-r, self.xpoint, self.ypoint+r, fill = markercolor)
                self.map.create_line(self.xpoint-r, self.ypoint, self.xpoint+r, self.ypoint, fill = markercolor)
            else:
                self.xpointtwo = root.winfo_pointerx() - root.winfo_rootx() - (self.mapx+1)
                self.ypointtwo = root.winfo_pointery() - root.winfo_rooty() - (self.mapy+1)
                self.map.create_oval(self.xpointtwo-r,self.ypointtwo-r, self.xpointtwo+r, self.ypointtwo+r, outline = markercolor)
                self.map.create_line(self.xpointtwo,self.ypointtwo-r, self.xpointtwo, self.ypointtwo+r, fill = markercolor)
                self.map.create_line(self.xpointtwo-r, self.ypointtwo, self.xpointtwo+r, self.ypointtwo, fill = markercolor)
                self.map.create_line(self.xpoint, self.ypoint, self.xpointtwo, self.ypointtwo, fill = markercolor)
        else:
            r = 2
            markercolor = "dark green"
            if not self.secondpoint:
                self.xstartpoint = root.winfo_pointerx() - root.winfo_rootx() - (self.mapx+1)
                self.ystartpoint = root.winfo_pointery() - root.winfo_rooty() - (self.mapy+1)
                self.map.create_image(0,0, anchor='nw', image=self.mapimage)
                self.map.image = self.mapimage
                self.map.create_oval(self.xstartpoint-r,self.ystartpoint-r, self.xstartpoint+r, self.ystartpoint+r, outline = markercolor)
                self.map.create_line(self.xstartpoint,self.ystartpoint-r, self.xstartpoint, self.ystartpoint+r, fill = markercolor)
                self.map.create_line(self.xstartpoint-r, self.ystartpoint, self.xstartpoint+r, self.ystartpoint, fill = markercolor)
                self.drawstart = False
            else:
                self.xstartpointtwo = root.winfo_pointerx() - root.winfo_rootx() - (self.mapx+1)
                self.ystartpointtwo = root.winfo_pointery() - root.winfo_rooty() - (self.mapy+1)
                self.map.create_line(self.xstartpoint, self.ystartpoint, self.xstartpointtwo, self.ystartpointtwo, fill = markercolor, arrow = LAST)
                self.drawstart = True
                #self.initposition = Pose(Point(xstartpoint,ystartpoint,0), Quaternion(0,0,(xstartpoint-xstartpointtwo),(ystartpoint-ystartpointtwo)))
                #self.covariance = [0.0]*36
                #self.covariance[0] = 0.25
                #self.covariance[7] = 0.25
                #self.covariance[35] = 0.06853891945200942
                #self.initialpose.publish(PoseWithCovariance(self.moveheader, self.initposition))
            
        self.draw()
        self.secondpoint = not self.secondpoint
        self.clicks += 1
        

    def movepoint(self, event):
        if event.char == 'w' or event.char == 'a' or event.char == 's' or event.char == 'd':
            dx = 0
            dy = 0
            if event.char == 'w':
                dy = -1
            elif event.char == 'a':
                dx = -1
            elif event.char == 's':
                dy = 1
            else:
                dx = 1
            r = 3
            markercolor = "purple"
            if self.secondpoint:
                self.xpoint += dx
                self.ypoint += dy
                self.map.create_image(0,0, anchor='nw', image=self.mapimage)
                self.map.image = self.mapimage
                self.map.create_oval(self.xpoint-r,self.ypoint-r, self.xpoint+r, self.ypoint+r, outline = markercolor)
                self.map.create_line(self.xpoint,self.ypoint-r, self.xpoint, self.ypoint+r, fill = markercolor)
                self.map.create_line(self.xpoint-r, self.ypoint, self.xpoint+r, self.ypoint, fill = markercolor)
            else:
                self.xpointtwo += dx
                self.ypointtwo += dy
                self.map.create_image(0,0, anchor='nw', image=self.mapimage)
                self.map.image = self.mapimage
                self.map.create_oval(self.xpoint-r,self.ypoint-r, self.xpoint+r, self.ypoint+r, outline = markercolor)
                self.map.create_line(self.xpoint,self.ypoint-r, self.xpoint, self.ypoint+r, fill = markercolor)
                self.map.create_line(self.xpoint-r, self.ypoint, self.xpoint+r, self.ypoint, fill = markercolor)
                self.map.create_oval(self.xpointtwo-r,self.ypointtwo-r, self.xpointtwo+r, self.ypointtwo+r, outline = markercolor)
                self.map.create_line(self.xpointtwo,self.ypointtwo-r, self.xpointtwo, self.ypointtwo+r, fill = markercolor)
                self.map.create_line(self.xpointtwo-r, self.ypointtwo, self.xpointtwo+r, self.ypointtwo, fill = markercolor)
                self.map.create_line(self.xpoint, self.ypoint, self.xpointtwo, self.ypointtwo, fill = markercolor)
            self.draw()

    def addpoint(self):
        if not self.secondpoint and self.clicks > 0 and self.clicks != self.pclicks:
            self.pclicks = self.clicks
            r = 3
            self.map.create_image(0,0, anchor='nw', image=self.mapimage)
            self.map.image = self.mapimage
            self.coordinates.append([self.xpoint, self.ypoint, self.xpointtwo, self.ypointtwo])
            self.draw()

    def deletepoint(self):
        if len(self.coordinates) > 0:
            self.coordinates.pop(self.listbox_points.index(ACTIVE))
            self.map.create_image(0,0, anchor='nw', image=self.mapimage)
            self.map.image = self.mapimage
            self.draw()
            i = 0
            while i < self.listbox_objectlist.size():
                place1 = int(self.listbox_objectlist.get(i).split(' : ')[1])
                place2 = int(self.listbox_objectlist.get(i).split(' : ')[2])
                if place1 > self.listbox_points.size() or place2 > self.listbox_points.size():
                    self.listbox_objectlist.delete(i)
                i += 1

    def draw(self):
        r = 3
        markercolor = "red"
        self.listbox_dropplace.delete(0, END)
        self.listbox_pickplace.delete(0, END)
        self.listbox_points.delete(0, END)
        num = 0
        points_file = open("points.txt", "w")
        for coord in self.coordinates:
            num = num + 1
            self.map.create_oval(coord[0]-r,coord[1]-r, coord[0]+r, coord[1]+r, outline = markercolor)
            self.map.create_line(coord[0],coord[1]-r, coord[0], coord[1]+r, fill = markercolor)
            self.map.create_line(coord[0]-r, coord[1], coord[0]+r, coord[1], fill = markercolor)
            self.map.create_text(coord[0], coord[1]-10, text = str(num), font = "calibri 14", fill = markercolor)
            self.map.create_oval(coord[2]-r,coord[3]-r, coord[2]+r, coord[3]+r, outline = markercolor)
            self.map.create_line(coord[2],coord[3]-r, coord[2], coord[3]+r, fill = markercolor)
            self.map.create_line(coord[2]-r, coord[3], coord[2]+r, coord[3], fill = markercolor)
            self.map.create_line(coord[0], coord[1], coord[2], coord[3], fill = markercolor)
            self.listbox_dropplace.insert(END, num)
            self.listbox_pickplace.insert(END, num)
            self.listbox_points.insert(END, str(str(num) + " " + str(self.coordinates[num-1])))
            points_file.write(str(str(coord[0]) + ":" + str(coord[1]) + ":" + str(coord[2]) + ":" + str(coord[3]) + "\n"))
        points_file.close()
        if self.drawstart: 
            r = 2
            markercolor = "dark green"
            self.map.create_oval(self.xstartpoint-r,self.ystartpoint-r, self.xstartpoint+r, self.ystartpoint+r, outline = markercolor)
            self.map.create_line(self.xstartpoint,self.ystartpoint-r, self.xstartpoint, self.ystartpoint+r, fill = markercolor)
            self.map.create_line(self.xstartpoint-r, self.ystartpoint, self.xstartpoint+r, self.ystartpoint, fill = markercolor)
            self.map.create_line(self.xstartpoint, self.ystartpoint, self.xstartpointtwo, self.ystartpointtwo, fill = markercolor, arrow = LAST)
        self.map.create_oval(self.robotposx-r,self.robotposy-r, self.robotposx+r, self.robotposy+r, outline = "dark green")
        
class update(threading.Thread):     
    def run(self):
        lastupdatetime = 0.0
        while 1:
            app.selectobject()
            try:
                mtime = os.path.getmtime("map.pgm")
            except OSError:
                mtime = 0
            if lastupdatetime != mtime:
                print "UPDATE Map"
                lastupdatetime = mtime
                app.mapimage = ImageTk.PhotoImage(Image.open("map.pgm"))
                app.mapimagewidth, app.mapimageheight = Image.open("map.pgm").size
                app.map.create_image(0,0, anchor='nw', image=app.mapimage)
                app.map.image = app.mapimage
                app.draw()
            sleep(0.1)
        print "Stop update thread"
        return
    
class rosupdate(threading.Thread):     
    def run(self):
        rospy.spin()

def signal_handler(signal, frame):
    print "Close interface"
    root.destroy()
    rospy.signal_shutdown("Close node")
    print "Exit"
    sys.exit(0)
    
#create window
root = Tk()

#mod window
root.title("Gui")
root.geometry("1000x800")
root.configure(background='gray99')

app = application(root)

#kick off event loop
updateThread = update()
updateThread.start()
signal.signal(signal.SIGINT, signal_handler)
root.bind("<KeyPress>", app.movepoint)
root.attributes('-fullscreen', False) #toggle fullscreen on or off
root.mainloop()#start the application after everything is defined.
