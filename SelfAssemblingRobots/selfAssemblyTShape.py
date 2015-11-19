# Self Assembly of a T Alphabet 
# With Graph Grammar and no Repair mechanism



import sys, os, random, time
from math import *
from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *

import math
import ode
import serial
import threading
import numpy as np
import time
quadratic = None #necessary for cylinder

ESCAPE = '\033'

X = 0;

aveKECounter = 0;
aveaveKE = 0;
bodies = []
geoms = []
COUNTERFUCKER = 0  
pullCounter = 0
startTime = time.time();
finishAssembly = 0;
a0 = 1.0;
a1 = 1.0;
a2 = 3.0;
a3 = 2.0;     

f0 = 1.3/(2*math.pi);
randomComponent1 = random.gauss(0,0.01)

f2 = 1.3/(2*math.pi);
randomComponent2 = random.gauss(0,0.01)

# delayCounter = 0;

# geometric utility functions
def scalp (vec, scal):
    vec[0] *= scal
    vec[1] *= scal
    vec[2] *= scal

def keyPressed(*args):
        if args[0] == ESCAPE:
                sys.exit()


def length (vec):
    return sqrt (vec[0]**2 + vec[1]**2 + vec[2]**2)

# prepare_GL
def prepare_GL():
    """Prepare drawing.
    """

    # Viewport
    global quadratic
    quadratic = gluNewQuadric()
    glViewport(0,0,800,600)

    # Initialize

    glClearColor(0.8, 0.8, 0.9, 0.0)
    glClearDepth(10.0) 
    glDepthFunc(GL_LESS)
    glDisable(GL_DEPTH_TEST)
    glShadeModel(GL_FLAT)   
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective (55,1.5,0.2,25)
    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()
    gluLookAt (3.0, 2.75, 6.0, 0.5, 1.25, 0, 0, 1, 0)

    # glAlphaFunc(GL_GREATER, 0.2);
    # glEnable(GL_ALPHA_TEST);

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
    glBegin(GL_QUADS);

     
    glColor4f(1.0,1.0,0.0,0.4)
    glVertex3f(-2,0,2);
    glVertex3f(2,0,2);
    glVertex3f(2,0,-2);
    glVertex3f(-2,0,-2);

    glColor4f(0.0,0.0,1.0,0.2)
    glVertex3f(-2,0,2);
    glVertex3f(2,0,2);
    glVertex3f(2,3,2);
    glVertex3f(-2,3,2);

    glColor4f(0.0,0.0,1.0,0.4)
    
    glVertex3f(-2,3,2);
    glVertex3f(2,3,2);
    glVertex3f(2,3,-2);
    glVertex3f(-2,3,-2);

    glColor4f(0.0,0.0,1.0,0.4)
    
    glVertex3f(2,3,2);
    glVertex3f(2,3,-2);
    glVertex3f(2,0,-2);
    glVertex3f(2,0,2);

    glColor4f(1.0,0.0,0.0,0.1)

    glVertex3f(-2,3,-2);
    glVertex3f(2,3,-2);
    glVertex3f(2,0,-2);
    glVertex3f(-2,0,-2);

    glColor4f(1.0,0.0,0.0,0.4)

    glVertex3f(-2,3,2);
    glVertex3f(-2,3,-2);
    glVertex3f(-2,0,-2);
    glVertex3f(-2,0,2);

    glEnd();



    glDisable(GL_BLEND)

    glEnable(GL_DEPTH_TEST)

  

# draw_body
def draw_body(body):
    """Draw an ODE body.
    """

    
    x,y,z = body.getPosition()
    R = body.getRotation()
    rot = [R[0], R[3], R[6], 0.,
           R[1], R[4], R[7], 0.,
           R[2], R[5], R[8], 0.,
           x, y, z, 1.0]
    glPushMatrix()
    glMultMatrixd(rot)
    if body.shape=="ccylinder":
        radius = body.radius
        height = body.height
        gluCylinder(quadratic,radius+0.25,radius,height,32,32)
    elif body.shape=="box":
        sx,sy,sz = body.boxsize
        glScale(0.5, 0.5, 0.5)
       # glColorMaterial ( GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE ) ;
        #glEnable ( GL_COLOR_MATERIAL ) ;
        glBegin(GL_QUADS)
        

        #TOP Centre = 0,0.25,0
        glColor3f(0.0,0.0,1.0)
        glVertex3f( sx, sy,-sz)
        glVertex3f(-sx, sy,-sz)
        glVertex3f(-sx, sy, sz)
        glVertex3f( sx, sy, sz) 

        
        #Bottom Centre = 0,-0.25,0
        glColor3f(1.0,0.0,0.0)
        glVertex3f( sx,-sy, sz)
        glVertex3f(-sx,-sy, sz)
        glVertex3f(-sx,-sy,-sz)
        glVertex3f( sx,-sy,-sz)
        
        #Front Centre = 0,0,0.25
        glColor3f(1.0,1.0,1.0)
        glVertex3f( sx, sy, sz)
        glVertex3f(-sx, sy, sz)
        glVertex3f(-sx,-sy, sz)
        glVertex3f( sx,-sy, sz)
 
        # Back Centre = 0,0,-0.25
        glColor3f(1.0,1.0,1.0)
        glVertex3f( sx,-sy,-sz)
        glVertex3f(-sx,-sy,-sz)
        glVertex3f(-sx, sy,-sz)
        glVertex3f( sx, sy,-sz)
 
        # Left Centre =-0.25,0,0
        glColor3f(1.0,1.0,1.0)
        glVertex3f(-sx, sy, sz) 
        glVertex3f(-sx, sy,-sz)
        glVertex3f(-sx,-sy,-sz) 
        glVertex3f(-sx,-sy, sz)  
    
        # Right Centre = 0.25,0,0
        glColor3f(1.0,1.0,1.0)
        glVertex3f( sx, sy,-sz) 
        glVertex3f( sx, sy, sz)
        glVertex3f( sx,-sy, sz)
        glVertex3f( sx,-sy,-sz)

        glEnd()

    glPopMatrix()

def create_cylinder(world, space, density, radius, height):
    
    body = ode.Body(world)
    M = ode.Mass()  
    M.setCappedCylinder(density, direction=1, r = radius, h = height)
    body.setMass(M)
    # Set parameters for drawing the body
    body.shape = "ccylinder"
    body.radius = radius
    body.height = height
# Create a CCylinder geom for collision detection
    geom = ode.GeomCCylinder(space, radius, height)
    geom.setBody(body)
    return body, geom



# create_box
def create_box(world, space, density, lx, ly, lz):
    """Create a box body and its corresponding geom."""

    # Create body
    body = ode.Body(world)
    M = ode.Mass()
    M.setBox(density, lx, ly, lz)
    body.setMass(M)

    # print body.getMass()

    # Set parameters for drawing the body
    body.shape = "box"
    body.id = objcount + 1 
    body.attached = 0
    body.attachedTo = []
    body.facesActivated = []
    body.facesActivatedTo = []
    body.boxsize = (lx, ly, lz)
    body.delayCounter = 0;
    body.state = 'a'

    # Create a box geom for collision detection
    geom = ode.GeomBox(space, lengths=body.boxsize)
    geom.setBody(body)

    return body, geom

def computeForces():
    
    global aveKECounter,aveaveKE;
    global objcount;
    global bodies
    g = 9.81 
    global startTime,a0,a1,a2,a3,f0,f1,f2,f3;
    # global delayCounter;
    aveKE = 0;
    for b in bodies:
        m = ode.Mass()
        m = b.getMass()
        lx,ly,lz = b.boxsize
        
        # print lx,ly,lz

        # print m

        buoyancy = 1000*lx*ly*lz* 9.81
        b.addForce((0,buoyancy,0))  #COG is same as COB for fully submerged cube.
        
        weight = 1000*lx*ly*lz*-9.81
        b.addForce((0,weight,0))
        
        

        
        xCurr1 = random.uniform(-5.0,5.0)
        yCurr1 = random.uniform(-5.0,5.0)
        zCurr1 = random.uniform(-5.0,5.0)



        b.delayCounter = b.delayCounter+1;

        nowTime = time.time() - startTime ;
        # print nowTime;
        for i in range(0,8):
            facezzz = i+1;
            pos = testing(facezzz)


            xVel1,yVel1,zVel1 = b.getRelPointVel(pos)
            xpos,ypos,zpos = b.getRelPointPos(pos)

            rC1 = random.gauss(0,1.0);
            rC2 = random.gauss(0,1.0);

            
            xVel1Trans,yVel1Trans,zVel1Trans = b.vectorFromWorld((xVel1,yVel1,zVel1));
            xCurr1Trans,yCurr1Trans,zCurr1Trans = b.vectorFromWorld((xCurr1,yCurr1,zCurr1));

            xCurrent1 = xVel1Trans - xCurr1Trans;
            yCurrent1 = yVel1Trans - yCurr1Trans;
            zCurrent1 = zVel1Trans - zCurr1Trans;


            xdrag1 = -0.5*1000*ly/2*lz/2*0.8*xCurrent1*math.fabs(xCurrent1)
            ydrag1 = -0.5*1000*lx/2*lz/2*0.8*yCurrent1*math.fabs(yCurrent1)
            zdrag1 = -0.5*1000*lx/2*ly/2*0.8*zCurrent1*math.fabs(zCurrent1)
            if b.delayCounter == 1:
                b.addRelForceAtRelPos((xdrag1,ydrag1,zdrag1),pos);
  
            if b.delayCounter == 15:
                b.delayCounter = 0;



    

def computeImpulse():
    global bodies
    for b in bodies:
        m = ode.Mass()
        m = b.getMass()
        lx,ly,lz = b.boxsize
        xCurr1 = 10.5
        yCurr1 = 0
        zCurr1 = 0
        xCurrent1,yCurrent1,zCurrent1 = b.vectorFromWorld((xCurr1,yCurr1,zCurr1));
        xdrag1 = -0.5*1000*ly*lz*0.5*(xCurrent1)*math.fabs(xCurrent1)
        ydrag1 = -0.5*1000*lx*lz*0.5*(yCurrent1)*math.fabs(yCurrent1)
        zdrag1 = -0.5*1000*lx*ly*0.5*(zCurrent1)*math.fabs(zCurrent1)
        b.addRelForceAtRelPos((xdrag1,ydrag1,zdrag1),(lx/2,0,0))





# drop_object
def drop_object():
    """Drop an object into the scene."""

    global bodies, geoms, counter, objcount

    body, geom = create_box(world, space, 1000,0.15, 0.15,0.15)
    geom.shape = "box"

    gx,gy,gz = world.getGravity()

    body.setPosition((random.uniform(-1.75,1.75),random.uniform(0.25,2.75),random.uniform(-1.75,1.75)))
    theta = random.uniform(0,2*math.pi);
    phi = random.uniform(0,2*math.pi);
    si = random.uniform(0,2*math.pi);

    ct = math.cos (theta)
    st = math.sin (theta)
    cp = math.cos(phi)
    sp = math.sin(phi)
    cs = math.cos(si)
    ss = math.sin(si)
    
    body.setRotation([ct*cs, -ct*ss, st, cp*ss+st * sp*cs, cp*cs - sp*st*ss, -sp*ct, sp*ss - cp*st*cs, sp*cs + cp*st*ss, ct*cp])
    bodies.append(body)
    geoms.append(geom)
    counter=0
    objcount+=1

# pull
def pull():
    """Pull the objects back to the origin.

    Every object will be pulled back to the origin.
    Every couple of frames there'll be a thrust upwards so that
    the objects won't stick to the ground all the time.
    """
    global bodies 
    
    
    for b in bodies:

        if b.attached == 1:
            # x0,y0,z0 = b.getPosition()
            attachedToArray = []
            attachedToArray = b.attachedTo
            # counterPull = 0
            for k in range(0,len(attachedToArray)):
                
                p = f(b.facesActivated[k])
                x0,y0,z0 = b.getRelPointPos(f(b.facesActivated[k]))
                # counterPull = counterPull + 1
                x1,y1,z1 = bodies[attachedToArray[k]-1].getRelPointPos(f(b.facesActivatedTo[k]))
                   
                l=[x1-x0,y1-y0,z1-z0]
                scalp (l, 200 / length (l))
                b.addForceAtRelPos(l,p)
                
               
   

def computeDistanceToDetach():
    global bodies 
    for b in bodies:
        if b.attached == 1:

            # counterPull = 0
            k = 0;
            while k < len(b.attachedTo): 

                x0,y0,z0 = b.getRelPointPos(f(b.facesActivated[k]))
                x1,y1,z1 = bodies[b.attachedTo[k]-1].getRelPointPos(f(b.facesActivatedTo[k]))
                   
                l=[x1-x0,y1-y0,z1-z0]
                if len(bodies[b.attachedTo[k]-1].attachedTo) > 0: 
                    if length(l) > 0.1:
                        if len(b.attachedTo) == 1:
                            b.attached = 0;
                        if len(bodies[b.attachedTo[k]-1].attachedTo) == 1:
                            bodies[b.attachedTo[k]-1].attached = 0; 

                        bodies[b.attachedTo[k]-1].attachedTo.remove(b.id);
                        bodies[b.attachedTo[k]-1].facesActivated.remove(b.facesActivatedTo[k]);
                        bodies[b.attachedTo[k]-1].facesActivatedTo.remove(b.facesActivated[k]);

                        b.attachedTo.pop(k);
                        b.facesActivated.pop(k);
                        b.facesActivatedTo.pop(k);
                        k = k-1
                        print "faces detached"
                k = k+1 


    


# Collision callback
def near_callback(args, geom1, geom2):
    """Callback function for the collide() method.

    This function checks if the given geoms do collide and
    creates contact joints if they do.
    """
    # Check if the objects do collide
    contacts = ode.collide(geom1, geom2)

    #pos, normal, depth, geom1, geom2 = contacts[1].getContactGeomParams()
    # Create contact joints
    global COUNTERFUCKER, X
    global pullCounter,finishAssembly
    world,contactgroup = args
    global bodies;

    
    b1 = ode.Body(world)
    b2 = ode.Body(world)
    
    b1 = geom1.getBody()
    b2 = geom2.getBody()
    
    pointOfContacts1 = []
    pointOfContacts2 = []

    face1 = []
    face2 = []

    X = 0

    if (geom1.shape == "floor" and geom2.shape == "box") or (geom1.shape == "box" and geom2.shape == "floor") :
        for c in contacts:
            c.setBounce(0.2)
            c.setMu(5000)
            pos, normal, depth, geom1, geom2 = c.getContactGeomParams()
            j = ode.ContactJoint(world, contactgroup, c)
            j.attach(geom1.getBody(), geom2.getBody())



    if (geom2.shape == "box") and (geom1.shape == "box"): 
    
        for c in contacts:
        
            c.setBounce(0.2)
            c.setMu(5000)
            pos, normal, depth, geom1, geom2 = c.getContactGeomParams()
            
            if b2.id not in b1.attachedTo:
                X = X+1
                        
                centreOfBox1 = geom1.getPosition();
                centreOfBox2 = geom2.getPosition();
                posRelBox1 = np.subtract(pos,centreOfBox1);
                posRelBox2 = np.subtract(pos,centreOfBox2);



                x = geom2.getRotation()
                y = geom1.getRotation()    
                    
                matrixX2 = np.matrix([[x[0],x[1],x[2],centreOfBox2[0]],[x[3],x[4],x[5],centreOfBox2[1]],[x[6],x[7],x[8],centreOfBox2[2]],[0.0,0.0,0.0,1.0]]) 
                matrixX2 = np.linalg.inv(matrixX2)
                    
                matrixX1 = np.matrix([[y[0],y[1],y[2],centreOfBox1[0] ],[y[3],y[4],y[5],centreOfBox1[1]],[y[6],y[7],y[8],centreOfBox1[2]],[0.0,0.0,0.0,1.0]]) 
                matrixX1 = np.linalg.inv(matrixX1)     
                    
                matrixY = np.matrix([[pos[0]],[pos[1]],[pos[2]],[1]])

                posRelBox2 = matrixX2 * matrixY
                posRelBox1 = matrixX1 * matrixY

                pointOfContacts1.append(posRelBox1)
                pointOfContacts2.append(posRelBox2)
                             

            j1 = ode.ContactJoint(world, contactgroup, c)
            j1.attach(geom1.getBody(), geom2.getBody())

        if X > 0:
            face1,face2 = findFace(pointOfContacts1,pointOfContacts2,geom1,geom2)
        # print len(pointOfContacts1)
        # print face1 
        # print face2

        distanceBetweenFaces = []
        distanceOfInterest = -0.1
        activatedFace1 = 0
        activatedFace2 = 0
        listOfFaces1 = [3,4,5,6];
        listOfFaces2 = [1,2];
    
        if len(face1) > 0 and len(face2) > 0: 
            if b1.state == 'a' and b2.state == 'a':
                for i in range(len(face1)):
                    for j in range(len(face2)):
                        xpos1,ypos1,zpos1 = b1.getRelPointPos(f(face1[i]))
                        xpos2,ypos2,zpos2 = b2.getRelPointPos(f(face2[j]))
                        dist = sqrt( (xpos1-xpos2)**2 + (ypos1-ypos2)**2 + (zpos1-zpos2)**2 ) 
                        # print "distance between face ", face1[i], "of block 1 and face ", face2[j], " of block 2 is ", dist  
                        distanceBetweenFaces.append(dist)
                        if (face1[i] == 1 or face1[i] == 2) and (face2[j] == 1 or face2[j] == 2):
                            distanceOfInterest = dist
                            norm1 = b1.vectorToWorld(f(face1[i]))
                            norm2 = b2.vectorToWorld(f(face2[j]))
                            activatedFace1 = face1[i]
                            activatedFace2 = face2[j]
                            theta = math.atan2(np.linalg.norm(np.cross(norm1,norm2)),np.dot(norm1,norm2))
                            theta = theta * 180/math.pi

                if len(distanceBetweenFaces) > 0:
                    if distanceOfInterest == min(distanceBetweenFaces) and theta > 135.0:
                        if (b2.id not in b1.attachedTo) and (activatedFace1 not in b1.facesActivated) and (activatedFace2 not in b2.facesActivated):
                                b1.state = 'b' 
                                b2.state = 'b'
                                b1.attached = 1
                                b2.attached = 1
                                b1.attachedTo.append(b2.id)
                                b2.attachedTo.append(b1.id)
                                b1.facesActivated.append(activatedFace1)
                                b1.facesActivatedTo.append(activatedFace2)
                                b2.facesActivated.append(activatedFace2)
                                b2.facesActivatedTo.append(activatedFace1)

            if b1.state == 'b' and b2.state == 'b':
                for i in range(len(face1)):
                    for j in range(len(face2)):
                        if ((face1[i] in listOfFaces1 and face2[j] in listOfFaces2) or (face1[i] in listOfFaces2 and face2[j] in listOfFaces1)): 
                            
                            norm1 = b1.vectorToWorld(f(face1[i]))
                            norm2 = b2.vectorToWorld(f(face2[j]))
                            activatedFace1 = face1[i]
                            activatedFace2 = face2[j]
                            theta = math.atan2(np.linalg.norm(np.cross(norm1,norm2)),np.dot(norm1,norm2))
                            theta = theta * 180/math.pi
                                   
                            if (b2.id not in b1.attachedTo) and theta > 135.0 and (activatedFace1 not in b1.facesActivated) and (activatedFace2 not in b2.facesActivated):
                                b1.state = 'c';
                                b2.state = 'c';
                                bodies[b1.attachedTo[0] - 1].state = 'd';
                                bodies[b2.attachedTo[0] - 1].state = 'd';

                                b1.attachedTo.append(b2.id)
                                b2.attachedTo.append(b1.id)
                                b1.facesActivated.append(activatedFace1)
                                b1.facesActivatedTo.append(activatedFace2)
                                b2.facesActivated.append(activatedFace2)
                                b2.facesActivatedTo.append(activatedFace1)
                                print "LShape Complete, Adding Final Block"
                                # os._exit(1);

            if ((b1.state == 'a' and b2.state == 'c') or (b2.state == 'a' and b1.state == 'c')):
                if b2.state == 'c':
                    for i in range(len(face1)):
                        for j in range(len(face2)):
                            if (face2[j] in listOfFaces2 and face1[i] in listOfFaces2):
                                norm1 = b1.vectorToWorld(f(face1[i]))
                                norm2 = b2.vectorToWorld(f(face2[j]))
                                activatedFace1 = face1[i]
                                activatedFace2 = face2[j]
                                theta = math.atan2(np.linalg.norm(np.cross(norm1,norm2)),np.dot(norm1,norm2))
                                theta = theta * 180/math.pi
               
                                if (b2.id not in b1.attachedTo) and theta > 135.0  and (activatedFace1 not in b1.facesActivated) and (activatedFace2 not in b2.facesActivated):
                                    b1.attached = 1
                                    b2.attached = 1

                                    b2.state = 'e';
                                    b1.state = 'd';
                                    b1.attachedTo.append(b2.id)
                                    b2.attachedTo.append(b1.id)
                                    b1.facesActivated.append(activatedFace1)
                                    b1.facesActivatedTo.append(activatedFace2)
                                    b2.facesActivated.append(activatedFace2)
                                    b2.facesActivatedTo.append(activatedFace1)
                                    print "Final Assembly over"
                                    os._exit(1);
                
                if b1.state == 'c':
                    for i in range(len(face1)):
                        for j in range(len(face2)):
                            if (face1[i] in listOfFaces2 and face2[j] in listOfFaces2):
                                norm1 = b1.vectorToWorld(f(face1[i]))
                                norm2 = b2.vectorToWorld(f(face2[j]))
                                activatedFace1 = face1[i]
                                activatedFace2 = face2[j]
                                theta = math.atan2(np.linalg.norm(np.cross(norm1,norm2)),np.dot(norm1,norm2))
                                theta = theta * 180/math.pi
               
                                if (b2.id not in b1.attachedTo) and theta > 135.0  and (activatedFace1 not in b1.facesActivated) and (activatedFace2 not in b2.facesActivated):
                                    b1.attached = 1
                                    b2.attached = 1

                                    b1.state = 'e';
                                    b2.state = 'd';
                                    b1.attachedTo.append(b2.id)
                                    b2.attachedTo.append(b1.id)
                                    b1.facesActivated.append(activatedFace1)
                                    b1.facesActivatedTo.append(activatedFace2)
                                    b2.facesActivated.append(activatedFace2)
                                    b2.facesActivatedTo.append(activatedFace1)
                                    print "Final Assembly over"
                                    os._exit(1);


def f(x):

    global bodies

    lx,ly,lz = bodies[0].boxsize

    return {
        1: (0.0,ly/2,0.0),
        2: (0.0,-ly/2,0.0),
        3: (lx/2,0.0,0.0),
        4: (-lx/2,0.0,0.0),
        5: (0.0,0.0,lz/2),
        6: (0.0,0.0,-lz/2),
        }.get(x, (0.0,0.0,0.0))

def testing(x):

    global bodies

    lx,ly,lz = bodies[0].boxsize

    return {
        1: (lx/4,ly/4,lz/4),
        2: (lx/4,ly/4,-lz/4),
        3: (lx/4,-ly/4,lz/4),
        4: (lx/4,-ly/4,-lz/4),
        5: (-lx/4,ly/4,lz/4),
        6: (-lx/4,ly/4,-lz/4),
        7: (-lx/4,-ly/4,lz/4),
        8: (-lx/4,-ly/4,-lz/4),
        }.get(x, (0.0,0.0,0.0))

def approx_equal(a, b, tol):

    x = 0

    if abs(a-b) < tol:
        x = 1

    return x   
   


def findFace(pointOfContacts1,pointOfContacts2,geom1,geom2):

    b1 = ode.Body(world)

    b1 = geom1.getBody()

    lx,ly,lz = b1.boxsize

    #print "Number of points of contacts = "
    l = len(pointOfContacts1)
    xArray = []
    yArray = []
    zArray = []

    xxArray = []
    yyArray = []
    zzArray = []
    

    for p in range(0,l):

          x1 = pointOfContacts1[p][0] 
          y1 = pointOfContacts1[p][1]
          z1 = pointOfContacts1[p][2]  
          
          x = 0.0
          y = 0.0
          z = 0.0

          if approx_equal(x1.item(0),lx/2,0.01):
            x = 0.25
          elif approx_equal(x1.item(0),-lx/2,0.01):
            x = -0.25 
                
          if approx_equal(y1.item(0),ly/2,0.01):
            y = 0.25
          elif approx_equal(y1.item(0),-ly/2,0.01):
            y = -0.25

          if approx_equal(z1.item(0),lz/2,0.01):
            z = 0.25
          elif approx_equal(z1.item(0),-lz/2,0.01):
            z = -0.25
        
          xArray.append(x)
          yArray.append(y)
          zArray.append(z)

          xx1 = pointOfContacts2[p][0] 
          yy1 = pointOfContacts2[p][1]
          zz1 = pointOfContacts2[p][2]  
          
          xx = 0.0
          yy = 0.0
          zz = 0.0

          if approx_equal(xx1.item(0),lx/2,0.01):
            xx = 0.25
          elif approx_equal(xx1.item(0),-lx/2,0.01):
            xx = -0.25 
                
          if approx_equal(yy1.item(0),ly/2,0.01):
            yy = 0.25
          elif approx_equal(yy1.item(0),-ly/2,0.01):
            yy = -0.25

          if approx_equal(zz1.item(0),lz/2,0.01):
            zz = 0.25
          elif approx_equal(zz1.item(0),-lz/2,0.01):
            zz = -0.25
        
          xxArray.append(xx)
          yyArray.append(yy)
          zzArray.append(zz)
  
    
    iter1 = returnFace(xArray,yArray,zArray)
    iter2 = returnFace(xxArray,yyArray,zzArray)





    s1 = len(iter1)
    s2 = len(iter2)

    

    return iter1,iter2

def returnFace(xArray,yArray,zArray):

    # Top face 1
    # Bottom Face 2
    # Right Face 3
    # Left Face 4
    # Front Face 5
    # Back Face 6 

    l = len(xArray)
    face = []
    
    if l == 1:

        if xArray[0] == 0.25:
            face.append(3)
        elif xArray[0] == -0.25:
            face.append(4)
   
        if yArray[0] == 0.25:
            face.append(1)
        elif yArray[0] == -0.25:
            face.append(2)
    
        if zArray[0] == 0.25:
            face.append(5)
        elif zArray[0] == -0.25:
            face.append(6)    

    if l >= 2:
        if xArray[1:] == xArray[:-1]:
            if xArray[0] == 0.25:
                face.append(3)
            elif xArray[0] == -0.25:
                face.append(4)
        elif yArray[1:] == yArray[:-1]:
            if yArray[0] == 0.25:
                face.append(1)
            elif yArray[0] == -0.25:
                face.append(2)
        elif zArray[1:] == zArray[:-1]:
            if zArray[0] == 0.25:
                face.append(5)
            elif zArray[0] == -0.25:
                face.append(6)

    if face == 0:
        print xArray,yArray,zArray
    
    return face



######################################################################

# Initialize Glut
glutInit ([])

# Open a window
glutInitDisplayMode (GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH)

x = 200
y = 1000
width = 640
height = 480
glutInitWindowPosition (x, y);
glutInitWindowSize (width, height);
glutCreateWindow ("testode")

# Create a world object
world = ode.World()
world.setGravity((0.0,0.0,0.0))
world.setERP(0.8)
world.setCFM(1E-9)

# Create a space object
space = ode.Space()


# Create a plane geom which prevent the objects from falling forever

floor = ode.GeomPlane(space, (0,1,0), 0)
floor.shape = "floor"


# create a tank to run the simulation 
#Left wall
leftWall = ode.GeomPlane(space, (1,0,0),-2.0)
leftWall.shape = "floor"

# # #Right wall
rightWall = ode.GeomPlane(space, (-1,0,0),-2.0)
rightWall.shape = "floor"

# #Front wall


# #Back Wall
frontWall = ode.GeomPlane(space, (0,0,-1),-2.0)
frontWall.shape = "floor"

backWall = ode.GeomPlane(space, (0,0,1),-2.0)
backWall.shape = "floor"

# ceiling 

ceilingWall = ode.GeomPlane(space,(0,-1,0),-3.0)
ceilingWall.shape = "floor" 

#floor.setBody(body_Floor)
# A list with ODE bodies


# The geoms for each of the bodies


# A joint group for the contact joints that are generated whenever
# two bodies collide
contactgroup = ode.JointGroup()

# Some variables used inside the simulation loop
fps = 150
dt = 1.0/fps
running = True
state = 0
counter = 0
objcount = 0
lasttime = time.time()


# keyboard callback
def _keyfunc (c, x, y):
    sys.exit (0)

glutKeyboardFunc (_keyfunc)

# draw callback
def _drawfunc ():
    # Draw the scene

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    # glLoadIdentity()
    

    global bodies 

    prepare_GL()
    for b in bodies:
        draw_body(b)

    glutSwapBuffers ()

glutDisplayFunc (_drawfunc)

    

# idle callback
def _idlefunc ():
    global counter, state, lasttime

    
    
    t = dt - (time.time() - lasttime)
    if (t > 0):
        time.sleep(t)

    objcount = 60;

    if state==0:
            for ii in range(0,objcount):
                drop_object()
            state=1

    # State 1: Explosion and pulling back the objects
    elif state==1:
        
        # computeDistanceToDetach()
        computeForces()
        pull()




    glutPostRedisplay ()

    # Simulate
    n = 2


    for i in range(n):
        # Detect collisions and create contact joints
        space.collide((world,contactgroup), near_callback)
        
        # Simulation step
        world.step(dt)

        # Remove all contact joints
        contactgroup.empty()

    lasttime = time.time()
    

glutIdleFunc (_idlefunc)
glutMainLoop ()
