#!/usr/bin/env python 

"""
TurnkeyLaserExporter

-----------------------------------
Maintained by Turnkey Tyranny (https://github.com/TurnkeyTyranny/laser-gcode-exporter-inkscape-plugin)
Designed to run on Ramps 1.4 + Marlin firmware on a K40 CO2 Laser Cutter.
Based on think|haus gcode inkscape extension
Based on a script by Nick Drobchenko from the CNC club

***

Copyright (C) 2009 Nick Drobchenko, nick@cnc-club.ru
based on gcode.py (C) 2007 hugomatic... 
based on addnodes.py (C) 2005,2007 Aaron Spike, aaron@ekips.org
based on dots.py (C) 2005 Aaron Spike, aaron@ekips.org
based on interp.py (C) 2005 Aaron Spike, aaron@ekips.org

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
"""

"""

Changelog 2015-02-01:
* Beginning of the project. Based on a fork from ShinyLaser(https://github.com/ajfoul/thlaser-inkscape-plugin)

Changelog 2015-02-16: 
Added an option to export as Marlin or Smoothie Power levels

Changelog 2015-03-07: 
Added capability to pick out power, ppm, feedrate etc from the layer names
Added code to support Pulse Per Minute burning or continuous burning. Will default to continuous.
M649 S100 L300 P10 - Set Laser settings to 100 percent power, pulses are each 300ms, and 10 pulses per mm.	
G0 : Move to a new location with the laser off.
G1 : Move to a new location with the laser on.
G2 : Move in a Clockwise Arc   
G3 : Move in a Counter Clockwise Arc
Name your layer like 10 [feed=600,ppm=40] for 10% power, 600mm per minute cut and 40 pulse per millimetre at 60ms duration
"""

###
###        Gcode tools
###

import inkex, simplestyle, simplepath
import cubicsuperpath, simpletransform, bezmisc

import os
import math
import bezmisc
import re
import copy
import sys
import time

import base64
from PIL import Image
import ImageOps

import getopt
from io import BytesIO
#_ = inkex._


################################################################################
###
###        Constants
###
################################################################################

VERSION = "1.0.1"

STRAIGHT_TOLERANCE = 0.0001
STRAIGHT_DISTANCE_TOLERANCE = 0.0001
LASER_ON = "M3 ;turn the laser on"          # LASER ON MCODE
LASER_OFF = "M5 ;turn the laser off\n"        # LASER OFF MCODE

HEADER_TEXT = ""
FOOTER_TEXT = ""

BIARC_STYLE = {
        'biarc0':    simplestyle.formatStyle({ 'stroke': '#88f', 'fill': 'none', 'stroke-width':'1' }),
        'biarc1':    simplestyle.formatStyle({ 'stroke': '#8f8', 'fill': 'none', 'stroke-width':'1' }),
        'line':        simplestyle.formatStyle({ 'stroke': '#f88', 'fill': 'none', 'stroke-width':'1' }),
        'area':        simplestyle.formatStyle({ 'stroke': '#777', 'fill': 'none', 'stroke-width':'0.1' }),
    }

# Inkscape group tag
SVG_GROUP_TAG = inkex.addNS("g", "svg")
SVG_PATH_TAG = inkex.addNS('path','svg')
SVG_IMAGE_TAG = inkex.addNS('image')
SVG_LABEL_TAG = inkex.addNS("label", "inkscape")


GCODE_EXTENSION = ".g" # changed to be Marlin friendly (ajf)

options = {}

################################################################################
###
###        Common functions
###
################################################################################


###
###        Just simple output function for better debugging
###

class Logger(object):
    first = True
    enabled = True
    def __init__(self):
        home = os.getenv("HOME") or os.getenv("USERPROFILE")
        self.logpath = os.path.join(home, "thlaser.log")

    def write(self, s):
        if (not self.enabled):
            return

        if self.first and os.path.isfile(self.logpath):
            os.remove(self.logpath)
        self.first = False

        f = open(self.logpath, "a")
        f.write(str(s)+"\n")
        f.close()

# The global logger object
logger = Logger()


###
###        Point (x,y) operations
###
## Pretty much what it sounds like: defines some arithmetic functions that can be applied to points.
class P:
    def __init__(self, x, y=None):
        if not y==None:
            self.x, self.y = float(x), float(y)
        else:
            self.x, self.y = float(x[0]), float(x[1])
    def __add__(self, other): return P(self.x + other.x, self.y + other.y)
    def __sub__(self, other): return P(self.x - other.x, self.y - other.y)
    def __neg__(self): return P(-self.x, -self.y)
    def __mul__(self, other):
        if isinstance(other, P):
            return self.x * other.x + self.y * other.y
        return P(self.x * other, self.y * other)
    __rmul__ = __mul__
    def __div__(self, other): return P(self.x / other, self.y / other)
    def mag(self): return math.hypot(self.x, self.y)
    def unit(self):
        h = self.mag()
        if h: return self / h
        else: return P(0,0)
    def dot(self, other): return self.x * other.x + self.y * other.y
    def rot(self, theta):
        c = math.cos(theta)
        s = math.sin(theta)
        return P(self.x * c - self.y * s,  self.x * s + self.y * c)
    def angle(self): return math.atan2(self.y, self.x)
    def __repr__(self): return '%f,%f' % (self.x, self.y)
    def pr(self): return "%.2f,%.2f" % (self.x, self.y)
    def to_list(self): return [self.x, self.y]    


###
###        Functions to operate with CubicSuperPath
###

def csp_at_t(sp1,sp2,t):
    bez = (sp1[1][:],sp1[2][:],sp2[0][:],sp2[1][:])
    return     bezmisc.bezierpointatt(bez,t)

def cspbezsplit(sp1, sp2, t = 0.5):
    s1,s2 = bezmisc.beziersplitatt((sp1[1],sp1[2],sp2[0],sp2[1]),t)
    return [ [sp1[0][:], sp1[1][:], list(s1[1])], [list(s1[2]), list(s1[3]), list(s2[1])], [list(s2[2]), sp2[1][:], sp2[2][:]] ]
    
def cspbezsplitatlength(sp1, sp2, l = 0.5, tolerance = 0.01):
    bez = (sp1[1][:],sp1[2][:],sp2[0][:],sp2[1][:])
    t = bezmisc.beziertatlength(bez, l, tolerance)
    return cspbezsplit(sp1, sp2, t)    
    
def cspseglength(sp1,sp2, tolerance = 0.001):
    bez = (sp1[1][:],sp1[2][:],sp2[0][:],sp2[1][:])
    return bezmisc.bezierlength(bez, tolerance)    

def csplength(csp):
    total = 0
    lengths = []
    for sp in csp:
        for i in xrange(1,len(sp)):
            l = cspseglength(sp[i-1],sp[i])
            lengths.append(l)
            total += l            
    return lengths, total


###
###        Distance calculattion from point to arc
###

def between(c,x,y):
        return x-STRAIGHT_TOLERANCE<=c<=y+STRAIGHT_TOLERANCE or y-STRAIGHT_TOLERANCE<=c<=x+STRAIGHT_TOLERANCE

def distance_from_point_to_arc(p, arc):
    P0,P2,c,a = arc
    dist = None
    p = P(p)
    r = (P0-c).mag()
    if r>0 :
        i = c + (p-c).unit()*r
        alpha = ((i-c).angle() - (P0-c).angle())
        if a*alpha<0: 
            if alpha>0:    alpha = alpha-2*math.pi
            else: alpha = 2*math.pi+alpha
        if between(alpha,0,a) or min(abs(alpha),abs(alpha-a))<STRAIGHT_TOLERANCE : 
            return (p-i).mag(), [i.x, i.y]
        else : 
            d1, d2 = (p-P0).mag(), (p-P2).mag()
            if d1<d2 : 
                return (d1, [P0.x,P0.y])
            else :
                return (d2, [P2.x,P2.y])

def get_distance_from_csp_to_arc(sp1,sp2, arc1, arc2, tolerance = 0.001 ): # arc = [start,end,center,alpha]
    n, i = 10, 0
    d, d1, dl = (0,(0,0)), (0,(0,0)), 0
    while i<1 or (abs(d1[0]-dl[0])>tolerance and i<2):
        i += 1
        dl = d1*1    
        for j in range(n+1):
            t = float(j)/n
            p = csp_at_t(sp1,sp2,t) 
            d = min(distance_from_point_to_arc(p,arc1), distance_from_point_to_arc(p,arc2))
            d1 = max(d1,d)
        n=n*2
    return d1[0]

################################################################################
###
###        Biarc function
###
###        Calculates biarc approximation of cubic super path segment
###        splits segment if needed or approximates it with straight line
###
################################################################################


def biarc(sp1, sp2, z1, z2, depth=0,):
    def biarc_split(sp1,sp2, z1, z2, depth): 
        if depth<options.biarc_max_split_depth:
            sp1,sp2,sp3 = cspbezsplit(sp1,sp2)
            l1, l2 = cspseglength(sp1,sp2), cspseglength(sp2,sp3)
            if l1+l2 == 0 : zm = z1
            else : zm = z1+(z2-z1)*l1/(l1+l2)
            return biarc(sp1,sp2,depth+1,z1,zm)+biarc(sp2,sp3,depth+1,z1,zm)
        else: return [ [sp1[1],'line', 0, 0, sp2[1], [z1,z2]] ]

    P0, P4 = P(sp1[1]), P(sp2[1])
    TS, TE, v = (P(sp1[2])-P0), -(P(sp2[0])-P4), P0 - P4
    tsa, tea, va = TS.angle(), TE.angle(), v.angle()
    if TE.mag()<STRAIGHT_DISTANCE_TOLERANCE and TS.mag()<STRAIGHT_DISTANCE_TOLERANCE:    
        # Both tangents are zerro - line straight
        return [ [sp1[1],'line', 0, 0, sp2[1], [z1,z2]] ]
    if TE.mag() < STRAIGHT_DISTANCE_TOLERANCE:
        TE = -(TS+v).unit()
        r = TS.mag()/v.mag()*2
    elif TS.mag() < STRAIGHT_DISTANCE_TOLERANCE:
        TS = -(TE+v).unit()
        r = 1/( TE.mag()/v.mag()*2 )
    else:    
        r=TS.mag()/TE.mag()
    TS, TE = TS.unit(), TE.unit()
    tang_are_parallel = ((tsa-tea)%math.pi<STRAIGHT_TOLERANCE or math.pi-(tsa-tea)%math.pi<STRAIGHT_TOLERANCE )
    if ( tang_are_parallel  and 
                ((v.mag()<STRAIGHT_DISTANCE_TOLERANCE or TE.mag()<STRAIGHT_DISTANCE_TOLERANCE or TS.mag()<STRAIGHT_DISTANCE_TOLERANCE) or
                    1-abs(TS*v/(TS.mag()*v.mag()))<STRAIGHT_TOLERANCE)    ):
                # Both tangents are parallel and start and end are the same - line straight
                # or one of tangents still smaller then tollerance

                # Both tangents and v are parallel - line straight
        return [ [sp1[1],'line', 0, 0, sp2[1], [z1,z2]] ]

    c,b,a = v*v, 2*v*(r*TS+TE), 2*r*(TS*TE-1)
    if v.mag()==0:
        return biarc_split(sp1, sp2, z1, z2, depth)
    asmall, bsmall, csmall = abs(a)<10**-10,abs(b)<10**-10,abs(c)<10**-10 
    if         asmall and b!=0:    beta = -c/b
    elif     csmall and a!=0:    beta = -b/a 
    elif not asmall:     
        discr = b*b-4*a*c
        if discr < 0:    raise ValueError, (a,b,c,discr)
        disq = discr**.5
        beta1 = (-b - disq) / 2 / a
        beta2 = (-b + disq) / 2 / a
        if beta1*beta2 > 0 :    raise ValueError, (a,b,c,disq,beta1,beta2)
        beta = max(beta1, beta2)
    elif    asmall and bsmall:    
        return biarc_split(sp1, sp2, z1, z2, depth)
    alpha = beta * r
    ab = alpha + beta 
    P1 = P0 + alpha * TS
    P3 = P4 - beta * TE
    P2 = (beta / ab)  * P1 + (alpha / ab) * P3

    def calculate_arc_params(P0,P1,P2):
        D = (P0+P2)/2
        if (D-P1).mag()==0: return None, None
        R = D - ( (D-P0).mag()**2/(D-P1).mag() )*(P1-D).unit()
        p0a, p1a, p2a = (P0-R).angle()%(2*math.pi), (P1-R).angle()%(2*math.pi), (P2-R).angle()%(2*math.pi)
        alpha =  (p2a - p0a) % (2*math.pi)                    
        if (p0a<p2a and  (p1a<p0a or p2a<p1a))    or    (p2a<p1a<p0a) : 
            alpha = -2*math.pi+alpha 
        if abs(R.x)>1000000 or abs(R.y)>1000000  or (R-P0).mag<options.min_arc_radius :
            return None, None
        else :    
            return  R, alpha
    R1,a1 = calculate_arc_params(P0,P1,P2)
    R2,a2 = calculate_arc_params(P2,P3,P4)
    if R1==None or R2==None or (R1-P0).mag()<STRAIGHT_TOLERANCE or (R2-P2).mag()<STRAIGHT_TOLERANCE    : return [ [sp1[1],'line', 0, 0, sp2[1], [z1,z2]] ]
    
    d = get_distance_from_csp_to_arc(sp1,sp2, [P0,P2,R1,a1],[P2,P4,R2,a2])
    if d > options.biarc_tolerance and depth<options.biarc_max_split_depth     : return biarc_split(sp1, sp2, z1, z2, depth)
    else:
        if R2.mag()*a2 == 0 : zm = z2
        else : zm  = z1 + (z2-z1)*(R1.mag()*a1)/(R2.mag()*a2+R1.mag()*a1)  
        return [    [ sp1[1], 'arc', [R1.x,R1.y], a1, [P2.x,P2.y], [z1,zm] ], [ [P2.x,P2.y], 'arc', [R2.x,R2.y], a2, [P4.x,P4.y], [zm,z2] ]        ]



################################################################################
###
###        Inkscape helper functions
###
################################################################################

# Returns true if the given node is a layer
def is_layer(node):
    return (node.tag == SVG_GROUP_TAG and
            node.get(inkex.addNS("groupmode", "inkscape")) == "layer")

def get_layers(document):
    layers = []
    root = document.getroot()
    for node in root.iterchildren():
        if (is_layer(node)):
            # Found an inkscape layer
            layers.append(node)
    return layers

def parse_layer_name(txt):
    params = {}
    try:
        n = txt.index("[")
    except ValueError:
        layerName = txt.strip()
    else:
        layerName = txt[0:n].strip()
        args = txt[n+1:].strip()
        if (args.endswith("]")): 
            args = args[0:-1]

        for arg in args.split(","):
            try:
                (field, value) = arg.split("=")
            except:
                raise ValueError("Invalid argument in layer '%s'" % layerName)
            if (field == "feed" or field == "ppm"):
                try:
                    value = float(value)
                except:
                    raise ValueError("Invalid layer name '%s'" % value)
            params[field] = value
            logger.write("%s == %s" % (field, value))

    return (layerName, params)

################################################################################
###
###        Gcode tools class
###
################################################################################




class Gcode_tools(inkex.Effect):

    def __init__(self):
        inkex.Effect.__init__(self)

        outdir = os.getenv("HOME") or os.getenv("USERPROFILE")
        if (outdir):
            outdir = os.path.join(outdir, "Desktop")
        else:
            outdir = os.getcwd()

        self.OptionParser.add_option("-d", "--directory",                action="store", type="string",         dest="directory", default=outdir,                help="Directory for gcode file")
        self.OptionParser.add_option("-f", "--filename",                action="store", type="string",         dest="file", default="-1.0",                    help="File name")            
        self.OptionParser.add_option("-u", "--Xscale",                    action="store", type="float",         dest="Xscale", default="1.0",                    help="Scale factor X")    
        self.OptionParser.add_option("-v", "--Yscale",                    action="store", type="float",         dest="Yscale", default="1.0",                    help="Scale factor Y")
        self.OptionParser.add_option("-x", "--Xoffset",                    action="store", type="float",         dest="Xoffset", default="0.0",                    help="Offset along X")    
        self.OptionParser.add_option("-y", "--Yoffset",                    action="store", type="float",         dest="Yoffset", default="0.0",                    help="Offset along Y")
        # added move (laser off) feedrate and laser intensity; made all int rather than float - (ajf)																								   

        self.OptionParser.add_option("-m", "--Mfeed",                    action="store", type="int",         dest="Mfeed", default="2000",                        help="Default Move Feed rate in unit/min")
        self.OptionParser.add_option("-p", "--feed",                    action="store", type="int",         dest="feed", default="300",                        help="Default Cut Feed rate in unit/min")
        self.OptionParser.add_option("-l", "--laser",                    action="store", type="int",         dest="laser", default="10",                        help="Default Laser intensity (0-100 %)")
        self.OptionParser.add_option("-b",   "--homebefore",                 action="store", type="inkbool",    dest="homebefore", default=True, help="Home all beofre starting (G28)")
        self.OptionParser.add_option("-a",   "--homeafter",                 action="store", type="inkbool",    dest="homeafter", default=False, help="Home X Y at end of job")


        self.OptionParser.add_option("",   "--biarc-tolerance",            action="store", type="float",         dest="biarc_tolerance", default="1",        help="Tolerance used when calculating biarc interpolation.")                
        self.OptionParser.add_option("",   "--biarc-max-split-depth",    action="store", type="int",         dest="biarc_max_split_depth", default="4",        help="Defines maximum depth of splitting while approximating using biarcs.")                

        self.OptionParser.add_option("",   "--unit",                    action="store", type="string",         dest="unit", default="G21 (All units in mm)\n",    help="Units")
        self.OptionParser.add_option("",   "--function",                action="store", type="string",         dest="function", default="Curve",                help="What to do: Curve|Area|Area inkscape")
        self.OptionParser.add_option("",   "--tab",                        action="store", type="string",         dest="tab", default="",                            help="Means nothing right now. Notebooks Tab.")
        self.OptionParser.add_option("",   "--generate_not_parametric_code",action="store", type="inkbool",    dest="generate_not_parametric_code", default=False,help="Generated code will be not parametric.")        
        self.OptionParser.add_option("",   "--double_sided_cutting",action="store", type="inkbool",    dest="double_sided_cutting", default=False,help="Generate code for double-sided cutting.")
        self.OptionParser.add_option("",   "--draw-curves",                action="store", type="inkbool",    dest="drawCurves", default=False,help="Draws curves to show what geometry was processed")        
        self.OptionParser.add_option("",   "--logging",                 action="store", type="inkbool",    dest="logging", default=False, help="Enable output logging from the plugin")

        self.OptionParser.add_option("",   "--loft-distances",            action="store", type="string",         dest="loft_distances", default="10",            help="Distances between paths.")
        self.OptionParser.add_option("",   "--loft-direction",            action="store", type="string",         dest="loft_direction", default="crosswise",        help="Direction of loft's interpolation.")
        self.OptionParser.add_option("",   "--loft-interpolation-degree",action="store", type="float",        dest="loft_interpolation_degree", default="2",    help="Which interpolation use to loft the paths smooth interpolation or staright.")

        self.OptionParser.add_option("",   "--min-arc-radius",            action="store", type="float",         dest="min_arc_radius", default="0.0005",            help="All arc having radius less than minimum will be considered as straight line")        
        self.OptionParser.add_option("",   "--mainboard",                    action="store", type="string",         dest="mainboard", default="ramps",    help="Mainboard")
	    
		
    def parse_curve(self, path):
#        if self.options.Xscale!=self.options.Yscale:
#            xs,ys = self.options.Xscale,self.options.Yscale
#            self.options.Xscale,self.options.Yscale = 1.0, 1.0
#        else :
        xs,ys = 1.0,1.0

#            ### Sort to reduce Rapid distance    
#            np = [p[0]]
#            del p[0]
#            while len(p)>0:
#                end = np[-1][-1][1]
#                dist = None
#                for i in range(len(p)):
#                    start = p[i][0][1]
#    
#                    dist = max(   ( -( ( end[0]-start[0])**2+(end[1]-start[1])**2 ) ,i)    ,   dist )
#                np += [p[dist[1]][:]]
#                del p[dist[1]]
#            p = np[:]        

        lst = []
        for subpath in path:
            lst.append(
                [[subpath[0][1][0]*xs, subpath[0][1][1]*ys], 'move', 0, 0]
            )
            for i in range(1,len(subpath)):
                sp1 = [  [subpath[i-1][j][0]*xs, subpath[i-1][j][1]*ys] for j in range(3)]
                sp2 = [  [subpath[i  ][j][0]*xs, subpath[i  ][j][1]*ys] for j in range(3)]
                lst += biarc(sp1,sp2,0,0)

            lst.append(
                [[subpath[-1][1][0]*xs, subpath[-1][1][1]*ys], 'end', 0, 0]
            )

        return lst

    def draw_curve(self, curve, group=None, style=BIARC_STYLE):
        if group==None:
            group = inkex.etree.SubElement( self.biarcGroup, SVG_GROUP_TAG )
        s, arcn = '', 0
        for si in curve:
            if s!='':
                if s[1] == 'line':
                    inkex.etree.SubElement(    group, SVG_PATH_TAG, 
                            {
                                'style': style['line'],
                                'd':'M %s,%s L %s,%s' % (s[0][0], s[0][1], si[0][0], si[0][1]),
                                'comment': str(s)
                            }
                        )
                elif s[1] == 'arc':
                    arcn += 1
                    sp = s[0]
                    c = s[2]
                    a =  ( (P(si[0])-P(c)).angle() - (P(s[0])-P(c)).angle() )%(2*math.pi) #s[3]
                    if s[3]*a<0: 
                            if a>0:    a = a-2*math.pi
                            else: a = 2*math.pi+a
                    r = math.sqrt( (sp[0]-c[0])**2 + (sp[1]-c[1])**2 )
                    a_st = ( math.atan2(sp[0]-c[0],- (sp[1]-c[1])) - math.pi/2 ) % (math.pi*2)
                    if a>0:
                        a_end = a_st+a
                    else: 
                        a_end = a_st*1
                        a_st = a_st+a    
                    inkex.etree.SubElement(    group, inkex.addNS('path','svg'), 
                         {
                            'style': style['biarc%s' % (arcn%2)],
                             inkex.addNS('cx','sodipodi'):        str(c[0]),
                             inkex.addNS('cy','sodipodi'):        str(c[1]),
                             inkex.addNS('rx','sodipodi'):        str(r),
                             inkex.addNS('ry','sodipodi'):        str(r),
                             inkex.addNS('start','sodipodi'):    str(a_st),
                             inkex.addNS('end','sodipodi'):        str(a_end),
                             inkex.addNS('open','sodipodi'):    'true',
                             inkex.addNS('type','sodipodi'):    'arc',
                            'comment': str(s)
                        })
            s = si
    

    def check_dir(self):
        if (os.path.isdir(self.options.directory)):
            if (os.path.isfile(self.options.directory+'/header')):
                f = open(self.options.directory+'/header', 'r')
                self.header = f.read()
                f.close()
            else:
                self.header = HEADER_TEXT
            if (os.path.isfile(self.options.directory+'/footer')):
                f = open(self.options.directory+'/footer','r')
                self.footer = f.read()
                f.close()
            else:
                self.footer = FOOTER_TEXT
        else: 
            inkex.errormsg(("Directory does not exist!"))
            return

    # Turns a list of arguments into gcode-style parameters (eg (1, 2, 3) -> "X1 Y2 Z3"),
    # taking scaling, offsets and the "parametric curve" setting into account
    def make_args(self, c):
        c = [c[i] if i<len(c) else None for i in range(6)]
        if c[5] == 0:
            c[5] = None
        # next few lines generate the stuff at the front of the file - scaling, offsets, etc (adina)
        if self.options.generate_not_parametric_code: 
            s = ["X", "Y", "Z", "I", "J", "K"]
            s1 = ["","","","","",""]

            # my replacement that hopefully makes sense (adina, june 22 2010)
            m = [self.options.Xscale, -self.options.Yscale, 1,
                 self.options.Xscale, -self.options.Yscale, 1]
            a = [self.options.Xoffset, self.options.Yoffset, 0, 0, 0, 0]
        else:
            s = ["X", "Y", "Z", "I", "J", "K"]
            s1 = ["", "", "", "",  "", ""]
            m = [1, -1, 1, 1, -1, 1]
            a = [0, 0, 0, 0, 0, 0]

        a[1] += self.pageHeight

        #I think this is the end of generating the header stuff (adina, june 22 2010)
        args = []
        for i in range(6):
            if c[i]!=None:
                value = self.unitScale*(c[i]*m[i]+a[i])
                args.append(s[i] + ("%f" % value) + s1[i])
        return " ".join(args)
    
    def generate_gcode(self, curve, depth, laserPower, altfeed=None, altppm=None):
        gcode = ''
        
        #Setup our feed rate, either from the layer name or from the default value.
        if (altfeed):
            # Use the "alternative" feed rate specified
            cutFeed = "F%i" % altfeed
        else:
            if self.options.generate_not_parametric_code: 
                cutFeed = "F%i" % self.options.feed
            else:
                cutFeed = "F%i" % self.options.feed
                
        #Setup our pulse per millimetre option, if applicable
        #B: laser firing mode (0 = continuous, 1 = pulsed, 2 = raster)
        if (altppm):
            # Use the "alternative" ppm - L60000 is 60ms
            ppmValue = "L60000 P%.2f B1 D0" % altppm
        else:
            #Set the laser firing mode to continuous.
            ppmValue = "B0 D0"

        cwArc = "G02"
        ccwArc = "G03"
		
		# The geometry is reflected, so invert the orientation of the arcs to match
        if (self.flipArcs):
            (cwArc, ccwArc) = (ccwArc, cwArc)

        # The 'laser on' and 'laser off' m-codes get appended to the GCODE generation
        lg = 'G00'
        for i in range(1,len(curve)):
            s, si = curve[i-1], curve[i]

			#G00 : Move with the laser off to a new point
            if s[1] == 'move':
                #Turn off the laser if it was on previously.
                if lg != "G00":
                    gcode += LASER_OFF + "\n"
				
                gcode += "G00" + " " + self.make_args(si[0]) + " F%i" % self.options.Mfeed + "\n"
                lg = 'G00'

            elif s[1] == 'end':
                lg = 'G00'

			#G01 : Move with the laser turned on to a new point
            elif s[1] == 'line':
                #No longer needed because G01, G02 and G03 will be forced in marlin to automatically fire the laser.
                #If the laser was turned off, turn it back on.
                #if lg == "G00": 
                #   gcode += LASER_ON + "\n"	 

                gcode += "G01 " + "S%.2f " % laserPower + self.make_args(si[0]) + " %s " % cutFeed + "%s" % ppmValue + "\n"
                lg = 'G01'

            #G02 and G03 : Move in an arc with the laser turned on.
            elif s[1] == 'arc':
                #No longer needed because G01, G02 and G03 will be forced in marlin to automatically fire the laser.
                #If the laser was turned off, turn it back on.
                #if lg == "G00":
                #    gcode += LASER_ON + "\n"

                dx = s[2][0]-s[0][0]
                dy = s[2][1]-s[0][1]
                if abs((dx**2 + dy**2)*self.options.Xscale) > self.options.min_arc_radius:
                    r1 = P(s[0])-P(s[2])
                    r2 = P(si[0])-P(s[2])
                    if abs(r1.mag() - r2.mag()) < 0.001:
                        if (s[3] > 0):
                            gcode += cwArc
                        else:
                            gcode += ccwArc
                        gcode += " " + "S%.2f " % laserPower + self.make_args(si[0] + [None, dx, dy, None]) + " %s " % cutFeed + "%s" % ppmValue + "\n"

                    else:
                        r = (r1.mag()+r2.mag())/2
                        if (s[3] > 0):
                            gcode += cwArc
                        else:
                            gcode += ccwArc
                        gcode += " " + "S%.2f " % laserPower + self.make_args(si[0]) + " R%f" % (r*self.options.Xscale) + " %s " % cutFeed + "%s" % ppmValue + "\n"

                    lg = cwArc
                #The arc is less than the minimum arc radius, draw it as a straight line.
                else:
                    gcode += "G01 " + "S%.2f " % laserPower + self.make_args(si[0]) + " %s " % cutFeed + "%s" % ppmValue + "\n"
                    lg = 'G01'

    
        #The end of the layer.
        if si[1] == 'end':
            gcode += LASER_OFF


        return gcode

    def tool_change(self):
        # Include a tool change operation
        gcode = TOOL_CHANGE % (self.currentTool+1)
        # Select the next available tool
        self.currentTool = (self.currentTool+1) % 32
        return gcode

    ################################################################################
    ###
    ###        Curve to Gcode
    ###
    ################################################################################

    def effect_curve(self, selected):
        selected = list(selected)

        # Set group
        if self.options.drawCurves and len(selected)>0:
            self.biarcGroup = inkex.etree.SubElement( selected[0].getparent(), SVG_GROUP_TAG )
            options.Group = self.biarcGroup

        # Recursively compiles a list of paths that are decendant from the given node
        self.skipped = 0
        def compile_paths(node, trans):
            # Apply the object transform, along with the parent transformation
            mat = node.get('transform', None)
            if mat:
                mat = simpletransform.parseTransform(mat)
                trans = simpletransform.composeTransform(trans, mat)

            if node.tag == SVG_PATH_TAG:
                # This is a path object
                if (not node.get("d")): return []
                csp = cubicsuperpath.parsePath(node.get("d"))
                if (trans):
                    simpletransform.applyTransformToPath(trans, csp)
                return csp

            elif node.tag == SVG_GROUP_TAG:
                # This node is a group of other nodes
                path = []
                for child in node.iterchildren():
                    path += compile_paths(child, trans)
                return path
            else:
                #inkex.errormsg( ) 
                
                #visit https://www.python.org/downloads/ and download python 2.7.9
                #Install it
                #In the folder : C:\Program Files\Inkscape you will need to rename the folder "Python" to "Python-old" so it uses the new system install instead.
                #pip install wheel
                #From http://www.lfd.uci.edu/~gohlke/pythonlibs/#pil , download "Pillow-2.7.0-cp27-none-win32.whl"
                #pip install Pillow-2.7.0-cp27-none-win32.whl
                #You're good to go!     
                
                #Export the image data.
                width = int(float(node.get("width")))
                height = int(float(node.get("height")))
                data = str((node.get(inkex.addNS('href','xlink')) )).replace("data:image/jpeg;base64,","")
                im = Image.open(BytesIO(base64.b64decode(data))).convert('L')
                img = ImageOps.invert(im)
                pixels = list(im.getdata())
                pixels = [pixels[i * width:(i + 1) * width] for i in xrange(height)]
                
                inkex.errormsg( str(pixels)) 

            inkex.errormsg("skipping " + str(node.tag))
            self.skipped += 1
            return []

        # Compile a list of layers in this document. We compile a list of only the layers
        # we need to use, so we can know ahead of time whether to put tool change 
        # operations between them.
        layers = []
        for layer in reversed(get_layers(self.document)):
            for node in layer.iterchildren():
                if (node in selected):
                    layers.append(layer)
                    break

        layers = list(reversed(get_layers(self.document)))

        # Loop over the layers and objects
        gcode = ""
        for layer in layers:
            label = layer.get(SVG_LABEL_TAG).strip()
            if (label.startswith("#")):
                # Ignore everything selected in this layer
                for node in layer.iterchildren():
                    if (node in selected):
                        selected.remove(node)
                continue

            # Parse the layer label text, which consists of the layer name followed
            # by an optional number of arguments in square brackets.
            try:
                (layerName, layerParams) = parse_layer_name(label)
            except ValueError,e:
                inkex.errormsg(str(e))
                return

            # Check if the layer specifies an alternative (from the default) feed rate
            altfeed = layerParams.get("feed", None)
            altppm = layerParams.get("ppm", None)

            logger.write("layer %s" % layerName)
            if (layerParams):
                logger.write("layer params == %s" % layerParams)
            pathList = []
            # Apply the layer transform to all objects within the layer
            trans = layer.get('transform', None)
            trans = simpletransform.parseTransform(trans)

            for node in layer.iterchildren():
                if (node in selected):
                    logger.write("node %s" % str(node.tag))
                    selected.remove(node)
                    pathList += compile_paths(node, trans)
                else:
                    logger.write("skipping node %s" % node)

            if (not pathList):
                logger.write("no objects in layer")
                continue
            curve = self.parse_curve(pathList)
            
            #Determind the power of the laser that this layer should be cut at.
            #If the layer is not named as an integer value then default to the laser intensity set at the export settings.
            #Fetch the laser power from the export dialog box.
            laserPower = self.options.laser
            
            if (int(layerName) > 0 and int(layerName) <= 100):
                laserPower = int(layerName)
            else :
                laserPower = self.options.laser
            
            #Switch between smoothie power levels and ramps+marlin power levels
            #ramps and marlin expect 0 to 100 while smoothie wants 0.0 to 1.0
            if (self.options.mainboard == 'smoothie'):
                laserPower = float(laserPower) / 100
                

            # If there are several layers, start with a tool change operation
            #Turnkey : Always output the layer header for information.
            if (len(layers) > 0):
                gcode += LASER_OFF+"\n"
                size = 60
                gcode += ";(%s)\n" % ("*"*size)
                gcode += (";(***** Layer: %%-%ds *****)\n" % (size-19)) % (layerName)
                gcode += (";(***** Laser Power: %%-%ds *****)\n" % (size-25)) % (laserPower)
                gcode += (";(***** Feed Rate: %%-%ds *****)\n" % (size-23)) % (altfeed)
                gcode += (";(***** Pulse Rate: %%-%ds *****)\n" % (size-24)) % (altppm)
                gcode += ";(%s)\n" % ("*"*size)
                gcode += ";(MSG,Starting layer '%s')\n\n" % layerName
                # Move the laser into the starting position (so that way it is positioned
                # for testing the power level, if the user wants to change that).
                arg = curve[0]
                pt = arg[0]
                #gcode += "G00 " + self.make_args(pt) + "\n"


            if (self.options.drawCurves):
                self.draw_curve(curve)
                
            #Generate the GCode for this layer
            gcode += self.generate_gcode(curve, 0, laserPower, altfeed=altfeed, altppm=altppm)

        # If there are any objects left over, it's because they don't belong
        # to any inkscape layer (bug in inkscape?). Output those now.
        if (selected):
            pathList = []
            # Use the identity transform (eg no transform) for the root objects
            trans = simpletransform.parseTransform("")
            for node in selected:
                pathList += compile_paths(node, trans)

            if (pathList):
                curve = self.parse_curve(pathList)
                if (self.options.drawCurves):
                    self.draw_curve(curve)

                gcode += "\n;(*** Root objects ***)\n"
                
                #Fetch the laser power from the export dialog box.
                laserPower = self.options.laser
                
                #Switch between smoothie power levels and ramps+marlin power levels
                #ramps and marlin expect 0 to 100 while smoothie wants 0.0 to 1.0
                if (self.options.mainboard == 'smoothie'):
                    laserPower = float(laserPower) / 100
                
                gcode += self.generate_gcode(curve, 0, laserPower)
                
        if self.options.homeafter:
            gcode += "\n\nG00 X0 Y0 F4000 ; home"
                
        return gcode

    def effect(self):
        global options
        options = self.options
        selected = self.selected.values()

        root = self.document.getroot()
        self.pageHeight = float(root.get("height", None))
        self.flipArcs = (self.options.Xscale*self.options.Yscale < 0)
        self.currentTool = 0

        self.filename = options.file.strip()
        if (self.filename == "-1.0" or self.filename == ""):
            inkex.errormsg(("Please select an output file name."))
            return

        if (not self.filename.lower().endswith(GCODE_EXTENSION)):
            # Automatically append the correct extension
            self.filename += GCODE_EXTENSION

        logger.enabled = self.options.logging
        logger.write("thlaser script started")
        logger.write("output file == %s" % self.options.file)

        if len(selected)<=0:
            inkex.errormsg(("This extension requires at least one selected path."))
            return

        self.check_dir()

        gcode = self.header;

        if (self.options.unit == "mm"):
            self.unitScale = 0.282222
            gcode += "G21 ; All units in mm\n"
        elif (self.options.unit == "in"):
            self.unitScale = 0.011111
            gcode += "G20 ; All units in in\n"
        else:
            inkex.errormsg(("You must choose mm or in"))
            return

        if not self.options.generate_not_parametric_code:
            gcode += """
; Default Cut Feedrate %i mm per minute
; Default Move Feedrate %i mm per minute
; Default Laser Intensity %i percent\n""" % (self.options.feed, self.options.Mfeed, self.options.laser)

        if self.options.homebefore:
            gcode += "G28 ; home all\n\n"

        #if self.options.function == 'Curve':
        gcode += self.effect_curve(selected)

        if (self.options.double_sided_cutting):
            gcode += "\n\n;(MSG,Please flip over material)\n\n"
            # Include a tool change operation
            gcode += self.tool_change()

            logger.write("*** processing mirror image")

            self.options.Yscale *= -1
            self.flipArcs = not(self.flipArcs)
            self.options.generate_not_parametric_code = True
            self.pageHeight = 0
            gcode += self.effect_curve(selected)

        try:
            f = open(self.options.directory+'/'+self.options.file, "w")    
            f.write(gcode + self.footer)
            f.close()                            
        except:
            inkex.errormsg(("Can not write to specified file!"))
            return

        if (self.skipped > 0):
            inkex.errormsg(("Warning: skipped %d object(s) because they were not paths. Please convert them to paths using the menu 'Path->Object To Path'" % self.skipped))

e = Gcode_tools()
e.affect()

