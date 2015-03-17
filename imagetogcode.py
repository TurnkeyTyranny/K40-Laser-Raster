#!/usr/bin/python
#
#    imagetogcode
#    Copyright 2015 Turnkey Tyranny
#
#    Changelog :
#    18 March 2015 : Added image rastering optimisations to remove printing of whitespace. This was a pain in the ass to get right!
#
#
#    Based off the work by Brian Adams
#
#    This program is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    This program is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with this program.  If not, see <http://www.gnu.org/licenses/>

"""Convert an image file to gcode a Marlin powered laser cutter can understand"""
import sys, getopt

import Image
import ImageOps

import base64

def imagetogcode(image, f):
    
    img = ImageOps.invert(image)
    width, height = img.size

    f.write("; Image pixel size: "+str(width)+"x"+str(height)+"\n");
    f.write("M649 S11 B2 D0 R0.0846 F6000\nG28\nG0 X70 Y45 F6000\n");

    pixels = list(img.getdata())
    pixels = [pixels[i * width:(i + 1) * width] for i in xrange(height)]
    forward = True

#    def get_chunks(arr, chunk_size = 51):
    def get_chunks(arr, chunk_size = 51):
        chunks  = [ arr[start:start+chunk_size] for start in range(0, len(arr), chunk_size)]
        return chunks 

#   return the last pixel that holds data.
    def last_in_list(arr):
        end = 0
        for i in range(len(arr)):
            if (arr[i] > 0):
                end = i
                
        return end


#   return the last pixel that holds data.
    def first_in_list(arr):
        end = 0
        for i in range(len(arr)):
            if (arr[i] == 0):
                end = i
            if (arr[i] > 0):
                break
                
        return end
		
    first = True   
    row = pixels[::-1]   


    previousRight = 99999999999
    previousLeft  = 0
    firstRow = True
    
    for index, rowData in enumerate(row):
        print "Line "+str(index+1)+" ="
        #print rowData
       
        splitRight = 0
        splitLeft = 0
        
        if(index+1 < len(row)):
            # Determine where to split the lines.
            ##################################################
            
            #If the left most pixel of the next row is earlier than the current row, then extend.
            if(first_in_list(row[index +1]) > first_in_list(rowData)):
                splitLeft = first_in_list(rowData)
            else:
                splitLeft = first_in_list(row[index +1])

            #If the end pixel of the next line is later than the current line, extend.
            if(last_in_list(row[index +1]) > last_in_list(rowData)):
                splitRight = last_in_list(row[index +1])
            else:
                splitRight = last_in_list(rowData)
            
            print "Prior Left cut = "+str(splitLeft)+" Right Cut == "+str(splitRight) 
        else:
            splitLeft = first_in_list(rowData)
            splitRight = last_in_list(rowData)
        
            
        #Positive direction
        if forward:
            print "Forward!"
            
            #Split the right side.
            ###########################################

            #Don't split more than the start of the last row as we print in reverse for alternate lines
            splitLeft = previousLeft
            previousRight = splitRight
            
        #Negative direction
        else:
            print "Backward!"
            
            #Split the left side.
            ###########################################
            
            #Don't split more than the end of the last row as we print in reverse for alternate lines
            splitRight = previousRight
            previousLeft = splitLeft
                
            
        
        #Exception to the rule : Don't split the left of the first row.
        if(firstRow):
            splitLeft = (previousLeft)
            
        firstRow = False
        print "After : Left cut = "+str(splitLeft)+" Right Cut == "+str(splitRight) 
            
        row2 = rowData[(splitLeft+1):(splitRight+1)]
        print row2
        
        #if(index == 5): 
        #    raise Exception('End')
        
        if not forward:
            result_row = row2[::-1]
        else:
            result_row = row2
            
        
        for chunk in get_chunks(result_row,51):
            if first:
                if forward:
                    f.write("\nG7 $1 ")
#                    f.write("G7 $1\nG7 ")
                else:
                    f.write("\nG7 $0 ")
#                    f.write("G7 $0\nG7 ")
                first = not first
            else:
                f.write ("G7 ")
                
            b64 = base64.b64encode("".join(chr(y) for y in chunk))
            f.write("L"+str(len(b64))+" ")
            f.write("D"+b64+ "\n")
        forward = not forward
        first = not first
            
    f.write("M5 \n");
def main(argv):
    inputfile = None
    
    def showhelp():
        print "imagetogcode: Process an input image to gcode for a Marlin laser cutter."
        print
        print "Usage: imagetogcode -i <input file> -o [output file]"
    
    outputfile = None
    try:
        opts, args = getopt.getopt(argv, "hi:o:", ["input=", "output="])
    except getopt.GetoptError:
        showhelp()    
        sys.exit(2)
    for opt, arg in opts:
        if opt == '-h':
            showhelp()
            sys.exit()
        if opt in ('-i', '--input'):
            inputfile = arg
        elif opt in ('-o', '--output'):
            outputfile = arg
    if inputfile is None:
        showhelp()
        sys.exit(2)
    try:
        image = Image.open(inputfile).convert('L')
    except IOError:
        print "Unable to open image file."
        sys.exit(2)
    if outputfile is None:
        gcode = sys.stdout
    else:
        try:
            gcode = open(outputfile, "w")
        except IOError:
            print "Unable to open output file."
    
    imagetogcode(image, gcode)
    

if __name__ == "__main__":
    main(sys.argv[1:])
