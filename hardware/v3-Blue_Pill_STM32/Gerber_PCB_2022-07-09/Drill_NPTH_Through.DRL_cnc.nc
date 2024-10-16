(G-CODE GENERATED BY FLATCAM v8.994 - www.flatcam.org - Version Date: 2020/11/7)

(Name: Drill_NPTH_Through.DRL_cnc)
(Type: G-code from Geometry)
(Units: MM)

(Created on Sunday, 10 July 2022 at 00:44)

(This preprocessor is used with a motion controller loaded with GRBL firmware.)
(It is configured to be compatible with almost any version of GRBL firmware.)


(TOOLS DIAMETER: )
(Tool: 1 -> Dia: 3.0)

(FEEDRATE Z: )
(Tool: 1 -> Feedrate: 100.0)

(FEEDRATE RAPIDS: )
(Tool: 1 -> Feedrate Rapids: 1500)

(Z_CUT: )
(Tool: 1 -> Z_Cut: -1.9000000000000001)

(Tools Offset: )
(Tool: 1 -> Offset Z: 0.0)

(Z_MOVE: )
(Tool: 1 -> Z_Move: 2)

(Z Toolchange: 20.0 mm)
(X,Y Toolchange: 0.0000, 0.0000 mm)
(Z Start: None mm)
(Z End: 2.0 mm)
(X,Y End: 0.0000, 0.0000 mm)
(Steps per circle: 64)
(Steps per circle: 64)
(Preprocessor Excellon: grbl_11)

(X range:   11.5000 ...   14.5000  mm)
(Y range:  -21.5000 ...   -1.5000  mm)

(Spindle Speed: 100 RPM)
G21
G90
G17
G94


G01 F100.00

M5             
G00 Z20.0000
G00 X0.0000 Y0.0000                
T1
M6
(MSG, Change to Tool Dia = 3.0000 ||| Total drills for tool T1 = 2)
M0
G00 Z20.0000

G01 F100.00
M03 S100
G00 X13.0000 Y-3.0000
G01 Z-1.9000
G01 Z0
G00 Z2.0000
G00 X13.0000 Y-20.0000
G01 Z-1.9000
G01 Z0
G00 Z2.0000
M05
G00 Z2.00
G00 X0.0 Y0.0


