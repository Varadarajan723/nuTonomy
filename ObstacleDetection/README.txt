These three files attached form the most important part of my master's work.

 		
FlsDetection.cc			
————————————
C code to interface with the FLS (forward looking sonar) and receive scans from 
Processes the scans and build a local occupancy grid which uses a Bayesian filter (details can be found in miscellaneous/ISER2014.pdf).
Extracts the obstacles at the end of each scan and copies into onto a file "obstacleDetection.bin"

Navigator.groovy
———————————	
	
Reads from the file obstacleDetection.bin and checks for collision. 
If there exits a collision, the Navigator re-plans the path and sends it 
to the Pilot 

Pilot.groovy 			
—————————-

Receives the path from the Navigator and converts it to thrust and bearing values
or the AUV to execute