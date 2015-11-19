package org.arl.jc2.agent

import org.arl.fjage.*
import org.arl.jc2.*
import org.arl.jc2.enums.*
import org.arl.jc2.messages.*

import java.util.List;
import java.io.*;
import org.apache.commons.math3.linear.*;
import org.arl.jc2.agent.pathfinding.*;
import org.apache.commons.math3.exception.*;

/**
 * @author  Varadarajan
 * @version $Revision: 10807 $, $Date: 2013-05-22 06:03:37 +0800 (Wed, 22 May 2013) $
 */

class Navigator extends Agent {

	float PathPlanningStepSize = 5

	
	Position curPos = null, lastPos = new Position(), nextPos = new Position()
	float curBearing, curAltitude, curDepth
	LinearPlanner pathPlanner
	WayPoints previousWayPoints = null

	WayPoints wayPts = new WayPoints()// used to store waypoints passed to the EO
	
	//extra holder to store future waypoint for smooth transition, should only be filled
	//when the pilot is at its last point of navigation!
	WayPoints wayPtsHolder = new WayPoints()
	
	//Transform the waypoints to the AUV's frame of reference 
	WayPoints wayPtsTransformed = new WayPoints()

	int wpcnt = 0
	int missionOvercheck = 0;	
	float depthForMissions_1;
	float depthForMissions_2;
	def msgHandler=[:]

	@Override
	void init() {
		register(C2Services.NAVIGATOR)
		subscribe(topic(C2Topics.ABORTSIGNAL))
		subscribe(topic(C2Topics.MISSIONSTATUS))
		subscribe(topic(C2Topics.OBSTACLEDETECTION))
		subscribe(topic(C2Topics.MISSIONPOSITIONSTATUS))	
		subscribe(topic(C2Topics.WAYPOSITIONSTATUS))
		//subscript to Vehicle state topic, for position update
		subscribe(topic(C2Topics.VEHICLESTATUS))

		//add in all the handler.
		msgHandler << [ (VehicleStatus.class)    : {handleVehicleStatus(it)},
			(C2CommandReq.class) : {handleC2CommandReq(it)},
			(WayPtReq.class) : {handleWayPtReq(it)},
			(ObstacleDetection.class):{handleObstacleDetection(it)},
			(MissionStatusNtf.class) : {handleMissionStatusNtf(it)},
			(MissionPointStatusNtf.class) : {handleMissionPointStatusNtf(it)},
			(WayPointStatusNtf.class) : {handleWayPointStatusNtf(it)},
		]

		add(new MessageBehavior(){
					@Override
					void onReceive(Message msg){
						if(msgHandler.containsKey(msg.class))
							msgHandler[msg.class](msg)
					}
				})
		

	}

	void handleWayPointStatusNtf(Message msg)
	{

		WayPointStatusNtf mm = msg as WayPointStatusNtf
		wpcnt = mm.wayPointNumber -1;
		log.fine("WayPoint Being executed is " +wpcnt)
	}

	void handleMissionPointStatusNtf(Message msg)
	{

		
		MissionPointStatusNtf mm = msg as MissionPointStatusNtf
		if(mm.getStatus() == Status.COMPLETED)
		{
			log.info("Mission Point Completed")

			if(wayPtsHolder.positions.size() > 0)
				{
					wayPts = wayPtsHolder;
					wayPtsHolder = new WayPoints();
					depthForMissions_1 = depthForMissions_2;
					wpcnt = 0;
				}
				else 
					{
						wayPts = new WayPoints()
						wayPtsHolder = new WayPoints()
						missionOvercheck = 0 ;

					}
		}


	}

	void handleObstacleDetection(ObstacleDetection msg)
	{

	if(missionOvercheck != 0){
		log.fine("Grid Received")
		double angle = Math.toRadians(curBearing);
       
    	int flagNextMissionPoint = 0;

    	Position mPointTransformed = new Position(); 
    	Position wPointBeforeMPointTransformed = new Position();

       double [][] T = new double[3][3]	
       
       T[0][0] = Math.cos(angle);
       T[0][1] = Math.sin(angle);
       T[0][2] = curPos.x;
       T[1][0] = -1*Math.sin(angle);
       T[1][1] = Math.cos(angle);
       T[1][2] = curPos.y;
       T[2][0] = 0;
       T[2][1] = 0;
       T[2][2] = 1;


        //double [][] T = new double {{ Math.cos(angle), Math.sin(angle) , curPos.x }, { -1*Math.sin(angle), Math.cos(angle) , curPos.y } , { 0 , 0 , 1} };


	    RealMatrix a = new Array2DRowRealMatrix(T);
       	RealMatrix aInverse = new LUDecomposition(a).getSolver().getInverse();
  
      wayPtsTransformed = new WayPoints();
      for(int i = wpcnt; i < wayPts.positions.size();i++)
      {

      		double [][] P = new double [3][1]
      		P[0][0] = wayPts.positions.get(i).x;
      		P[1][0] = wayPts.positions.get(i).y;
      		P[2][0] = 1;

      		RealMatrix p = new Array2DRowRealMatrix(P);
      		RealMatrix inter = aInverse.multiply(p);
      		Position transformed = new Position(inter.getEntry(0,0),inter.getEntry(1,0),wayPts.positions.get(i).z);
      		wayPtsTransformed.positions.add(transformed);
			//log.info("Way points transformed are " +wayPtsTransformed.positions.get(wayPtsTransformed.positions.size()-1).x +" " +wayPtsTransformed.positions.get(wayPtsTransformed.positions.size()-1).y);

      }	

	
	if(wayPts.positions.size() - wpcnt >= 2)	{      
      mPointTransformed = wayPtsTransformed.positions.get(wayPtsTransformed.positions.size()-1);
      wPointBeforeMPointTransformed = wayPtsTransformed.positions.get(wayPtsTransformed.positions.size()-2);
  	}else{

  		if(wayPts.positions.size() - wpcnt == 1){
  				mPointTransformed = wayPtsTransformed.positions.get(wayPtsTransformed.positions.size()-1);
      			wPointBeforeMPointTransformed = wayPtsTransformed.positions.get(wayPtsTransformed.positions.size()-1);
      	}
	}

      if(wayPts.positions.size() - wpcnt <= 4 && wayPtsHolder.positions.size() != 0) 
      {

      		flagNextMissionPoint = 1;
      		for(int i = 0; i < wayPtsHolder.positions.size();i++)
     		{
	      		double [][] P = new double [3][1]
	      		P[0][0] = wayPtsHolder.positions.get(i).x;
	      		P[1][0] = wayPtsHolder.positions.get(i).y;
	      		P[2][0] = 1;
	      		RealMatrix p = new Array2DRowRealMatrix(P);
	      		RealMatrix inter = aInverse.multiply(p);
	      		Position transformed = new Position(inter.getEntry(0,0),inter.getEntry(1,0),wayPtsHolder.positions.get(i).z);
	      		wayPtsTransformed.positions.add(transformed);
      		}	

      }


     
      int wayPointLastX = Math.ceil(wayPtsTransformed.positions.get(wayPtsTransformed.positions.size()-1).x);
      int wayPointLastY = Math.ceil(wayPtsTransformed.positions.get(wayPtsTransformed.positions.size()-1).y);
      int flagGridToUse = 0;

      int size_x = 0;
      int size_y = 0;

	      	if ((wayPointLastX <=45 && wayPointLastX > -45) && (wayPointLastY <= 50 && wayPointLastY > 0)){
            	flagGridToUse = 1;
            	size_x = 18;
            	size_y = 10;
       		}
        
            
            if(wayPointLastX > 45 && (wayPointLastY <=50 && wayPointLastY > 0))
            {
                flagGridToUse = 2;
                int rem = wayPointLastX%5;
                size_x = 2*((wayPointLastX + (5-rem))/5);
                size_y = 10;

            }
            if(wayPointLastX > 45 && wayPointLastY >50)
            {
                flagGridToUse = 3;
                int rem_x = wayPointLastX%5;
                int rem_y = wayPointLastY%5;
                size_x = 2*((wayPointLastX + (5-rem_x))/5);
                size_y = ((wayPointLastY + (5-rem_y))/5);

            }
            if(wayPointLastX < -45 && (wayPointLastY <=50 && wayPointLastY > 0))
            {
                flagGridToUse = 4;
                int trick = Math.abs(wayPointLastX) + 1;
                int rem = trick%5;
                size_x = 2*((trick + (5-rem))/5);
                size_y = 10;

            }
            if(wayPointLastX < -45 && wayPointLastY >50)
            {    
            	flagGridToUse = 5;
            	int trick = Math.abs(wayPointLastX) + 1;
                int rem_x = trick%5;
                size_x = 2*((trick + (5-rem_x))/5);
                int rem_y = wayPointLastY%5;
                size_y = ((wayPointLastY + (5-rem_y))/5);

            }
            if(wayPointLastY > 50 && (wayPointLastX >=-45 && wayPointLastX <=45))
            {    
            	flagGridToUse = 6;
            	size_x = 18;
            	int rem_y = wayPointLastY%5;
                size_y = ((wayPointLastY + (5-rem_y))/5);

            }


      Map<ExampleNode> myMap = new Map<ExampleNode>(size_x, size_y, new ExampleFactory());
      boolean k = false;

      	
      	if ( flagGridToUse != 0 ) {
      	 RealMatrix adjusting = new Array2DRowRealMatrix(5*size_x,5*size_y);
		 double [][] intermediate = new double[74][50]
		 msg.occupancyGrid.copySubMatrix(0,73,1,50,intermediate);
		 int xstart = (5*size_x - 74)/2;
		 adjusting.setSubMatrix(intermediate,xstart,0);
		 int flag_to_break = 0;
	      for(int ii=0;ii<size_x;ii++){
	      	for(int jj=0;jj<size_y;jj++)
	      		{	
	      			RealMatrix check = adjusting.getSubMatrix(5*ii,5*(ii+1)-1,5*jj,5*(jj+1)-1)
	      			for(int l=0;l<5;l++){
	      				for(int m= 0;m<5;m++){
	      					if(check.getEntry(l,m) == 1.0){
	      						flag_to_break = 1;
	      						myMap.setWalkable(ii,jj,k);
	      						
	      						break;
	      					}
	      				}
	      				if(flag_to_break == 1)
	      				{
	      					flag_to_break = 0;
	      					break;
	      				}
	      				}
	      			}
			}
		}

		int flag_obstacle_avoidance = 0;
		
		for(int zz = 0; zz< wayPtsTransformed.positions.size() ; zz++ ){
				int wayPointCheck = 1;
				int x_Coordinate = Math.ceil(wayPtsTransformed.positions.get(zz).x) + ((size_x*5)/2) ;
				int y_Coordinate = Math.ceil(wayPtsTransformed.positions.get(zz).y) ;
				x_Coordinate = Math.ceil(x_Coordinate/5) - 1;
				y_Coordinate = Math.ceil(y_Coordinate/5) - 1;

		

				if(x_Coordinate < 0 || y_Coordinate < 0)
					wayPointCheck = 0;

				if(flagGridToUse != 0 && wayPointCheck !=0){		

				if ( zz == 0){	
					if(myMap.getWalkable(x_Coordinate,y_Coordinate) == false)
					{
						log.info("Abort Mission , Obstacle very close");
						flag_obstacle_avoidance = 3;
						break;
					}
				}

				if (myMap.getWalkable(x_Coordinate,y_Coordinate) == false)
				{
					double dist = Math.sqrt(Math.pow(wayPtsTransformed.positions.get(zz).x - mPointTransformed.x,2) + Math.pow(wayPtsTransformed.positions.get(zz).y - mPointTransformed.y,2));
					if ( dist < 10){
								if( flagNextMissionPoint == 1)
                               		 flag_obstacle_avoidance = 4;
                           		else{

                           			if(wayPtsHolder.positions.size() == 0){
 	                         			 	log.info("Abort!! Last Mission Point reached and obstacle too close");
 	                         			 	flag_obstacle_avoidance = 3;
 	                         			 	break;
                           			 	}
                           			 else
                           			 	{
                       			 			log.info("Wait for next set of waypoints to carry out avoidance since this mission point needs to deleted");
                           			 	}	
                           			}
                                 }
                    else{
                            if((dist > 10) && (wayPtsTransformed.positions.get(zz).y < mPointTransformed.y))
                               flag_obstacle_avoidance = 1;
                            else
                                if((dist > 10) && (wayPtsTransformed.positions.get(zz).y >  mPointTransformed.y))
                                    flag_obstacle_avoidance = 0;
                                                            
                   }

				}

			}
								
		}

		if (flag_obstacle_avoidance == 1 || flag_obstacle_avoidance==4){
			log.info("Obstacle Avoidanace Initiated")

			Position goal = new Position();
			
			if (flag_obstacle_avoidance == 1) 
				goal = wPointBeforeMPointTransformed;
			else
				goal =  wayPtsTransformed.positions.get(wayPtsTransformed.positions.size() - 2);


			int goal_x = Math.ceil(goal.x) + ((size_x*5)/2);
			goal_x = Math.ceil(goal_x/5) - 1;
			int goal_y = Math.ceil(goal.y);
			goal_y = Math.ceil(goal_y/5) - 1;

			int start_x = Math.ceil(wayPtsTransformed.positions.get(0).x) + ((size_x*5)/2);
			start_x = Math.ceil(start_x/5) - 1;
			int start_y;
			if(start_x < size_x/2){
				start_x = (size_x/2) -1;
    			start_y = 0;
    		}
			else{
			    start_x = (size_x/2);
    			start_y = 0;
			}

 			float depthForMissionToUse=0;

 			if (flag_obstacle_avoidance == 1) 
				depthForMissionToUse = depthForMissions_1;
			else
				{
					depthForMissionToUse = depthForMissions_2;
					depthForMissions_1 = depthForMissions_2;
				}

			WayPoints newForAvoidance = new WayPoints();
			List<ExampleNode> path = myMap.findPath(start_x, start_y, goal_x, goal_y) ;

			if (path.size() == 0)
				log.info(" No such path exists. Need to abort/delete missionpoint");
			else{
				 for (int i = 0; i < path.size(); i++) {
	           			log.info("(" + path.get(i).getxPosition() + ", " + path.get(i).getyPosition() + ") -> ");
	    				double [][] I = new double [3][1]
	      				I[0][0] = 5*path.get(i).getxPosition() - (size_x*5)/2 + 2.5;
	      				I[1][0] = 5*path.get(i).getyPosition() + 2.5;
	      				I[2][0] = 1;

	      				RealMatrix smallI = new Array2DRowRealMatrix(I);
	      				RealMatrix inter = a.multiply(smallI);
	      				Position interim = new Position(inter.getEntry(0,0),inter.getEntry(1,0),depthForMissionToUse);
	      				log.info("New Way points generated are " +interim.x +" " +interim.y+" " +depthForMissionToUse);
	      				
		  				if(i == path.size()-1){
		  					if (flag_obstacle_avoidance == 4)
		      					newForAvoidance.positions.add(wayPtsHolder.positions.get(wayPtsHolder.positions.size()-1));
							else 
								newForAvoidance.positions.add(wayPts.positions.get(wayPts.positions.size()-1))
						}
	      				else
	      					newForAvoidance.positions.add(interim);

	    		}

				AgentID server = this.agentForService(C2Services.PILOT);
				if(server != null){
					WayPtObstacleAvoidanceReqExe m = new WayPtObstacleAvoidanceReqExe(server)
					m.setAlteredWayPoints(newForAvoidance)
					if (flag_obstacle_avoidance == 1)
						m.setmPointID(wayPts.getmPointBroadcastID())
					else
					{
						m.setmPointID(wayPtsHolder.getmPointBroadcastID())
						log.info(" Currently exectuting mission point has been aborted because the obstacle is too close to AUV and MissionPoint ")

						// broadcasting that a mission point has been deleted. 
						AgentID mserver = topic(C2Topics.MISSIONPOSITIONSTATUS)
						MissionPointStatusNtf deleteMsg = new MissionPointStatusNtf(mserver)
						deleteMsg.setmPointID(wayPts.getmPointBroadcastID())
						deleteMsg.setStatus(Status.DELETED)
						send(deleteMsg)
						log.finest("WayPts with MpID "+wayPts.getmPointBroadcastID().toString()+" deleted")

					}
					this.send(m)
					log.finest("Altered WayPoints have been sent to pilot")

					wayPts = newForAvoidance;
					wayPtsHolder = new WayPoints();
					wpcnt = 0;
					previousWayPoints = new WayPoints(newForAvoidance);


				}else
					log.warning("pilot not found")
			}


		}
		

		}
	}


	void handleMissionStatusNtf(Message msg){
		MissionStatusNtf mm = msg as MissionStatusNtf
		if(mm.getStatus() == Status.STARTED){
		    previousWayPoints = null
		    missionOvercheck = 1;
		}
	    if(mm.getStatus() == Status.COMPLETED){
	    	 wayPts = new WayPoints()
			 wayPtsHolder = new WayPoints()
	    }

	}

	void handleVehicleStatus(Message msg) {
		VehicleStatus mm = msg as VehicleStatus
		curPos = mm.getPos()
		curBearing = mm.getBearing()
		curDepth = mm.getDepth()
		curAltitude = mm.getAltitude()
	}

	void handleC2CommandReq(Message msg) {
		C2CommandReq mm = msg as C2CommandReq
		//listening to abort signal
		C2Command cmd = mm.getCommand()
		if(cmd == C2Command.ABORT ||
		cmd == C2Command.ABORT_TO_HOME ||
		cmd == C2Command.ABORT_TO_START ||
		cmd == C2Command.CLEARALL){
			previousWayPoints = null
			wayPts = null
			wayPtsHolder = null
			wayPtsWorking = null
			wayPtsHolderWorking = null
		}
	}

	void handleWayPtReq(Message msg) {
		WayPtReq mm = msg as WayPtReq
		WayPtRes m = new WayPtRes(msg,Performative.INFORM)
		ArrayList<MissionPosition> missionPts = mm.getMps()
		MissionPosition mp = missionPts.get(0)
		WayPoints wps = planWayPoints(missionPts)
		wps.setParams(mp.getParams())
		//make sure the path planner copy the mpID of the MissionPosition so that the Pilot can broadcast it
		wps.setmPointBroadcastID(mp.getBroadcastMpID())
		m.setWps(wps)
		//keep the waypoints for storage
		previousWayPoints = new WayPoints(wps)
		send(m)

		if(wayPts.positions.size() == 0) {
			wayPts = wps;
			depthForMissions_1 = wayPts.positions.get(0).z;
		}else {
			wayPtsHolder = wps;
			depthForMissions_2 = wayPtsHolder.positions.get(0).z;
		}
	}


	WayPoints planWayPoints(ArrayList<MissionPosition> missionPosList){
		MissionPosition mp = (MissionPosition)missionPosList.get(0)
		//get curPlanBearing
		float curPlanBearing
		Position curPlanPos
		if(previousWayPoints != null && previousWayPoints.positions.size()>1){
			Position p1 = previousWayPoints.positions.get(previousWayPoints.positions.size()-1)
			Position p2 = previousWayPoints.positions.get(previousWayPoints.positions.size()-2)
			curPlanBearing = C2Utils.getAngle(p2, p1)
			curPlanPos = new Position(p1)
			
		}else{
			curPlanBearing = curBearing
			curPlanPos = new Position(curPos)
			
		}

		log.fine("missionPoint received "+mp.toString()+" curPlanPos "+curPlanPos.toString()+" curPlanBearing "+curPlanBearing.toString())
		pathPlanner = new LinearPlanner(PathPlanningStepSize,mp.params)
		return pathPlanner.planPath(curPlanPos, missionPosList, curPlanBearing)
	}

	@Override
	protected void die(Exception ex) {
		log.severe(getClass().getName() + " died: " + ex.getMessage());
		log.severe("restarting JC2");
		Runtime.getRuntime().exec("killall jc2");
	}
}
