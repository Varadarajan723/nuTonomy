package org.arl.jc2.agent

import org.arl.fjage.*
import org.arl.jc2.*
import org.arl.jc2.enums.*
import org.arl.jc2.messages.*



/**
 * @author  Varadarajan
 * @version $Revision: 10807 $, $Date: 2013-05-22 06:03:37 +0800 (Wed, 22 May 2013) $
 */

class Pilot extends Agent {
	private final static float THRUST_UPDATE_INTERVAL = 2000


	//TODO:read from configuration file
	def sideDist = 20
	def secAngle = 10
	boolean enablePathFollowing = false // enable or disable path following

	def bearingLastUpdate = 0.0
	def thrustLastUpdateValue = 0.0
	def thrustLastUpdateTime

	State myState
	WayPoints wayPts = new WayPoints()// used to store waypoints passed by EO
	//extra holder to store future waypoint for smooth transition, should only be filled
	//when the pilot is at its last point of navigation!
	WayPoints wayPtsHolder = new WayPoints()
	int wpcnt = 0
	boolean isWpReached = true

	Position curPos = null, lastPos = new Position(), nextPos = new Position()
	float curBearing,curHeading, curAltitude, curDepth, curYaw, curSpeed

	def msgHandler = [:]

	@Override
	void init() {
		register(C2Services.PILOT)

		//subscrbe to handle abort message
		subscribe(topic(C2Topics.ABORTSIGNAL))
		//subscript to Vehicle state topic, for position update
		subscribe(topic(C2Topics.VEHICLESTATUS))

		//add in all the handler.
		msgHandler << [ (VehicleStatus.class)    : {handleVehicleStatus(it)},
			(WayPtReqExe.class) : {handleWayPtReqExe(it)},
			(C2CommandReq.class) : {handleC2CommandReq(it)},
			(StateReportReq.class) : {handleStateReportReq(it)},
			(WayPtObstacleAvoidanceReqExe.class) : {handleWayPtObstacleAvoidanceReqExe(it)},
		]

		add(new MessageBehavior(){
					@Override
					void onReceive(Message msg){
						if(msgHandler.containsKey(msg.class))
							msgHandler[msg.class](msg)
					}
				})

		add(new TickerBehavior(500){
					@Override
					void onTick() {
						if(myState == State.RUNNING){
							if(wayPts.positions.size() > 0) {
								steer()
								//println "Pilot steering"
							} else if(wayPtsHolder.positions.size() > 0){
								wayPts = wayPtsHolder
								wpcnt = 0
								isWpReached = true
								//broadcast the MissionPointID start
								AgentID server = agent.topic(C2Topics.MISSIONPOSITIONSTATUS)
								MissionPointStatusNtf msg = new MissionPointStatusNtf(server)
								msg.setmPointID(wayPts.getmPointBroadcastID())
								log.finer("WayPts with MpID "+wayPts.getmPointBroadcastID().toString()+" started")
								msg.setStatus(Status.STARTED)
								agent.send(msg)

								//let EO know that pilot is ready for waypoints
								wayPtsHolder = new WayPoints()
								server  = agent.agentForService(C2Services.EXEOFFICER)
								if(server != null){
									StateReport informMsg = new StateReport(server,State.READY)
									agent.send(informMsg)
								}else
									log.warning("ExeOfficer not found")

							}else {
								AgentID server = agent.agentForService(C2Services.EXEOFFICER)
								if(server != null){
									StateReport msg = new StateReport(server,State.STOP)
									agent.send(msg)
									myState = State.STOP
									log.finer("Pilot in stop state")
									setThrust(0)
								}else log.warning("Pilot: Can not find ExeOfficer")
							}
						}else if (myState == State.ABORTING){
							if(abortToSurface())
								myState = State.STOP
						}
					}
				})

		thrustLastUpdateTime = this.currentTimeMillis()
		myState = State.RUNNING
	}

	void handleVehicleStatus(Message msg) {
		VehicleStatus mm = msg as VehicleStatus
		curPos = mm.getPos()
		curBearing = mm.getBearing()
		curHeading = mm.getHeading()
		curDepth = mm.getDepth()
		curAltitude = mm.getAltitude()
		curYaw = mm.getYaw()
		curSpeed = mm.getSpeed()
	}

	void handleWayPtReqExe(Message msg) {
		WayPtReqExe mm = msg as WayPtReqExe
		if(wayPtsHolder.positions.size() == 0) {
			wayPtsHolder = mm.getWps()
			myState = State.RUNNING
			println "Pilot wayPtsHolder is being filled"
		}else {
			log.warning("Pilot is not ready to receive more waypoints yet !")
			WayPtRes rm = new WayPtRes(mm,Performative.REFUSE)
			send(rm)
		}
	}

	void handleWayPtObstacleAvoidanceReqExe(Message msg){
		
		WayPtObstacleAvoidanceReqExe mm = msg as WayPtObstacleAvoidanceReqExe

		if (wayPts.getmPointBroadcastID().equals(mm.mPointID))
		{
			log.info("Current mission point's way points are being altered to carry out avoidance")

			wayPts = new WayPoints();
			wayPts = mm.alteredWayPoints;
			wayPts.setmPointBroadcastID(mm.mPointID)

			log.info("WayPts with MpID "+wayPts.getmPointBroadcastID().toString()+" has already been altered")
			wpcnt = 0;
			isWpReached = true;
		}
		else 
		{
			log.info("Next mission point's way points are being altered to carry out avoidance")
			wayPts = new WayPoints();
			wayPts = mm.alteredWayPoints;
			wayPts.setmPointBroadcastID(mm.mPointID)
			log.info("WayPts with MpID "+wayPts.getmPointBroadcastID().toString()+" has already been altered")

			wpcnt = 0;
			isWpReached = true;
			wayPtsHolder = new WayPoints();


		}



	} 


	void handleC2CommandReq(Message msg) {
		C2CommandReq mm = msg as C2CommandReq
		C2Command cmd = mm.getCommand()
		if(cmd == C2Command.ABORT ||
		cmd == C2Command.ABORT_TO_HOME ||
		cmd == C2Command.ABORT_TO_START ||
		cmd == C2Command.CLEARALL){
			//broadcast abort
			if(wayPts.positions.size() > 0){
				AgentID server = topic(C2Topics.MISSIONPOSITIONSTATUS)
				MissionPointStatusNtf abortMsg = new MissionPointStatusNtf(server)
				abortMsg.setmPointID(wayPts.getmPointBroadcastID())
				abortMsg.setStatus(Status.ABORTED)
				send(abortMsg)
				log.finer("WayPts with MpID "+wayPts.getmPointBroadcastID().toString()+" aborted")
			}

			wayPts = new WayPoints()
			wayPtsHolder = new WayPoints()
			wpcnt = 0
			isWpReached = true
			lastPos = new Position()
			nextPos = new Position()
			if(cmd == C2Command.ABORT){
				myState = State.ABORTING
			}
		}
	}

	void handleStateReportReq(Message msg) {
		//TODO:Should I have type here ?
		StateReport rMsg = new StateReport(msg,myState)
		send(rMsg)
	}

	boolean abortToSurface(){
		if(curDepth > 1) {
			setThrust(0.5)
			setDepth(-2)
		}else{
			setThrust(0)
			return true
		}
	}

	def steer() {
		if(isWpReached) {
			lastPos = nextPos
			if(wpcnt <= wayPts.positions.size() - 1) {
				nextPos = wayPts.positions.get(wpcnt)

				//send the last waypoint notice to EO to send the next waypoints
				//so that the wayPtsHolder can be filled
				//				if(wpcnt == wayPts.positions.size() - 1) {
				//					def server  = this.agentForService(C2Services.EXEOFFICER)
				//					def msg = new C2Message(server,Performative.INFORM)
				//					this.send(msg)
				//				}

				//increment the wpcnt
				wpcnt++
				AgentID lserver = topic(C2Topics.WAYPOSITIONSTATUS)
				WayPointStatusNtf msg = new WayPointStatusNtf(lserver)
				msg.setmPointID(wayPts.getmPointBroadcastID())
				msg.setWayPointNumber(wpcnt)
				send(msg)



				//make sure the nextPos is reachable
				//that's to make sure the nextPos is not too close or at the side of the AUV
				isWpReached = checkDistAngle() //return false by default
				if(isWpReached)
					return

				//setting altitude/depth here for the particular waypoint,
				//to save number of messages being sent to the VCS in navigate()
				//determine if it is altitude or depth control
				if(wayPts.params.containsKey('CruisingAltitude')&&wayPts.params.getAt('CruisingAltitude') != -1) {
					setAltitude(wayPts.params['CruisingAltitude'])
				}else {
					setDepth(nextPos.getZ())
				}
			}else if (wpcnt == wayPts.positions.size()) {
				//broadcast completion
				AgentID server = this.topic(C2Topics.MISSIONPOSITIONSTATUS)
				MissionPointStatusNtf msg = new MissionPointStatusNtf(server)
				msg.setmPointID(wayPts.getmPointBroadcastID())
				msg.setStatus(Status.COMPLETED)
				this.send(msg)
				log.info("WayPts with MpID "+wayPts.getmPointBroadcastID().toString()+" completed")
				wayPts = new WayPoints()
				wpcnt = 0
				return
			}
		}

		isWpReached = navigateTo(nextPos)
	}

	boolean navigateTo(Position pos) {
		float dist = C2Utils.getDist2D(curPos, pos)
		if(dist > wayPts.params['WaypointRadius']) {
			//println "time "+this.currentTimeMillis()+" Pilot:dist "+dist+" wyRadius"+wayPts.params['WaypointRadius']
			//println " Pilot:dist "+dist+" wyRadius"+wayPts.params['WaypointRadius']

			//calculate and set the bearing setpoint
			//TODO: decide on path following or waypoint following
			float auvBearing = calAngle(curPos,pos)
			if(auvBearing != null)
				setBearing(auvBearing)

			//log the setpoint and pos
			log.info("AuvPosBearing|"+curPos.toString()+" "+auvBearing+" "+dist)

			//determine if it is speed or thrust control
			if(wayPts.params.containsKey('CruisingSpeed')) {
				setSpeed(wayPts.params['CruisingSpeed'])
			}else {
				setThrust(wayPts.params['CruisingThrust'])
			}

			//determine if the wp is on the side of the AUV (make sure to change to AUV's body Frame).
			float angle1 = C2Utils.getAngle(curPos,pos)
			float ang1 = (curBearing-90)/180*C2Utils.PI;
			if(ang1<0) ang1+=2*C2Utils.PI;
			float ang2 = ang1+(secAngle/180*C2Utils.PI);
			if(ang2>2*C2Utils.PI) ang2-=2*C2Utils.PI;
			//float ang3 = (bearing-(270-secAngle/2))/180*PI;
			float ang3 = (curBearing-270)/180*C2Utils.PI;
			if(ang3<0) ang3+=2*C2Utils.PI;
			float ang4 = ang3+(secAngle/180*C2Utils.PI);
			if(ang4>2*C2Utils.PI) ang4-=2*C2Utils.PI;
			if(((angle1>=ang1&&angle1<=ang2)||(angle1>=ang3&&angle1<=ang4))&&dist<sideDist)
			{
				log.info("wayPoint on the side, procceed to next one");
				//log.info("angToNextPos=%f, ang1=%f,ang2=%f,ang3=%f,ang4=%f" ,angle1/PI*180,ang1/PI*180,ang2/PI*180,ang3/PI*180,ang4/PI*180);
				return true;
			}
			//TODO: decide on where to determine the waypoint progress ? here or higher level ?

			return false
		}else log.info "WayPt "+pos.toString()+" Reached !"
		return true
	}

	float calAngle(final Position p1, final Position p2) {
		float angle=C2Utils.getAngle(p1,p2)
		float angleBef = angle;
		float diff = Math.abs(curHeading-curBearing)
		if(diff > 1 && enablePathFollowing){ //TODO: adjust this parameter accordingly
			float tSpeed = 0.0//thrust induce speed
			if(wayPts.params.containsKey('CruisingSpeed')) {
				tSpeed = wayPts.params['CruisingSpeed']
			}else {
				tSpeed = C2Utils.thrust2Speed(wayPts.params['CruisingThrust'])
			}
			float curHeadingRad = Math.toRadians(curHeading)
			float curBearingRad = Math.toRadians(curBearing)
			float xComp = (curSpeed*Math.sin(angle))-((curSpeed*Math.sin(curHeadingRad))-(tSpeed*Math.sin(curBearingRad)))
			float yComp = (curSpeed*Math.cos(angle))-((curSpeed*Math.cos(curHeadingRad))-(tSpeed*Math.cos(curBearingRad)))
			angle = -(Math.atan2(yComp, xComp)-C2Utils.PIdiv2)
			if (angle < 0) angle += C2Utils.PItim2
			return Math.toDegrees(angle)
		}
		return Math.toDegrees(angleBef)
	}

	boolean checkDistAngle() {
		def angle = C2Utils.getAngle(curPos, nextPos)/C2Utils.PI*180
		def dist = C2Utils.getDist2D(curPos, nextPos)
		def bearingDiff = Math.abs(curBearing - angle)

		log.info("angleToNextPos= "+angle+" bearing= "+curBearing+" angleDiff= "+bearingDiff)

		if(bearingDiff > 45 && bearingDiff < 315) {
			//advance at most two points
			def cnt = 0;
			while(dist < sideDist && cnt < 2){
				if(wpcnt < wayPts.positions.size()) {
					lastPos = nextPos
					nextPos = wayPts.positions.get(wpcnt)
					wpcnt++
					AgentID lserver = topic(C2Topics.WAYPOSITIONSTATUS)
					WayPointStatusNtf msg = new WayPointStatusNtf(lserver)
					msg.setmPointID(wayPts.getmPointBroadcastID())
					msg.setWayPointNumber(wpcnt)
					send(msg)
					cnt++
					log.info("too close for comfort,waypoint incremented by 1")
				}else if (wpcnt == wayPts.positions.size()){
					return true
				}
			}
		}
		return false
	}

	//TODO:have the option to send VAL_DISABLE and VAL_ENABLE


	def setDepth(float depth) {
		def server = this.agentForService(C2Services.DIVINGOFFICER)
		if(server != null){
			DivingOfficerReq msg = new DivingOfficerReq(server)
			msg.setDepth(depth)
			this.send(msg)
		}else log.warning("pilot: divingOfficer not available")
	}

	def setAltitude(float altitude) {
		def server = this.agentForService(C2Services.DIVINGOFFICER)
		if(server != null){
			DivingOfficerReq msg = new DivingOfficerReq(server)
			msg.setAltitude(altitude)
			msg.setIsAltitudeControl(true)
			this.send(msg)
		}else log.warning("pilot: divingOfficer not available")
	}

	def setThrust(float thrust) {
		def time = this.currentTimeMillis()
		if(thrust == thrustLastUpdateValue) {
			if((time - thrustLastUpdateTime) > THRUST_UPDATE_INTERVAL)
			{
				AgentID server = this.agentForService(C2Services.ENGINEROOM)
				if(server != null){
					def msg = new EngineRoomReq(server)
					msg.setThrust(thrust)
					this.send(msg)
					thrustLastUpdateTime = time
				}else log.warning("pilot: EngineRoom not available")
			}
		} else {
			def server = this.agentForService(C2Services.ENGINEROOM)
			if(server != null){
				def msg = new EngineRoomReq(server)
				msg.setThrust(thrust)
				this.send(msg)
				thrustLastUpdateTime = time
				thrustLastUpdateValue = thrust
			}else log.warning("pilot: EngineRoom not available")
		}
	}

	def setSpeed(float speed) {
		def time = this.currentTimeMillis()
		if((time - thrustLastUpdateTime) > THRUST_UPDATE_INTERVAL)
		{
			def server = this.agentForService(C2Services.ENGINEROOM)
			if(server != null){
				def msg = new EngineRoomReq(server)
				msg.setSpeed(speed)
				msg.setIsSpeedControl(true)
				this.send(msg)
				thrustLastUpdateTime = time
			}else log.warning("pilot: EngineRoom not available")
		}
	}

	def setBearing(float bearing) {
		if(Math.abs(bearing - bearingLastUpdate) >= 0.1) {
			def server = this.agentForService(C2Services.HELMSMAN)
			if(server != null){
				HelmsmanReq msg = new HelmsmanReq(server)
				msg.setBearing(bearing)
				this.send(msg)
				bearingLastUpdate = bearing
				//log.info("bearing "+bearing+" sent")
			}else log.warning("pilot: Helmsman not available")
		}
	}

	@Override
	protected void die(Exception ex) {
		log.severe(getClass().getName() + " died: " + ex.getMessage());
		log.severe("restarting JC2");
		Runtime.getRuntime().exec("killall jc2");
	}

	
}
