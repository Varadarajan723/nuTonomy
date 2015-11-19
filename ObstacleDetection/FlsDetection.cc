
#include <dsaav/dsaav.h>
#include <iostream>
#include <stdio.h>
#include <fstream>
#include <math.h>
#include <starfish/FlsDetection.h>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/LU>
#include <vector>
#include <endian.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdlib.h>
#include <errno.h>

double bivnor ( double ah, double ak, double r );
double gauss ( double t );
double r8_abs ( double x );
double r8_max ( double x, double y );
double r8_min ( double x, double y );



//static int a;
using namespace std;
using namespace Eigen;

static int sonarIntensityData[44];
static int bin_occupied[44];
static int threshold[44];
static double p_detect[44];
static double p_fa;
static double flsAngle;
static float noOfBins;
static float angleRemainder;
static float angleInteger; 
static double timeBeforeLastUpdate;
static double bearingOffset;
static double angleEff;
static double deltaR;
static double deltaTheta;
static double flsTime;
static int numOfBins;
static int flag;
static int flag1;
static int broadcast;


int fd;

static const float MAXAGE = 0.05;


static double pos_x;
static double pos_y;
static double heading;
static double prev_pos_x;
static double prev_pos_y;
static double prev_heading;
static double deltaHeading;
static double displacementHeading;
static double dist;
static double angleDiff;
float pn;
static double deltaX;
static double deltaY;
static Sentuator* attSensor;

vector<int> no_go_region_x;
vector<int> no_go_region_y;

FILE* my_ptr;

ofstream output;
MatrixXd occupancy_grids(74,52);
MatrixXi obstacle_detection(74,52);
MatrixXd ***area_left_shift;
MatrixXd ***area_right_shift;
MatrixXd ***area_Overlap;

MatrixXd bin_centres(2,44);
MatrixXd covarianceMatrices[44];
MatrixXd centreOfSquares(2,44);
MatrixXd pLocal(2,2);
MatrixXd processNoise(2,2);
MatrixXd displacement(2,1);
Matrix2i dilate;

MatrixXd*** initialize_4dMatrix(MatrixXd*** matrix, int sizes[], MatrixXd area_2D) {
    matrix = new MatrixXd** [sizes[0]];
    for (int d1 = 0; d1 < sizes[0]; d1++) {
        matrix[d1] = new MatrixXd* [sizes[1]];
        for (int d2 = 0; d2 < sizes[1]; d2++)
            matrix[d1][d2] = new(MatrixXd)(area_2D.block<3,3>(3*d1,3*d2));
    }
    return matrix;
	delete matrix;
}

MatrixXd*** initialize_5dMatrix(MatrixXd*** matrix, int sizes[], MatrixXd area_overlap) {
    matrix = new MatrixXd** [sizes[0]];
    for (int d1 = 0; d1 < sizes[0]; d1++) {
        matrix[d1] = new MatrixXd* [sizes[1]];
        for (int d2 = 0; d2 < sizes[1]; d2++)
            matrix[d1][d2] = new(MatrixXd)(area_overlap.block<5,5>(5*d1,5*d2));
    }
    return matrix;
	delete matrix;
}



MatrixXi fill(MatrixXi OccupancyGrids)
{
	
	for (int i=1;i<OccupancyGrids.rows()-1;i++)
		for (int j=1;j<OccupancyGrids.cols()-1;j++)
			if(OccupancyGrids(i,j)==0)
				if(OccupancyGrids(i,j-1) == 1 && OccupancyGrids(i,j+1) == 1 && OccupancyGrids(i-1,j) == 1 && OccupancyGrids(i+1,j) == 1 )
					OccupancyGrids(i,j) = 1;
	
	
	return OccupancyGrids;
	
}

MatrixXi dilation(MatrixXi OccupancyGrid, Matrix2i Dilate)
{
	int i,j;
	MatrixXi OccupancyGrid_Intermediate;
	OccupancyGrid_Intermediate = MatrixXi::Zero(OccupancyGrid.rows()+2,OccupancyGrid.cols()+2);
	
	OccupancyGrid_Intermediate.block(1,1,OccupancyGrid.rows(),OccupancyGrid.cols()) = OccupancyGrid;
	MatrixXi OccupancyGrid_Dilated;
	OccupancyGrid_Dilated = MatrixXi::Zero(OccupancyGrid.rows(),OccupancyGrid.cols());
	
	for (int row = 0; row < OccupancyGrid_Intermediate.rows()-2; row++ ) { 
		for (int col = 0; col < OccupancyGrid_Intermediate.cols()-2; col++ ) {
			
			for (i = 1; i <= 2; i++ ) {
				for (j = 1; j <= 2; j++ ) {
					if(OccupancyGrid_Intermediate(i+row-1,j+col-1) == Dilate(i-1,j-1)){
						OccupancyGrid_Dilated(row,col) = 1;
						break;
					}
					
				}
				
				if (OccupancyGrid_Dilated(row,col) == 1)
					break;
				
			} 
		}
	}
	
	
	return OccupancyGrid_Dilated;
	
	
}


void cmAlternative(double angle, MatrixXd PLocal,MatrixXd &BinCentres, MatrixXd *CovarianceMatrices,MatrixXd &CentreOfSquares)
{

    double r = 50.0/44.0;
	for (int i=0;i<44;i++)
	{
		
		CovarianceMatrices[i](0,0) = (i+0.5)*(i+0.5)*PLocal(1,1)*sin(angle)*sin(angle) + PLocal(0,0)*cos(angle)*cos(angle);
		CovarianceMatrices[i](1,1) = (i+0.5)*(i+0.5)*PLocal(1,1)*cos(angle)*cos(angle) + PLocal(0,0)*sin(angle)*sin(angle);
		CovarianceMatrices[i](1,0) = (PLocal(0,0) -((i+0.5)*(i+0.5)*PLocal(1,1)))*sin(angle)*cos(angle);
		CovarianceMatrices[i](0,1) = CovarianceMatrices[i](1,0);
		
		BinCentres(0,i) = ((r*i)+r/2.0)*cos(angle);
		BinCentres(1,i) = ((r*i)+r/2.0)*sin(angle);
		
		CentreOfSquares(0,i) = ceil(BinCentres(0,i)) - 0.5;
		CentreOfSquares(1,i) = ceil(BinCentres(1,i)) - 0.5;
		
	}
	
}



void MeasurementModel(MatrixXd BinCentres,MatrixXd CentreOfSquares,int BinOccupied[],MatrixXd &OccupancyGrids,vector<int> &No_Go_Region_X, vector<int> &No_Go_Region_Y)
{
	
	
	MatrixXd OccupancyGridsIntermediate = MatrixXd::Ones(76,54)*0.4;
    OccupancyGridsIntermediate.block(1,1,74,52) = OccupancyGrids;
	double prob_no_hit;
	double prob_hit;
	float r = 50.0/44.0; 
    float range;
    double theta;
    float frac;
    float integer;
    int ra;
    int th;
	int start_x;
	int start_y;
	MatrixXd likelihood_notoccupancy(1,25);
	MatrixXd likelihood_occupancy(1,25);
    MatrixXd kk(1,25);

	
	for (int numberOfBins= 0;numberOfBins<44;numberOfBins++)
	{
		
		
		start_x = ceil(CentreOfSquares(0,numberOfBins)) + 37 - 1 -1;
		start_y = ceil(CentreOfSquares(1,numberOfBins))-1;
        
		likelihood_occupancy = MatrixXd::Zero(1,25);
		likelihood_notoccupancy = MatrixXd::Zero(1,25);
		
        theta = atan2(BinCentres(1,numberOfBins),BinCentres(0,numberOfBins))*180/M_PI;
        range = sqrt((BinCentres(1,numberOfBins)*BinCentres(1,numberOfBins)) + (BinCentres(0,numberOfBins)*BinCentres(0,numberOfBins)));
        range = (range+r/2.0)/r;
        frac = modf(range,&integer);
        
        ra = integer;        
        th = ((int)theta - 45)/3 + 1;
        prob_hit = 0;
		prob_no_hit = 0;
        
	
		
		if( BinOccupied[numberOfBins] == 0 )
		{
	
		    MatrixXd prob_distribution(5,5);
            prob_distribution = *area_Overlap[ra-1][th-1];
        	prob_distribution.transposeInPlace();
            prob_distribution.resize(1,25);
 
            MatrixXd intermediate(5,5);
			intermediate = OccupancyGridsIntermediate.block(start_x,start_y,5,5).transpose();
            intermediate.resize(1,25);	
			kk = -1*intermediate.array() + 1;
        
			intermediate = prob_distribution.cwiseProduct(intermediate)*(1-p_detect[numberOfBins]) + (1-p_fa)*prob_distribution.cwiseProduct(kk);
            
        
            prob_no_hit = intermediate.sum();
			likelihood_notoccupancy = (-1*intermediate).array() + prob_no_hit + (1-p_fa)*prob_distribution.array();
	        likelihood_notoccupancy.resize(5,5);
			likelihood_notoccupancy.transposeInPlace();
			
			MatrixXd inter(5,5);
			inter = (-1 * OccupancyGridsIntermediate.block(start_x,start_y,5,5)).array() + 1;
			inter = likelihood_notoccupancy.cwiseProduct(inter)/prob_no_hit;
			OccupancyGridsIntermediate.block(start_x,start_y,5,5) = 1 + (-1*inter).array();
			
		}
		
		else
		{
			
            MatrixXd prob_distribution(5,5);
            prob_distribution = *area_Overlap[ra-1][th-1];
            prob_distribution.transposeInPlace();
            prob_distribution.resize(1,25);
            
            MatrixXd intermediate(5,5);
			intermediate = OccupancyGridsIntermediate.block(start_x,start_y,5,5).transpose();                  
			intermediate.resize(1,25);
			kk = -1*intermediate.array() + 1;
			intermediate = prob_distribution.cwiseProduct(intermediate)*p_detect[numberOfBins] + p_fa*prob_distribution.cwiseProduct(kk);
			
   
			prob_hit = intermediate.sum();
			
			likelihood_occupancy = (-1*intermediate).array() + prob_hit + p_detect[numberOfBins]*prob_distribution.array();
			likelihood_occupancy.resize(5,5);
			likelihood_occupancy.transposeInPlace();
			
			OccupancyGridsIntermediate.block(start_x,start_y,5,5) =likelihood_occupancy.cwiseProduct(OccupancyGridsIntermediate.block(start_x,start_y,5,5))/prob_hit;
			No_Go_Region_X.push_back(start_x+2-1);
			No_Go_Region_Y.push_back(start_y+2-1);
			
		}
		
	}
	
    OccupancyGrids = OccupancyGridsIntermediate.block(1,1,74,52);
}

MatrixXd conv2d(MatrixXd OccupancyGrid, Matrix2d ProcessNoise, MatrixXd Displacement)

{
	
    VectorXd v;
    v.setLinSpaced(3,-0.5f,1.5f);
    MatrixXd X(3,3);
    for (int i=1;i<=3;i++)
        X.row(i-1) = v.transpose();
    MatrixXd Y = X.transpose();
    
    X.resize(9,1);
    Y.resize(9,1);
    MatrixXd Z(2,9);
    Z.row(0) = X.transpose();
    Z.row(1) = Y.transpose();
    
    Matrix2d inverse;
    bool invertible;
    double determinant;
	
	MatrixXd mean(2,1);
    
    mean << 1.5,
	1.5;
	
	MatrixXd Kernel(3,3);
    Kernel = MatrixXd::Zero(3,3);
	
    
	double delX = Displacement(1,0);
	double delY = Displacement(0,0);
	
	
	MatrixXd OccupancyGrid_Convolved(OccupancyGrid.rows()-2,OccupancyGrid.cols()-2);
    
	ProcessNoise.computeInverseAndDetWithCheck(inverse,determinant,invertible);

	if (determinant == 0)
	{
		if ( delX <= 0 && delY <= 0){
			
			Kernel << 0 , 0, 0,
			          0	,(1-abs(delX))*(1-abs(delY)),abs(delY)*(1-abs(delX)),
			          0	, abs(delX)*(1-abs(delY)),abs(delY)*abs(delX);				 
			
		}
		if ( delX >= 0 && delY >= 0){
			
			
			Kernel << abs(delY)*abs(delX),abs(delX)*(1-abs(delY)) ,0,
			          abs(delY)*(1-abs(delX)),(1-abs(delX))*(1-abs(delY)) ,0,
			          0,0, 0;
		}
		if ( delX <= 0 && delY >= 0){
			
			
			
			Kernel<< 0, 0, 0,
                     abs(delY)*(1-abs(delX)) ,(1-abs(delX))*(1-abs(delY)),0,
			         abs(delY)*abs(delX), abs(delX)*(1-abs(delY)) ,0;
			
			
		}							 
		
		if (delX >= 0 && delY <= 0)
		{
			
			Kernel<< 0,abs(delX)*(1-abs(delY)),abs(delY)*abs(delX),
			         0,(1-abs(delX))*(1-abs(delY)),abs(delY)*(1-abs(delX)),
                     0, 0, 0;
        }
	
	}
	else {
        

        double gaussianKernel_1_1= bivnor(-(Z(0,0)+delY)/sqrt(ProcessNoise(0,0)),-(Z(1,0) + delX)/sqrt(ProcessNoise(1,1)),0);
        double gaussianKernel_1_2= bivnor(-(Z(0,1)+delY)/sqrt(ProcessNoise(0,0)),-(Z(1,1) + delX)/sqrt(ProcessNoise(1,1)),0);
        double gaussianKernel_2_1= bivnor(-(Z(0,3)+delY)/sqrt(ProcessNoise(0,0)),-(Z(1,3) + delX)/sqrt(ProcessNoise(1,1)),0);
        double gaussianKernel_2_2 = bivnor(-(Z(0,4)+delY)/sqrt(ProcessNoise(0,0)),-(Z(1,4) + delX)/sqrt(ProcessNoise(1,1)),0);
        
        double X_lim = INFINITY;
        double Y_lim = INFINITY;
        
        
        Kernel(2,0) = gaussianKernel_1_1;
        Kernel(1,0) = gaussianKernel_1_2 - gaussianKernel_1_1;
        Kernel(0,0) = bivnor(-(1-1.5+delY)/sqrt(ProcessNoise(0,0)),-Y_lim,0) - gaussianKernel_1_2;
        
        Kernel(2,1) = gaussianKernel_2_1 - gaussianKernel_1_1;
        Kernel(1,1) = gaussianKernel_2_2 - gaussianKernel_2_1 - Kernel(1,0);
        Kernel(0,1) = bivnor(-(2-1.5+delY)/sqrt(ProcessNoise(0,0)),-Y_lim,0) - gaussianKernel_2_2 - Kernel(0,0);
        
        Kernel(2,2) = bivnor(-X_lim,-(1-1.5+delX)/sqrt(ProcessNoise(1,1)),0) - gaussianKernel_2_1;
        Kernel(1,2) = bivnor(-X_lim,-(2-1.5+delX)/sqrt(ProcessNoise(1,1)),0) - gaussianKernel_2_2 - Kernel(2,2);
        Kernel(0,2) =  1 - Kernel.sum();
        
        
	}
	
	for (int row = 1; row < OccupancyGrid.rows()-1; row++ ) {
		for (int col = 1; col < OccupancyGrid.cols()-1; col++ ) {
			double accumulation = 0;
			for (int i = -1; i <= 1; i++ ) {
				for (int j = -1; j <= 1; j++ ) {
					double k = OccupancyGrid(row-j, col-i);
					accumulation += k * Kernel(1+j,1+i);
					
				}
			}
			OccupancyGrid_Convolved(row-1,col-1)=accumulation;
		}
	}
	
	
	return OccupancyGrid_Convolved;
	
	
}

void MotionUpdate(MatrixXd &OccupancyGrids,int DeltaHeading,MatrixXd Displacement, Matrix2d ProcessNoise)
{
	
	MatrixXd occupancy_grids_intermediate = MatrixXd::Ones(76,54)*0.4;
    occupancy_grids_intermediate.block(1,1,74,52) = OccupancyGrids;    
	OccupancyGrids = conv2d(occupancy_grids_intermediate,ProcessNoise,Displacement);
    occupancy_grids_intermediate.block(1,1,74,52) = OccupancyGrids;

	
	if(DeltaHeading!=0)
	{
		if(DeltaHeading == - 1)
			for (int i =0 ; i<74;i++)
				for (int j = 0;j<52;j++)
				{
					OccupancyGrids(i,j) = occupancy_grids_intermediate.block(i,j,3,3).cwiseProduct(*area_right_shift[i][j]).sum();
				}
		else
			
			for (int i =0 ; i<74;i++)
				for (int j = 0;j<52;j++)
				{
					OccupancyGrids(i,j) = occupancy_grids_intermediate.block(i,j,3,3).cwiseProduct(*area_left_shift[i][j]).sum();
				}
		
	}
	
	

	
	
}



FlsDetection::FlsDetection(Rpc* rpc) : SentuatorService(rpc,"FlsDetection")
{
	log->info("$Id: FLS DATA COLLECTION AND OBJECT DETECTION:01-07-2013: 14:52:00");
	numOfBins = cfg->getInteger("numOfBins",44);
	p_fa = cfg->getFloat("ProbabilityOfFalseAlarm",0.02);
	attSensor = cfg->getSentuator("Attitude");
    if (attSensor == NULL) log->warning("Unable to find sentuator [Attitude]");
	
	
	int sizes[4] = {74,52,3,3};
	
    int size_area_overlap[4] = {44,31,5,5};
    
    ifstream file,file2,file3,file4,file5;
	file.open( "/mnt/hda3/STARFISHRUN/bin/area_2D_shift_left.txt");
	file2.open("/mnt/hda3/STARFISHRUN/bin/area_2D_shift_right.txt");
	file3.open("/mnt/hda3/STARFISHRUN/bin/p_detection.txt");
	file4.open("/mnt/hda3/STARFISHRUN/bin/threshold.txt");
	file5.open("/mnt/hda3/STARFISHRUN/bin/areaOverlap.txt");
	
	
	my_ptr = fopen("result.bin","wb");
	
	double mat_vec_right[34632] = {0};
	double mat_vec_left[34632] = {0};
    double mat_area[34100] = {0};
    
    int counter = 0;
	
	for( int i=0; i<222; i++){
        for(int j=0;j<156;j++){
            file >> mat_vec_left[counter];
			file2 >> mat_vec_right[counter];
            counter++;
        }
    }          
	int counter_2 = 0;
    
    for( int i=0; i<220; i++){
        for(int j=0;j<155;j++){
            file5 >> mat_area[counter_2];
            counter_2++;
        }
    
    }  
    
	
	for(int i=0;i<44;i++){
		file3 >> p_detect[i];
		file4 >> threshold[i];
	}
	
	
    file.close();
    file2.close();
	file3.close();
	file4.close();
    file5.close();
    MatrixXd area_2d_left = Map<MatrixXd>(mat_vec_left,156,222).transpose();
	MatrixXd area_2d_right = Map<MatrixXd>(mat_vec_right,156,222).transpose();
	MatrixXd area_overlap = Map<MatrixXd>(mat_area,155,220).transpose();
    
	area_left_shift = initialize_4dMatrix(area_left_shift, sizes,area_2d_left);
	area_right_shift = initialize_4dMatrix(area_right_shift,sizes,area_2d_right);
	area_Overlap = initialize_5dMatrix(area_Overlap,size_area_overlap,area_overlap);
      

	deltaR = 1;
	deltaTheta = 3*M_PI/180;
	deltaHeading = 0;
	bearingOffset = 90;
	
	
	
	pLocal <<    pow(deltaR,2)/12 , 0,
	0,   pow(deltaTheta,2)/12;
	
	
	occupancy_grids = MatrixXd::Ones(74,52)*0.4;
	centreOfSquares = MatrixXd::Ones(2,44);
	for(int i = 0;i<44;i++){
        covarianceMatrices[i] = MatrixXd::Ones(2,2);
    }
	flag = 0;
	flag1 = 0;
	
	dilate = Matrix2i::Ones(2,2);
	obstacle_detection = MatrixXi::Zero(74,52);
	broadcast = 0;
	bind(MT_OBSTACLE);
	
	
}


Measurement* FlsDetection::get(MeasurementType key, float maxAge)
{
	if (key != MT_OBSTACLE) return NULL;
	
	if (broadcast)
	{
	FILE* my_ptr2;
	my_ptr2 = fopen("obstacleDetection.bin","wb");
	int a; 
	for(int i=0;i<74;i++){
		for(int j=0;j<52;j++)
		{
			a = htobe32(obstacle_detection.colwise().reverse()(i,j));
			fwrite(&a,sizeof(int),1,my_ptr2);
			//a = obstacle_detection(i,j);
			//write(fd,&a,sizeof(a));
		}
	}
	
	
	fflush(my_ptr2);
	delete my_ptr2;
	
	}
	
	Measurement* obs = NULL;
	obs = new Measurement(1,getTime());
	obs->put(MQ_MEMORY,broadcast);
	return obs;	
}



void FlsDetection::notification(MeasurementType key, Measurement* m){
	
	switch (key){
			
		case MT_OBSTACLE:
			

			flsAngle = m->get(MQ_FLS_BEARING);
            noOfBins = m->get(MQ_FLS_NBINS);

			angleRemainder = modf(flsAngle,&angleInteger);
			int i;				        
            
			
			
			for (int index=0;index<44;index++)  //44 is the number of bins to expect
			{
				if (index < noOfBins){      
					sonarIntensityData[index] = int(m->get(MQ_INTENSITY,index));    //reading an array through measurement quantity

					if(index<9)
						bin_occupied[index] = 0;
					
					else
						if(sonarIntensityData[index] < threshold[index]){
		
							bin_occupied[index] = 0;
						}
						else
						{
							bin_occupied[index] = 1;
						}
				}
								
			}      
			
			
			
			angleEff = (flsAngle - bearingOffset)*M_PI/180;
			
			cmAlternative(angleEff,pLocal,bin_centres,covarianceMatrices,centreOfSquares);
			MeasurementModel(bin_centres,centreOfSquares,bin_occupied,occupancy_grids, no_go_region_x,no_go_region_y);
	//		log->debug("Measurement Update Done");
			
			
			
			
			if(angleInteger == 135 && angleRemainder == 0 ){

	
				broadcast = 1;
				timeBeforeLastUpdate = getTime();
				obstacle_detection = MatrixXi::Zero(74,52);
				
				for (i = 0; i<no_go_region_x.size(); ++i)
					if (occupancy_grids.block(no_go_region_x[i]-1,no_go_region_y[i]-1,3,3).sum() > 0.8)
						obstacle_detection.block(no_go_region_x[i]-1,no_go_region_y[i]-1,3,3) = MatrixXi::Ones(3,3);
				
					obstacle_detection=dilation(obstacle_detection,dilate);
					obstacle_detection=fill(obstacle_detection);
		
								
				no_go_region_x.clear();
				no_go_region_y.clear();
				


				
	
			    
			}
			
			if(angleInteger == 225 && angleRemainder == 0){
				
				
			

				broadcast = 1;
				timeBeforeLastUpdate = getTime();
				obstacle_detection = MatrixXi::Zero(74,52);
			    

				
				for (i = 0; i<no_go_region_x.size(); ++i)
					if (occupancy_grids.block(no_go_region_x[i]-1,no_go_region_y[i]-1,3,3).sum() > 0.8)
						obstacle_detection.block(no_go_region_x[i]-1,no_go_region_y[i]-1,3,3) = MatrixXi::Ones(3,3);
				
				obstacle_detection=dilation(obstacle_detection,dilate);
				obstacle_detection=fill(obstacle_detection);
							
				no_go_region_x.clear();
				no_go_region_y.clear();
				
								
		

				

			}
			break;
			
		case MT_POSITION:
			
 

			if(flag == 0){
				prev_pos_x = m->get(MQ_XPOS);
				prev_pos_y = m->get(MQ_YPOS);
                pn = m->get(MQ_PN);
				flag = 1;
			}
			else if(flag1 == 1)
			{
				pos_x = m->get(MQ_XPOS);
				pos_y = m->get(MQ_YPOS);
                pn = m->get(MQ_PN);
       
				
				deltaX =  pos_x - prev_pos_x;
				deltaY = pos_y - prev_pos_y;
				
				
                
                displacementHeading = atan2(deltaY,deltaX);
                displacementHeading = displacementHeading * 180/M_PI; 
                
                if (displacementHeading <= 0)
                {
                    displacementHeading = 90 + abs(displacementHeading);
                }
                else{
                    if (displacementHeading <= 90)
                    {
                        displacementHeading = 90 - displacementHeading;
                    }
                    else 
                    {
                        displacementHeading = 360 - displacementHeading + 90;
                    }
                }
                
                angleDiff = displacementHeading - heading;
                
                dist = sqrt((deltaX*deltaX) + (deltaY*deltaY));
                
                deltaX = dist*sin(angleDiff*M_PI/180);
                deltaY = dist*cos(angleDiff*M_PI/180);
                displacement << deltaY,
                                deltaX;
                
                              
                              
				prev_pos_x = pos_x;
				prev_pos_y = pos_y;
				
                     
 
                if (pn == 1.0)
                {
    
                    processNoise << 0.0 , 0.0,
                                    0.0 , 0.0;
                }
                else 
                { if (pn == 2.0)
                  {
         
                    processNoise << 0.00, 0.0,
                                    0.0,  0.00;
                  }
                  else
                  {
                    processNoise << 0.0, 0.0,
                                    0.0, 0.0;
                  }
                }
                
                
                
				MotionUpdate(occupancy_grids,deltaHeading,displacement,processNoise);

				
			}
			break;
			
		case MT_ATTITUDE:
			
		//	log->debug("Heading received");
			if (flag1 == 0){
				heading = m->get(MQ_BEARING);
				prev_heading = m->get(MQ_BEARING);
				flag1 = 1;
			}
			else {
				heading = m->get(MQ_BEARING);
				
				deltaHeading = prev_heading - heading;
				if (deltaHeading < -1 || deltaHeading > 1){
					if (deltaHeading < -1){
						deltaHeading = -1;
	
					}
					else
					{
						deltaHeading = 1;
					
					}
					prev_heading = heading;
				}
				else
					deltaHeading = 0;
			}
			break;
			
			
		default :
			//do nothing
			break;
			
			
			
	}
}


void FlsDetection::tick(void)
{
	

	
	if(getTime() - timeBeforeLastUpdate > 20.0)
		broadcast = 0;
	
	
	
}



double bivnor ( double ah, double ak, double r )
{
	double a2;
	double ap;
	double b;
	double cn;
	double con;
	double conex;
	double ex;
	double g2;
	double gh;
	double gk;
	double gw;
	double h2;
	double h4;
	int i;
	static int idig = 15;
	int is;
	double rr;
	double s1;
	double s2;
	double sgn;
	double sn;
	double sp;
	double sqr;
	double t;
	static double twopi = 6.283185307179587;
	double w2;
	double wh;
	double wk;
	
	b = 0.0;
	
	gh = gauss ( - ah ) / 2.0;
	gk = gauss ( - ak ) / 2.0;
	
	if ( r == 0.0 )
	{
		b = 4.00 * gh * gk;
		b = r8_max ( b, 0.0 );
		b = r8_min ( b, 1.0 );
		return b;
	}
	
	rr = ( 1.0 + r ) * ( 1.0 - r );
	
	if ( rr < 0.0 )
	{
		fprintf ( stderr, "\n" );
		fprintf ( stderr, "BIVNOR - Fatal error!\n" );
		fprintf ( stderr, "  1 < |R|.\n" );
		exit ( 0 );
	}
	
	if ( rr == 0.0 )
	{
		if ( r < 0.0 )
		{
			if ( ah + ak < 0.0 )
			{
				b = 2.0 * ( gh + gk ) - 1.0;
			}
		}
		else
		{
			if ( ah - ak < 0.0 )
			{
				b = 2.0 * gk;
			}
			else
			{
				b = 2.0 * gh;
			}
		}
		b = r8_max ( b, 0.0 );
		b = r8_min ( b, 1.0 );
		return b;
	}
	
	sqr = sqrt ( rr );
	
	if ( idig == 15 )
	{
		con = twopi * 1.0E-15 / 2.0;
	}
	else
	{
		con = twopi / 2.0;
		for ( i = 1; i <= idig; i++ )
		{
			con = con / 10.0;
		}
	}
	/*
	 (0,0)
	 */
	if ( ah == 0.0 && ak == 0.0 )
	{
		b = 0.25 + asin ( r ) / twopi;
		b = r8_max ( b, 0.0 );
		b = r8_min ( b, 1.0 );
		return b;
	}
	/*
	 (0,nonzero)
	 */
	if ( ah == 0.0 && ak != 0.0 )
	{
		b = gk;
		wh = -ak;
		wk = ( ah / ak - r ) / sqr;
		gw = 2.0 * gk;
		is = 1;
	}
	/*
	 (nonzero,0)
	 */
	else if ( ah != 0.0 && ak == 0.0 )
	{
		b = gh;
		wh = -ah;
		wk = ( ak / ah - r ) / sqr;
		gw = 2.0 * gh;
		is = -1;
	}
	/*
	 (nonzero,nonzero)
	 */
	else if ( ah != 0.0 && ak != 0.0 )
	{
		b = gh + gk;
		if ( ah * ak < 0.0 )
		{
			b = b - 0.5;
		}
		wh = - ah;
		wk = ( ak / ah - r ) / sqr;
		gw = 2.0 * gh;
		is = -1;
	}
	
	for ( ; ; )
	{
		sgn = -1.0;
		t = 0.0;
		
		if ( wk != 0.0 )
		{
			if ( r8_abs ( wk ) == 1.0 )
			{
				t = wk * gw * ( 1.0 - gw ) / 2.0;
				b = b + sgn * t;
			}
			else
			{
				if ( 1.0 < r8_abs ( wk ) )
				{
					sgn = -sgn;
					wh = wh * wk;
					g2 = gauss ( wh );
					wk = 1.0 / wk;
					
					if ( wk < 0.0 )
					{
						b = b + 0.5;
					}
					b = b - ( gw + g2 ) / 2.0 + gw * g2;
				}
				h2 = wh * wh;
				a2 = wk * wk;
				h4 = h2 / 2.0;
				ex = exp ( - h4 );
				w2 = h4 * ex;
				ap = 1.0;
				s2 = ap - ex;
				sp = ap;
				s1 = 0.0;
				sn = s1;
				conex = r8_abs ( con / wk );
				
				for ( ; ; )
				{
					cn = ap * s2 / ( sn + sp );
					s1 = s1 + cn;
					
					if ( r8_abs ( cn ) <= conex )
					{
						break;
					}
					sn = sp;
					sp = sp + 1.0;
					s2 = s2 - w2;
					w2 = w2 * h4 / sp;
					ap = - ap * a2;
				}
				t = ( atan ( wk ) - wk * s1 ) / twopi;
				b = b + sgn * t;
			}
		}
		if ( 0 <= is )
		{
			break;
		}
		if ( ak == 0.0 )
		{
			break;
		}
		wh = -ak;
		wk = ( ah / ak - r ) / sqr;
		gw = 2.0 * gk;
		is = 1;
	}
	
	b = r8_max ( b, 0.0 );
	b = r8_min ( b, 1.0 );
	
	return b;
}


double gauss ( double t )
{
	double value;
	
	value = ( 1.0 + erf ( t / sqrt ( 2.0 ) ) ) / 2.0;
	
	return value;
}

double r8_abs ( double x )
{
	double value;
	
	if ( 0.0 <= x )
	{
		value = + x;
	}
	else
	{
		value = - x;
	}
	return value;
}


double r8_max ( double x, double y )
{
	double value;
	
	if ( y < x )
	{
		value = x;
	}
	else
	{
		value = y;
	}
	return value;
}


double r8_min ( double x, double y )

{
	double value;
	
	if ( y < x )
	{
		value = y;
	}
	else
	{
		value = x;
	}
	return value;
}




