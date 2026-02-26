/*
* Copyright(c) 2021
* Rodrigues, T. E. P.; Durandau, G. V. All rights reserved.

* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
* 1. Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
* 2. Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the distribution.
* 3. All advertising materials mentioning features or use of this software
*    must display the following acknowledgement:
*      This product includes software developed by Rodrigues, T. and Durandau G.
* 4. Neither the name of the copyright holder nor the names of its contributors
*    may be used to endorse or promote products derived from this software
*    without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED.  IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
* OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
* OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
* SUCH DAMAGE.
*/


// Muscle-in-the-loop Optimization
// Tiago Rodrigues, Guillaume Durandau, 2021
// tiagoepr@ua.pt | t.e.pereirarodrigues@student.utwente.nl

#ifndef OPTIMIZATIONPLUGIN_H_
#define OPTIMIZATIONPLUGIN_H_


#include <iostream>
#include <iomanip>
#include <fstream>
#include <cstdlib> // for exit function of computation time logger
#include <cfloat> // for DBL_MAX


// Libraries from Boost
#include <boost/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/bind.hpp>

// Headers for running the CEINMS-RT plugin
#include "OptimizationPlugin.h"
#include "OpenSimFileLogger.h"
#include "ExecutionXmlReader.h"
#include "ExecutionOptimizationXmlReader.h"
#include "SetupDataStructure.h"
#include "Activation/ExponentialActivation.h"
#include "Activation/ExponentialActivationRT.h"
#include "Tendon/StiffTendon.h"
#include "Tendon/ElasticTendon.h"
#include "Tendon/ElasticTendon_BiSec.h"
#include "NMSmodel.h"

// Client to send torque to the Exoskeleton
#include <tcAdsClient.h>
// Had to had the ADS library into the Linker's additional libraries: https://infosys.beckhoff.com/english.php?content=../content/1033/tc3_adsdll2/123108747.html&id=


// Headers of the optimization plugin
#include "SimulatedAnnealingBase.h"
#include <ErrorMinimizerAnnealing.h>
#include <chrono>

// Pagmo
#include <pagmo/pagmo.hpp>
#include "MusclePagmoProblem.cpp" // For the UDP Class



using namespace pagmo;
using namespace std;


#ifdef WIN32
template <typename NMSmodelT>
class __declspec(dllexport) OptimizationMusclePlugin : public OptimizationPlugin<NMSmodelT>
#endif
#ifdef UNIX
	template <typename NMSmodelT>
class OptimizationMusclePlugin : public OptimizationPlugin<NMSmodelT>
#endif
{
public:
	OptimizationMusclePlugin();
	~OptimizationMusclePlugin();
	void init(NMSmodelT& model, const std::string& executionXMLFileName, const std::string& configurationFileName);
	void newData();

	/*
		>> Functions to get and set the plugin data
	*/
	std::vector<double> getDofTorque();
	std::vector<double> getShapeFactor();
	std::vector<double> getTendonSlackLengths();
	std::vector<double> getOptimalFiberLengths();
	std::vector<double> getGroupMusclesBasedOnStrengthCoefficients();


	// Optimization plugin's mandatory CEINMS-RT virtual get and set functions
	void setLmt(const std::vector<double>& lmt);
	void setMA(const std::vector<std::vector<double> >& ma);
	void setMuscleForce(const std::vector<double>& muscleForces);
	void setDOFTorque(const std::vector<double>& dofTorque);
	void setExternalDOFTorque(const std::vector<double>& externalDOFTorque);
	void setActivations(const std::vector<double>& activations);
	void setFibreLengths(const std::vector<double>& fibreLengths);
	void setFibreVelocities(const std::vector<double>& fibreVelocities);
	void setPennationAngle(const std::vector<double>& pennationAngle);
	void setTendonLength(const std::vector<double>& tendonLength);
	void setTime(const double& time);
	void setEmgs(const std::vector<double>& Emgs);
	void setJointAngle(const std::vector<double>& jointAngle) {}
	bool getFromBuffer(double& time, std::vector<double>& Emgs, std::vector<double>& externalDOFTorque, std::vector<double>& lmt, std::vector<std::vector<double> >& ma);

	void setSR(vector<double> sr);

	void start(); //Start the plugin
	void stop(); // Stop the plugin
	void setDirectories(const std::string& outDirectory, const std::string& inDirectory = std::string());
	void setVerbose(const int& verbose);
	void setRecord(const bool& record);


protected:
	/*
		>> Local variables and functions
	*/

	// Threads
	boost::thread* threadOptimization_;
	boost::thread* threadTorque_;
	boost::thread* threadAlgorithmR_;
	boost::thread* threadAlgorithmL_;
	bool threadStop_;
	bool algorithmStartR_ = false;
	bool algorithmStartL_ = false;
	bool optimizationDone_R_ = false;
	bool optimizationDone_L_ = false;

	// For the new data validator
	boost::mutex DataMutex_;
	boost::mutex newDataMutex_;
	bool newData_;
	boost::condition_variable conditionNewData_;

	// Loops
	void torqueLoop(); //Loop for assistance torque computation
	void optimizationLoop(); //Loop for optimization
	void algorithmLoopR(); // worker thread to update the SR, wait for the mean EMG, and compute the cost function
	void algorithmLoopL(); // worker thread to update the SR, wait for the mean EMG, and compute the cost function


	/*
		>> Personal Functions/Methods
	*/

	// Torque Loop
	vector<double> computeTorque(); // For computing the torque
	void sendTorque(vector<double> dofTorque);
	void printGeneralTable();

	// Optimization Loop
	bool optimizationBool_; // To state if we want to do the optimization or use a static SR
	void newEMGvalidator();
	void populateEMG(); // To add current EMG to list
	void getGRFEvent(); // To get a step event
	void computeEMGmean(vector<unsigned> muscleIndexList); // To compute the EMG mean of the indexed muscles
	void addRefEMGmean(vector<unsigned> muscleIndexList); // To add the EMG mean of each muscle to the vector firstsEMGmean_
	void computeRefEMGmean(vector<unsigned> muscleIndexList); //To compute the EMG REF mean and SD of the indexed muscles
	void printSR(vector<unsigned> muscleIndexList);
	double computeFitness(vector<unsigned> musclesToOptimizeIndex_, vector<unsigned> musclesNonOptimizedIndex_); // To compute the fitness after optimization so we can have an idea of the variance of the EMG with the same SR


	// Loggers
	void loggersInit(string directoriOptimization);
	void loggers(string foot, float f, string type);
	void loggersRef(string foot);
	std::string outDirectory_;
	bool record_;
	OpenSimFileLogger<NMSmodelT>* logger_;


	/*
		>> NMS model and calibration
	*/
	NMSmodelT* model_;
	void setupSubject(NMSmodelT& mySubject, string configurationFile);

	// Model Variables
	std::vector<std::string> muscleName_; // Muscle List
	std::vector<std::string> dofName_;
	std::vector<unsigned int> dofsUsed_;
	std::vector<double> shapeFactor_;
	std::vector<double> tendonSlackLengths_;
	std::vector<double> optimalFiberLengths_;
	std::vector<double> groupMusclesBasedOnStrengthCoefficients_;
	std::vector<double> lmt_;
	std::vector<double> dofTorque_;
	std::vector<double> externalDOFTorque_;
	std::vector<double> activations_;
	std::vector<double> fibreLengths_;
	std::vector<double> fibreVelocities_;
	std::vector<double> pennationAngle_;
	std::vector<double> tendonLength_;
	double time_;
	std::vector<std::vector<double>> ma_; // (m) > Moment Arm: It is a vector of a vetor since it describes the moment arm of each muscle for every DOF
	std::vector<double> muscleForces_; // (N) > Muscle-tendon Forces
	std::vector<double> Emgs_; // Post-processed Normalized EMG [0,1]
	// Note: The order of the vector is the same of the vector "muscleName_"

	// Buffers: In case they are needed in the future
	std::list<double> timeBuffer_;
	std::list<std::vector<double>> EmgsBuffer_;
	std::list<std::vector<double>> externalDOFTorqueBuffer_;
	std::list<std::vector<double>> lmtBuffer_;
	std::list<std::vector<std::vector<double>>> maBuffer_;
	bool bufferEmpty_;
	double totalTime_;

	// EtherCAT variables
	bool sendTorque_;
	double timeStamp_;
	tcAdsClient* client_;
	std::vector<unsigned long> varNameVect_;
	double timeStampEthercat_;
	std::map<std::string, double> dataTorque_;
	std::map<std::string, double> dataTorqueEthercat_;

	enum VarName {
		//TorqueControlRightAnkle,
		//TorqueControlLeftAnkle,
		//Time,
		AchOutput,
		AchInputs,
	};


	/*
		>> Personal Variables
	*/

	// Pagmo
	MusclePagmoProblem<NMSmodelT> UDP_R_; // User-defined Problem for the right leg
	MusclePagmoProblem<NMSmodelT> UDP_L_; // User-defined Problem for the left leg
	double lowerBoundOpt_; // To set the min boundary for the optimization
	double upperBoundOpt_; // To set the Max boundary for the optimization
	vector<double> lowerBounds_R, upperBounds_R; // Boundaries for the right foot
	vector<double> lowerBounds_L, upperBounds_L; // Boundaries for the right foot
	vector<vector<double>> setOfSR_R; // For each generation
	vector<vector<double>> setOfEMGs_R; // For each generation
	vector<vector<double>> setOfSR_L; // For each generation
	vector<vector<double>> setOfEMGs_L; // For each generation
	int generationR_; // number of the generation
	int generationL_; // number of the generation
	std::vector<double> xR; // Champion for the right leg
	std::vector<double> xL; // Champion for the left leg
	std::vector<double> best_fR; // Best f(SR) of the right leg
	std::vector<double> best_fL; // Best f(SR) of the left leg
	SrAndEmg SrAndEmg_R_; // variable for passing data
	condition_variable cv_R; // Conditional variable
	SrAndEmg SrAndEmg_L_; // variable for passing data
	condition_variable cv_L; // Conditional variable
	unsigned int numGen_; // Number of generations for CMAES
	unsigned int numPop_; // Number of candidates per generation


	// Muscles Indexes for the NMS model, Pagmo, etc
	std::vector<unsigned> musclesListIndex_; // Index of the used muscles
	std::vector<unsigned> musclesListIndexR_; // Index of the muscles of the Right Foot
	std::vector<unsigned> musclesListIndexL_; // Index of the muscles of the Left Foot
	std::vector<unsigned> musclesToOptimizeIndex_;
	std::vector<unsigned> musclesToOptimizeIndexR_;
	std::vector<unsigned> musclesToOptimizeIndexL_;
	std::vector<unsigned> musclesNonOptimizedIndex_;
	std::vector<unsigned> musclesNonOptimizedIndexR_;
	std::vector<unsigned> musclesNonOptimizedIndexL_;

	// Ground-reaction forces
	vector<double> GRFz_; // (N) > Ground Reaction Forces: 2D Vector with GRF or perecentage of the gait cycle for each leg [Rleg, Lleg]

	// For validating new EMG
	bool newEMG_;
	vector<double> oldEMG_;

	// Parameters from the XML file 
	double thereshold_; // (N) > for getting the step event
	int refCount_; // Number of steps to compute the mean of the EMG Ref
	double cOpt_; // Objective function's wheight of the Optimized Muscle
	double cNOpt_; // Objective function's wheight of the Non-Optimized Muscles
	double r_; // Objective function's Reduction Factor
	int optimizationCount_; // Number of steps to optimization

	// For the torque computation
	vector<double> SR_; // Support Ratio
	vector<double> SR_b; // Baseline of the Support Ratio for the optimization
	double SR_static;   // If the optimization is set to false and we need to use a Static Assistance 
						// to look at the response of the muscles to an inpulse over time
	std::vector<double> newMuscleForces_; // MTForces multiplied by SR

	// To control the loops
	boolean loopTorque_; // To control the Torque computation loop
	boolean loopOptimizzation_; // To control the Optimization loop

	// Step event
	boolean stepEvent_R; // To look for step events
	boolean stepEvent_L;
	string stepEvent_foot_R; // Step validation Right
	string stepEvent_foot_L; // Step validation Left
	boolean stepValidationR_; // To observe it the last GRFz was under the thereshold
	boolean stepValidationL_;
	int stepCountR; // Total step count
	int stepCountL;
	int stepCountOptimizationR_; // Step count for the control of the optimization SR evaluation
	int stepCountOptimizationL_;

	// To compute the EMG mean, SD
	string optimizationCriterion_;
	vector<vector<double>> loopEMG_;
	/* List of EMG lists for each loop and each muscle that will be constantly appended:
	[[EMG1Muscle1, EMG2Muscle1, ..., EMGnMuscle1], ..., [EMG1MuscleM, EMG2MuscleM, ..., EMGnMuscleM]]
	>> Note: The order of this vector is the same of the muscleName_ vector, however, only the index of
	the muscles indicated in the XML file will be populated to minimize computation time*/
	vector<double> meanEMG_; // Mean of the EMG of each loop
	vector<double> refEMGmean_; // Mean of the fisrts 20 EMG means to use as reference in the cost function
	vector<double> refEMGsd_; // Standard Deviation of the fisrts 20 EMG means to use as reference in the cost function
	vector<vector<double>> firstsEMGmean_; // List of all the fisrts 20 EMG means to compute refEMGmean and refEMGsd > Same order as muscleName_
	bool CleanEMG_; // To state if we want to clean the EMG vector in order to count the EMG of just the lasts steps of each loop  
	int cleanEMGcount_; // Number of steps to clean the EMG


	// Loggers
	//std::ofstream Torque; // Optimized torque
	std::ofstream EMGRefLogger_R; // EMG mean, EMG SD
	std::ofstream SRLogger_R; // Time, Step No., SR
	std::ofstream EMGLogger_R; // Time, Step No., EMG mean, fitness
	std::ofstream EMGRefLogger_L; // EMG mean, EMG SD
	std::ofstream SRLogger_L; // Time, Step No., SR
	std::ofstream EMGLogger_L; // Time, Step No., EMG mean, fitness

};

#endif