// Muscle-in-the-loop Optimization
// Tiago Rodrigues, Guillaume Durandau, 2021
// tiagoepr@ua.pt | t.e.pereirarodrigues@student.utwente.nl

#include "OptimizationMusclePlugin.h"

template <typename NMSmodelT>
OptimizationMusclePlugin<NMSmodelT>::OptimizationMusclePlugin() :
	threadStop_(true), // Validator set by CEINMS-RT
	record_(false),  // Validator set by the user to CEINMS-RT, that also sets in the plugin 
	newData_(false), // Validator set by CEINMS-RT
	optimizationBool_(true), // To set if we want to use the optimization, If false it uses the SR_static for muscle specific static assistance
	SR_static(0.3), // double - To look at the response of the muscles to an inpulse over time
	lowerBoundOpt_(-0.15), // double - To set the min boundary for the optimization
	upperBoundOpt_(0.4), // double - To set the max boundary for the optimization
	sendTorque_(false), // Set to true to send torque to Exos and use the gait cycle percentage from TwinCAT
	numGen_(2u), // uint - Number of generations
	numPop_(5u), // uint - Number of candidates per generation > Minimum of 6 candidates!!
	CleanEMG_(true), // To state if we want to clean the EMG vector in order to count just the EMG of the lasts steps of each loop so that the muscle can adapt to the given assistance
	cleanEMGcount_(1) // int - Number of the firsts steps which EMG should be cleaned
{}

template <typename NMSmodelT>
OptimizationMusclePlugin<NMSmodelT>::~OptimizationMusclePlugin() {
}

// >> init function: NMS Model setup and definition, UDP
template <typename NMSmodelT>
void OptimizationMusclePlugin<NMSmodelT>::init(NMSmodelT& model, const std::string& executionXMLFileName, const std::string& configurationFileName) {

	std::cout << std::endl;
	std::cout << "	>> MUSCLE-IN-THE-LOOP OPTIMIZATION" << std::endl << std::flush;
	std::cout << "	>> T. Rodrigues, G. Durandau, 2021" << std::endl << std::flush;
	std::cout << std::endl;


	// >> Setup the NMS model core class
	model_ = new NMSmodelT();
	// >> Setup the subject's muscles, DOFs, and calibrated parameters from file
	setupSubject(*model_, configurationFileName);

	model_->getMuscleNames(muscleName_); // Get the muscles names from the model
	/*std::cout << "muscleName_: " << std::endl;
	for (auto it : muscleName_) {
		std::cout << it << " ";
	}
	std::cout << std::endl;*/
	// Output: med_gas_r lat_gas_r soleus_r tib_ant_r per_brev_r per_long_r per_tert_r med_gas_l lat_gas_l soleus_l tib_ant_l per_brev_l per_long_l per_tert_l


	// >> Load XML files reader classes to obtain the parameters that will be used in the plugin
	ExecutionXmlReader execution(executionXMLFileName); // For getting the parameters from the executionRT.xml file (namely the optimization file path)
	ExecutionOptimizationXmlReader executionOptimization(execution.getOptimizationFile()); // For getting the paramenters from the optimization file path indicated in the executioRT.xml file
	// - executionOptimization is now the class that has the muscle in the loop optimization parameters/variables


	/*
		>> Define plugin variables
	*/

	// Define Optimization Criterion
	optimizationCriterion_ = executionOptimization.getOptimizationCriterion();

	// >> Get the DOFs from the XML file and print
	dofName_ = executionOptimization.getDofsList();
	/*std::cout << "dofName_: " << std::endl;
	for (auto it : dofName_)
	{
		std::cout << it << " ";
	}
	std::cout << std::endl;
	std::cout << std::endl;*/
	// Outupt: ankle_angle_l ankle_angle_r

	// >> Erase unused DOFs from the NMS model for faster computation
	model_->eraseUnusedDofs(dofName_, dofsUsed_); // Defined in NMSmodel.cpp
	/* // Check if the dofs are indeed erased:
	model_->getDoFNames(dofName_);
	std::cout << "Model_ dofName: " << std::endl;
	for (auto it : dofName_)
	{
		std::cout << it << std::endl;
	}
	std::cout << "Model_ muscleName: " << std::endl;
	for (auto it : muscleName_)
	{
		std::cout << it << std::endl;
	}*/

	// >> Get the index of the muscles on the NMS model list
	model_->getMusclesIndexFromMusclesList(musclesListIndex_, executionOptimization.getMusclesList());
	/*std::cout << "Muscles List_: " << std::endl;
	for (auto it : musclesListIndex_)
	{
		std::cout << it << " ";
	}
	std::cout << std::endl;
	std::cout << std::endl;*/
	// Outupt: 7 8 9 10 0 1 2 3


	model_->getMusclesIndexFromMusclesList(musclesToOptimizeIndex_, executionOptimization.getMusclesToOptimize());
	/*std::cout << "Muscles To Optimize: " << std::endl;
	for (auto it : musclesToOptimizeIndex_)
	{
		std::cout << it << " ";
	}
	std::cout << std::endl;
	std::cout << std::endl;*/
	// Output: 9 2

	// Index of the muscles for each foot for optimization and torque computation
	for (int i = 0; i < musclesListIndex_.size(); i++) {
		int index = musclesListIndex_[i];
		if (muscleName_[index].compare(muscleName_[index].size() - 2, 2, "_r") == 0) {
			musclesListIndexR_.push_back(index);

			// For the optimization muscles
			if (find(musclesToOptimizeIndex_.begin(), musclesToOptimizeIndex_.end(), index) != musclesToOptimizeIndex_.end()) {
				// if it finds the index on the list of muscles'index to be optimized 
				musclesToOptimizeIndexR_.push_back(index);
			}
			else {
				musclesNonOptimizedIndexR_.push_back(index);
			}
		}
		else if (muscleName_[index].compare(muscleName_[index].size() - 2, 2, "_l") == 0) {
			musclesListIndexL_.push_back(index);
			if (find(musclesToOptimizeIndex_.begin(), musclesToOptimizeIndex_.end(), index) != musclesToOptimizeIndex_.end()) {
				// if it finds the index on the list of muscles'index to be optimized 
				musclesToOptimizeIndexL_.push_back(index);
			}
			else {
				musclesNonOptimizedIndexL_.push_back(index);
			}
		}
	}

	newMuscleForces_.resize(muscleName_.size()); // MTForces multiplied by SR

	// Contruction of the SR and SR baseline
	SR_.resize(muscleName_.size()); // Will be all zeros for computing the reference EMG
	SR_b.resize(muscleName_.size());
	for (int i = 0; i < musclesToOptimizeIndex_.size(); i++) {
		int it = musclesToOptimizeIndex_[i];
		SR_b.at(it) = SR_static;
	}


	// Print initial table:
	std::cout << "Support Ratio (Initial Guess): " << std::endl;
	const char separator = ' ';
	const int nameWidth = 10;
	std::cout << right << setw(nameWidth) << "Muscle:  ";
	for (int i = 0; i < muscleName_.size(); i++) {
		cout << left << setw(nameWidth) << setfill(separator) << muscleName_[i];
		cout << " | ";
	}
	std::cout << std::endl;
	std::cout << right << setw(nameWidth) << "SR:  ";
	for (int i = 0; i < muscleName_.size(); i++) {
		cout << left << setw(nameWidth) << setfill(separator) << SR_[i];
		cout << " | ";
	}
	std::cout << std::endl;
	std::cout << std::endl;


	// >> Optimization Variables
	stepEvent_R = false;
	stepEvent_L = false;
	stepEvent_foot_R = 'NaN';
	stepEvent_foot_L = 'NaN';
	refCount_ = executionOptimization.getnCyclesRef(); // Number of steps to compute the mean of the EMG Ref
	cOpt_ = executionOptimization.getWheightOptimizedMuscles(); // Objective function's wheight of the Optimized Muscles
	cNOpt_ = executionOptimization.getWheightNonOptimizedMuscles(); // Objective function's wheight of the Non-Optimized Muscles
	r_ = executionOptimization.getReductionFactor(); // Objective function's Reduction Factor
	optimizationCount_ = executionOptimization.getnCyclesOptimization(); // Number of steps to optimization
	numPop_ = numPop_ - 1u; // Because 0 means using 1 candidate

	newEMG_ = false; //Validating if there is new EMG
	stepValidationR_ = false;
	stepValidationL_ = false;
	stepCountR = 0;
	stepCountL = 0;
	stepCountOptimizationR_ = 0;
	stepCountOptimizationL_ = 0;

	// Resize vectors so we can access and edit its elements
	oldEMG_.resize(muscleName_.size());
	loopEMG_.resize(muscleName_.size());
	GRFz_.resize(dofsUsed_.size());
	meanEMG_.resize(muscleName_.size());
	firstsEMGmean_.resize(muscleName_.size());
	refEMGmean_.resize(muscleName_.size());
	refEMGsd_.resize(muscleName_.size());
	lowerBounds_R.resize(muscleName_.size());
	upperBounds_R.resize(muscleName_.size());
	lowerBounds_L.resize(muscleName_.size());
	upperBounds_L.resize(muscleName_.size());

	// Set Initial Bounds for the SR
	double range = 1; //so the support ratio is between [-1, 1] for each muscle
	// Muscles to optimize on the Right leg
	for (unsigned i = 0; i < musclesListIndexR_.size(); ++i) {
		int it = musclesListIndexR_[i];
		lowerBounds_R.at(it) = lowerBoundOpt_;//SR_b.at(it) - range;
		upperBounds_R.at(it) = upperBoundOpt_;//SR_b.at(it) + range;
	} // The rest will be zero (It will save computation time)
	// Muscles to optimize on the Left leg
	for (unsigned i = 0; i < musclesListIndexL_.size(); ++i) {
		int it = musclesListIndexL_[i];
		lowerBounds_L.at(it) = lowerBoundOpt_;//SR_b.at(it) - range;
		upperBounds_L.at(it) = upperBoundOpt_;//SR_b.at(it) + range;
	}// The rest will be zero (It will save computation time)

	std::cout << right << setw(nameWidth) << "R min:  ";
	for (int i = 0; i < muscleName_.size(); i++) {
		cout << left << setw(nameWidth) << setfill(separator) << lowerBounds_R[i];
		cout << " | ";
	}
	std::cout << std::endl;
	std::cout << right << setw(nameWidth) << "R Max:  ";
	for (int i = 0; i < muscleName_.size(); i++) {
		cout << left << setw(nameWidth) << setfill(separator) << upperBounds_R[i];
		cout << " | ";
	}
	std::cout << std::endl;
	std::cout << right << setw(nameWidth) << "L min:  ";
	for (int i = 0; i < muscleName_.size(); i++) {
		cout << left << setw(nameWidth) << setfill(separator) << lowerBounds_L[i];
		cout << " | ";
	}
	std::cout << std::endl;
	std::cout << right << setw(nameWidth) << "L Max:  ";
	for (int i = 0; i < muscleName_.size(); i++) {
		cout << left << setw(nameWidth) << setfill(separator) << upperBounds_L[i];
		cout << " | ";
	}
	std::cout << std::endl;
	std::cout << std::endl;


	// >> Define Pagmo's UDP > init set functions
	// Right Leg
	UDP_R_.setWeights(cOpt_, cNOpt_);
	UDP_R_.setReductionFactor(r_);
	UDP_R_.setMusclesIndex(musclesToOptimizeIndexR_, musclesNonOptimizedIndexR_);
	UDP_R_.set_bounds(lowerBounds_R, upperBounds_R);
	UDP_R_.setSrAndEmg(&SrAndEmg_R_); // set the pointer of the SrAndEmg object
	UDP_R_.setcv(&cv_R); // Set the pointer of CV
	// Left Leg
	UDP_L_.setWeights(cOpt_, cNOpt_);
	UDP_L_.setReductionFactor(r_);
	UDP_L_.setMusclesIndex(musclesToOptimizeIndexL_, musclesNonOptimizedIndexL_);
	UDP_L_.set_bounds(lowerBounds_L, upperBounds_L);
	UDP_L_.setSrAndEmg(&SrAndEmg_L_); // set the pointer of the SrAndEmg object
	UDP_L_.setcv(&cv_L); // Set the pointer of CV


	// >> Write the data into their output files 
	if (record_) {
		std::string directoriOptimization = outDirectory_ + "/MuscleInTheLoopOptimization/";

		// Torque loop > CEINMS-RT already logs the values
		logger_ = new OpenSimFileLogger<NMSmodelT>(*model_, directoriOptimization);
		logger_->addLog(Logger::Torques, dofName_);

		// Optimization loop
		loggersInit(directoriOptimization);
	}

	if (sendTorque_) {
		client_ = new tcAdsClient(350); // Port hard coded for now

		// var index as conventioned in CEINMS-RT
		//std::string var5 = "AdsTalker_Obj1 (CAdsTalker).Outputs.TorqueCommandRightAnkle";
		//std::string var6 = "AdsTalker_Obj1 (CAdsTalker).Outputs.TorqueCommandLeftAnkle";
		//std::string var8 = "MAIN.Time1";
		std::string var9 = "AdsTalker_Obj1 (CAdsTalker).Inputs.AchOutput";
		std::string var10 = "AdsTalker_Obj1 (CAdsTalker).Outputs.AchInputs";

		varNameVect_.resize(2);
		//varNameVect_[VarName::TorqueControlRightAnkle] = client_->getVariableHandle(&var5[0], var5.size());
		//varNameVect_[VarName::TorqueControlLeftAnkle] = client_->getVariableHandle(&var6[0], var6.size());
		//varNameVect_[VarName::Time] = client_->getVariableHandle(&var8[0], var8.size());
		varNameVect_[VarName::AchOutput] = client_->getVariableHandle(&var9[0], var9.size());
		varNameVect_[VarName::AchInputs] = client_->getVariableHandle(&var10[0], var10.size());

		if (varNameVect_[VarName::AchOutput] == -1 || varNameVect_[VarName::AchInputs] == -1)
			exit(-1);

		timeStamp_ = time_;
	}

}



// >> Setup the subject's muscles, DOFs, and calibrated parameters from the configuration file
template <typename NMSmodelT>
void OptimizationMusclePlugin<NMSmodelT>::setupSubject(NMSmodelT& mySubject, string configurationFile) {
	SetupDataStructure<NMSmodelT, Curve<CurveMode::Online> > setupData(configurationFile);
	setupData.createCurves();
	setupData.createMuscles(mySubject);
	setupData.createDoFs(mySubject);
	setupData.createMusclesNamesOnChannel(mySubject);
}


// >> Public Get and Set functions needed for setting the NMS model 
template <typename NMSmodelT>
std::vector<double> OptimizationMusclePlugin<NMSmodelT>::getDofTorque() {
	boost::mutex::scoped_lock lock(DataMutex_);
	return dofTorque_;
}

template <typename NMSmodelT>
std::vector<double> OptimizationMusclePlugin<NMSmodelT>::getShapeFactor() {
	boost::mutex::scoped_lock lock(DataMutex_);
	return shapeFactor_;
}

template <typename NMSmodelT>
std::vector<double> OptimizationMusclePlugin<NMSmodelT>::getTendonSlackLengths() {
	boost::mutex::scoped_lock lock(DataMutex_);
	return tendonSlackLengths_;
}

template <typename NMSmodelT>
std::vector<double> OptimizationMusclePlugin<NMSmodelT>::getOptimalFiberLengths() {
	boost::mutex::scoped_lock lock(DataMutex_);
	return optimalFiberLengths_;
}

template <typename NMSmodelT>
std::vector<double> OptimizationMusclePlugin<NMSmodelT>::getGroupMusclesBasedOnStrengthCoefficients() {
	boost::mutex::scoped_lock lock(DataMutex_);
	return groupMusclesBasedOnStrengthCoefficients_;
}

template <typename NMSmodelT>
void OptimizationMusclePlugin<NMSmodelT>::setMuscleForce(const std::vector<double>& muscleForces) {
	boost::mutex::scoped_lock lock(DataMutex_);
	muscleForces_ = muscleForces;

}

template <typename NMSmodelT>
void OptimizationMusclePlugin<NMSmodelT>::setDOFTorque(const std::vector<double>& dofTorque) {
	boost::mutex::scoped_lock lock(DataMutex_);
	dofTorque_ = dofTorque;
	//newData_ = true;
}

template <typename NMSmodelT>
void OptimizationMusclePlugin<NMSmodelT>::setExternalDOFTorque(const std::vector<double>& externalDOFTorque) {
	boost::mutex::scoped_lock lock(DataMutex_);
	externalDOFTorque_ = externalDOFTorque; // NORMAL
	//externalDOFTorqueBuffer_.push_back(externalDOFTorque);

	//std::cout << "setExternalDOFTorque: " << externalDOFTorque.at(3) << std::endl;
	//std::cout << externalDOFTorque.size() << std::endl;
}

template <typename NMSmodelT>
void OptimizationMusclePlugin<NMSmodelT>::setActivations(const std::vector<double>& activations) {
	boost::mutex::scoped_lock lock(DataMutex_);
	activations_ = activations;
}

template <typename NMSmodelT>
void OptimizationMusclePlugin<NMSmodelT>::setFibreLengths(const std::vector<double>& fibreLengths) {
	boost::mutex::scoped_lock lock(DataMutex_);
	fibreLengths_ = fibreLengths;
}

template <typename NMSmodelT>
void OptimizationMusclePlugin<NMSmodelT>::setFibreVelocities(const std::vector<double>& fibreVelocities) {
	boost::mutex::scoped_lock lock(DataMutex_);
	fibreVelocities_ = fibreVelocities;
}

template <typename NMSmodelT>
void OptimizationMusclePlugin<NMSmodelT>::setPennationAngle(const std::vector<double>& pennationAngle) {
	boost::mutex::scoped_lock lock(DataMutex_);
	pennationAngle_ = pennationAngle;
}

template <typename NMSmodelT>
void OptimizationMusclePlugin<NMSmodelT>::setTendonLength(const std::vector<double>& tendonLength) {
	boost::mutex::scoped_lock lock(DataMutex_);
	tendonLength_ = tendonLength;
}

template <typename NMSmodelT>
void OptimizationMusclePlugin<NMSmodelT>::setTime(const double& time) {
	boost::mutex::scoped_lock lock(DataMutex_);
	time_ = time;	// NORMAL
	//timeBuffer_.push_back(time);
	//std::cout << "setTime: " << time << std::endl;
}

template <typename NMSmodelT>
void OptimizationMusclePlugin<NMSmodelT>::setEmgs(const std::vector<double>& Emgs) {
	boost::mutex::scoped_lock lock(DataMutex_);
	Emgs_ = Emgs; // NORMAL
	//EmgsBuffer_.push_back(Emgs);
}

template <typename NMSmodelT>
bool OptimizationMusclePlugin<NMSmodelT>::getFromBuffer(double& time, std::vector<double>& Emgs, std::vector<double>& externalDOFTorque, std::vector<double>& lmt, std::vector<std::vector<double> >& ma) {
	boost::mutex::scoped_lock lock(DataMutex_);
	time = timeBuffer_.front();
	timeBuffer_.pop_front();
	Emgs = EmgsBuffer_.front();
	EmgsBuffer_.pop_front();
	externalDOFTorque = externalDOFTorqueBuffer_.front();
	externalDOFTorqueBuffer_.pop_front();
	lmt = lmtBuffer_.front();
	lmtBuffer_.pop_front();
	ma = maBuffer_.front();
	maBuffer_.pop_front();

	/*std::cout << "getTime: " << time << std::endl;
	std::cout << "getExternalDOFTorque: " << externalDOFTorque.at(3) << std::endl;

	std::cout << timeBuffer_.size() << std::endl;
	std::cout << externalDOFTorqueBuffer_.size() << std::endl;*/

	return EmgsBuffer_.empty();
}

template <typename NMSmodelT>
void OptimizationMusclePlugin<NMSmodelT>::setLmt(const std::vector<double>& lmt) {
	boost::mutex::scoped_lock lock(DataMutex_);
	lmt_ = lmt; // NORMAL
	//lmtBuffer_.push_back(lmt);
}

template <typename NMSmodelT>
void OptimizationMusclePlugin<NMSmodelT>::setMA(const std::vector<std::vector<double> >& ma) {
	boost::mutex::scoped_lock lock(DataMutex_);
	ma_ = ma; // NORMAL
	//maBuffer_.push_back(ma);
}


template <typename NMSmodelT>
void OptimizationMusclePlugin<NMSmodelT>::setDirectories(const std::string& outDirectory, const std::string& inDirectory) {
	outDirectory_ = outDirectory;
}

template <typename NMSmodelT>
void OptimizationMusclePlugin<NMSmodelT>::setVerbose(const int& verbose) {
}

template <typename NMSmodelT>
void OptimizationMusclePlugin<NMSmodelT>::setRecord(const bool& record) {
	record_ = record;
}


template <typename NMSmodelT>
void OptimizationMusclePlugin<NMSmodelT>::newData() {
	boost::mutex::scoped_lock lock(newDataMutex_);
	newData_ = true;
	conditionNewData_.notify_one();
}


template <typename NMSmodelT>
void OptimizationMusclePlugin<NMSmodelT>::setSR(vector<double> sr) {
	boost::mutex::scoped_lock lock(DataMutex_);
	SR_ = sr;
}




/*

				++++-----------------------------------------------------------------------------++++
				+++-------------------------------------------------------------------------------+++
				+++-------------------------------------------------------------------------------+++
				+++-------------------------------------------------------------------------------+++
				+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
				+++-------------------------------------------------------------------------------+++
				+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
				+++-------------------------------------------------------------------------------+++
				+++-------------------------------------------------------------------------------+++
				++++++++++++++++++++++++++++++++++ Tiago Rodrigues ++++++++++++++++++++++++++++++++++
				++++++++++++++++++++++++++++++++ Guillaume Durandau +++++++++++++++++++++++++++++++++
				+++++++++++++++++++++++++++++++++++++++ 2021 ++++++++++++++++++++++++++++++++++++++++
				+++-------------------------------------------------------------------------------+++
				+++-------------------------------------------------------------------------------+++
				+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
				+++-------------------------------------------------------------------------------+++
				+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
				+++-------------------------------------------------------------------------------+++
				+++-------------------------------------------------------------------------------+++
				+++-------------------------------------------------------------------------------+++
				++++-----------------------------------------------------------------------------++++

*/



/*
	>> Start the Plugin
*/
template <typename NMSmodelT>
void OptimizationMusclePlugin<NMSmodelT>::start() {
	// Create Torque threads
	threadTorque_ = new boost::thread(boost::bind(&OptimizationMusclePlugin<NMSmodelT>::torqueLoop, this));
	// Create Optimization thread
	threadOptimization_ = new boost::thread(boost::bind(&OptimizationMusclePlugin<NMSmodelT>::optimizationLoop, this));
	// Create Optimization thread for right foot
	threadAlgorithmR_ = new boost::thread(boost::bind(&OptimizationMusclePlugin<NMSmodelT>::algorithmLoopR, this));
	// Create Optimization thread for left foot
	threadAlgorithmL_ = new boost::thread(boost::bind(&OptimizationMusclePlugin<NMSmodelT>::algorithmLoopL, this));
}


/*
	>> Stop the Plugin
*/
template <typename NMSmodelT>
void OptimizationMusclePlugin<NMSmodelT>::stop() {
	threadStop_ = false;
	algorithmStartR_ = false;
	algorithmStartL_ = false;

	{
		boost::mutex::scoped_lock lock(newDataMutex_);
		newData_ = true;
		conditionNewData_.notify_one();
	}

	// Join Threads
	threadTorque_->join();
	threadOptimization_->join();
	threadAlgorithmR_->join();
	threadAlgorithmL_->join();

	// stop the loggers
	if (record_) {
		logger_->stop();
		delete logger_;

		EMGRefLogger_R.close();
		SRLogger_R.close();
		EMGLogger_R.close();
		EMGRefLogger_L.close();
		SRLogger_L.close();
		EMGLogger_L.close();
	}


	delete threadAlgorithmR_;
	delete threadAlgorithmL_;
	delete threadOptimization_;
	delete threadTorque_;
	delete model_;
}


/*
	>> Loop for computing the assistive torque in real time
*/
template <typename NMSmodelT>
void OptimizationMusclePlugin<NMSmodelT>::torqueLoop() {
	while (threadStop_) {

		// Wait until there is new data
		{
			boost::mutex::scoped_lock lock(newDataMutex_);
			while (!newData_) conditionNewData_.wait(lock);
			newData_ = false; // NORMAL
		}

		// >> Set the NMS model
		// Not necessairy since it will only encrease computation time
		/*DataMutex_.lock();
		model_->setTime(time_);
		model_->setEmgs(Emgs_);

		model_->setMuscleTendonLengths(lmt_);
		std::cout << "ok" << std::endl;
		unsigned int j = 0;
		std::vector<vector<double>> ma_log; // for logging the moment arms
		for (auto it = dofsUsed_.begin(); it != dofsUsed_.end(); ++it) // set moment arms for the DOFs that are optimized
		{
			//UDP_.setMomentArms(ma_.at(*it), j);
			model_->setMomentArms(ma_.at(*it), j);
			j++;
			ma_log.push_back(ma_.at(*it));
			//std::cout << "MA *it:" << *it << "\n";
			//for (unsigned int i_ma = 0; i_ma < ma_.at(*it).size(); ++i_ma)
			//	std::cout << ma_.at(*it).at(i_ma) << " ";
		}

		DataMutex_.unlock();*/

		if (muscleForces_.size() > 0) {
			// Compute the assistive torque
			vector<double> assistiveTorque = computeTorque();

			if (record_) {
				DataMutex_.lock();
				dofTorque_ = assistiveTorque;
				logger_->log(Logger::Torques, time_);
				DataMutex_.unlock();
			}

			// Send Assistive Torque to Exoskeleton
			if (sendTorque_) { // If using TwinCAT
				sendTorque(assistiveTorque);
			}
		}
	}
}


/*
	>> Loop for the optimization each step
*/
template <typename NMSmodelT>
void OptimizationMusclePlugin<NMSmodelT>::optimizationLoop() {

	// Define lock with the mutex of the SrAndEmg objects 
	std::unique_lock<std::mutex> lk_R(*SrAndEmg_R_.getMutex(), std::defer_lock);
	std::unique_lock<std::mutex> lk_L(*SrAndEmg_L_.getMutex(), std::defer_lock);
	// mutex is not locked, only defined > std::defer_lock makes the lock not acquiring ownership of the mutex
	// If defined inside the while statement, it would get outof scope and unlock by itself  

	while (threadStop_) {

		// Validate if there is new EMG data
		newEMGvalidator();

		// If there is new EMG data received from CEINMS
		if (newEMG_) {

			// Populate the EMG vector to compute the EMG mean of each gait cycle
			// Note: If "MTForces" are defined in the XML file, Muscle forces will be used instead of EMG
			populateEMG();

			// Get gait cycle event
			getGRFEvent(); // To see if a step from the R or L foot was observed 

			// If there is a step event:
			if (stepEvent_R || stepEvent_L) {

				// >> Right Foot Before Optimization
				if (stepEvent_foot_R == "R" && !optimizationDone_R_) { // Right foot event
					stepCountR += 1;
					//std::cout << "Right Step No: " << stepCountR << std::endl;

					// 1) First steps to get EMG reference
					if (stepCountR <= refCount_) {

						// Compute EMG Mean and clear loopEMG_ of the right leg
						computeEMGmean(musclesListIndexR_);

						std::cout << "Right Step No: " << stepCountR << std::endl;
						addRefEMGmean(musclesListIndexR_); // Add EMG mean to list

						// If we have the last EMG (EMG from the last step) to compute the Ref EMG
						if (stepCountR == refCount_) {
							std::cout << std::endl;
							std::cout << "Reference EMG - Right Foot: " << std::endl;
							// If stepCountR == refCount_ we can now compute the EMG Ref mean and SD for each leg
							computeRefEMGmean(musclesListIndexR_);
							UDP_R_.setRefDecisionVector(refEMGmean_);

							if (record_) {
								loggersRef("R");
							}
						}

						// Log values for the Reference Type
						if (record_) {
							loggers("R", -1.0, "R");
						}
					}

					// 2) Optimization after computing the reference
					else {
						if (optimizationBool_) {
							// Start Optimization
							if (stepCountR == refCount_ + 1) {

								// Compute EMG Mean and clear loopEMG_ of the right leg
								computeEMGmean(musclesListIndexR_);

								// Release the lock of the mutex so the other thread can access the data
								// Initialize the algorithm loop
								algorithmStartR_ = true;

								// Log values for the Reference Type
								if (record_) {
									loggers("R", -1.0, "R");
								}
							}
							else {
								// Clean the EMG vector in case the CleanEMG_ is set to true and the step count for optimization is the same as the stated
								if (CleanEMG_ && stepCountOptimizationR_ == cleanEMGcount_) {
									// Muscles need to adapt to the updated SR and the firsts EMG may lead to misleading results 
									// since they are not mirroring the real muscle effort corresponding to that SR
									// This is just a easy way to solve the problem, however, 
									// we are still appending EMG to a vector, filling memory, when it is not realy need
									for (int i = 0; i < musclesListIndexR_.size(); i++) {
										int mIndex_ = musclesListIndexR_[i];
										// clear loopEMG_ of each muscle
										loopEMG_[mIndex_].clear();
									}
								}

								// If we already have the last EMG from the last step of the evaluation
								if (stepCountOptimizationR_ == optimizationCount_) {

									// Evaluate global EMG mean:
									// Compute EMG Mean and clear loopEMG_ of the right leg
									computeEMGmean(musclesListIndexR_);

									// we already have the meanEMG from the old SR, so we can now set it to the object
									SrAndEmg_R_.setEMG(meanEMG_);
									SrAndEmg_R_.setEMGvalidator(true);

									// Print EMG results of the last SR
									//std::cout << right << setw(90) << ">> ";
									//cout << "5) Optimization Thread: Mean EMG computed and set to the object" << endl;
									/*const char separator = ' ';
									const int nameWidth = 10;
									std::cout << right << setw(nameWidth) << "EMG:  ";
									for (int i = 0; i < musclesListIndexR_.size(); i++) {
										int it = musclesListIndexR_[i];
										cout << left << setw(nameWidth) << setfill(separator) << meanEMG_[it];
										cout << " | ";
									}
									std::cout << std::endl;
									std::cout << std::endl;*/

									lk_R.unlock();
									cv_R.notify_one(); // Notify of the new EMG

									stepCountOptimizationR_ = 0;
								}
							}

							// If the last loop was finished, we can get the new SR candidate from the algorithm
							if (stepCountOptimizationR_ == 0) {

								// Wait until the algorithm sets the SR and the mutex is released from the other thread
								lk_R.lock();
								cv_R.wait(lk_R, [this] {
									if (optimizationDone_R_) {
										return optimizationDone_R_; // In case the optimization had been concluded
									}
									else {
										return SrAndEmg_R_.getSRvalidator();
									}
								});
								if (optimizationDone_R_) {
									lk_R.unlock(); // unlock the mutex in case the optimization have been finished
								}
								else {
									SrAndEmg_R_.setSRvalidator(false);
								}

								// If we already have the SR and the correspondent EMG
								if (stepCountR > refCount_ + 1) {
									// Get the f(EMG(SR)) of the previous SR
									double fp_ = SrAndEmg_R_.getF();
									/*std::cout << ">> fp(EMG(SR)) - Right Foot = " << fp_ << std::endl;
									std::cout << std::endl;
									std::cout << std::endl;*/

									// Log old values
									if (record_) {
										loggers("R", fp_, "O");
									}
								}

								if (!optimizationDone_R_) {
									// After the wait, we own the lock and we can get the new SR
									vector<double> copy_sr = SrAndEmg_R_.getSR();

									//std::cout << right << setw(90) << ">> ";
									//cout << "3) Optimization Thread: SR gotten from the object" << endl;

									// Now we can update the SR for the optimized muscles
									for (int i = 0; i < musclesListIndexR_.size(); i++) {
										int mIndex = musclesListIndexR_[i];
										SR_.at(mIndex) = copy_sr.at(mIndex);
									}

									//std::cout << right << setw(90) << ">> ";
									//cout << "4) Optimization Thread: SR updated" << endl;

									// Print results of the new SR
									/*std::cout << "Support Ratio (EMG) - step No.: " << stepCountR << std::endl;
									printSR(musclesListIndexR_);*/
								}
							}
							stepCountOptimizationR_ += 1;
						}
						else {
							// Set the SR baseline
							for (int i = 0; i < musclesListIndexR_.size(); i++) {
								int mIndex = musclesListIndexR_[i];
								SR_.at(mIndex) = SR_b.at(mIndex);
							}

							// 1) Compute EMG Mean and clear loopEMG_ of the right leg
							computeEMGmean(musclesListIndexR_);

							// 2) Print the step number
							// Print results of the new SR
							/*std::cout << "Support Ratio (EMG) - step No.: " << stepCountR << std::endl;
							printSR(musclesListIndexR_);
							const char separator = ' ';
							const int nameWidth = 15;
							std::cout << right << setw(nameWidth) << "EMG Reduc.:  ";
							for (int i = 0; i < musclesListIndexR_.size(); i++) {
								int it = musclesListIndexR_[i];
								cout << left << setw(nameWidth) << setfill(separator) << refEMGmean_[it] - meanEMG_[it];
								cout << " | ";
							}
							std::cout << std::endl;
							std::cout << std::endl;*/

							// 3) Record data
							if (record_) {
								double f = computeFitness(musclesToOptimizeIndexR_, musclesNonOptimizedIndexR_);
								loggers("R", f, "V");
							}
						}
					}
					stepEvent_R = false;
				}

				// >> Left Foot Before Optimization
				else if (stepEvent_foot_L == "L" && !optimizationDone_L_) {
					stepCountL += 1;

					// 1) First steps to get EMG reference
					if (stepCountL <= refCount_) {

						// Compute EMG Mean and clear loopEMG_ of the right leg
						computeEMGmean(musclesListIndexL_);

						std::cout << "Left Step No: " << stepCountL << std::endl;
						addRefEMGmean(musclesListIndexL_); // Add EMG mean to list
						if (stepCountL == refCount_) {
							std::cout << std::endl;
							std::cout << "Reference EMG - Left Foot: " << std::endl;
							// If stepCountR == refCount_ we can now compute the EMG Ref mean and SD for each leg
							computeRefEMGmean(musclesListIndexL_);
							UDP_L_.setRefDecisionVector(refEMGmean_);

							if (record_) {
								loggersRef("L");
							}
						}

						// Log values for the Reference Type
						if (record_) {
							loggers("L", -1.0, "R");
						}
					}

					// 2) Optimization after computing the reference
					else {
						if (optimizationBool_) {

							//std::cout << "Left Step No: " << stepCountL << std::endl;
							// Start Optimization
							if (stepCountL == refCount_ + 1) {

								// Compute EMG Mean and clear loopEMG_ of the right leg
								computeEMGmean(musclesListIndexL_);

								// Initialize the algorithm loop
								algorithmStartL_ = true;

								std::cout << " start Optimization Left" << std::endl;

								// Log values for the Reference Type
								if (record_) {
									loggers("L", -1.0, "R");
								}
							}
							else {
								// Clean the EMG vector in case the CleanEMG_ is set to true and the step count for optimization is the same as the stated
								if (CleanEMG_ && stepCountOptimizationL_ == cleanEMGcount_) {
									// Muscles need to adapt to the updated SR and the firsts EMG may lead to misleading results 
									// since they are not mirroring the real muscle effort corresponding to that SR
									// This is just a easy way to solve the problem, however, 
									// we are still appending EMG to a vector, filling memory, when it is not realy need
									for (int i = 0; i < musclesListIndexL_.size(); i++) {
										int mIndex_ = musclesListIndexL_[i];
										// clear loopEMG_ of each muscle
										loopEMG_[mIndex_].clear();
									}
									cout << "EMG left cleared. Optimization Step: " << stepCountOptimizationL_ << endl;
								}

								// If we already have the last EMG from the last step of the evaluation
								if (stepCountOptimizationL_ == optimizationCount_) {

									// Evaluate global EMG mean:
									// Compute EMG Mean and clear loopEMG_ of the right leg
									computeEMGmean(musclesListIndexL_);

									// we already have the meanEMG from the old SR, so we can now set it to the object
									SrAndEmg_L_.setEMG(meanEMG_);
									SrAndEmg_L_.setEMGvalidator(true);

									const char separator = ' ';
									const int nameWidth = 15;
									std::cout << right << setw(nameWidth) << "EMG Reduc.:  ";
									for (int i = 0; i < musclesListIndexL_.size(); i++) {
										int it = musclesListIndexL_[i];
										cout << left << setw(nameWidth) << setfill(separator) << refEMGmean_[it] - meanEMG_[it];
										cout << " | ";
									}
									std::cout << std::endl;
									std::cout << std::endl;

									lk_L.unlock();
									cv_L.notify_one(); // Notify of the new EMG

									stepCountOptimizationL_ = 0;
								}
							}

							// If the last loop was finished, we can get the new SR candidate from the algorithm
							if (stepCountOptimizationL_ == 0) {

								// Wait until the algorithm sets the SR and the mutex is released from the other thread
								lk_L.lock();
								cv_L.wait(lk_L, [this] {
									if (optimizationDone_L_) {
										return optimizationDone_L_; // In case the optimization had been concluded
									}
									else {
										return SrAndEmg_L_.getSRvalidator();
									}
								});
								if (optimizationDone_L_) {
									lk_L.unlock(); // unlock the mutex in case the optimization have been finished
								}
								else {
									SrAndEmg_L_.setSRvalidator(false);
								}

								// If we already have the SR and the correspondent EMG
								if (stepCountL > refCount_ + 1) {
									// Get the f(EMG(SR)) of the previous SR
									double fp_ = SrAndEmg_L_.getF();
									std::cout << ">> fp(EMG(SR)) - Left Foot = " << fp_ << std::endl;
									std::cout << std::endl;
									std::cout << std::endl;

									// Log old values
									if (record_) {
										loggers("L", fp_, "O");
									}
								}

								if (!optimizationDone_L_) { // If the optimization haven't finished yet

									// After the wait, we own the lock and we can get the new SR
									vector<double> copy_sr = SrAndEmg_L_.getSR();

									// Now we can update the SR for the optimized muscles
									for (int i = 0; i < musclesListIndexL_.size(); i++) {
										int mIndex = musclesListIndexL_[i];
										SR_.at(mIndex) = copy_sr.at(mIndex);
									}

									// Print results of the new SR
									std::cout << "Support Ratio (EMG) - step No.: " << stepCountL << std::endl;
									printSR(musclesListIndexL_);
								}
							}
							stepCountOptimizationL_ += 1;
						}
						else {
							// Set the SR baseline
							for (int i = 0; i < musclesListIndexL_.size(); i++) {
								int mIndex = musclesListIndexL_[i];
								SR_.at(mIndex) = SR_b.at(mIndex);
							}

							// 1) Compute EMG Mean and clear loopEMG_ of the right leg
							computeEMGmean(musclesListIndexL_);

							// 2) Print the step number
							// Print results of the new SR
							std::cout << "Support Ratio (EMG) - step No.: " << stepCountL << std::endl;
							printSR(musclesListIndexL_);
							const char separator = ' ';
							const int nameWidth = 15;
							std::cout << right << setw(nameWidth) << "EMG Reduc.:  ";
							for (int i = 0; i < musclesListIndexL_.size(); i++) {
								int it = musclesListIndexL_[i];
								cout << left << setw(nameWidth) << setfill(separator) << refEMGmean_[it] - meanEMG_[it];
								cout << " | ";
							}
							std::cout << std::endl;
							std::cout << std::endl;


							// 3) Record data
							if (record_) {
								double f = computeFitness(musclesToOptimizeIndexL_, musclesNonOptimizedIndexL_);
								loggers("L", f, "V");
							}
						}
					}
					stepEvent_L = false;
				}

				// >> Right Foot After Optimization
				else if (stepEvent_foot_R == "R" && optimizationDone_R_) { // Right foot event
					stepCountR += 1;

					// 1) Compute EMG Mean and clear loopEMG_ of the right leg
					computeEMGmean(musclesListIndexR_);

					// 2) Print the step number
					std::cout << "Right Step No: " << stepCountR << std::endl;

					// 3) Record data
					if (record_) {
						double f = computeFitness(musclesToOptimizeIndexR_, musclesNonOptimizedIndexR_);
						loggers("R", f, "V");
					}

					stepEvent_R = false;
				}

				// >> Right Foot After Optimization
				else if (stepEvent_foot_L == "L" && optimizationDone_L_) { // Right foot event
					stepCountL += 1;

					// 1) Compute EMG Mean and clear loopEMG_ of the right leg
					computeEMGmean(musclesListIndexL_);

					// 2) Print the step number
					std::cout << "Left Step No: " << stepCountL << std::endl;

					// 3) Record data
					if (record_) {
						double f = computeFitness(musclesToOptimizeIndexL_, musclesNonOptimizedIndexL_);
						loggers("L", f, "V");
					}
					stepEvent_L = false;
				}

			}
			newEMG_ = false; // To validate if there is a new EMG set in the next iteration
		}
	}
}


/*
	>> Loops where the algorithms takes places
*/

// >> Right foot
template <typename NMSmodelT>
void OptimizationMusclePlugin<NMSmodelT>::algorithmLoopR() {
	while (threadStop_) {
		if (algorithmStartR_) {

			algorithmStartR_ = false;

			/*std::cout << right << setw(90) << ">> ";
			cout << "0) Optimization Thread: Begin Algorithm - Right" << endl;*/

			// Create the problem from the UDP class
			problem probR{ UDP_R_ };

			// Create Algorithm

			// ->> CMA-ES: https://esa.github.io/pagmo2/docs/cpp/algorithms/cmaes.html
			algorithm algoR{ pagmo::cmaes(numGen_, -1., -1., -1., -1., 0.5, 1e-4,1e-6, false, true) };
			// 6 Generation, f tolerance of 1e-4, Force_bounds are set to true

			// Create Population with the same sizeof refCount_ based on the given bounds
			population popR{ probR, population::size_type(numPop_) }; // (problem, pop_size(), (seed))




			/*
			// ->> Corana's SA: https://esa.github.io/pagmo2/docs/cpp/algorithms/simulated_annealing.html
			algorithm algoR{ pagmo::simulated_annealing(30.,.1,2u,1u,10u,1.) };
			//Ts(10.), Tf(.1), n_T_adj(10), n_range_adj(1), bin_size(20), start_range(1.), (seed)
			// At each call of the evolve method the number of fitness evaluations will be n_T_adj * n_range_adj * bin_size times the problem dimension
			population popR{ probR, 0 }; // (problem, pop_size(), (seed))
			popR.push_back(SR_b); // have SR baseline as initial guess
			*/


			// Evolve the population to get the optimized population
			popR = algoR.evolve(popR);

			// State that the optimization have been finished
			optimizationDone_R_ = true;
			cv_R.notify_one(); // To notify the wait() method so we can unlock the mutex
			std::cout << endl;
			std::cout << endl;
			cout << "Optimization Right Leg Concluded" << endl;

			// Get the SR champion and its fitness
			vector<double> x = popR.champion_x(); // SR champion
			double f = popR.champion_f().at(0); // Best fitness, correspondent to the SR champion

			// Set the SR champion to the Right foot
			for (int i = 0; i < musclesListIndexR_.size(); i++) {
				int it = musclesListIndexR_[i];
				SR_[it] = x[it];
			}

			// Print the results
			std::cout << "Support Ratio Champion - Right Foot: " << std::endl;
			printSR(musclesListIndexR_);
			std::cout << "Best Fitness - Right Foot: " << f << std::endl;
			std::cout << endl;
		}
	}
}

// >> Left foot
template <typename NMSmodelT>
void OptimizationMusclePlugin<NMSmodelT>::algorithmLoopL() {

	while (threadStop_) {
		if (algorithmStartL_) {

			algorithmStartL_ = false;

			std::cout << right << setw(90) << ">> ";
			cout << "0) Optimization Thread: Begin Algorithm - Left" << endl;

			// Create the first problem from the UDP class
			problem probL{ UDP_L_ };

			// Create Algorithm

			// ->> CMA-ES: https://esa.github.io/pagmo2/docs/cpp/algorithms/cmaes.html
			algorithm algoL{ pagmo::cmaes(numGen_, -1., -1., -1., -1., 0.5, 1e-4,1e-6, false, true) };
			// 6 Generation, f tolerance of 1e-4, Force_bounds are set to true

			// Create Population with the same sizeof refCount_ based on the given bounds
			population popL{ probL, population::size_type(numPop_) }; // (problem, pop_size(), (seed))

			/*
			// ->> Corana's SA: https://esa.github.io/pagmo2/docs/cpp/algorithms/simulated_annealing.html
			algorithm algoL{ pagmo::simulated_annealing(30.,.1,2u,1u,10u,1.) };
			//Ts(10.), Tf(.1), n_T_adj(10), n_range_adj(1), bin_size(20), start_range(1.), (seed)
			// At each call of the evolve method the number of fitness evaluations will be n_T_adj * n_range_adj * bin_size times the problem dimension

			population popL{ probL, 0 }; // (problem, pop_size(), (seed))
			popL.push_back(SR_b); // have SR baseline as initial guess
			*/

			// Evolve the population to get the optimized population
			popL = algoL.evolve(popL);

			// State that the optimization have been finished
			optimizationDone_L_ = true;
			cv_L.notify_one(); // To notify the wait() method so we can unlock the mutex
			std::cout << endl;
			std::cout << endl;
			cout << "Optimization Left Leg Concluded" << endl;

			// Get the SR champion and its fitness
			vector<double> x = popL.champion_x(); // SR champion
			double f = popL.champion_f().at(0); // Best fitness, correspondent to the SR champion

			// Set the SR champion to the Left foot
			for (int i = 0; i < musclesListIndexL_.size(); i++) {
				int it = musclesListIndexL_[i];
				SR_[it] = x[it];
			}

			// Print the results
			std::cout << "Support Ratio Champion - Left Foot: " << std::endl;
			printSR(musclesListIndexL_);
			std::cout << "Best Fitness - Left Foot: " << f << std::endl << std::endl;
			std::cout << endl;
		}
	}
}



/*
	>> Personalized Functions
*/

/*
	-> Torque Loop
*/

// >> Compute Assistive torque with the new set of SRs
template <typename NMSmodelT>
vector<double> OptimizationMusclePlugin<NMSmodelT>::computeTorque() {


	// 1) Get the MTForces from the model/CEINMS
	// Automatically sended by ModelEvaluationRealTime and saved into muscleForces_
	DataMutex_.lock(); // Lock the data access on the memory address 

	// 2) Set MTForces*SR in the model
	// Multiply the Forces by the SR

	for (int i = 0; i < musclesListIndex_.size(); i++) {
		int index = musclesListIndex_.at(i);
		newMuscleForces_.at(index) = muscleForces_.at(index) * SR_.at(index);
	}

	// Set  Moment Arms and New Muscle Forces into the NMS model
	unsigned int j = 0;
	std::vector<vector<double>> ma_log; // for logging the moment arms
	for (auto it = dofsUsed_.begin(); it != dofsUsed_.end(); ++it) // set moment arms for the DOFs that are optimized
	{
		model_->setMomentArms(ma_.at(*it), j);
		j++;
		ma_log.push_back(ma_.at(*it));
	}

	model_->setMuscleForces(newMuscleForces_); // Set MTForces*SR in the model

	// Print the Tables > It can increase computation time
	// printGeneralTable();

	DataMutex_.unlock();

	// Update the model
	model_->updateTorques(); // Compute the torque given the updated MTForces*SR
	model_->pushState();


	// 3) Get the computed DOF torque from the model
	vector<double> aTorque;
	DataMutex_.lock();
	model_->getTorques(dofTorque_);
	DataMutex_.unlock();
	aTorque = dofTorque_;

	return aTorque;
}

// >> Send torque to Exoskeleton
template <typename NMSmodelT>
void OptimizationMusclePlugin<NMSmodelT>::sendTorque(vector<double> dofTorque) {

	double timeLocal;
	timeLocal = time_;

	int length = 2; //Lankle, Rankle
	std::vector<double> temp;
	temp.resize(2);
	for (std::vector<std::string>::const_iterator it = dofName_.begin(); it != dofName_.end(); it++) {
		int cpt = std::distance<std::vector<std::string>::const_iterator>(dofName_.begin(), it);
		if (*it == "ankle_angle_l") {
			temp[0] = dofTorque[cpt];
			//client_->write(varNameVect_[VarName::TorqueControlLeftAnkle], &temp[cpt], sizeof(double));
			//std::cout << "ankle_angle_l" << temp[cpt] << std::endl << std::flush;
		}
		if (*it == "ankle_angle_r") {
			temp[1] = dofTorque[cpt];
			//client_->write(varNameVect_[VarName::TorqueControlRightAnkle], &temp[cpt], sizeof(double));
			//std::cout << "ankle_angle_r" << temp[cpt] << std::endl << std::flush;
		}
	}
	if (dofTorque.size() >= 2) {
		client_->write(varNameVect_[VarName::AchInputs], &temp[0], 2 * sizeof(double));
	}
}

// >> Print into the console General Information
template <typename NMSmodelT>
void OptimizationMusclePlugin<NMSmodelT>::printGeneralTable() {
	// for the table format:
	const char separator = ' ';
	const int nameWidth = 10;
	const int titleWidth = 23;


	std::cout << "\nGeneral Information: " << std::endl;
	std::cout << right << setw(titleWidth) << "Muscle:  ";
	for (int i = 0; i < muscleName_.size(); i++) {
		cout << left << setw(nameWidth) << setfill(separator) << muscleName_[i];
		cout << " | ";
	}
	std::cout << std::endl;
	std::cout << right << setw(titleWidth) << "SR:  ";
	for (int i = 0; i < muscleName_.size(); i++) {
		cout << left << setw(nameWidth) << setfill(separator) << SR_[i];
		cout << " | ";
	}
	std::cout << std::endl;
	std::cout << right << setw(titleWidth) << "MTForces CEINMS (N):  ";
	for (int i = 0; i < muscleName_.size(); i++) {
		cout << left << setw(nameWidth) << setfill(separator) << muscleForces_[i];
		cout << " | ";
	}
	std::cout << std::endl;
	std::cout << right << setw(titleWidth) << "MTForces*SR (N):  ";
	for (int i = 0; i < muscleName_.size(); i++) {
		cout << left << setw(nameWidth) << setfill(separator) << newMuscleForces_[i];
		cout << " | ";
	}
	std::cout << std::endl;
	std::cout << std::endl;


	std::cout << "Optimized Torque: " << std::endl;
	std::cout << right << setw(titleWidth) << "DOF:  ";
	for (int i = 0; i < dofName_.size(); i++) {
		cout << left << setw(16) << setfill(separator) << dofName_[i];
		cout << " | ";
	}
	std::cout << std::endl;
	std::cout << right << setw(titleWidth) << "Torque (Nm):  ";
	for (int i = 0; i < dofTorque_.size(); i++) {
		cout << left << setw(16) << setfill(separator) << dofTorque_[i];
		cout << " | ";
	}
	std::cout << std::endl;
	std::cout << std::endl;

}


/*
	-> Optimization Loop
*/


// >> To see if there is new EMG (Or MTForces)
template <typename NMSmodelT>
void OptimizationMusclePlugin<NMSmodelT>::newEMGvalidator() {
	if (optimizationCriterion_ == "EMG") {
		// Validate if there is new EMG data
		DataMutex_.lock();
		if (Emgs_ != oldEMG_ && Emgs_.size() > 0) {
			oldEMG_ = Emgs_;
			newEMG_ = true;
		}
		else {
			newEMG_ = false;
		}
		DataMutex_.unlock();
	}
	else if (optimizationCriterion_ == "MTForces") {
		// Validate if there is new MTF data
		DataMutex_.lock();
		if (muscleForces_ != oldEMG_ && muscleForces_.size() > 0) {
			oldEMG_ = muscleForces_;
			newEMG_ = true;
		}
		else {
			newEMG_ = false;
		}
		DataMutex_.unlock();
	}

}


// >> Populate EMG to compute mean
template <typename NMSmodelT>
void OptimizationMusclePlugin<NMSmodelT>::populateEMG() {
	if (optimizationCriterion_ == "EMG") {
		for (int i = 0; i < musclesListIndex_.size(); i++) {
			int indexMuxcle_ = musclesListIndex_[i];
			loopEMG_[indexMuxcle_].push_back(Emgs_[indexMuxcle_]);
			// for each considered muscle, add the current EMG to the end of the vector
			/*>> Note: The order of th vector loopEMG_ is the same of the muscleName_ vector, however, only the index of
				the muscles indicated in the XML file will be populated to minimize computation time */
		}
	}
	else if (optimizationCriterion_ == "MTForces") {
		for (int i = 0; i < musclesListIndex_.size(); i++) {
			int indexMuxcle_ = musclesListIndex_[i];
			loopEMG_[indexMuxcle_].push_back(muscleForces_[indexMuxcle_]);
		}
	}
}

// >> Get Step event
template <typename NMSmodelT>
void OptimizationMusclePlugin<NMSmodelT>::getGRFEvent() {

	// Will use DOF torque for offline debuging
	// For online execution will use the gait cycle percentage/GRF

	int length = 2; //Lankle, Rankle
	std::vector<double> temp;
	temp.resize(7);

	DataMutex_.lock();

	if (sendTorque_) {
		/// If executing online > Using Gait cycle percentage
		client_->read(varNameVect_[VarName::AchOutput], &temp[0], 7 * sizeof(double));
		GRFz_[0] = temp[6]; // Right gait cycle
		GRFz_[1] = temp[5]; // Left gait cycle

		thereshold_ = 50; //Percentage of the gait cycle 
		// > 50% Is when the EMGs of the GM, GL, and SOL are minimal, so any difference/perturbance between steps will have minimal impact
	}
	else {
		if (externalDOFTorque_.size() > 0) {
			// To validate that it is not empty
		/// If debuging offline > Using DOF Torque instead of GRF
			GRFz_[0] = abs(externalDOFTorque_[1]);
			GRFz_[1] = abs(externalDOFTorque_[3]);
			thereshold_ = 30;
		}
	}

	if (externalDOFTorque_.size() > 0 || sendTorque_) {

		// Right Foot evaluation
		if (GRFz_[0] > thereshold_) {
			if (!stepValidationR_) { // to see if there wasn't a declared step in the previous iteration 
				stepEvent_R = true;
				stepEvent_foot_R = 'R';
				// Declare that a right step was already declared
				stepValidationR_ = true;
			}
			else {
				// So we don't define a new step
				stepEvent_R = false;
				stepValidationR_ = true;
				stepEvent_foot_R = 'NaN';
			}
		}
		else if (GRFz_[0] <= thereshold_) {
			if (stepValidationR_) {
				stepValidationR_ = false; // So a new step can be observed 
				stepEvent_R = false;
				stepEvent_foot_R = 'NaN';
			}
		}

		// Left Foot evaluation
		if (GRFz_[1] > thereshold_) {
			if (!stepValidationL_) { // to see if there wasn't a declared step in the previous iteration 
				stepEvent_L = true;
				stepEvent_foot_L = 'L';
				// Declare that a right step was already declared

				stepValidationL_ = true;
			}
			else {
				// So we don't define a new step
				stepEvent_L = false;
				stepValidationL_ = true;
				stepEvent_foot_L = 'NaN';
			}
		}
		else if (GRFz_[1] <= thereshold_) {
			if (stepValidationL_) {
				stepValidationL_ = false; // So a new step can be observed 
				stepEvent_L = false;
				stepEvent_foot_L = 'NaN';
			}
		}
	}
	DataMutex_.unlock();

}

// >> Compute EMG mean and clear the loopEMG_ for each called muscle
template <typename NMSmodelT>
void OptimizationMusclePlugin<NMSmodelT>::computeEMGmean(vector<unsigned> muscleIndexList) {
	for (int i = 0; i < muscleIndexList.size(); i++) {
		int mIndex_ = muscleIndexList[i];
		auto Nemg = loopEMG_[mIndex_].size();
		double meanEMGloop = 0.0;
		if (Nemg != 0) {
			meanEMGloop = accumulate(loopEMG_[mIndex_].begin(), loopEMG_[mIndex_].end(), 0.0) / Nemg;
		}
		meanEMG_[mIndex_] = meanEMGloop;

		// clear loopEMG_ of the computed muscle
		loopEMG_[mIndex_].clear();
	}
}

// >> Add the EMG mean of each muscle to the vector firstsEMGmean_
template <typename NMSmodelT>
void OptimizationMusclePlugin<NMSmodelT>::addRefEMGmean(vector<unsigned> muscleIndexList) {
	for (int i = 0; i < muscleIndexList.size(); i++) {
		int mIndex_ = muscleIndexList[i];
		firstsEMGmean_[mIndex_].push_back(meanEMG_[mIndex_]);
	}
}

// >> Compute EMG REF mean and SD for each called muscle for better representation
template <typename NMSmodelT>
void OptimizationMusclePlugin<NMSmodelT>::computeRefEMGmean(vector<unsigned> muscleIndexList) {
	for (int i = 0; i < muscleIndexList.size(); i++) {
		int mIndex_ = muscleIndexList[i];
		auto NmeanEmg = firstsEMGmean_[mIndex_].size();
		// Compute the Ref EMG mean
		double meanEMGRef = 0.0;
		if (NmeanEmg != 0) {
			meanEMGRef = accumulate(firstsEMGmean_[mIndex_].begin(), firstsEMGmean_[mIndex_].end(), 0.0) / NmeanEmg;
		}
		refEMGmean_[mIndex_] = meanEMGRef; // Mean for each muscle

		// Compute the Ref EMG SD
		double var = 0.0; // Variance
		if (NmeanEmg != 0) {
			for (int i = 0; i < firstsEMGmean_[mIndex_].size(); i++) {
				var += pow(firstsEMGmean_[mIndex_].at(i) - meanEMGRef, 2);
			}
			var /= NmeanEmg;
		}
		refEMGsd_[mIndex_] = sqrt(var); // Standard Deviation for each muscle
	}


	// table printing:
	const char separator = ' ';
	const int nameWidth = 10;


	std::cout << right << setw(nameWidth) << "Muscle:  ";
	for (int i = 0; i < muscleIndexList.size(); i++)
	{
		cout << left << setw(nameWidth) << setfill(separator) << muscleName_[muscleIndexList[i]];
		cout << " | ";
	}
	std::cout << std::endl;
	std::cout << right << setw(nameWidth) << "Mean:  ";
	for (int i = 0; i < muscleIndexList.size(); i++)
	{
		cout << left << setw(nameWidth) << setfill(separator) << refEMGmean_[muscleIndexList[i]];
		cout << " | ";
	}
	std::cout << std::endl;
	std::cout << right << setw(nameWidth) << "SD:  ";
	for (int i = 0; i < muscleIndexList.size(); i++)
	{
		cout << left << setw(nameWidth) << setfill(separator) << refEMGsd_[muscleIndexList[i]];
		cout << " | ";
	}
	std::cout << std::endl;
	std::cout << std::endl;


}

// >> Print Table of the SR
template <typename NMSmodelT>
void OptimizationMusclePlugin<NMSmodelT>::printSR(vector<unsigned> muscleIndexList) {
	const char separator = ' ';
	const int nameWidth = 15;
	std::cout << right << setw(nameWidth) << "Muscle:  ";
	for (int i = 0; i < muscleIndexList.size(); i++) {
		int it = muscleIndexList[i];
		cout << left << setw(nameWidth) << setfill(separator) << muscleName_[it];
		cout << " | ";
	}
	std::cout << std::endl;
	std::cout << right << setw(nameWidth) << "SR:  ";
	for (int i = 0; i < muscleIndexList.size(); i++) {
		int it = muscleIndexList[i];
		cout << left << setw(nameWidth) << setfill(separator) << SR_[it];
		cout << " | ";
	}
	std::cout << std::endl;
	/*std::cout << right << setw(nameWidth) << "M_EMG:  ";
	for (int i = 0; i < muscleIndexList.size(); i++){
		int it = muscleIndexList[i];
		cout << left << setw(nameWidth) << setfill(separator) << meanEMG_[it];
		cout << " | ";
	}
	std::cout << std::endl;
	std::cout << std::endl;*/
}

// To compute the fitness after optimization so we can have an idea of the variance of the EMG with the same SR
template <typename NMSmodelT>
double OptimizationMusclePlugin<NMSmodelT>::computeFitness(vector<unsigned> musclesToOptimizeIndex_, vector<unsigned> musclesNonOptimizedIndex_) {
	// Compute fitness - Evaluate the Objective Function 
	// A) Optimized Muscles
	double fpOpt = 0.0;
	for (int i = 0; i < musclesToOptimizeIndex_.size(); i++) {
		int muscleIndex_ = musclesToOptimizeIndex_[i];
		fpOpt += abs(((1 - r_) * refEMGmean_[muscleIndex_]) - meanEMG_[muscleIndex_]);
	}
	fpOpt *= cOpt_;

	// B) Non Optimized Muscles
	double fpNOpt = 0.0;
	for (int i = 0; i < musclesNonOptimizedIndex_.size(); i++) {
		int muscleIndex_ = musclesNonOptimizedIndex_[i];
		fpNOpt += abs(refEMGmean_[muscleIndex_] - meanEMG_[muscleIndex_]);
	}
	fpNOpt *= cNOpt_;

	// C) Compute and return Objective Function
	return fpOpt + fpNOpt;
}

/*
	>> Loggers: Used just if Record is set to true to save data into file
*/

template <typename NMSmodelT>
void OptimizationMusclePlugin<NMSmodelT>::loggersInit(string directoriOptimization) {

	/*
		>> Right Foot
	*/

	// EMG Reference
	EMGRefLogger_R.open(directoriOptimization + "EMGRef_R.csv"); // opens the file
	if (!EMGRefLogger_R) {
		std::cerr << "Error: EMGRef_R.csv file could not be opened" << std::endl;
		exit(1);
	}
	EMGRefLogger_R << "Muscles,";
	for (int i = 0; i < musclesListIndexR_.size(); i++) {
		int it = musclesListIndexR_[i];
		EMGRefLogger_R << muscleName_[it] << ",";

	}
	EMGRefLogger_R << std::endl;

	// SR
	SRLogger_R.open(directoriOptimization + "SR_R.csv"); // opens the file
	if (!SRLogger_R) {
		std::cerr << "Error: SR_R.csv file could not be opened" << std::endl;
		exit(1);
	}
	SRLogger_R << "Type," << "Time," << "Step No.,";
	for (int i = 0; i < musclesListIndexR_.size(); i++) {
		int it = musclesListIndexR_[i];
		SRLogger_R << "SR " << muscleName_[it] << ",";

	}
	SRLogger_R << std::endl;

	// EMG
	EMGLogger_R.open(directoriOptimization + "EMG_R.csv"); // opens the file
	if (!EMGLogger_R) {
		std::cerr << "Error: EMG_R.csv file could not be opened" << std::endl;
		exit(1);
	}
	EMGLogger_R << "Type," << "Time," << "Step No.," << "f(EMG(SR)),";
	for (int i = 0; i < musclesListIndexR_.size(); i++) {
		int it = musclesListIndexR_[i];
		EMGLogger_R << "EMG " << muscleName_[it] << ",";
	}
	EMGLogger_R << std::endl;


	/*
		>> Left Foot
	*/

	// EMG Reference
	EMGRefLogger_L.open(directoriOptimization + "EMGRef_L.csv"); // opens the file
	if (!EMGRefLogger_L) {
		std::cerr << "Error: EMGRef_L.csv file could not be opened" << std::endl;
		exit(1);
	}
	EMGRefLogger_L << "Muscles,";
	for (int i = 0; i < musclesListIndexL_.size(); i++) {
		int it = musclesListIndexL_[i];
		EMGRefLogger_L << muscleName_[it] << ",";

	}
	EMGRefLogger_L << std::endl;

	// SR
	SRLogger_L.open(directoriOptimization + "SR_L.csv"); // opens the file
	if (!SRLogger_L) {
		std::cerr << "Error: SR_L.csv file could not be opened" << std::endl;
		exit(1);
	}
	SRLogger_L << "Type," << "Time," << "Step No.,";
	for (int i = 0; i < musclesListIndexR_.size(); i++) {
		int it = musclesListIndexL_[i];
		SRLogger_L << "SR " << muscleName_[it] << ",";

	}
	SRLogger_L << std::endl;

	// EMG
	EMGLogger_L.open(directoriOptimization + "EMG_L.csv"); // opens the file
	if (!EMGLogger_L) {
		std::cerr << "Error: EMG_L.csv file could not be opened" << std::endl;
		exit(1);
	}
	EMGLogger_L << "Type," << "Time," << "Step No.," << "f(EMG(SR)),";
	for (int i = 0; i < musclesListIndexL_.size(); i++) {
		int it = musclesListIndexL_[i];
		EMGLogger_L << "EMG " << muscleName_[it] << ",";

	}
	EMGLogger_L << std::endl;

}

template <typename NMSmodelT>
void OptimizationMusclePlugin<NMSmodelT>::loggers(string foot, float f, string type) {

	if (foot == "R") {
		// SR
		SRLogger_R << type << "," << std::setprecision(15) << time_ << "," << stepCountR << ",";
		for (int i = 0; i < musclesListIndexR_.size(); i++) {
			int it = musclesListIndexR_[i];
			SRLogger_R << SR_[it] << ",";
		}
		SRLogger_R << std::endl;

		// EMG
		EMGLogger_R << type << "," << std::setprecision(15) << time_ << "," << stepCountR << "," << f << ",";
		for (int i = 0; i < musclesListIndexR_.size(); i++) {
			int it = musclesListIndexR_[i];
			EMGLogger_R << meanEMG_[it] << ",";

		}
		EMGLogger_R << std::endl;

	}
	else if (foot == "L") {
		// SR
		SRLogger_L << type << "," << std::setprecision(15) << time_ << "," << stepCountL << ",";
		for (int i = 0; i < musclesListIndexL_.size(); i++) {
			int it = musclesListIndexL_[i];
			SRLogger_L << SR_[it] << ",";
		}
		SRLogger_L << std::endl;

		// EMG
		EMGLogger_L << type << "," << std::setprecision(15) << time_ << "," << stepCountL << "," << f << ",";
		for (int i = 0; i < musclesListIndexL_.size(); i++) {
			int it = musclesListIndexL_[i];
			EMGLogger_L << meanEMG_[it] << ",";
		}
		EMGLogger_L << std::endl;
	}



}

template <typename NMSmodelT>
void OptimizationMusclePlugin<NMSmodelT>::loggersRef(string foot) {

	if (foot == "R") {
		// EMG Reference mean
		EMGRefLogger_R << "Mean,";
		for (int i = 0; i < musclesListIndexR_.size(); i++) {
			int it = musclesListIndexR_[i];
			EMGRefLogger_R << refEMGmean_[it] << ",";

		}
		EMGRefLogger_R << std::endl;

		// EMG Reference SD
		EMGRefLogger_R << "SD,";
		for (int i = 0; i < musclesListIndexR_.size(); i++) {
			int it = musclesListIndexR_[i];
			EMGRefLogger_R << refEMGsd_[it] << ",";

		}
		EMGRefLogger_R << std::endl;
	}
	else if (foot == "L") {
		// EMG Reference mean
		EMGRefLogger_L << "Mean,";
		for (int i = 0; i < musclesListIndexL_.size(); i++) {
			int it = musclesListIndexL_[i];
			EMGRefLogger_L << refEMGmean_[it] << ",";

		}
		EMGRefLogger_L << std::endl;

		// EMG Reference SD
		EMGRefLogger_L << "SD,";
		for (int i = 0; i < musclesListIndexL_.size(); i++) {
			int it = musclesListIndexL_[i];
			EMGRefLogger_L << refEMGsd_[it] << ",";

		}
		EMGRefLogger_L << std::endl;
	}

}





/*




*/




// >> Important for building, linking, and executing the plugin

#ifdef UNIX
extern "C" {
	OptimizationPlugin<NMSmodel<ExponentialActivationRT, StiffTendon<Curve<CurveMode::Online> >, CurveMode::Online> >* createEAS()
	{
		return new OptimizationMusclePlugin<NMSmodel<ExponentialActivationRT, StiffTendon<Curve<CurveMode::Online> >, CurveMode::Online> >;
	}

	OptimizationPlugin<NMSmodel<ExponentialActivationRT, ElasticTendon_BiSec<Curve<CurveMode::Online> >, CurveMode::Online> >* createEAEB()
	{
		return new OptimizationMusclePlugin<NMSmodel<ExponentialActivationRT, ElasticTendon_BiSec<Curve<CurveMode::Online> >, CurveMode::Online> >;
	}

	OptimizationPlugin<NMSmodel<ExponentialActivationRT, ElasticTendon<Curve<CurveMode::Online> >, CurveMode::Online> >* createEAE()
	{
		return new OptimizationMusclePlugin<NMSmodel<ExponentialActivationRT, ElasticTendon<Curve<CurveMode::Online> >, CurveMode::Online> >;
	}
}


extern "C" {
	void destroyEAS(OptimizationPlugin<NMSmodel<ExponentialActivationRT, StiffTendon<Curve<CurveMode::Online> >, CurveMode::Online> >* p)
	{
		delete p;
	}

	void destroyEAEB(OptimizationPlugin<NMSmodel<ExponentialActivationRT, ElasticTendon_BiSec<Curve<CurveMode::Online> >, CurveMode::Online> >* p)
	{
		delete p;
	}

	void destroyEAE(OptimizationPlugin<NMSmodel<ExponentialActivationRT, ElasticTendon<Curve<CurveMode::Online> >, CurveMode::Online> >* p)
	{
		delete p;
	}
}
#endif
#ifdef WIN32 // __declspec (dllexport) id important for dynamic loading
extern "C" {
	__declspec (dllexport) OptimizationPlugin<NMSmodel<ExponentialActivationRT, StiffTendon<Curve<CurveMode::Online> >, CurveMode::Online> >* __cdecl createEAS()
	{
		return new OptimizationMusclePlugin<NMSmodel<ExponentialActivationRT, StiffTendon<Curve<CurveMode::Online> >, CurveMode::Online> >;
	}

	__declspec (dllexport) OptimizationPlugin<NMSmodel<ExponentialActivationRT, ElasticTendon_BiSec<Curve<CurveMode::Online> >, CurveMode::Online> >* __cdecl createEAEB()
	{
		return new OptimizationMusclePlugin<NMSmodel<ExponentialActivationRT, ElasticTendon_BiSec<Curve<CurveMode::Online> >, CurveMode::Online> >;
	}

	__declspec (dllexport) OptimizationPlugin<NMSmodel<ExponentialActivationRT, ElasticTendon<Curve<CurveMode::Online> >, CurveMode::Online> >* __cdecl createEAE()
	{
		return new OptimizationMusclePlugin<NMSmodel<ExponentialActivationRT, ElasticTendon<Curve<CurveMode::Online> >, CurveMode::Online> >;
	}
}


extern "C" {
	__declspec (dllexport) void __cdecl destroyEAS(OptimizationPlugin<NMSmodel<ExponentialActivationRT, StiffTendon<Curve<CurveMode::Online> >, CurveMode::Online> >* p)
	{
		delete p;
	}

	__declspec (dllexport) void __cdecl destroyEAEB(OptimizationPlugin<NMSmodel<ExponentialActivationRT, ElasticTendon_BiSec<Curve<CurveMode::Online> >, CurveMode::Online> >* p)
	{
		delete p;
	}

	__declspec (dllexport) void __cdecl destroyEAE(OptimizationPlugin<NMSmodel<ExponentialActivationRT, ElasticTendon<Curve<CurveMode::Online> >, CurveMode::Online> >* p)
	{
		delete p;
	}
}
#endif
template class OptimizationMusclePlugin<NMSmodel<ExponentialActivationRT, StiffTendon<Curve<CurveMode::Online> >, CurveMode::Online> >;
template class OptimizationMusclePlugin<NMSmodel<ExponentialActivationRT, ElasticTendon_BiSec<Curve<CurveMode::Online> >, CurveMode::Online> >;
template class OptimizationMusclePlugin<NMSmodel<ExponentialActivationRT, ElasticTendon<Curve<CurveMode::Online> >, CurveMode::Online> >;


