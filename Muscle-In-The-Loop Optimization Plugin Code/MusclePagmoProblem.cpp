// Muscle-in-the-loop Optimization
// Tiago Rodrigues, Guillaume Durandau, 2021
// tiagoepr@ua.pt | t.e.pereirarodrigues@student.utwente.nl

#include "MusclePagmoProblem.h"

template<typename NMSmodelT>
MusclePagmoProblem<NMSmodelT>::MusclePagmoProblem() {

}

template<typename NMSmodelT>
MusclePagmoProblem<NMSmodelT>::~MusclePagmoProblem() {
}

/*
	Mandatory functions (see pagmo user-defined problem documentation):
*/

template<typename NMSmodelT>
std::vector<double> MusclePagmoProblem<NMSmodelT>::fitness(const std::vector<double>& dv) const {
	// Evaluate the Objective Function for the new population

	//std::cout << right << setw(90) << ">> ";
	//cout << "1) Algorithm Thread: SR gotten from the algorithm" << endl;


	// Call a non-const function whithin a const function: Is not reliable, but it is the best we can do for now
	const_cast<MusclePagmoProblem<NMSmodelT>*>(this)->getFitness(const_cast<const vector<double>&>(dv)); // Computes the fitness

	return { fp_ };
}


template<typename NMSmodelT>
std::pair<std::vector<double>, std::vector<double>> MusclePagmoProblem<NMSmodelT>::get_bounds() const {
	return bounds_;
}



/*
	>> Public Get/set functions
*/


template<typename NMSmodelT>
void MusclePagmoProblem<NMSmodelT>::setRefDecisionVector(vector<double> refDecisionVector) {
	refDecisionVector_ = refDecisionVector;
}

template<typename NMSmodelT>
void MusclePagmoProblem<NMSmodelT>::setWeights(double cOpt, double cNOpt) {
	cOpt_ = cOpt;
	cNOpt_ = cNOpt;
}

template<typename NMSmodelT>
void MusclePagmoProblem<NMSmodelT>::setReductionFactor(double r) {
	r_ = r;
}


template<typename NMSmodelT>
void MusclePagmoProblem<NMSmodelT>::setMusclesIndex(vector<unsigned> musclesToOptimizeIndex, vector<unsigned> musclesNonOptimizedIndex) {
	musclesToOptimizeIndex_ = musclesToOptimizeIndex;
	musclesNonOptimizedIndex_ = musclesNonOptimizedIndex;
}


template<typename NMSmodelT>
void MusclePagmoProblem<NMSmodelT>::set_bounds(std::vector<double> lowerBounds, std::vector<double> upperBounds) {
	bounds_ = { lowerBounds, upperBounds };
}


template<typename NMSmodelT>
void MusclePagmoProblem<NMSmodelT>::setSrAndEmg(SrAndEmg* SrAndEmg) {
	SrAndEmg_ = SrAndEmg;
}


template<typename NMSmodelT>
void MusclePagmoProblem<NMSmodelT>::setcv(condition_variable* cv) {
	cv_ = cv;
}


/*
	>> Private Functions
*/

template<typename NMSmodelT>
void MusclePagmoProblem<NMSmodelT>::getFitness(vector<double> dv) {

	// References for waiting for the SR evaluation: Mutex + condition variable + semaphore + class with pointers
	// https://en.cppreference.com/w/cpp/thread/condition_variable
	// http://www.gerald-fahrnholz.eu/sw/online_doc_multithreading/html/group___grp_mutexes_and_locks.html
	// https://stackoverflow.com/questions/20516773/stdunique-lockstdmutex-or-stdlock-guardstdmutex
	// https://stackoverflow.com/questions/43019598/stdlock-guard-or-stdscoped-lock

	// 1) Set the SR into the object and notify the optimization thread that it can now access the SR
	{
		std::lock_guard<std::mutex> lk(*SrAndEmg_->getMutex());
		// 2) Set the SR into the torque loop
		SrAndEmg_->setSR(dv);
		//std::cout << right << setw(90) << ">> ";
		//cout << "2) Algorithm Thread: SR set to the object" << endl;

		SrAndEmg_->setSRvalidator(true); // set the SR validator to true so that the other thread knows that it already can access to the SR
		SrAndEmg_->setEMGvalidator(false); // set the EMG validator to false so that the thread can wait until we have data and it is turned to true
	}
	cv_->notify_one();


	// 2) Lock the thread until we get the new EMG data corresponding to the current SR from the optimization thread
	std::unique_lock<std::mutex> lk(*SrAndEmg_->getMutex());
	// Wait until we get the validation that we have the mean EMG, i.e., the condition variable is true
	cv_->wait(lk, [this] {return SrAndEmg_->getEMGvalidator(); });


	// 3) After the wait, we now own the lock, and we can access the new EMG, corresponding to this SR/dv, that we set before
	decisionVector_ = SrAndEmg_->getEMG();
	//std::cout << right << setw(90) << ">> ";
	//cout << "6) Algorithm Thread: Mean EMG gotten from the object" << endl;

	lk.unlock();


	// 4) Compute fitness - Evaluate the Objective Function 
	// A) Optimized Muscles
	double fpOpt = 0.0;
	for (int i = 0; i < musclesToOptimizeIndex_.size(); i++) {
		// we can optimize more than one muscle
		int muscleIndex_ = musclesToOptimizeIndex_[i];
		fpOpt += abs(((1 - r_) * refDecisionVector_[muscleIndex_]) - decisionVector_[muscleIndex_]);
	}
	fpOpt *= cOpt_;

	// B) Non Optimized Muscles
	double fpNOpt = 0.0;
	for (int i = 0; i < musclesNonOptimizedIndex_.size(); i++) {
		int muscleIndex_ = musclesNonOptimizedIndex_[i];
		fpNOpt += abs(refDecisionVector_[muscleIndex_] - decisionVector_[muscleIndex_]);
	}
	fpNOpt *= cNOpt_;

	// C) Compute Objective Function
	fp_ = fpOpt + fpNOpt;

	SrAndEmg_->setF(fp_);

	//cv.notify_one(); // Not needed since the lock will get out of scope and close by itself
}