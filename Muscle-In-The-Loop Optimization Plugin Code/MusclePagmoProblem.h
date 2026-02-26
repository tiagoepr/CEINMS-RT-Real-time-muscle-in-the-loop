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

#ifndef MusclePagmoProblem_h
#define MusclePagmoProblem_h


#include <utility> // for std::pair
#include <vector>
#include <string>

#include <boost/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/bind.hpp>
#include <future>

#include "SetupDataStructure.h"

#include "SrAndEmg.h" // For the data structure - for passing EMG and SR data

using namespace std;

template<typename NMSmodelT>
class MusclePagmoProblem
{
public:
	MusclePagmoProblem();
	~MusclePagmoProblem();

	// Mandatory functions (see pagmo user-defined problem documentation):
	std::vector<double> fitness(const std::vector<double>& dv) const; // dv: decision vector (values of (optimized muscles) EMG)
	std::pair<std::vector<double>, std::vector<double>> get_bounds() const;

	// Set functions (init)
	void setWeights(double cOpt, double cNOpt); // For setting the weights of the objective function 
	void setReductionFactor(double r); // For setting the Reduction Factor of the objective function 
	void setRefDecisionVector(vector<double> refDecisionVector); // for setting the Reference EMG or Muscle Forces
	void setMusclesIndex(vector<unsigned> musclesToOptimizeIndex, vector<unsigned> musclesNonOptimizedIndex); //for setting the muscles' index to optimize
	void setSrAndEmg(SrAndEmg* SrAndEmg); // To set the pointer of the SrAndEmg object
	void setcv(condition_variable* cv); // To set the pointer of the conditional variable
	void set_bounds(std::vector<double> lowerBounds, std::vector<double> upperBounds); // For setting bounds in the SR
	

private:
	// Function to compute the value of the objective function
	void getFitness(vector<double> dv);

	// variable for passing data
	SrAndEmg* SrAndEmg_; // Pointer to the object defined in the plugin
	condition_variable* cv_; // Pointer to the object defined in the plugin

	// Variables for the objective function
	vector<double> refDecisionVector_; // EMGs or MTForces
	vector<double> decisionVector_; // EMGs or MTForces
	double cOpt_; // Objective function's wheight of the Optimized Muscle
	double cNOpt_; // Objective function's wheight of the Non-Optimized Muscles
	double r_; // Objective function's Reduction Factor
	vector<unsigned> musclesToOptimizeIndex_; // index of the muscles to optimize 
	vector<unsigned> musclesNonOptimizedIndex_; // index of the muscles to optimize 
	vector<unsigned> musclesIndex_;
	vector<double> sr_;
	double fp_;

	// Variables for the optimization algorithm
	std::pair<std::vector<double>, std::vector<double>> bounds_;


};

#endif