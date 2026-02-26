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

// >> SR and EMG semaphore


#ifndef SrAndEmg_h
#define SrAndEmg_h

#include <utility> // for std::pair
#include <vector>
#include <string>

#include <boost/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/bind.hpp>
#include <future>


#include "SetupDataStructure.h"


using namespace std;

class SrAndEmg 
{
public:
	
	SrAndEmg() : EMGvalidator_(false), SRvalidator_(false), sr_({}), emg_({}) {};
	~SrAndEmg() {};
	

	void setSR(vector<double> sr) {
		setMutex_.lock();
		/*
		std::cout << "dv Structure = ";
		for (int i = 0; i < sr.size(); i++) {
			std::cout << sr[i] << ", ";
		}
		std::cout << " " << std::endl;
		*/
		sr_ = sr;
		setMutex_.unlock();
	};

	void setEMG(vector<double> emg) {
		setMutex_.lock();
		emg_ = emg;
		setMutex_.unlock();
	};

	void setF(double f_emg_sr) {
		setMutex_.lock();
		f_emg_sr_ = f_emg_sr;
		setMutex_.unlock();
	};

	void setEMGvalidator(bool condition) {
		setMutex_.lock();
		EMGvalidator_ = condition;
		setMutex_.unlock();
	}

	void setSRvalidator(bool condition) {
		setMutex_.lock();
		SRvalidator_ = condition;
		setMutex_.unlock();
	}

	bool getSRvalidator() {
		return SRvalidator_;
	};


	bool getEMGvalidator() { 
		return EMGvalidator_; 
	};
	
	vector<double> getSR() {
		return sr_;
	};
	
	vector<double> getEMG() {
		return emg_;
	};

	double getF() {
		return f_emg_sr_;
	}
	
	std::mutex* getMutex() {
		// Mutex aren't copiable, what actually makes sense.
		// In this case we return a pointer of the SRMutex (memory address), that coud be read and edited by all the threads
		return &SRMutex_; 
	};

private:
	bool EMGvalidator_;
	bool SRvalidator_;
	vector<double> sr_;
	vector<double> emg_;
	double f_emg_sr_;
	std::mutex SRMutex_;
	std::mutex setMutex_; // Needs to be a different Mutex since SRMutex would be always blocked from one of the threads
};


#endif