#pragma once

#ifndef __stereoexpander__
#define __stereoexpander__

#include "fxobjects.h"

/**
\struct stereoexpanderParameters
\ingroup FX-Objects
\brief
Custom parameter structure for the stereoexpander object.

\author <Your Name> <http://www.yourwebsite.com>
\remark <Put any remarks or notes here>
\version Revision : 1.0
\date Date : 2019 / 01 / 31
*/
struct stereoexpanderParameters
{
	stereoexpanderParameters() {}

	/** all FXObjects parameter objects require overloaded= operator so remember to add new entries if you add new variables. */
	stereoexpanderParameters& operator=(const stereoexpanderParameters& params)	// need this override for collections to work
	{
		// --- it is possible to try to make the object equal to itself
		//     e.g. thisObject = thisObject; so this code catches that
		//     trivial case and just returns this object
		if (this == &params)
			return *this;

		// --- copy from params (argument) INTO our variables
\		center = params.center;
		sides = params.sides;
		highPass = params.highPass;
		outLvl = params.outLvl;


		// --- MUST be last
		return *this;
	}

	// --- individual parameters
	//data_type myVariable = 0.0;	///< init
	double center = -6.0;
	double sides = -6.0;
	double highPass = 0.0;
	double outLvl = -12.0;

};


/**
\class stereoexpander
\ingroup FX-Objects
\brief
The stereoexpander object implements ....

Audio I/O:
- Processes mono input to mono output.
- *** Optionally, process frame *** Modify this according to your object functionality

Control I/F:
- Use stereoexpanderParameters structure to get/set object params.

\author <Your Name> <http://www.yourwebsite.com>
\remark <Put any remarks or notes here>
\version Revision : 1.0
\date Date : 2019 / 01 / 31
*/
class stereoexpander : public IAudioSignalProcessor
{
public:
	stereoexpander(void) {}	/* C-TOR */
	~stereoexpander(void) {}	/* D-TOR */

public:
	/** reset members to initialized state */
	virtual bool reset(double _sampleRate)
	{
		// --- store the sample rate
		sampleRate = _sampleRate;



		AudioFilterParameters hpfParams = highPassFilter.getParameters();
		hpfParams.algorithm = filterAlgorithm::kHPF1;
		// --- PRESET VALUES:
		hpfParams.fc = 100.0;
		highPassFilter.setParameters(hpfParams);
		highPassFilter.reset(_sampleRate);

		AudioFilterParameters lpfParams = parametric.getParameters();
		lpfParams.algorithm = filterAlgorithm::kCQParaEQ;
		// --- PRESET VALUES:
		lpfParams.fc = 100.0;
		lpfParams.Q = 12.0;
		lpfParams.boostCut_dB = 19.0;
		parametric.setParameters(lpfParams);
		parametric.reset(_sampleRate);

		AudioFilterParameters hsfParams = highShelving.getParameters();
		hsfParams.algorithm = filterAlgorithm::kHiShelf;
		hsfParams.fc = 1900.0;
		hsfParams.boostCut_dB = 10.0;
		highShelving.setParameters(hsfParams);
		highShelving.reset(_sampleRate);

		AudioFilterParameters apfParams = allPassFilter.getParameters();
		apfParams.algorithm = filterAlgorithm::kAPF1;
		// --- PRESET VALUES:
		apfParams.fc = 20480.0;
		allPassFilter.setParameters(apfParams);
		allPassFilter.reset(_sampleRate);

		// --- do any other per-audio-run inits here

		return true;
	}

	/** process MONO input */
	/**
	\param xn input
	\return the processed sample
	*/
	virtual double processAudioSample(double xn)
	{	/*
		// --- the output variable
		double yn = 0.0;

		// --- do your DSP magic here to create yn

		// --- done
		return yn;
		*/
		return xn;
	}

	/** query to see if this object can process frames */
	virtual bool canProcessAudioFrame() { return true; } // <-- change this!

	/** process audio frame: implement this function if you answer "true" to above query */
	virtual bool processAudioFrame(const float* inputFrame,	/* ptr to one frame of data: pInputFrame[0] = left, pInputFrame[1] = right, etc...*/
					     float* outputFrame,
					     uint32_t inputChannels,
					     uint32_t outputChannels)
	{
		double diffGainCooked = pow(10.0, parameters.sides / 20);
		double sumGainCooked = pow(10.0, parameters.center / 20);
		double outputCooked = pow(10.0, parameters.outLvl / 20);
		double xLn = inputFrame[0];
		double xRn = inputFrame[1];
		double yLn = 0.0;
		double yRn = 0.0;
		double leftOutput = 0.0;
		double rightOutput = 0.0;
		double output = 0.0;

		double drySum = xLn + xRn;

		double leftHighPass = highPassFilter.processAudioSample(xLn);
		double rightHighPass = highPassFilter.processAudioSample(xRn);

		double sum = (leftHighPass + rightHighPass) * sumGainCooked;
		double difference = (leftHighPass - rightHighPass) * diffGainCooked;


		// Parametric
		double diffPara = parametric.processAudioSample(difference);

		//All Pass
		double diffAllPass = allPassFilter.processAudioSample(difference);

		//High Shelf
		double diffHiShelf = highShelving.processAudioSample(difference);

		//Outputs
		// --- Right
		rightOutput = xRn + sum - (diffPara + diffAllPass + diffHiShelf);

		// --- Left
		leftOutput = xLn + sum + (diffPara + diffAllPass + diffHiShelf);

		outputFrame[0] = leftOutput * outputCooked;
		outputFrame[1] = rightOutput * outputCooked;

		// --- do nothing
		return true; //handled
	}


	/** get parameters: note use of custom structure for passing param data */
	/**
	\return stereoexpanderParameters custom data structure
	*/
	stereoexpanderParameters getParameters()
	{
		return parameters;
	}

	/** set parameters: note use of custom structure for passing param data */
	/**
	\param stereoexpanderParameters custom data structure
	*/
	void setParameters(const stereoexpanderParameters& _params)
	{
		// --- copy them; note you may choose to ignore certain items
		//     and copy the variables one at a time, or you may test
		//     to see if cook-able variables have changed; if not, then
		//     do not re-cook them as it just wastes CPU
		parameters = _params;

		AudioFilterParameters params = highShelving.getParameters();
		params.boostCut_dB = parameters.highPass;
		highShelving.setParameters(params);

		// --- cook parameters here
	}

private:
	///< object parameters
	stereoexpanderParameters parameters;

	// --- local variables used by this object
	double sampleRate = 0.0;	///< sample rate
	AudioFilter highPassFilter;
	AudioFilter parametric;
	AudioFilter highShelving;
	AudioFilter allPassFilter;

};

#endif