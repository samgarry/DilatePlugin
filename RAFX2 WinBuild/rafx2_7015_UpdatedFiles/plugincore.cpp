// -----------------------------------------------------------------------------
//    ASPiK Plugin Kernel File:  plugincore.cpp
//
/**
    \file   plugincore.cpp
    \author Will Pirkle
    \date   17-September-2018
    \brief  Implementation file for PluginCore object
    		- http://www.aspikplugins.com
    		- http://www.willpirkle.com
*/
// -----------------------------------------------------------------------------
#include "plugincore.h"
#include "plugindescription.h"

/**
\brief PluginCore constructor is launching pad for object initialization

Operations:
- initialize the plugin description (strings, codes, numbers, see initPluginDescriptors())
- setup the plugin's audio I/O channel support
- create the PluginParameter objects that represent the plugin parameters (see FX book if needed)
- create the presets
*/
PluginCore::PluginCore()
{
    // --- describe the plugin; call the helper to init the static parts you setup in plugindescription.h
    initPluginDescriptors();

    // --- default I/O combinations
	// --- for FX plugins
	if (getPluginType() == kFXPlugin)
	{
		addSupportedIOCombination({ kCFMono, kCFMono });
		addSupportedIOCombination({ kCFMono, kCFStereo });
		addSupportedIOCombination({ kCFStereo, kCFStereo });
	}
	else // --- synth plugins have no input, only output
	{
		addSupportedIOCombination({ kCFNone, kCFMono });
		addSupportedIOCombination({ kCFNone, kCFStereo });
	}

	// --- for sidechaining, we support mono and stereo inputs; auxOutputs reserved for future use
	addSupportedAuxIOCombination({ kCFMono, kCFNone });
	addSupportedAuxIOCombination({ kCFStereo, kCFNone });

	// --- create the parameters
    initPluginParameters();

    // --- create the presets
    initPluginPresets();
}

/**
\brief create all of your plugin parameters here

\return true if parameters were created, false if they already existed
*/
bool PluginCore::initPluginParameters()
{
	if (pluginParameterMap.size() > 0)
		return false;

    // --- Add your plugin parameter instantiation code bewtween these hex codes
	// **--0xDEA7--**


	// --- Declaration of Plugin Parameter Objects 
	PluginParameter* piParam = nullptr;

	// --- continuous control: Sum Level
	piParam = new PluginParameter(controlID::center, "Sum Level", "dB", controlVariableType::kDouble, -20.000000, 1.000000, -20.000000, taper::kLinearTaper);
	piParam->setParameterSmoothing(true);
	piParam->setSmoothingTimeMsec(20.00);
	piParam->setBoundVariable(&center, boundVariableType::kDouble);
	addPluginParameter(piParam);

	// --- continuous control: Diff Level
	piParam = new PluginParameter(controlID::sides, "Diff Level", "dB", controlVariableType::kDouble, -20.000000, 16.000000, 5.000000, taper::kLinearTaper);
	piParam->setParameterSmoothing(true);
	piParam->setSmoothingTimeMsec(20.00);
	piParam->setBoundVariable(&sides, boundVariableType::kDouble);
	addPluginParameter(piParam);

	// --- continuous control: High Pass
	piParam = new PluginParameter(controlID::highPass, "High Pass", "Hz", controlVariableType::kDouble, 10.000000, 2000.000000, 10.000000, taper::kVoltOctaveTaper);
	piParam->setParameterSmoothing(true);
	piParam->setSmoothingTimeMsec(20.00);
	piParam->setBoundVariable(&highPass, boundVariableType::kDouble);
	addPluginParameter(piParam);

	// --- continuous control: EQ Frequency
	piParam = new PluginParameter(controlID::eqFreq, "EQ Frequency", "Hz", controlVariableType::kDouble, 25.000000, 1200.000000, 401.000000, taper::kVoltOctaveTaper);
	piParam->setParameterSmoothing(true);
	piParam->setSmoothingTimeMsec(20.00);
	piParam->setBoundVariable(&eqFreq, boundVariableType::kDouble);
	addPluginParameter(piParam);

	// --- continuous control: EQ Gain
	piParam = new PluginParameter(controlID::eqGain, "EQ Gain", "dB", controlVariableType::kDouble, -24.000000, 12.000000, 0.000000, taper::kLinearTaper);
	piParam->setParameterSmoothing(true);
	piParam->setSmoothingTimeMsec(20.00);
	piParam->setBoundVariable(&eqGain, boundVariableType::kDouble);
	addPluginParameter(piParam);

	// --- continuous control: EQ Q
	piParam = new PluginParameter(controlID::eqQ, "EQ Q", "Units", controlVariableType::kDouble, 0.200000, 20.000000, 0.707000, taper::kLinearTaper);
	piParam->setParameterSmoothing(true);
	piParam->setSmoothingTimeMsec(20.00);
	piParam->setBoundVariable(&eqQ, boundVariableType::kDouble);
	addPluginParameter(piParam);

	// --- continuous control: High Shelf Fc
	piParam = new PluginParameter(controlID::hiShelfFreq, "High Shelf Fc", "Hz", controlVariableType::kDouble, 1200.000000, 12000.000000, 2000.000000, taper::kVoltOctaveTaper);
	piParam->setParameterSmoothing(true);
	piParam->setSmoothingTimeMsec(20.00);
	piParam->setBoundVariable(&hiShelfFreq, boundVariableType::kDouble);
	addPluginParameter(piParam);

	// --- continuous control: High Shelf Gain
	piParam = new PluginParameter(controlID::hiShelfGain, "High Shelf Gain", "dB", controlVariableType::kDouble, -24.000000, 24.000000, 0.000000, taper::kLinearTaper);
	piParam->setParameterSmoothing(true);
	piParam->setSmoothingTimeMsec(20.00);
	piParam->setBoundVariable(&hiShelfGain, boundVariableType::kDouble);
	addPluginParameter(piParam);

	// --- continuous control: All Pass Fc
	piParam = new PluginParameter(controlID::allPassFreq, "All Pass Fc", "Hz", controlVariableType::kDouble, 20.000000, 20480.000000, 20480.000000, taper::kVoltOctaveTaper);
	piParam->setParameterSmoothing(true);
	piParam->setSmoothingTimeMsec(20.00);
	piParam->setBoundVariable(&allPassFreq, boundVariableType::kDouble);
	addPluginParameter(piParam);

	// --- continuous control: All Pass Gain
	piParam = new PluginParameter(controlID::allPassGain, "All Pass Gain", "dB", controlVariableType::kDouble, -60.000000, 20.000000, 0.000000, taper::kLinearTaper);
	piParam->setParameterSmoothing(true);
	piParam->setSmoothingTimeMsec(20.00);
	piParam->setBoundVariable(&allPassGain, boundVariableType::kDouble);
	addPluginParameter(piParam);

	// --- discrete control: Enable APF
	piParam = new PluginParameter(controlID::apfSwitch, "Enable APF", "SWITCH OFF,SWITCH ON", "SWITCH OFF");
	piParam->setBoundVariable(&apfSwitch, boundVariableType::kInt);
	piParam->setIsDiscreteSwitch(true);
	addPluginParameter(piParam);

	// --- discrete control: EQ Listen
	piParam = new PluginParameter(controlID::eqSwitch, "EQ Listen", "SWITCH OFF,SWITCH ON", "SWITCH OFF");
	piParam->setBoundVariable(&eqSwitch, boundVariableType::kInt);
	piParam->setIsDiscreteSwitch(true);
	addPluginParameter(piParam);

	// --- continuous control: Output Level
	piParam = new PluginParameter(controlID::outLvl, "Output Level", "dB", controlVariableType::kDouble, -60.000000, -1.000000, -5.000000, taper::kLinearTaper);
	piParam->setParameterSmoothing(true);
	piParam->setSmoothingTimeMsec(20.00);
	piParam->setBoundVariable(&outLvl, boundVariableType::kDouble);
	addPluginParameter(piParam);

	// --- continuous control: High Shelf Q
	piParam = new PluginParameter(controlID::hiShelfQ, "High Shelf Q", "Units", controlVariableType::kDouble, 2.000000, 16.000000, 2.000000, taper::kLinearTaper);
	piParam->setParameterSmoothing(true);
	piParam->setSmoothingTimeMsec(20.00);
	piParam->setBoundVariable(&hiShelfQ, boundVariableType::kDouble);
	addPluginParameter(piParam);

	// --- Aux Attributes
	AuxParameterAttribute auxAttribute;

	// --- RAFX GUI attributes
	// --- controlID::center
	auxAttribute.reset(auxGUIIdentifier::guiControlData);
	auxAttribute.setUintAttribute(2147483676);
	setParamAuxAttribute(controlID::center, auxAttribute);

	// --- controlID::sides
	auxAttribute.reset(auxGUIIdentifier::guiControlData);
	auxAttribute.setUintAttribute(2147483676);
	setParamAuxAttribute(controlID::sides, auxAttribute);

	// --- controlID::highPass
	auxAttribute.reset(auxGUIIdentifier::guiControlData);
	auxAttribute.setUintAttribute(2147483677);
	setParamAuxAttribute(controlID::highPass, auxAttribute);

	// --- controlID::eqFreq
	auxAttribute.reset(auxGUIIdentifier::guiControlData);
	auxAttribute.setUintAttribute(2147483666);
	setParamAuxAttribute(controlID::eqFreq, auxAttribute);

	// --- controlID::eqGain
	auxAttribute.reset(auxGUIIdentifier::guiControlData);
	auxAttribute.setUintAttribute(2147483667);
	setParamAuxAttribute(controlID::eqGain, auxAttribute);

	// --- controlID::eqQ
	auxAttribute.reset(auxGUIIdentifier::guiControlData);
	auxAttribute.setUintAttribute(2147483666);
	setParamAuxAttribute(controlID::eqQ, auxAttribute);

	// --- controlID::hiShelfFreq
	auxAttribute.reset(auxGUIIdentifier::guiControlData);
	auxAttribute.setUintAttribute(2147483675);
	setParamAuxAttribute(controlID::hiShelfFreq, auxAttribute);

	// --- controlID::hiShelfGain
	auxAttribute.reset(auxGUIIdentifier::guiControlData);
	auxAttribute.setUintAttribute(2147483675);
	setParamAuxAttribute(controlID::hiShelfGain, auxAttribute);

	// --- controlID::allPassFreq
	auxAttribute.reset(auxGUIIdentifier::guiControlData);
	auxAttribute.setUintAttribute(2147483666);
	setParamAuxAttribute(controlID::allPassFreq, auxAttribute);

	// --- controlID::allPassGain
	auxAttribute.reset(auxGUIIdentifier::guiControlData);
	auxAttribute.setUintAttribute(2147483666);
	setParamAuxAttribute(controlID::allPassGain, auxAttribute);

	// --- controlID::apfSwitch
	auxAttribute.reset(auxGUIIdentifier::guiControlData);
	auxAttribute.setUintAttribute(1073741825);
	setParamAuxAttribute(controlID::apfSwitch, auxAttribute);

	// --- controlID::eqSwitch
	auxAttribute.reset(auxGUIIdentifier::guiControlData);
	auxAttribute.setUintAttribute(1073741824);
	setParamAuxAttribute(controlID::eqSwitch, auxAttribute);

	// --- controlID::outLvl
	auxAttribute.reset(auxGUIIdentifier::guiControlData);
	auxAttribute.setUintAttribute(2147483670);
	setParamAuxAttribute(controlID::outLvl, auxAttribute);

	// --- controlID::hiShelfQ
	auxAttribute.reset(auxGUIIdentifier::guiControlData);
	auxAttribute.setUintAttribute(2147483674);
	setParamAuxAttribute(controlID::hiShelfQ, auxAttribute);


	// **--0xEDA5--**
   
    // --- BONUS Parameter
    // --- SCALE_GUI_SIZE
    PluginParameter* piParamBonus = new PluginParameter(SCALE_GUI_SIZE, "Scale GUI", "tiny,small,medium,normal,large,giant", "normal");
    addPluginParameter(piParamBonus);

	// --- create the super fast access array
	initPluginParameterArray();

    return true;
}

/**
\brief initialize object for a new run of audio; called just before audio streams

Operation:
- store sample rate and bit depth on audioProcDescriptor - this information is globally available to all core functions
- reset your member objects here

\param resetInfo structure of information about current audio format

\return true if operation succeeds, false otherwise
*/
bool PluginCore::reset(ResetInfo& resetInfo)
{
    // --- save for audio processing
    audioProcDescriptor.sampleRate = resetInfo.sampleRate;
    audioProcDescriptor.bitDepth = resetInfo.bitDepth;

	// --- reset it (HPF)
	AudioFilterParameters hpfParamsOne = highPassFilter[0].getParameters();
	hpfParamsOne.algorithm = filterAlgorithm::kHPF1;
	highPassFilter[0].setParameters(hpfParamsOne);
	highPassFilter[0].reset(resetInfo.sampleRate);

	AudioFilterParameters hpfParamsTwo = highPassFilter[1].getParameters();
	hpfParamsTwo.algorithm = filterAlgorithm::kHPF1;
	highPassFilter[1].setParameters(hpfParamsTwo);
	highPassFilter[1].reset(resetInfo.sampleRate);

	AudioFilterParameters lpfParams = parametric.getParameters();
	lpfParams.algorithm = filterAlgorithm::kCQParaEQ;
	parametric.setParameters(lpfParams);
	parametric.reset(resetInfo.sampleRate);

	AudioFilterParameters hsfParams = highShelving.getParameters();
	hsfParams.algorithm = filterAlgorithm::kHiShelf;
	highShelving.setParameters(hsfParams);
	highShelving.reset(resetInfo.sampleRate);

	AudioFilterParameters apfParams = allPassFilter.getParameters();
	apfParams.algorithm = filterAlgorithm::kAPF1;
	allPassFilter.setParameters(apfParams);
	allPassFilter.reset(resetInfo.sampleRate);



    // --- other reset inits
    return PluginBase::reset(resetInfo);
}

/**
\brief one-time initialize function called after object creation and before the first reset( ) call

Operation:
- saves structure for the plugin to use; you can also load WAV files or state information here
*/
bool PluginCore::initialize(PluginInfo& pluginInfo)
{
	// --- add one-time init stuff here

	return true;
}

/**
\brief do anything needed prior to arrival of audio buffers

Operation:
- syncInBoundVariables when preProcessAudioBuffers is called, it is *guaranteed* that all GUI control change information
  has been applied to plugin parameters; this binds parameter changes to your underlying variables
- NOTE: postUpdatePluginParameter( ) will be called for all bound variables that are acutally updated; if you need to process
  them individually, do so in that function
- use this function to bulk-transfer the bound variable data into your plugin's member object variables

\param processInfo structure of information about *buffer* processing

\return true if operation succeeds, false otherwise
*/
bool PluginCore::preProcessAudioBuffers(ProcessBufferInfo& processInfo)
{
    // --- sync internal variables to GUI parameters; you can also do this manually if you don't
    //     want to use the auto-variable-binding
    syncInBoundVariables();

    return true;
}


void PluginCore::updateParameters()
{

	diffGainCooked = pow(10.0, sides / 20);
	sumGainCooked = pow(10.0, center / 20);
	outputCooked = pow(10.0, outLvl / 20);
	
	AudioFilterParameters hpfParams = highPassFilter[0].getParameters();
	hpfParams.fc = highPass;
	highPassFilter[0].setParameters(hpfParams);

	AudioFilterParameters hpfParamsTwo = highPassFilter[1].getParameters();
	hpfParamsTwo.fc = highPass;
	highPassFilter[1].setParameters(hpfParamsTwo);
	
	
	AudioFilterParameters eqParams = parametric.getParameters();
	eqParams.fc = eqFreq;
	eqParams.Q = eqQ;
	eqParams.boostCut_dB = eqGain;
	parametric.setParameters(eqParams);
	
	
	AudioFilterParameters hsfParams = highShelving.getParameters();
	hsfParams.fc = hiShelfFreq;
	hsfParams.boostCut_dB = hiShelfGain;
	hsfParams.Q = hiShelfQ;
	highShelving.setParameters(hsfParams);
	
	
	AudioFilterParameters apfParams = allPassFilter.getParameters();
	apfParams.fc = allPassFreq;
	apfParams.boostCut_dB = allPassGain;
	allPassFilter.setParameters(apfParams);
}


/**
\brief frame-processing method

Operation:
- decode the plugin type - for synth plugins, fill in the rendering code; for FX plugins, delete the if(synth) portion and add your processing code
- note that MIDI events are fired for each sample interval so that MIDI is tightly sunk with audio
- doSampleAccurateParameterUpdates will perform per-sample interval smoothing

\param processFrameInfo structure of information about *frame* processing

\return true if operation succeeds, false otherwise
*/
bool PluginCore::processAudioFrame(ProcessFrameInfo& processFrameInfo)
{
    // --- fire any MIDI events for this sample interval
    processFrameInfo.midiEventQueue->fireMidiEvents(processFrameInfo.currentFrame);

	// --- do per-frame updates; VST automation and parameter smoothing
	doSampleAccurateParameterUpdates();
	updateParameters();


    // --- FX Plugin:
    if(processFrameInfo.channelIOConfig.inputChannelFormat == kCFMono &&
       processFrameInfo.channelIOConfig.outputChannelFormat == kCFMono)
    {
		// --- pass through code: change this with your signal processing
        processFrameInfo.audioOutputFrame[0] = processFrameInfo.audioInputFrame[0];

        return true; /// processed
    }

    // --- Mono-In/Stereo-Out
    else if(processFrameInfo.channelIOConfig.inputChannelFormat == kCFMono &&
       processFrameInfo.channelIOConfig.outputChannelFormat == kCFStereo)
    {
		// --- pass through code: change this with your signal processing
        processFrameInfo.audioOutputFrame[0] = processFrameInfo.audioInputFrame[0];
        processFrameInfo.audioOutputFrame[1] = processFrameInfo.audioInputFrame[0];

        return true; /// processed
    }

    // --- Stereo-In/Stereo-Out
    else if(processFrameInfo.channelIOConfig.inputChannelFormat == kCFStereo &&
       processFrameInfo.channelIOConfig.outputChannelFormat == kCFStereo)
    {

		double xLn = processFrameInfo.audioInputFrame[0];
		double xRn = processFrameInfo.audioInputFrame[1];
		double yLn = 0.0;
		double yRn = 0.0;
		double leftOutput = 0.0;
		double rightOutput = 0.0;
		double output = 0.0;

		double drySum = xLn + xRn;
		
		double leftHighPass = highPassFilter[0].processAudioSample(xLn);
		double rightHighPass = highPassFilter[1].processAudioSample(xRn);

		double sum = (leftHighPass + rightHighPass) * sumGainCooked;
		double difference = (leftHighPass - rightHighPass) * diffGainCooked;



		if (compareEnumToInt(eqSwitchEnum::SWITCH_OFF, eqSwitch))
		{
			// Parametric
			double diffPara = parametric.processAudioSample(difference);

			//All Pass
			double diffAllPass = allPassFilter.processAudioSample(difference);

			//High Shelf
			double diffHiShelf = highShelving.processAudioSample(difference);

			//Outputs
			// --- Right
			if (compareEnumToInt(apfSwitchEnum::SWITCH_ON, apfSwitch))
			{
				rightOutput = xRn + sum - (diffPara + diffAllPass + diffHiShelf);
			}
			else
				rightOutput = xRn + sum - (diffPara + diffHiShelf);

			// --- Left
			if (compareEnumToInt(apfSwitchEnum::SWITCH_ON, apfSwitch))
			{
				leftOutput = xLn + sum + (diffPara + diffAllPass + diffHiShelf);
			}
			else
				leftOutput = xLn + sum + (diffPara + diffHiShelf);

			processFrameInfo.audioOutputFrame[0] = leftOutput * outputCooked;
			processFrameInfo.audioOutputFrame[1] = rightOutput * outputCooked;
		}
		else
		{
			// Parametric
			double sumPara = parametric.processAudioSample(sum);

			//All Pass
			double sumAllPass = allPassFilter.processAudioSample(sum);

			//High Shelf
			double sumHiShelf = highShelving.processAudioSample(sum);

			//Outputs
			if (compareEnumToInt(apfSwitchEnum::SWITCH_ON, apfSwitch))
			{
				output = xLn + sum + (sumPara + sumAllPass + sumHiShelf);
			}
			else
				output = xLn + sum + (sumPara + sumHiShelf);

			processFrameInfo.audioOutputFrame[0] = output * outputCooked;
			processFrameInfo.audioOutputFrame[1] = output * outputCooked;
		}
		


		// --- pass through code: change this with your signal processing
        // processFrameInfo.audioOutputFrame[0] = processFrameInfo.audioInputFrame[0];
        // processFrameInfo.audioOutputFrame[1] = processFrameInfo.audioInputFrame[1];

        return true; /// processed
    }

    return false; /// NOT processed
}


/**
\brief do anything needed prior to arrival of audio buffers

Operation:
- updateOutBoundVariables sends metering data to the GUI meters

\param processInfo structure of information about *buffer* processing

\return true if operation succeeds, false otherwise
*/
bool PluginCore::postProcessAudioBuffers(ProcessBufferInfo& processInfo)
{
	// --- update outbound variables; currently this is meter data only, but could be extended
	//     in the future
	updateOutBoundVariables();

    return true;
}

/**
\brief update the PluginParameter's value based on GUI control, preset, or data smoothing (thread-safe)

Operation:
- update the parameter's value (with smoothing this initiates another smoothing process)
- call postUpdatePluginParameter to do any further processing

\param controlID the control ID value of the parameter being updated
\param controlValue the new control value
\param paramInfo structure of information about why this value is being udpated (e.g as a result of a preset being loaded vs. the top of a buffer process cycle)

\return true if operation succeeds, false otherwise
*/
bool PluginCore::updatePluginParameter(int32_t controlID, double controlValue, ParameterUpdateInfo& paramInfo)
{
    // --- use base class helper
    setPIParamValue(controlID, controlValue);

    // --- do any post-processing
    postUpdatePluginParameter(controlID, controlValue, paramInfo);

    return true; /// handled
}

/**
\brief update the PluginParameter's value based on *normlaized* GUI control, preset, or data smoothing (thread-safe)

Operation:
- update the parameter's value (with smoothing this initiates another smoothing process)
- call postUpdatePluginParameter to do any further processing

\param controlID the control ID value of the parameter being updated
\param normalizedValue the new control value in normalized form
\param paramInfo structure of information about why this value is being udpated (e.g as a result of a preset being loaded vs. the top of a buffer process cycle)

\return true if operation succeeds, false otherwise
*/
bool PluginCore::updatePluginParameterNormalized(int32_t controlID, double normalizedValue, ParameterUpdateInfo& paramInfo)
{
	// --- use base class helper, returns actual value
	double controlValue = setPIParamValueNormalized(controlID, normalizedValue, paramInfo.applyTaper);

	// --- do any post-processing
	postUpdatePluginParameter(controlID, controlValue, paramInfo);

	return true; /// handled
}

/**
\brief perform any operations after the plugin parameter has been updated; this is one paradigm for
	   transferring control information into vital plugin variables or member objects. If you use this
	   method you can decode the control ID and then do any cooking that is needed. NOTE: do not
	   overwrite bound variables here - this is ONLY for any extra cooking that is required to convert
	   the GUI data to meaninful coefficients or other specific modifiers.

\param controlID the control ID value of the parameter being updated
\param controlValue the new control value
\param paramInfo structure of information about why this value is being udpated (e.g as a result of a preset being loaded vs. the top of a buffer process cycle)

\return true if operation succeeds, false otherwise
*/
bool PluginCore::postUpdatePluginParameter(int32_t controlID, double controlValue, ParameterUpdateInfo& paramInfo)
{
    // --- now do any post update cooking; be careful with VST Sample Accurate automation
    //     If enabled, then make sure the cooking functions are short and efficient otherwise disable it
    //     for the Parameter involved
    /*switch(controlID)
    {
        case 0:
        {
            return true;    /// handled
        }

        default:
            return false;   /// not handled
    }*/

    return false;
}

/**
\brief has nothing to do with actual variable or updated variable (binding)

CAUTION:
- DO NOT update underlying variables here - this is only for sending GUI updates or letting you
  know that a parameter was changed; it should not change the state of your plugin.

WARNING:
- THIS IS NOT THE PREFERRED WAY TO LINK OR COMBINE CONTROLS TOGETHER. THE PROPER METHOD IS
  TO USE A CUSTOM SUB-CONTROLLER THAT IS PART OF THE GUI OBJECT AND CODE.
  SEE http://www.willpirkle.com for more information

\param controlID the control ID value of the parameter being updated
\param actualValue the new control value

\return true if operation succeeds, false otherwise
*/
bool PluginCore::guiParameterChanged(int32_t controlID, double actualValue)
{
	/*
	switch (controlID)
	{
		case controlID::<your control here>
		{

			return true; // handled
		}

		default:
			break;
	}*/

	return false; /// not handled
}

/**
\brief For Custom View and Custom Sub-Controller Operations

NOTES:
- this is for advanced users only to implement custom view and custom sub-controllers
- see the SDK for examples of use

\param messageInfo a structure containing information about the incoming message

\return true if operation succeeds, false otherwise
*/
bool PluginCore::processMessage(MessageInfo& messageInfo)
{
	// --- decode message
	switch (messageInfo.message)
	{
		// --- add customization appearance here
	case PLUGINGUI_DIDOPEN:
	{
		return false;
	}

	// --- NULL pointers so that we don't accidentally use them
	case PLUGINGUI_WILLCLOSE:
	{
		return false;
	}

	// --- update view; this will only be called if the GUI is actually open
	case PLUGINGUI_TIMERPING:
	{
		return false;
	}

	// --- register the custom view, grab the ICustomView interface
	case PLUGINGUI_REGISTER_CUSTOMVIEW:
	{

		return false;
	}

	case PLUGINGUI_REGISTER_SUBCONTROLLER:
	case PLUGINGUI_QUERY_HASUSERCUSTOM:
	case PLUGINGUI_USER_CUSTOMOPEN:
	case PLUGINGUI_USER_CUSTOMCLOSE:
	case PLUGINGUI_EXTERNAL_SET_NORMVALUE:
	case PLUGINGUI_EXTERNAL_SET_ACTUALVALUE:
	{

		return false;
	}

	default:
		break;
	}

	return false; /// not handled
}


/**
\brief process a MIDI event

NOTES:
- MIDI events are 100% sample accurate; this function will be called repeatedly for every MIDI message
- see the SDK for examples of use

\param event a structure containing the MIDI event data

\return true if operation succeeds, false otherwise
*/
bool PluginCore::processMIDIEvent(midiEvent& event)
{
	return true;
}

/**
\brief (for future use)

NOTES:
- MIDI events are 100% sample accurate; this function will be called repeatedly for every MIDI message
- see the SDK for examples of use

\param vectorJoysickData a structure containing joystick data

\return true if operation succeeds, false otherwise
*/
bool PluginCore::setVectorJoystickParameters(const VectorJoystickData& vectorJoysickData)
{
	return true;
}

/**
\brief use this method to add new presets to the list

NOTES:
- see the SDK for examples of use
- for non RackAFX users that have large paramter counts, there is a secret GUI control you
  can enable to write C++ code into text files, one per preset. See the SDK or http://www.willpirkle.com for details

\return true if operation succeeds, false otherwise
*/
bool PluginCore::initPluginPresets()
{
	// **--0xFF7A--**

	// --- Plugin Presets 
	int index = 0;
	PresetInfo* preset = nullptr;

	// --- Preset: Factory Preset
	preset = new PresetInfo(index++, "Factory Preset");
	initPresetParameters(preset->presetParameters);
	setPresetParameter(preset->presetParameters, controlID::center, -20.000000);
	setPresetParameter(preset->presetParameters, controlID::sides, 5.000000);
	setPresetParameter(preset->presetParameters, controlID::highPass, 10.000000);
	setPresetParameter(preset->presetParameters, controlID::eqFreq, 401.000000);
	setPresetParameter(preset->presetParameters, controlID::eqGain, 0.000000);
	setPresetParameter(preset->presetParameters, controlID::eqQ, 0.707000);
	setPresetParameter(preset->presetParameters, controlID::hiShelfFreq, 2000.000000);
	setPresetParameter(preset->presetParameters, controlID::hiShelfGain, 0.000000);
	setPresetParameter(preset->presetParameters, controlID::allPassFreq, 20480.000000);
	setPresetParameter(preset->presetParameters, controlID::allPassGain, 0.000000);
	setPresetParameter(preset->presetParameters, controlID::apfSwitch, -0.000000);
	setPresetParameter(preset->presetParameters, controlID::eqSwitch, -0.000000);
	setPresetParameter(preset->presetParameters, controlID::outLvl, -10.000000);
	setPresetParameter(preset->presetParameters, controlID::hiShelfQ, 2.000000);
	addPreset(preset);


	// **--0xA7FF--**

    return true;
}

/**
\brief setup the plugin description strings, flags and codes; this is ordinarily done through the ASPiKreator or CMake

\return true if operation succeeds, false otherwise
*/
bool PluginCore::initPluginDescriptors()
{
    pluginDescriptor.pluginName = PluginCore::getPluginName();
    pluginDescriptor.shortPluginName = PluginCore::getShortPluginName();
    pluginDescriptor.vendorName = PluginCore::getVendorName();
    pluginDescriptor.pluginTypeCode = PluginCore::getPluginType();

	// --- describe the plugin attributes; set according to your needs
	pluginDescriptor.hasSidechain = kWantSidechain;
	pluginDescriptor.latencyInSamples = kLatencyInSamples;
	pluginDescriptor.tailTimeInMSec = kTailTimeMsec;
	pluginDescriptor.infiniteTailVST3 = kVSTInfiniteTail;

    // --- AAX
    apiSpecificInfo.aaxManufacturerID = kManufacturerID;
    apiSpecificInfo.aaxProductID = kAAXProductID;
    apiSpecificInfo.aaxBundleID = kAAXBundleID;  /* MacOS only: this MUST match the bundle identifier in your info.plist file */
    apiSpecificInfo.aaxEffectID = "aaxDeveloper.";
    apiSpecificInfo.aaxEffectID.append(PluginCore::getPluginName());
    apiSpecificInfo.aaxPluginCategoryCode = kAAXCategory;

    // --- AU
    apiSpecificInfo.auBundleID = kAUBundleID;
	apiSpecificInfo.auBundleName = kAUBundleName;   /* MacOS only: this MUST match the bundle identifier in your info.plist file */
    apiSpecificInfo.auBundleName = kAUBundleName;

    // --- VST3
    apiSpecificInfo.vst3FUID = PluginCore::getVSTFUID(); // OLE string format
    apiSpecificInfo.vst3BundleID = kVST3BundleID;/* MacOS only: this MUST match the bundle identifier in your info.plist file */
	apiSpecificInfo.enableVST3SampleAccurateAutomation = kVSTSAA;
	apiSpecificInfo.vst3SampleAccurateGranularity = kVST3SAAGranularity;

    // --- AU and AAX
    apiSpecificInfo.fourCharCode = PluginCore::getFourCharCode();

    return true;
}

// --- static functions required for VST3/AU only --------------------------------------------- //
const char* PluginCore::getPluginBundleName() { return kAUBundleName; }
const char* PluginCore::getPluginName(){ return kPluginName; }
const char* PluginCore::getShortPluginName(){ return kShortPluginName; }
const char* PluginCore::getVendorName(){ return kVendorName; }
const char* PluginCore::getVendorURL(){ return kVendorURL; }
const char* PluginCore::getVendorEmail(){ return kVendorEmail; }
const char* PluginCore::getAUCocoaViewFactoryName(){ return AU_COCOA_VIEWFACTORY_STRING; }
pluginType PluginCore::getPluginType(){ return kPluginType; }
const char* PluginCore::getVSTFUID(){ return kVSTFUID; }
int32_t PluginCore::getFourCharCode(){ return kFourCharCode; }
