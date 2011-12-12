/*
 
 Original comments from Apple:
 
    File: MixerHostAudio.m
Abstract: Audio object: Handles all audio tasks for the application.
 Version: 1.0

 

*/


#import "MixerHostAudio.h"      

#import "TPCircularBuffer.h"        // ring buffer
#import "SNFCoreAudioUtils.h"       // Chris Adamson's debug print util

// example file for fileplayer
// file needs to be in resources/sounds folder

#define FILE_PLAYER_FILE @"Sounds/dmxbeat"
#define FILE_PLAYER_FILE_TYPE @"aiff"


// preset and supporting audio files for MIDI sampler
// preset file should be in resources
// base aiff file should be in resources/sounds folder

#define AU_SAMPLER_FILE @"lead"
#define AU_SAMPLER_FILE_TYPE @"aif"
#define AU_SAMPLER_PRESET_FILE @"lead"

// function defs for fft code

float MagnitudeSquared(float x, float y);
void ConvertInt16ToFloat(MixerHostAudio *THIS, void *buf, float *outputBuf, size_t capacity);

// function defs for smb fft code

void smbPitchShift(float pitchShift, long numSampsToProcess, long fftFrameSize, long osamp, float sampleRate, float *indata, float *outdata);

void smb2PitchShift(float pitchShift, long numSampsToProcess, long fftFrameSize,
					long osamp, float sampleRate, float *indata, float *outdata,
					FFTSetup fftSetup, float * frequency);


// function defs for mic fx dsp methods used in callbacks


void ringMod( void *inRefCon,  UInt32 inNumberFrames, SInt16 *sampleBuffer );
OSStatus fftPassThrough ( void *inRefCon, UInt32 inNumberFrames, SInt16 *sampleBuffer);
OSStatus fftPitchShift ( void *inRefCon, UInt32 inNumberFrames, SInt16 *sampleBuffer);
OSStatus simpleDelay ( void *inRefCon, UInt32 inNumberFrames, SInt16 *sampleBuffer);
OSStatus movingAverageFilterFloat ( void *inRefCon, UInt32 inNumberFrames, SInt16 *sampleBuffer);
OSStatus logFilter ( void *inRefCon, UInt32 inNumberFrames, SInt16 *sampleBuffer);
OSStatus convolutionFilter ( void *inRefCon, UInt32 inNumberFrames, SInt16 *sampleBuffer);

// function defs for audio processing to support callbacks

void  lowPassWindowedSincFilter( float *buf , float fc );
float xslide(int sval, float x );
float getSynthEnvelope( void * inRefCon );
void fixedPointToSInt16( SInt32 * source, SInt16 * target, int length );
void SInt16ToFixedPoint( SInt16 * source, SInt32 * target, int length );
float getMeanVolumeSint16( SInt16 * vector , int length );

// audio callbacks

static OSStatus inputRenderCallback (void *inRefCon, AudioUnitRenderActionFlags  *ioActionFlags, const AudioTimeStamp *inTimeStamp,  UInt32  inBusNumber,   UInt32  inNumberFrames,  AudioBufferList *ioData );

static OSStatus synthRenderCallback (void *inRefCon, AudioUnitRenderActionFlags  *ioActionFlags, const AudioTimeStamp *inTimeStamp,  UInt32  inBusNumber,   UInt32  inNumberFrames,  AudioBufferList *ioData );

static OSStatus micLineInRenderCallback (void *inRefCon, AudioUnitRenderActionFlags  *ioActionFlags, const AudioTimeStamp *inTimeStamp,  UInt32  inBusNumber,   UInt32  inNumberFrames,  AudioBufferList *ioData );

void audioRouteChangeListenerCallback ( void   *inUserData,  AudioSessionPropertyID  inPropertyID,  UInt32 inPropertyValueSize,  const void  *inPropertyValue );

// midi callbacks

void MyMIDINotifyProc (const MIDINotification  *message, void *refCon);
static void	MyMIDIReadProc(const MIDIPacketList *pktlist, void *refCon, void *connRefCon);


// ring buffer buffer declarations

// SInt16 circular delay buffer (used for echo effect)

SInt16 *delayBuffer;
TPCircularBufferRecord delayBufferRecord;
NSLock *delayBufferRecordLock;
SInt16 *tempDelayBuffer;

// float circular filter buffer declarations (used for filters)

float *circularFilterBuffer;
TPCircularBufferRecord circularFilterBufferRecord;
NSLock *circularFilterBufferRecordLock;
float *tempCircularFilterBuffer;

// end of declarations //


/////////////////////
// callback functions


#pragma mark Mixer input bus 0 & 1 render callback (loops buffers)

//  Callback for guitar and beats loops - mixer channels 0 & 1
//
//  original comments from Apple:

//    This callback is invoked each time a Multichannel Mixer unit input bus requires more audio
//        samples. In this app, the mixer unit has two input buses. Each of them has its own render 
//        callback function and its own interleaved audio data buffer to read from.
//
//    This callback is written for an inRefCon parameter that can point to two noninterleaved 
//        buffers (for a stereo sound) or to one mono buffer (for a mono sound).
//
//    Audio unit input render callbacks are invoked on a realtime priority thread (the highest 
//    priority on the system). To work well, to not make the system unresponsive, and to avoid 
//    audio artifacts, a render callback must not:
//
//        * allocate memory
//        * access the file system or a network connection
//        * take locks
//        * waste time
//
//    In addition, it's usually best to avoid sending Objective-C messages in a render callback.
//
//    Declared as AURenderCallback in AudioUnit/AUComponent.h. See Audio Unit Component Services Reference.
static OSStatus inputRenderCallback (

    void                        *inRefCon,      // A pointer to a struct containing the complete audio data 
                                                //    to play, as well as state information such as the  
                                                //    first sample to play on this invocation of the callback.
    AudioUnitRenderActionFlags  *ioActionFlags, // Unused here. When generating audio, use ioActionFlags to indicate silence 
                                                //    between sounds; for silence, also memset the ioData buffers to 0.
    const AudioTimeStamp        *inTimeStamp,   // Unused here.
    UInt32                      inBusNumber,    // The mixer unit input bus that is requesting some new
                                                //        frames of audio data to play.
    UInt32                      inNumberFrames, // The number of frames of audio to provide to the buffer(s)
                                                //        pointed to by the ioData parameter.
    AudioBufferList             *ioData         // On output, the audio data to play. The callback's primary 
                                                //        responsibility is to fill the buffer(s) in the 
                                                //        AudioBufferList.
) {

    soundStructPtr    soundStructPointerArray   = (soundStructPtr) inRefCon;
    UInt32            frameTotalForSound        = soundStructPointerArray[inBusNumber].frameCount;
    BOOL              isStereo                  = soundStructPointerArray[inBusNumber].isStereo;

    // Declare variables to point to the audio buffers. Their data type must match the buffer data type.
    AudioUnitSampleType *dataInLeft;
    AudioUnitSampleType *dataInRight;
    
    dataInLeft                 = soundStructPointerArray[inBusNumber].audioDataLeft;
    if (isStereo) dataInRight  = soundStructPointerArray[inBusNumber].audioDataRight;
    
    // Establish pointers to the memory into which the audio from the buffers should go. This reflects
    //    the fact that each Multichannel Mixer unit input bus has two channels, as specified by this app's
    //    graphStreamFormat variable.
    AudioUnitSampleType *outSamplesChannelLeft;
    AudioUnitSampleType *outSamplesChannelRight;
    
    outSamplesChannelLeft                 = (AudioUnitSampleType *) ioData->mBuffers[0].mData;
    if (isStereo) outSamplesChannelRight  = (AudioUnitSampleType *) ioData->mBuffers[1].mData;

    // Get the sample number, as an index into the sound stored in memory,
    //    to start reading data from.
    UInt32 sampleNumber = soundStructPointerArray[inBusNumber].sampleNumber;

    // Fill the buffer or buffers pointed at by *ioData with the requested number of samples 
    //    of audio from the sound stored in memory.
    for (UInt32 frameNumber = 0; frameNumber < inNumberFrames; ++frameNumber) {

        outSamplesChannelLeft[frameNumber]                 = dataInLeft[sampleNumber];
        if (isStereo) outSamplesChannelRight[frameNumber]  = dataInRight[sampleNumber];
        
        sampleNumber++;

        // After reaching the end of the sound stored in memory--that is, after
        //    (frameTotalForSound / inNumberFrames) invocations of this callback--loop back to the 
        //    start of the sound so playback resumes from there.
        if (sampleNumber >= frameTotalForSound) sampleNumber = 0;
    }
    
    // Update the stored sample number so, the next time this callback is invoked, playback resumes 
    //    at the correct spot.
    soundStructPointerArray[inBusNumber].sampleNumber = sampleNumber;
    
    return noErr;
}


///////////////////////////////////////////////
//  synth callback - generates a sine wave with
//
//  freq = MixerHost.sinFreq
//  phase = MixerHost.sinPhase
//  note on = MixerHost.synthNoteOn
//
//  its a simple example of a synthesizer sound generator
//

static OSStatus synthRenderCallback (
								   void *							inRefCon,
								   AudioUnitRenderActionFlags *	ioActionFlags,
								   const AudioTimeStamp *			inTimeStamp,
								   UInt32							inBusNumber,
								   UInt32							inNumberFrames,
								   AudioBufferList *				ioData) {
	
    
	MixerHostAudio* THIS = (MixerHostAudio *)inRefCon;	// scope reference that allows access to everything in MixerHostAudio class
    
    
    
    float freq = THIS.sinFreq;      // get frequency data from instance variables
	float phase = THIS.sinPhase;

    
	float sinSignal;                //
    float envelope;                 // scaling factor from envelope generator 0->1
	
	
    //	NSLog(@"inside callback - freq: %f phase: %f", freq, phase );
	
   	
	double phaseIncrement = 2 * M_PI * freq / THIS.graphSampleRate; // phase change per sample
	
	
	AudioSampleType *outSamples;
    outSamples = (AudioSampleType *) ioData->mBuffers[0].mData;
    
   
    
// if a note isn't being triggered just fill the frames with zeroes and bail.
// interesting note: when we didn't zero out the buffer, the microphone was
// somehow activated on the synth channel... weird???
//
// synth note triggering is handled by envelope generator now but I left above comment - to illustrate
// what can happen if your callback doesn't fill its output data buffers
    
/*
    if( noteOn == NO ) {
        memset(outSamples, 0, inNumberFrames * sizeof(SInt16));
        return noErr;
    }

*/
    
// build a sine wave (not a teddy bear)
	
	for (UInt32 frameNumber = 0; frameNumber < inNumberFrames; ++frameNumber) {
		
		
		sinSignal = sin(phase); // if we were using float samples this would be the value
	
		
        // scale to half of maximum volume level for integer samples
        // and use envelope value to determine instantaneous level
        
    
        // envelope = 1.0;
        envelope = getSynthEnvelope( inRefCon );  // envelope ranges from 0->1

        
		outSamples[frameNumber] =  (SInt16) (((sinSignal * 32767.0f) / 2) * envelope);  		
		phase = phase + phaseIncrement; // increment phase
        
        if(phase >= (2 * M_PI * freq)) {         // phase wraps around every cycle
            phase = phase - (2 * M_PI * freq);
        }
		
	}
    
   
	
	THIS.sinPhase = phase;		// save for next time this callback is invoked
	
	return noErr;
	
}	




#pragma mark -
#pragma mark mic, line in Audio Rendering

////////////////////////////////
// callback for mic/lineIn input
//
// 
// this callback is now the clearinghouse for
// DSP fx processing 
//
//

OSStatus micLineInCallback (void					*inRefCon, 
						 AudioUnitRenderActionFlags 	*ioActionFlags, 
						 const AudioTimeStamp			*inTimeStamp, 
						 UInt32 						inBusNumber, 
						 UInt32 						inNumberFrames, 
						 AudioBufferList				*ioData)
{
	
	// set params & local variables
    
    // scope reference that allows access to everything in MixerHostAudio class

	MixerHostAudio *THIS = (MixerHostAudio *)inRefCon;
    
    AudioUnit rioUnit = THIS.ioUnit;    // io unit which has the input data from mic/lineIn
    int i;                              // loop counter
    
	OSStatus err;                       // error returns
	OSStatus renderErr;
	
    UInt32 bus1 = 1;                    // input bus
	
       
    AudioUnitSampleType *inSamplesLeft;         // convenience pointers to sample data
    AudioUnitSampleType *inSamplesRight;
    
    int isStereo;               // c boolean - for deciding how many channels to process.
    int numberOfChannels;       // 1 = mono, 2= stereo
    
    // Sint16 buffers to hold sample data after conversion 
    
    SInt16 *sampleBufferLeft = THIS.conversionBufferLeft;
    SInt16 *sampleBufferRight = THIS.conversionBufferRight;
    SInt16 *sampleBuffer;
    
	
    // start the actual processing 
    
    numberOfChannels = THIS.displayNumberOfInputChannels;
    isStereo = numberOfChannels > 1 ? 1 : 0;  // decide stereo or mono

    //  printf("isStereo: %d\n", isStereo);
    //  NSLog(@"frames: %lu, bus: %lu",inNumberFrames, inBusNumber );
	
	// copy all the input samples to the callback buffer - after this point we could bail and have a pass through
	
    renderErr = AudioUnitRender(rioUnit, ioActionFlags, 
								inTimeStamp, bus1, inNumberFrames, ioData);
	if (renderErr < 0) {
		return renderErr;
	}
    
    // this comment is open to debate:
    // it seems that you can process single channel audio input as SInt16 just fine
    // In fact thats what this program had previously done with mono audio input.
    
    // but you get format errors if you set Sint16 samples in an ASBD with 2 channels
    // So... now to simplify things, we're going to get all input as 8.24 and just 
    // convert it to SInt16 or float for processing
    //
    // There may be some 3 stage conversions here, ie., 8.24->Sint16->float 
    // that could probably obviously be replaced by direct 8.24->float conversion
    // 
    
    // convert to SInt16
    
 
    inSamplesLeft = (AudioUnitSampleType *) ioData->mBuffers[0].mData; // left channel
    fixedPointToSInt16(inSamplesLeft, sampleBufferLeft, inNumberFrames);

    if(isStereo) {
        inSamplesRight = (AudioUnitSampleType *) ioData->mBuffers[1].mData; // right channel
        fixedPointToSInt16(inSamplesRight, sampleBufferRight, inNumberFrames);
    }
        
      
    
    
    // get average input volume level for meter display
    // 
    // (note: there's a vdsp function to do this but it works on float samples
   
   
   
   THIS.displayInputLevelLeft = getMeanVolumeSint16( sampleBufferLeft, inNumberFrames); // assign to instance variable for display
    if(isStereo) {
        THIS.displayInputLevelRight = getMeanVolumeSint16(sampleBufferRight, inNumberFrames); // assign to instance variable for display
      }
 
 
    //     
    //  get user mic/line FX selection 
    //
    //  so... none of these effects except fftPassthrough and delay (echo) are fast enough to
    //  render in stereo at the default sample rate and buffer sizes - on the ipad2
    //  This is kind of sad but I didn't really do any optimization 
    //  and there's a lot of wasteful conversion and duplication going on... so there is hope

    // for now, run the effects in mono

    
    if(THIS.micFxOn == YES) {       // if user toggled on mic fx
        
        if(isStereo) {              // if stereo, combine left and right channels into left
            for( i = 0; i < inNumberFrames; i++ ) {
                sampleBufferLeft[i] = (SInt16) ((.5 * (float) sampleBufferLeft[i]) + (.5 * (float) sampleBufferRight[i]));
            }
        }    
        sampleBuffer = sampleBufferLeft;
        
        // do effect based on user selection
        switch (THIS.micFxType) {
                case 0:
                    ringMod( inRefCon, inNumberFrames, sampleBuffer );
                    break;
                case 1:
                    err = fftPassThrough ( inRefCon, inNumberFrames, sampleBuffer);
                    break;
                case 2:
                    err = fftPitchShift ( inRefCon, inNumberFrames, sampleBuffer);
                    break;
                case 3:
                    err = simpleDelay ( inRefCon, inNumberFrames, sampleBuffer);
                    break;
                case 4:
                    err = movingAverageFilterFloat ( inRefCon, inNumberFrames, sampleBuffer);
                    break;    
                case 5:
                    err = convolutionFilter ( inRefCon, inNumberFrames, sampleBuffer);
                    break;     

                default:
                    break;
            }
        // If stereo, copy left channel (mono) results to right channel 
        if(isStereo) {
            for(i = 0; i < inNumberFrames; i++ ) {
                sampleBufferRight[i] = sampleBufferLeft[i];
            }
        }
    }
    

    // convert back to 8.24 fixed point 
    
    SInt16ToFixedPoint(sampleBufferLeft, inSamplesLeft, inNumberFrames); 
    if(isStereo) {
        SInt16ToFixedPoint(sampleBufferRight, inSamplesRight, inNumberFrames); 
    }

    
    return noErr;	// return with samples in iOdata

        
	
}




#pragma mark -
#pragma mark Audio route change listener callback

// Audio session callback function for responding to audio route changes. If playing back audio and
//   the user unplugs a headset or headphones, or removes the device from a dock connector for hardware  
//   that supports audio playback, this callback detects that and stops playback. 
//
// Refer to AudioSessionPropertyListener in Audio Session Services Reference.
void audioRouteChangeListenerCallback (
   void                      *inUserData,
   AudioSessionPropertyID    inPropertyID,
   UInt32                    inPropertyValueSize,
   const void                *inPropertyValue
) {
    
    // Ensure that this callback was invoked because of an audio route change
    if (inPropertyID != kAudioSessionProperty_AudioRouteChange) return;

    // This callback, being outside the implementation block, needs a reference to the MixerHostAudio
    //   object, which it receives in the inUserData parameter. You provide this reference when
    //   registering this callback (see the call to AudioSessionAddPropertyListener).
    MixerHostAudio *audioObject = (MixerHostAudio *) inUserData;
    
    // if application sound is not playing, there's nothing to do, so return.
    if (NO == audioObject.isPlaying) {

        NSLog (@"Audio route change while application audio is stopped.");
        return;
        
    } else {

        // Determine the specific type of audio route change that occurred.
        CFDictionaryRef routeChangeDictionary = inPropertyValue;
        
        CFNumberRef routeChangeReasonRef =
                        CFDictionaryGetValue (
                            routeChangeDictionary,
                            CFSTR (kAudioSession_AudioRouteChangeKey_Reason)
                        );

        SInt32 routeChangeReason;
        
        CFNumberGetValue (
            routeChangeReasonRef,
            kCFNumberSInt32Type,
            &routeChangeReason
        );
        
        // "Old device unavailable" indicates that a headset or headphones were unplugged, or that 
        //    the device was removed from a dock connector that supports audio output. In such a case,
        //    pause or stop audio (as advised by the iOS Human Interface Guidelines).
        if (routeChangeReason == kAudioSessionRouteChangeReason_OldDeviceUnavailable) {

            NSLog (@"Audio output device was removed; stopping audio playback.");
            NSString *MixerHostAudioObjectPlaybackStateDidChangeNotification = @"MixerHostAudioObjectPlaybackStateDidChangeNotification";
            [[NSNotificationCenter defaultCenter] postNotificationName: MixerHostAudioObjectPlaybackStateDidChangeNotification object: audioObject]; 

        } else {

            NSLog (@"A route change occurred that does not require stopping application audio.");
        }
    }
}


/////////////////
// midi callbacks
//
void MyMIDINotifyProc (const MIDINotification  *message, void *refCon) {
	printf("MIDI Notify, messageId=%ld,", message->messageID);
}

static void	MyMIDIReadProc(const MIDIPacketList *pktlist, void *refCon, void *connRefCon) {
	MixerHostAudio *myVC = (__bridge MixerHostAudio *) refCon;
	
	MIDIPacket *packet = (MIDIPacket *)pktlist->packet;	
	for (int i=0; i < pktlist->numPackets; i++) {
		Byte midiStatus = packet->data[0];
		Byte midiCommand = midiStatus >> 4;
		// is it a note-on or note-off
		if ((midiCommand == 0x09) ||
			(midiCommand == 0x08)) {
			Byte note = packet->data[1] & 0x7F;
			Byte velocity = packet->data[2] & 0x7F;
			// printf("midiCommand=%d. Note=%d, Velocity=%d\n", midiCommand, note, velocity);
			
			// send to augraph
            
            // tz - this is where the sampler gets called
            
            
			CheckError(MusicDeviceMIDIEvent (myVC.auSamplerUnit,
											 midiStatus,
											 note,
											 velocity,
											 0), 
					   "Couldn't send MIDI event");
			
            
        }
		packet = MIDIPacketNext(packet);
	}
}





///////////////////
// end of callbacks


//////////////////////////////////////////////////////////
// functions to support audio processing done in callbacks


///////////////////////////////////////////////////
//
// recursive logarithmic smoothing (low pass filter)

// based on algorithm in Max/MSP slide object
// http://cycling74.com
//
float xslide(int sval, float x ) {
	
	static int firstTime = TRUE;
	static float yP;
	float y;
	
	if(sval <= 0) {
		sval = 1;
	}
	
	if(firstTime) {
		firstTime = FALSE;
		yP = x;
    }
	
	
	y = yP + ((x - yP) / sval);
	
	yP = y;
	
	return(y);
	
	
}



////////////////////////////////////////////////////////////////////////
//
// pitch shifter using stft - based on dsp dimension articles and source
// http://www.dspdimension.com/admin/pitch-shifting-using-the-ft/

OSStatus fftPitchShift (
                        void *inRefCon,                // scope (MixerHostAudio)
                        UInt32 inNumberFrames,        // number of frames in this slice
                        SInt16 *sampleBuffer) {      // frames (sample data)
    
    // scope reference that allows access to everything in MixerHostAudio class
    
	MixerHostAudio *THIS = (MixerHostAudio *)inRefCon;
    
    
  	float *outputBuffer = THIS.outputBuffer;        // sample buffers
	float *analysisBuffer = THIS.analysisBuffer;
    
    
	
	FFTSetup fftSetup = THIS.fftSetup;      // fft setup structures need to support vdsp functions
	
  
	uint32_t stride = 1;                    // interleaving factor for vdsp functions
	int bufferCapacity = THIS.fftBufferCapacity;    // maximum size of fft buffers
    	
    float pitchShift = 1.0;                 // pitch shift factor 1=normal, range is .5->2.0
    long osamp = 4;                         // oversampling factor
    long fftSize = 1024;                    // fft size 
    
	
	float frequency;                        // analysis frequency result
    
       
    //	ConvertInt16ToFloat
    
    vDSP_vflt16((SInt16 *) sampleBuffer, stride, (float *) analysisBuffer, stride, bufferCapacity );
    
    // run the pitch shift
    
    // scale the fx control 0->1 to range of pitchShift .5->2.0
    
    pitchShift = (THIS.micFxControl * 1.5) + .5;
    
    // osamp should be at least 4, but at this time my ipod touch gets very unhappy with 
    // anything greater than 2
    
    osamp = 4;
    fftSize = 1024;		// this seems to work in real time since we are actually doing the fft on smaller windows
    
    smb2PitchShift( pitchShift , (long) inNumberFrames,
                   fftSize,  osamp, (float) THIS.graphSampleRate,
                   (float *) analysisBuffer , (float *) outputBuffer,
                   fftSetup, &frequency);
    
    
    // display detected pitch
   
    
    THIS.displayInputFrequency = (int) frequency;
    
    
    // very very cool effect but lets skip it temporarily    
    //    THIS.sinFreq = THIS.frequency;   // set synth frequency to the pitch detected by microphone
    
    
    
    // now convert from float to Sint16
    
    vDSP_vfixr16((float *) outputBuffer, stride, (SInt16 *) sampleBuffer, stride, bufferCapacity );
    
    
       
    return noErr;
    
    
}






#pragma mark -
#pragma mark fft passthrough function

// called by audio callback function with a slice of sample frames
//
// note this is nearly identical to the code example in apple developer lib at
// http://developer.apple.com/library/ios/#documentation/Performance/Conceptual/vDSP_Programming_Guide/SampleCode/SampleCode.html%23//apple_ref/doc/uid/TP40005147-CH205-CIAEJIGF
//
// this code does a passthrough from mic input to mixer bus using forward and inverse fft
// it also analyzes frequency with the freq domain data
//-------------------------------------------------------------

OSStatus fftPassThrough (   void                        *inRefCon,          // scope referece for external data
                         UInt32 						inNumberFrames,     // number of frames to process
                         SInt16 *sampleBuffer)                           // frame buffer
{
	
    // note: the fx control slider does nothing during fft passthrough
    
    // set all the params
    
    // scope reference that allows access to everything in MixerHostAudio class
    
	MixerHostAudio *THIS = (MixerHostAudio *)inRefCon;
    
    COMPLEX_SPLIT A = THIS.fftA;                // complex buffers
	
	void *dataBuffer = THIS.dataBuffer;         // working sample buffers
	float *outputBuffer = THIS.outputBuffer;
	float *analysisBuffer = THIS.analysisBuffer;
	
	FFTSetup fftSetup = THIS.fftSetup;          // fft structure to support vdsp functions
        
    // fft params
    
	uint32_t log2n = THIS.fftLog2n;             
	uint32_t n = THIS.fftN;
	uint32_t nOver2 = THIS.fftNOver2;
	uint32_t stride = 1;
	int bufferCapacity = THIS.fftBufferCapacity;
	SInt16 index = THIS.fftIndex;
	
    
    
	// this next logic assumes that the bufferCapacity determined by maxFrames in the fft-setup is less than or equal to
	// the inNumberFrames (which should be determined by the av session IO buffer size (ie duration)
	//
	// If we can guarantee the fft buffer size is equal to the inNumberFrames, then this buffer filling step is unecessary
	//
	// at this point i think its essential to make the two buffers equal size in order to do the fft passthrough without doing
	// the overlapping buffer thing
	//
	
    
	// Fill the buffer with our sampled data. If we fill our buffer, run the
	// fft.
	
	// so I have a question - the fft buffer  needs to be an even multiple of the frame (packet size?) or what?
    
	// NSLog(@"index: %d", index);
	int read = bufferCapacity - index;
	if (read > inNumberFrames) {
		// NSLog(@"filling");
        
		memcpy((SInt16 *)dataBuffer + index, sampleBuffer, inNumberFrames * sizeof(SInt16));
		THIS.fftIndex += inNumberFrames;
	} else {
		// NSLog(@"processing");
		// If we enter this conditional, our buffer will be filled and we should 
		// perform the FFT.
        
		memcpy((SInt16 *)dataBuffer + index, sampleBuffer, read * sizeof(SInt16));
        
		
		// Reset the index.
		THIS.fftIndex = 0;
        
        
        // *************** FFT ***************		
        // convert Sint16 to floating point
        
        vDSP_vflt16((SInt16 *) dataBuffer, stride, (float *) outputBuffer, stride, bufferCapacity );
        
        
		//
		// Look at the real signal as an interleaved complex vector by casting it.
		// Then call the transformation function vDSP_ctoz to get a split complex 
		// vector, which for a real signal, divides into an even-odd configuration.
		//
        
        vDSP_ctoz((COMPLEX*)outputBuffer, 2, &A, 1, nOver2);
		
		// Carry out a Forward FFT transform.
        
        vDSP_fft_zrip(fftSetup, &A, stride, log2n, FFT_FORWARD);
		
			
		// The output signal is now in a split real form. Use the vDSP_ztoc to get
		// an interleaved complex vector.
        
        vDSP_ztoc(&A, 1, (COMPLEX *)analysisBuffer, 2, nOver2);
		
		// for display purposes...
        //
        // Determine the dominant frequency by taking the magnitude squared and 
		// saving the bin which it resides in. This isn't precise and doesn't
        // necessary get the "fundamental" frequency, but its quick and sort of works...
        
        // note there are vdsp functions to do the amplitude calcs
        
        float dominantFrequency = 0;
        int bin = -1;
        for (int i=0; i<n; i+=2) {
			float curFreq = MagnitudeSquared(analysisBuffer[i], analysisBuffer[i+1]);
			if (curFreq > dominantFrequency) {
				dominantFrequency = curFreq;
				bin = (i+1)/2;
			}
		}
        
        dominantFrequency = bin*(THIS.graphSampleRate/bufferCapacity);
        
        // printf("Dominant frequency: %f   \n" , dominantFrequency);
        THIS.displayInputFrequency = (int) dominantFrequency;   // set instance variable with detected frequency
		
        
        // Carry out an inverse FFT transform.
		
        vDSP_fft_zrip(fftSetup, &A, stride, log2n, FFT_INVERSE );
        
        // scale it
		
		float scale = (float) 1.0 / (2 * n);					
		vDSP_vsmul(A.realp, 1, &scale, A.realp, 1, nOver2 );
		vDSP_vsmul(A.imagp, 1, &scale, A.imagp, 1, nOver2 );
		
		
        // convert from split complex to interleaved complex form
		
		vDSP_ztoc(&A, 1, (COMPLEX *) outputBuffer, 2, nOver2);
		
        // now convert from float to Sint16
		
		vDSP_vfixr16((float *) outputBuffer, stride, (SInt16 *) sampleBuffer, stride, bufferCapacity );
        
        
		
        
	}
    
    
    return noErr;
    
    
	
    
}



/////////////////////////////////////////////
// ring modulator effect - for SInt16 samples
//
// called from callback function that passes in a slice of frames
//
void ringMod( 
             void *inRefCon,                // scope (MixerHostAudio)
             
             UInt32 inNumberFrames,        // number of frames in this slice
             SInt16 *sampleBuffer) {      // frames (sample data)
    
    //  scope reference that allows access to everything in MixerHostAudio class 
    
    MixerHostAudio* THIS = (MixerHostAudio *)inRefCon;	    
    
    UInt32 frameNumber;     // current frame number for looping 
    float theta;            // for frequency calculation
    static float phase = 0; // for frequency calculation
    float freq;             // etc.,
    AudioSampleType *outSamples;    // convenience pointer to result samples
	
    outSamples  = (AudioSampleType *) sampleBuffer; // pointer to samples
    
    freq = (THIS.micFxControl * 4000) + .00001; // get freq from fx control slider
    // .00001 prevents divide by 0
    
    // loop through the samples
    
    for (frameNumber = 0; frameNumber < inNumberFrames; ++frameNumber) {
		
        theta = phase * M_PI * 2;   // convert to radians 
        outSamples[frameNumber] = (AudioSampleType) (sin(theta) * outSamples[frameNumber]);
        
        phase += 1.0 / (THIS.graphSampleRate / freq);	// increment phase
        if (phase > 1.0) {                              // phase goes from 0 -> 1
            phase -= 1.0;
        }
        
    }
    
    
    
}

// simple AR envelope generator for synth note
//
// for now, attack and release value params hardcoded in this function
//

#define ENV_OFF 0
#define ENV_ATTACK 1
#define ENV_RELEASE 2

float getSynthEnvelope( void * inRefCon ) {
    
    MixerHostAudio* THIS = (MixerHostAudio *)inRefCon;  // access to mixerHostAudio scope
    
    static int state = ENV_OFF;           // current state
    static int keyPressed = 0;            // current(previous) state of key
    static float envelope = 0.0;          // current envelope value 0->1
    
    float attack = 1000.0;                     // attack time in samples
    float release = 40000.0;                   // release time in samples
    
    float attackStep;                       // amount to increment each sample during attack phase
    float releaseStep;                      // amount to decrement each sample during release phase
    
    int newKeyState;                       // new on/off state of key
    
    // start 
    
    attackStep = 1.0 / attack;              // calculate attack and release steps
    releaseStep = 1.0 / release;
    
    newKeyState = THIS.synthNoteOn == YES ? 1 : 0; 
    
    // printf("envelope: %f, state: %d, keyPressed: %d, newKeyState: %d\n", envelope, state, keyPressed, newKeyState);
    
    if(keyPressed == 0) {       // key has been up
        if(newKeyState == 0) {      //  if key is still up
            switch(state) 
            {
                case ENV_RELEASE:
                    // printf("dec: env: %f, rs: %f\n", envelope, releaseStep );
                    envelope -= releaseStep;
                    if(envelope <= 0.) {
                        envelope = 0.0;
                        state = ENV_OFF;
                    }
                    break;
                default:
                    state = ENV_OFF;    // this should already be the case
                    envelope = 0.0;     
                    break;
            }       
        }
        else {  // key was just pressed
            keyPressed = 1;                 // save new key state
            state = ENV_ATTACK;             // change state to attack
            
        }
    }    
    else {  // key has been down
        
        if(newKeyState == 0) {      // if key was just released
            keyPressed = 0;         // save new key state
            state = ENV_RELEASE;
            
        }
        else {                  // key is still down
            switch(state) 
            {
                    
                case ENV_ATTACK: 
                    // printf("inc: env: %f, as: %f\n", envelope, attackStep );    
                    envelope += attackStep;
                    if (envelope >= 1.0) {
                        envelope = 1.0;
                    }
                    break;
                    
                default:
                    state = ENV_ATTACK;         // this should already be the case
                    break;
            }    
        }    
        
    }
    
    
    return (envelope);
}





////////////////////////////////////////////
//
// simple (one tap) delay using ring buffer  
//
// called by callback with a slice of sample data in ioData
//
OSStatus simpleDelay (
                      void                          *inRefCon,              // scope reference
                      UInt32 						inNumberFrames,         // number of frames to process
                      SInt16 *sampleBuffer)                                 // frame data
{
	
	// set all the params
	
	MixerHostAudio *THIS = (MixerHostAudio *)inRefCon;	// scope reference that allows access to everything in MixerHostAudio class
    
    UInt32 i;                                       // loop counter
    //    UInt32 averageVolume = 0;                           // for tracking microphone level
    
    
    
    int32_t tail;           // tail of ring buffer (read pointer)
    // int32_t head;       // head of ring buffer (write pointer)
    SInt16 *targetBuffer, *sourceBuffer;   // convenience pointers to sample data
    
    
    SInt16 *buffer;         // 
    int sampleCount = 0;                    // number of samples processed in ring buffer
    int samplesToCopy = inNumberFrames;     // total number of samples to process
    int32_t length;                         // length of ring buffer
    int32_t delayLength;                    // size of delay in samples
    int delaySlices;    // number of slices to delay by
	
	
	// Put audio into circular delay buffer
    
    // write incoming samples into the ring at the current head position
    // head is incremented by inNumberFrames
    
    
    // The logic is a bit different than usual circular buffer because we don't care 
    // whether the head catches up to the tail - because we're going to manually
    // set the tail position based on the delay length each time this function gets
    // called. 
    
    samplesToCopy = inNumberFrames;
    
    sourceBuffer = sampleBuffer;
    length = TPCircularBufferLength(&delayBufferRecord);
    // printf("length: %d\n", length );
    
    //        [delayBufferRecordLock lock];     // skip locks 
    while(samplesToCopy > 0) {
        sampleCount =  MIN(samplesToCopy, length - TPCircularBufferHead(&delayBufferRecord));
        if(sampleCount == 0) {
            break;
        }
        buffer = delayBuffer + TPCircularBufferHead(&delayBufferRecord);
        memcpy( buffer, sourceBuffer, sampleCount*sizeof(SInt16)); // actual copy
        sourceBuffer += sampleCount;
        samplesToCopy -= sampleCount;
        TPCircularBufferProduceAnywhere(&delayBufferRecord, sampleCount);  // this increments head
    }
    
    // head = TPCircularBufferHead(&delayBufferRecord);
    // printf("new head is %d\n", head );
    
    
    //    [THIS.delayBufferRecordLock unlock];  // skip lock because processing is local
    
    
    
    
    // Now we need to calculate where to put the tail - note this will probably blow
    // up if you don't make the circular buffer big enough for the delay
    
    delaySlices = (int) (THIS.micFxControl * 80);
    
    delayLength = delaySlices * inNumberFrames;      // number of slices do delay by
    // printf("delayLength: %d\n", delayLength);
    tail = TPCircularBufferHead(&delayBufferRecord) - delayLength;
    if(tail < 0) {
        tail = length + tail;
    }
    
    
    // printf("new tail is %d", tail );
    
    TPCircularBufferSetTailAnywhere(&delayBufferRecord, tail);
    
    
    targetBuffer = tempDelayBuffer; // tail data will get copied into temporary buffer
    samplesToCopy = inNumberFrames;
    
    
    
    // Pull audio from playthrough buffer, in contiguous chunks
    
    //        [delayBufferRecordLock lock];     // skip locks
    
    // this is the tricky part of the ring buffer where we need to break the circular
    // illusion and do linear housekeeping. If we're within 1024 of the physical
    // end of buffer, then copy out the samples in 2 steps.
    
    while ( samplesToCopy > 0 ) {
        sampleCount = MIN(samplesToCopy, length - TPCircularBufferTail(&delayBufferRecord));
        if ( sampleCount == 0 ) {
            break;   
        }
        // set pointer based on location of the tail
        
        buffer = delayBuffer + TPCircularBufferTail(&delayBufferRecord);
        
        // printf("\ncopying %d to temp, head: %d, tail %d", sampleCount, head, tail );
        
        memcpy(targetBuffer, buffer, sampleCount*sizeof(SInt16)); // actual copy
        
        targetBuffer += sampleCount;    // move up target pointer
        samplesToCopy -= sampleCount;   // keep track of what's already written
        TPCircularBufferConsumeAnywhere(&delayBufferRecord, sampleCount);  // this increments tail
    }
    
    //        [THIS.delayBufferRecordLock unlock];      // skip locks
    
    
    
   
    
    // convenience pointers for looping
    
    AudioSampleType *outSamples;
    outSamples = (AudioSampleType *) sampleBuffer;
    
   
    
    // this is just a debug test to see if anything is in the delay buffer  
    // by calculating mean volume of the buffer
    // and displaying it to the screen
    
    // for ( i = 0; i < inNumberFrames ; i++ ) {
    //    averageVolume += abs((int) tempDelayBuffer[i]);
    // }
    //    THIS.micLevel = averageVolume / inNumberFrames; 
    //    printf("\naverageVolume = %lu", averageVolume);
    
 
     // mix the delay buffer with the input buffer
    
    // so here the ratio is .4 * input signal
    // and .6 * delayed signal
    
    for ( i = 0; i < inNumberFrames ; i++ ) {
        outSamples[i] = (.4 * outSamples[i]) + (.6 * tempDelayBuffer[i]);
    }
    
    
  	return noErr;  
    
    
    
    
}

//////////////////////////////////////////
// logarithmic smoothing (low pass) filter
//
// based on algorithm in Max/MSP slide object
// http://cycling74.com
//
// called by callback with sample data in ioData
//
OSStatus logFilter (
                    void                          *inRefCon,        // scope reference
                    UInt32 						inNumberFrames,     // number of frames to process
                    SInt16 *sampleBuffer)                           // frame data
{
    
    // set params
	
    // scope reference that allows access to everything in MixerHostAudio class
    
    MixerHostAudio *THIS = (MixerHostAudio *)inRefCon;	    
    
    
    int i;     // loop counter
    SInt16 *buffer;
    int slide;  // smoothing factor (1 = no smoothing)
    
    // map fx control slider 0->1 to 1->15 for slide range
    
    slide = (int) (THIS.micFxControl * 14) + 1;
    
    buffer = sampleBuffer;
    
    // logarihmic filter
    
    for(i = 0 ; i < inNumberFrames; i++ ) {
        sampleBuffer[i] = (SInt16) xslide( slide, (float) buffer[i]);
    }
    
    return noErr;
    
}





//////////////////////////////////////////
//
// recursive Moving Average filter (float)  

// from http://www.dspguide.com/
// table 15-2
//
// called by callback with a slice of sample data in ioData
//
// note - the integer version didn't work 
// but this version works fine
// integer version causes clipping regardless of length
//
OSStatus movingAverageFilterFloat (
                                   void                          *inRefCon,         // scope reference
                                   UInt32 						inNumberFrames,     // number of frames to process
                                   SInt16 *sampleBuffer)                            // frame data
{
	
	// set all the params
	
    MixerHostAudio *THIS = (MixerHostAudio *)inRefCon;	// scope reference that allows access to everything in MixerHostAudio class
    
    int i;                                       // loop counter
    //    UInt32 averageVolume = 0;                           // for tracking microphone level
    
    float *analysisBuffer = THIS.analysisBuffer;                // working sample data buffers
    size_t bufferCapacity = THIS.fftBufferCapacity;
    
    
    int32_t tail;               // tail of ring buffer (read pointer)
   
    float *targetBuffer, *sourceBuffer;         // convenience points for sample data
    
    float *buffer;                      //
    int sampleCount = 0;                // number of samples read in while processing ring buffer
    int samplesToCopy = inNumberFrames; // total number samples to process in ring buffer
    int32_t length;                     // length of ring buffer
    int32_t delayLength;                   //  
   
    int filterLength;                   // size of filter (in samples)
    int middle;                         // middle of filter
	
    float acc;   // accumulator for moving average calculation
    float *resultBuffer;   // output
    
    int stride = 1;             // interleaving factor for sample data for vdsp functions
    
    // convenience pointers for looping
    
    float *signalBuffer;            //
    
    // on first pass, move the head up far enough into the ring buffer so we 
    // have enough zero padding to process the incoming signal data
    
    
    
    // set filter size from mix fx control
    
    filterLength = (int) (THIS.micFxControl * 30) + 3;
    if((filterLength % 2) == 0) {   // if even
        filterLength += 1;          // make it odd
    }
    
    // printf("filterLength %d\n", filterLength );
    
    //    filterLength = 51;
    middle = (filterLength - 1) / 2;
    
    
    // convert vector to float 
    
    //	ConvertInt16ToFloat
    
    vDSP_vflt16((SInt16 *) sampleBuffer, stride, (float *) analysisBuffer, stride, bufferCapacity );
    
    // Put audio into circular delay buffer
    
    // write incoming samples into the ring at the current head position
    // head is incremented by inNumberFrames
    
    
    // The logic is a bit different than usual circular buffer because we don't care 
    // whether the head catches up to the tail - because we're doing all the processing
    // within this function. So tail position gets reset manually each time.
    
    samplesToCopy = inNumberFrames;
    
    sourceBuffer = analysisBuffer;
    length = TPCircularBufferLength(&circularFilterBufferRecord);
    // printf("length: %d\n", length );
    
    //        [delayBufferRecordLock lock];     // skip locks 
    while(samplesToCopy > 0) {
        sampleCount =  MIN(samplesToCopy, length - TPCircularBufferHead(&circularFilterBufferRecord));
        if(sampleCount == 0) {
            break;
        }
        buffer = circularFilterBuffer + TPCircularBufferHead(&circularFilterBufferRecord);
        memcpy( buffer, sourceBuffer, sampleCount*sizeof(float)); // actual copy
        sourceBuffer += sampleCount;
        samplesToCopy -= sampleCount;
        TPCircularBufferProduceAnywhere(&circularFilterBufferRecord, sampleCount);  // this increments head
    }
    
    // head = TPCircularBufferHead(&delayBufferRecord);
    // printf("new head is %d\n", head );
    
    
    //    [THIS.delayBufferRecordLock unlock];  // skip lock because processing is local
    
    
    // Now we need to calculate where to put the tail - note this will probably blow
    // up if you don't make the circular buffer big enough for the delay
    
    // delaySlices = (int) (THIS.micFxControl * 80);
    
    delayLength = (inNumberFrames + filterLength) - 1; 
    
    
    // printf("delayLength: %d\n", delayLength);
    tail = TPCircularBufferHead(&circularFilterBufferRecord) - delayLength;
    if(tail < 0) {
        tail = length + tail;
    }
    
    
    // printf("new tail is %d", tail );
    
    TPCircularBufferSetTailAnywhere(&circularFilterBufferRecord, tail);
    
    
    targetBuffer = tempCircularFilterBuffer; // tail data will get copied into temporary buffer
    samplesToCopy = delayLength;
    
    
    
    // Pull audio from playthrough buffer, in contiguous chunks
    
    //        [delayBufferRecordLock lock];     // skip locks
    
    // this is the tricky part of the ring buffer where we need to break the circular
    // illusion and do linear housekeeping. If we're within 1024 of the physical
    // end of buffer, then copy out the samples in 2 steps.
    
    while ( samplesToCopy > 0 ) {
        sampleCount = MIN(samplesToCopy, length - TPCircularBufferTail(&circularFilterBufferRecord));
        if ( sampleCount == 0 ) {
            break;   
        }
        // set pointer based on location of the tail
        
        buffer = circularFilterBuffer + TPCircularBufferTail(&circularFilterBufferRecord);
        
        // printf("\ncopying %d to temp, head: %d, tail %d", sampleCount, head, tail );
        
        memcpy(targetBuffer, buffer, sampleCount*sizeof(float)); // actual copy
        
        targetBuffer += sampleCount;    // move up target pointer
        samplesToCopy -= sampleCount;   // keep track of what's already written
        TPCircularBufferConsumeAnywhere(&circularFilterBufferRecord, sampleCount);  // this increments tail
    }
    
    //        [THIS.delayBufferRecordLock unlock];      // skip locks
    
    
    // ok now we have enough samples in the temp delay buffer to actually run the 
    // filter. For example, if slice size is 1024 and filterLength is 101 - then we
    // should have 1124 samples in the tempDelayBuffer
    
    
    signalBuffer = tempCircularFilterBuffer;
    resultBuffer = THIS.outputBuffer;
    
 
    
    acc = 0;  // accumulator - find y[50] by averaging points x[0] to x[100]
    
    for(i = 0; i < filterLength; i++ ) {
        acc += signalBuffer[i];
    }
    
    
    resultBuffer[0] = (float) acc / filterLength;
    
    // recursive moving average filter
    
    middle = (filterLength - 1) / 2;
    
    
    for ( i = middle + 1; i < (inNumberFrames + middle) ; i++ ) {
        acc = acc + signalBuffer[i + middle] - signalBuffer[i - (middle + 1)];
        resultBuffer[i - middle] = (float) acc / filterLength;
    }
    
    //    printf("last i-middle is: %d\n", i - middle);
    
    // now convert from float to Sint16
    
    vDSP_vfixr16((float *) resultBuffer, stride, (SInt16 *) sampleBuffer, stride, bufferCapacity );
    
    
    
    return noErr;  
    
    
}




//////////////////////////////////////////////////////////////////////
//
// 101 point windowed sinc lowpass filter from http://www.dspguide.com/
// table 16-1
//
void  lowPassWindowedSincFilter( float *buf , float fc ) {
    
    // re-calculate 101 point lowpass filter kernel    
    
    int i;
    int m = 100;
    float sum = 0;
    
    
    for( i = 0; i < 101 ; i++ ) {
        if((i - m / 2) == 0 ) {
            buf[i] = 2 * M_PI * fc;
        }
        else {
            buf[i] = sin(2 * M_PI * fc * (i - m / 2)) / (i - m / 2);
        }
        buf[i] = buf[i] * (.54 - .46 * cos(2 * M_PI * i / m ));
    }
    
    // normalize for unity gain at dc
    
    
    for ( i = 0 ; i < 101 ; i++ ) {
        sum = sum + buf[i]; 
    }
    
    for ( i = 0 ; i < 101 ; i++ ) {
        buf[i] = buf[i] / sum;
    }
    
}







//////////////////////////////////////
//
// Convoluation Filter example (float)  
//
// called by callback with a slice of sample data in ioData
//
OSStatus convolutionFilter (
                            void                          *inRefCon,        // scope reference
                            UInt32 						inNumberFrames,     // number of frames to process
                            SInt16 *sampleBuffer)                           // frame data
{
	
	// set all the params
	
    MixerHostAudio *THIS = (MixerHostAudio *)inRefCon;	// scope reference that allows access to everything in MixerHostAudio class
    
    //    int i;                                       // loop counter
    //    UInt32 averageVolume = 0;                           // for tracking microphone level
    
    float *analysisBuffer = THIS.analysisBuffer;            // working data buffers
    size_t bufferCapacity = THIS.fftBufferCapacity;
    
    
    int32_t tail;    // tail of ring buffer (read pointer)
    //    int32_t head;       // head of ring buffer (write pointer)
    float *targetBuffer, *sourceBuffer;   
    //    static BOOL firstTime = YES;        // flag for some buffer initialization
    
    float *buffer;
    int sampleCount = 0;
    int samplesToCopy = inNumberFrames;
    int32_t length;
    int32_t delayLength;    
    //    int delaySlices;    // number of slices to delay by
    //    int filterLength;
    //    int middle;
	
    //    float acc;   // accumulator for moving average calculation
    //    float *resultBuffer;   // output
    
    int stride = 1;
    // convolution stuff
	
    
   	float *filterBuffer = THIS.filterBuffer;        // impusle response buffer
    int filterLength = THIS.filterLength;           // length of filterBuffer
    float *signalBuffer = THIS.signalBuffer;        // signal buffer
    //    int signalLength = THIS.signalLength;           // signal length
    float *resultBuffer = THIS.resultBuffer;        // result buffer
    int resultLength = THIS.resultLength;           // result length
  
	
	int filterStride = -1;           // -1 = convolution, 1 = correlation
    float fc;   // cutoff frequency
    
    resultLength = 1024;
    filterLength = 101;
    
    
    // get mix fx control for cutoff freq (fc)
    
    fc = (THIS.micFxControl * .18) + .001;
    
    // make filter with this fc
    
    lowPassWindowedSincFilter( filterBuffer, fc);
    
    //	Convert input signal from Int16ToFloat
    
    vDSP_vflt16((SInt16 *) sampleBuffer, stride, (float *) analysisBuffer, stride, bufferCapacity );
    
    // Put audio into circular delay buffer
    
    // write incoming samples into the ring at the current head position
    // head is incremented by inNumberFrames
    
    
    // The logic is a bit different than usual circular buffer because we don't care 
    // whether the head catches up to the tail - because we're doing all the processing
    // within this function. So tail position gets reset manually each time.
    
    samplesToCopy = inNumberFrames;
    
    sourceBuffer = analysisBuffer;
    length = TPCircularBufferLength(&circularFilterBufferRecord);
    // printf("length: %d\n", length );
    
    //        [delayBufferRecordLock lock];     // skip locks 
    while(samplesToCopy > 0) {
        sampleCount =  MIN(samplesToCopy, length - TPCircularBufferHead(&circularFilterBufferRecord));
        if(sampleCount == 0) {
            break;
        }
        buffer = circularFilterBuffer + TPCircularBufferHead(&circularFilterBufferRecord);
        memcpy( buffer, sourceBuffer, sampleCount*sizeof(float)); // actual copy
        sourceBuffer += sampleCount;
        samplesToCopy -= sampleCount;
        TPCircularBufferProduceAnywhere(&circularFilterBufferRecord, sampleCount);  // this increments head
    }
    
    // head = TPCircularBufferHead(&delayBufferRecord);
    // printf("new head is %d\n", head );
    
    
    //    [THIS.delayBufferRecordLock unlock];  // skip lock because processing is local
    
    
    // Now we need to calculate where to put the tail - note this will probably blow
    // up if you don't make the circular buffer big enough for the delay
    
    // delaySlices = (int) (THIS.micFxControl * 80);
    
    delayLength = (inNumberFrames + filterLength) - 1; 
    
    
    // printf("delayLength: %d\n", delayLength);
    tail = TPCircularBufferHead(&circularFilterBufferRecord) - delayLength;
    if(tail < 0) {
        tail = length + tail;
    }
    
    
    // printf("new tail is %d", tail );
    
    TPCircularBufferSetTailAnywhere(&circularFilterBufferRecord, tail);
    
    
    //   targetBuffer = tempCircularFilterBuffer; // tail data will get copied into temporary buffer
    
    targetBuffer = signalBuffer; // tail data will get copied into temporary buffer
    
    
    samplesToCopy = delayLength;
    
    
    
    // Pull audio from playthrough buffer, in contiguous chunks
    
    //        [delayBufferRecordLock lock];     // skip locks
    
    // this is the tricky part of the ring buffer where we need to break the circular
    // illusion and do linear housekeeping. If we're within 1024 of the physical
    // end of buffer, then copy out the samples in 2 steps.
    
    while ( samplesToCopy > 0 ) {
        sampleCount = MIN(samplesToCopy, length - TPCircularBufferTail(&circularFilterBufferRecord));
        if ( sampleCount == 0 ) {
            break;   
        }
        // set pointer based on location of the tail
        
        buffer = circularFilterBuffer + TPCircularBufferTail(&circularFilterBufferRecord);
        
        // printf("\ncopying %d to temp, head: %d, tail %d", sampleCount, head, tail );
        
        memcpy(targetBuffer, buffer, sampleCount*sizeof(float)); // actual copy
        
        targetBuffer += sampleCount;    // move up target pointer
        samplesToCopy -= sampleCount;   // keep track of what's already written
        TPCircularBufferConsumeAnywhere(&circularFilterBufferRecord, sampleCount);  // this increments tail
    }
    
    //        [THIS.delayBufferRecordLock unlock];      // skip locks
    
    
    // ok now we have enough samples in the temp delay buffer to actually run the 
    // filter. For example, if slice size is 1024 and filterLength is 101 - then we
    // should have 1124 samples in the tempDelayBuffer
    
     
    // do convolution
    
    filterStride = -1;      // convolution
    vDSP_conv( signalBuffer, stride, filterBuffer + filterLength - 1, filterStride, resultBuffer, stride,  resultLength, filterLength ); 
    
    
      
    // now convert from float to Sint16
    
    vDSP_vfixr16((float *) resultBuffer, stride, (SInt16 *) sampleBuffer, stride, bufferCapacity );
    
    
    
    return noErr;  
    
    
    
}





////////////////////////////////////////////////////////
// convert sample vector from fixed point 8.24 to SInt16
void fixedPointToSInt16( SInt32 * source, SInt16 * target, int length ) {
    
    int i;
    
    for(i = 0;i < length; i++ ) {
        target[i] =  (SInt16) (source[i] >> 9);
        
    }
    
}

////////////////////////////////////////////////////////
// convert sample vector from SInt16 to fixed point 8.24 
void SInt16ToFixedPoint( SInt16 * source, SInt32 * target, int length ) {
    
    int i;
    
    for(i = 0;i < length; i++ ) {
        target[i] =  (SInt32) (source[i] << 9);
        if(source[i] < 0) { 
            target[i] |= 0xFF000000;
        }
        else {
            target[i] &= 0x00FFFFFF;
        }
        
    }
    
}



//////////////////////////////////////////////////
float getMeanVolumeSint16( SInt16 * vector , int length ) {
    
    
    // get average input volume level for meter display
    // by calculating log of mean volume of the buffer
    // and displaying it to the screen
    // (note: there's a vdsp function to do this but it works on float samples
    
    int sum;
    int i;
    int averageVolume;
    float logVolume;
    
    
    sum = 0;    
    for ( i = 0; i < length ; i++ ) {
        sum += abs((int) vector[i]);
    }
    
    averageVolume = sum / length;
    
    //    printf("\naverageVolume before scale = %lu", averageVolume );
    
    // now convert to logarithm and scale log10(0->32768) into 0->1 for display
    
    
    logVolume = log10f( (float) averageVolume ); 
    logVolume = logVolume / log10(32768);
    
    return (logVolume);
    
}




//////////////////////////
//
// calculate magnitude 

#pragma mark -
#pragma mark fft 

// for some calculation in the fft callback
// check to see if there is a vDsp library version
float MagnitudeSquared(float x, float y) {
	return ((x*x) + (y*y));
}



// end of audio functions supporting callbacks

///////////////////////
// mixerHostAudio class

#pragma mark -
@implementation MixerHostAudio

// properties (see header file for definitions and comments)

@synthesize stereoStreamFormat;         // stereo format for use in buffer and mixer input for "guitar" sound
@synthesize monoStreamFormat;           // mono format for use in buffer and mixer input for "beats" sound

@synthesize SInt16StreamFormat;

@synthesize floatStreamFormat;
@synthesize auEffectStreamFormat;
@synthesize auSamplerUnit;
@synthesize auFilePlayerUnit;

@synthesize graphSampleRate;            // sample rate to use throughout audio processing chain
@synthesize mixerUnit;                  // the Multichannel Mixer unit
@synthesize ioUnit;                  // the io unit
@synthesize auEffectUnit;  

@synthesize auInputEffect1;
@synthesize mixerNode;
@synthesize auEffectNode;
@synthesize samplerNode;
@synthesize filePlayerNode;
@synthesize inputEffect1Node;

@synthesize iONode;


@synthesize filePlayerFile;


@synthesize playing;                    // Boolean flag to indicate whether audio is playing or not
@synthesize interruptedDuringPlayback;  // Boolean flag to indicate whether audio was playing when an interruption arrived


@synthesize fftSetup;			// this is required by fft methods in the callback
@synthesize fftA;			
@synthesize fftLog2n;
@synthesize fftN;
@synthesize fftNOver2;		// params for fft setup

@synthesize dataBuffer;			// input buffer from mic
@synthesize outputBuffer;		// for fft conversion
@synthesize analysisBuffer;		// for fft frequency analysis

@synthesize conversionBufferLeft;
@synthesize conversionBufferRight;

@synthesize filterBuffer;
@synthesize filterLength;
@synthesize signalBuffer;
@synthesize signalLength;
@synthesize resultBuffer;
@synthesize resultLength;

@synthesize fftBufferCapacity;	// In samples
@synthesize fftIndex;	// In samples - this is a horrible variable name

@synthesize displayInputFrequency;
@synthesize displayInputLevelLeft;
@synthesize displayInputLevelRight;
@synthesize displayNumberOfInputChannels;

@synthesize sinFreq;
@synthesize sinPhase;
@synthesize synthNoteOn;

@synthesize micFxType;
@synthesize micFxOn;
@synthesize micFxControl;


@synthesize inputDeviceIsAvailable;

// end of properties


#pragma mark -
#pragma mark Initialize

//////////////////////////////////
// Get the app ready for playback.
- (id) init {
    
    
    self = [super init];
    
    if (!self) return nil;

    self.interruptedDuringPlayback = NO;
      

    [self setupAudioSession];		            
	
    [self FFTSetup];
    [self convolutionSetup];
    
    [self initDelayBuffer];

    [self obtainSoundFileURLs];
    [self setupStereoStreamFormat];
    [self setupMonoStreamFormat];

    [self setupSInt16StreamFormat];
  
	[self readAudioFilesIntoMemory];
    
    [self configureAndInitializeAudioProcessingGraph];
    
    
	
	return self;
}


#pragma mark -
#pragma mark Audio set up

//
//  AVAudioSession setup
//  This is all the external housekeeping needed in any ios coreaudio app
//
- (void) setupAudioSession {

// some debugging to find out about ourselves    
    
#if !CA_PREFER_FIXED_POINT
	NSLog(@"not fixed point");
#else
	NSLog(@"fixed point");
#endif
	

#if TARGET_IPHONE_SIMULATOR

// #warning *** Simulator mode: beware ***
	NSLog(@"simulator is running");
#else
	NSLog(@"device is running");
#endif
        
    
    if(UI_USER_INTERFACE_IDIOM() == UIUserInterfaceIdiomPhone) {
        NSLog(@"running iphone or ipod touch...\n");
    }
	
	NSString *deviceType = [UIDevice currentDevice].model;
    NSLog(@"device type is: %@", deviceType);
 

//    NSString *deviceUniqueId = [UIDevice currentDevice ].uniqueIdentifier;
//    NSLog(@"device Id is: %@", deviceUniqueId);


    NSString *operatingSystemVersion = [UIDevice currentDevice].systemVersion;
    NSLog(@"OS version is: %@", operatingSystemVersion);

//////////////////////////
// setup the session
	
    AVAudioSession *mySession = [AVAudioSession sharedInstance];

    // Specify that this object is the delegate of the audio session, so that
    //    this object's endInterruption method will be invoked when needed.
    [mySession setDelegate: self];


    // tz change to play and record
	// Assign the Playback category to the audio session.
    NSError *audioSessionError = nil;
    [mySession setCategory: AVAudioSessionCategoryPlayAndRecord
                     error: &audioSessionError];
    
    if (audioSessionError != nil) {
    
        NSLog (@"Error setting audio session category.");
        
    }
	
    
    //
	// check if input is available
    // this only really applies to older ipod touch without builtin mic
    //
    // There seems to be no graceful way to handle this
	//
    // what we do is:
    //
    //  1. set instance var: inputDeviceIsAvailable so app can make decisions based on input availability
    //  2. give the user a message saying input device is not available
    //  3. set the session for Playback only
    // 
    //
    // haven't tried this helpful tip:
    //
    //  set info.plist key: UIApplicationExitsOnSuspend to make the app terminate when
    //  the home button is pressed (instead of just suspending)
    //
    // another note on this: since ios5 the detection of the mic (headset) on ipod touch
    // is a lot less accurate.  It seems that sometimes you need to reboot the ipod or at the
    // very least terminate the app to 
    // get AVSession to actually detect the mic is plugged in...
    //
    
    
    
	inputDeviceIsAvailable = [mySession inputIsAvailable];
//    NSAssert( micIsAvailable, @"No audio input device available." );

	
    if(inputDeviceIsAvailable) {
            NSLog(@"input device is available");
    }
    else {
        NSLog(@"input device not available...");
        [mySession setCategory: AVAudioSessionCategoryPlayback
                         error: &audioSessionError];
       
    }
        
  
   
    
       
    // Request the desired hardware sample rate.
    self.graphSampleRate = 44100.0;    // Hertz
    
    [mySession setPreferredHardwareSampleRate: graphSampleRate
                                        error: &audioSessionError];
    
    if (audioSessionError != nil) {
    
        NSLog (@"Error setting preferred hardware sample rate.");
        
    }
	
	// refer to IOS developer library : Audio Session Programming Guide
	// set preferred buffer duration to 1024 using
	//  try ((buffer size + 1) / sample rate) - due to little arm6 floating point bug?
	// doesn't seem to help - the duration seems to get set to whatever the system wants...
	
	Float32 currentBufferDuration =  (Float32) (1024.0 / self.graphSampleRate);  
	UInt32 sss = sizeof(currentBufferDuration);
	
	AudioSessionSetProperty(kAudioSessionProperty_CurrentHardwareIOBufferDuration, sizeof(currentBufferDuration), &currentBufferDuration);
	NSLog(@"setting buffer duration to: %f", currentBufferDuration);
	
	
		
	
	
	

	// note: this is where ipod touch (w/o mic) erred out when mic (ie earbud thing) was not plugged - before we added
	// the code above to check for mic available 
    // Activate the audio session
    [mySession setActive: YES
                   error: &audioSessionError];

    if (audioSessionError != nil) {
    
        NSLog (@"Error activating audio session during initial setup.");
        
    }

    // Obtain the actual hardware sample rate and store it for later use in the audio processing graph.
    self.graphSampleRate = [mySession currentHardwareSampleRate];
	NSLog(@"Actual sample rate is: %f", self.graphSampleRate );
	
	// find out the current buffer duration
	// to calculate duration use: buffersize / sample rate, eg., 512 / 44100 = .012
	
	// Obtain the actual buffer duration - this may be necessary to get fft stuff working properly in passthru
	AudioSessionGetProperty(kAudioSessionProperty_CurrentHardwareIOBufferDuration, &sss, &currentBufferDuration);
	NSLog(@"Actual current hardware io buffer duration: %f ", currentBufferDuration );
	
	

    // Register the audio route change listener callback function with the audio session.
    AudioSessionAddPropertyListener (
        kAudioSessionProperty_AudioRouteChange,
        audioRouteChangeListenerCallback,
        self
    );
    
    // find out how many input channels are available 
    
    NSInteger numberOfChannels = [mySession currentHardwareInputNumberOfChannels];  
	NSLog(@"number of channels: %d", numberOfChannels );	
    displayNumberOfInputChannels = numberOfChannels;    // set instance variable for display

    return ;   // everything ok
    
}


/////////////////////////////
//
// housekeeping for loop files
//
- (void) obtainSoundFileURLs {

    // Create the URLs for the source audio files. The URLForResource:withExtension: method is new in iOS 4.0.
    
    // tz note: file references must added as resources to the xcode project bundle

    
    NSURL *guitarLoop   = [[NSBundle mainBundle] URLForResource: @"caitlin"
                                                  withExtension: @"caf"];
    
    NSURL *beatsLoop    = [[NSBundle mainBundle] URLForResource: @"congaloop"
                                                  withExtension: @"caf"];

    
    
    
    // ExtAudioFileRef objects expect CFURLRef URLs, so cast to CRURLRef here
    sourceURLArray[0]   = (CFURLRef) [guitarLoop retain];
    sourceURLArray[1]   = (CFURLRef) [beatsLoop retain];
}




// this converts the samples in the input buffer into floats
//
// there is an accelerate framework vdsp function 
// that does this conversion, so we're not using this function now
// but its good to know how to do it this way, although I would split it up into a setup and execute module
// I left this code to show how its done with an audio converter
//
void ConvertInt16ToFloat(MixerHostAudio *THIS, void *buf, float *outputBuf, size_t capacity) {
	AudioConverterRef converter;
	OSStatus err;
	
	size_t bytesPerSample = sizeof(float);
	AudioStreamBasicDescription outFormat = {0};
	outFormat.mFormatID = kAudioFormatLinearPCM;
	outFormat.mFormatFlags = kAudioFormatFlagIsFloat | kAudioFormatFlagIsPacked;
	outFormat.mBitsPerChannel = 8 * bytesPerSample;
	outFormat.mFramesPerPacket = 1;
	outFormat.mChannelsPerFrame = 1;	
	outFormat.mBytesPerPacket = bytesPerSample * outFormat.mFramesPerPacket;
	outFormat.mBytesPerFrame = bytesPerSample * outFormat.mChannelsPerFrame;		
	outFormat.mSampleRate = THIS->graphSampleRate;
	
	const AudioStreamBasicDescription inFormat = THIS->SInt16StreamFormat;
	
	UInt32 inSize = capacity*sizeof(SInt16);
	UInt32 outSize = capacity*sizeof(float);
	
	// this is the famed audio converter
	
	err = AudioConverterNew(&inFormat, &outFormat, &converter);
	if(noErr != err) {
		NSLog(@"error in audioConverterNew: %ld", err);
	}
	
	
	err = AudioConverterConvertBuffer(converter, inSize, buf, &outSize, outputBuf);
	if(noErr != err) {
		NSLog(@"error in audioConverterConvertBuffer: %ld", err);
	}
	
}




////////////////////////////
//
// setup asbd stream formats
//
//
- (void) setupStereoStreamFormat {

    // The AudioUnitSampleType data type is the recommended type for sample data in audio
    //    units. This obtains the byte size of the type for use in filling in the ASBD.
    size_t bytesPerSample = sizeof (AudioUnitSampleType);
//     NSLog (@"size of AudioUnitSampleType: %lu", bytesPerSample);

    // Fill the application audio format struct's fields to define a linear PCM, 
    //        stereo, noninterleaved stream at the hardware sample rate.
    stereoStreamFormat.mFormatID          = kAudioFormatLinearPCM;
    stereoStreamFormat.mFormatFlags       = kAudioFormatFlagsAudioUnitCanonical;
    stereoStreamFormat.mBytesPerPacket    = bytesPerSample;
    stereoStreamFormat.mFramesPerPacket   = 1;
    stereoStreamFormat.mBytesPerFrame     = bytesPerSample;
    stereoStreamFormat.mChannelsPerFrame  = 2;                    // 2 indicates stereo
    stereoStreamFormat.mBitsPerChannel    = 8 * bytesPerSample;
    stereoStreamFormat.mSampleRate        = graphSampleRate;


    NSLog (@"The stereo stream format:");
    [self printASBD: stereoStreamFormat];
}

//////////////////////////////
- (void) setupMonoStreamFormat {

    // The AudioUnitSampleType data type is the recommended type for sample data in audio
    //    units. This obtains the byte size of the type for use in filling in the ASBD.
    size_t bytesPerSample = sizeof (AudioUnitSampleType);

    // Fill the application audio format struct's fields to define a linear PCM, 
    //        stereo, noninterleaved stream at the hardware sample rate.
    monoStreamFormat.mFormatID          = kAudioFormatLinearPCM;
    monoStreamFormat.mFormatFlags       = kAudioFormatFlagsAudioUnitCanonical;
    monoStreamFormat.mBytesPerPacket    = bytesPerSample;
    monoStreamFormat.mFramesPerPacket   = 1;
    monoStreamFormat.mBytesPerFrame     = bytesPerSample;
    monoStreamFormat.mChannelsPerFrame  = 1;                  // 1 indicates mono
    monoStreamFormat.mBitsPerChannel    = 8 * bytesPerSample;
    monoStreamFormat.mSampleRate        = graphSampleRate;

    NSLog (@"The mono stream format:");
    [self printASBD: monoStreamFormat];

}

// this will be the stream format for anything that gets seriously processed by a render callback function
// it users 16bit signed int for sample data, assuming that this callback is probably on the input bus of a mixer
// or the input scope of the rio Output bus, in either case, we're assumeing that the AU will do the necessary format
// conversion to satisfy the output hardware - tz
//
// important distinction here with asbd's:
//
// note the difference between AudioUnitSampleType and AudioSampleType
//
// the former is an 8.24 (32 bit) fixed point sample format
// the latter is signed 16 bit (SInt16) integer sample format
//
// a subtle name differnce for a huge programming differece


- (void) setupSInt16StreamFormat {
    
    // Stream format for Signed 16 bit integers
    //
    // note: as of ios5 this works for signal channel mic/line input (not stereo)
    // and for mono audio generators (like synths) which pull no device data
    
    //    This obtains the byte size of the type for use in filling in the ASBD.
    size_t bytesPerSample = sizeof (AudioSampleType);	// Sint16
//    NSLog (@"size of AudioSampleType: %lu", bytesPerSample);
	
    // Fill the application audio format struct's fields to define a linear PCM, 
    //        stereo, noninterleaved stream at the hardware sample rate.
    SInt16StreamFormat.mFormatID          = kAudioFormatLinearPCM;
    SInt16StreamFormat.mFormatFlags       = kAudioFormatFlagsCanonical;
    SInt16StreamFormat.mBytesPerPacket    = bytesPerSample;
    SInt16StreamFormat.mFramesPerPacket   = 1;
    SInt16StreamFormat.mBytesPerFrame     = bytesPerSample;
    SInt16StreamFormat.mChannelsPerFrame  = 1;                  // 1 indicates mono
    SInt16StreamFormat.mBitsPerChannel    = 8 * bytesPerSample;
    SInt16StreamFormat.mSampleRate        = graphSampleRate;
	
    NSLog (@"The SInt16 (mono) stream format:");
    [self printASBD: SInt16StreamFormat];

    
    
}


// this is a test of using a float stream for the output scope of rio input bus
// and the input bus of a mixer channel
// the reason for this is that it would allow float algorithms to run without extra conversion
// that is, if it actually works
//
// so - apparently this doesn't work - at least in the context just described - there was no error in setting it
//
- (void) setupFloatStreamFormat {
	
    
    //    This obtains the byte size of the type for use in filling in the ASBD.
    	size_t bytesPerSample = sizeof(float);
	
    // Fill the application audio format struct's fields to define a linear PCM, 
    //        stereo, noninterleaved stream at the hardware sample rate.
    floatStreamFormat.mFormatID          = kAudioFormatLinearPCM;
    floatStreamFormat.mFormatFlags       = kAudioFormatFlagIsFloat | kAudioFormatFlagIsPacked;
    floatStreamFormat.mBytesPerPacket    = bytesPerSample;
    floatStreamFormat.mFramesPerPacket   = 1;
    floatStreamFormat.mBytesPerFrame     = bytesPerSample;
    floatStreamFormat.mChannelsPerFrame  = 1;                  // 1 indicates mono
    floatStreamFormat.mBitsPerChannel    = 8 * bytesPerSample;
    floatStreamFormat.mSampleRate        = graphSampleRate;
	
    NSLog (@"The float stream format:");
    [self printASBD: floatStreamFormat];
	
}







// initialize the circular delay buffer
// 
- (void) initDelayBuffer {
    
    // Allocate buffer
    
    delayBuffer = (SInt16*)malloc(sizeof(SInt16) * kDelayBufferLength);
    
    memset(delayBuffer,0, kDelayBufferLength );  // set to zero 
    
    // Initialise record
    TPCircularBufferInit(&delayBufferRecord, kDelayBufferLength);
    delayBufferRecordLock = [[NSLock alloc] init];
   
    // this should be set with a constant equal to frame buffer size
    // so we're using this for other big stuff, so...
    
     tempDelayBuffer = (SInt16*)malloc(sizeof(SInt16) * 4096);
    
    // now do the same thing for the float filter buffer
    
    
    // Allocate buffer
    
    circularFilterBuffer = (float *)malloc(sizeof(float) * kDelayBufferLength);
    
    memset(circularFilterBuffer,0, kDelayBufferLength );  // set to zero 
    
    // Initialise record
    
    TPCircularBufferInit(&circularFilterBufferRecord, kDelayBufferLength);
    circularFilterBufferRecordLock = [[NSLock alloc] init];
    
    // this should be set with a constant equal to frame buffer size
    // so we're using this for other big stuff, so...
    
    tempCircularFilterBuffer = (float *)malloc(sizeof(float) * 4096);
    
    
    
}

//////////////////////////////////////////////////
// Setup FFT - structures needed by vdsp functions
//
- (void) FFTSetup {
	
	// I'm going to just convert everything to 1024
	
	
	// on the simulator the callback gets 512 frames even if you set the buffer to 1024, so this is a temp workaround in our efforts
	// to make the fft buffer = the callback buffer, 
	
	
	// for smb it doesn't matter if frame size is bigger than callback buffer
	
	UInt32 maxFrames = 1024;    // fft size
	
	
	// setup input and output buffers to equal max frame size
	
	dataBuffer = (void*)malloc(maxFrames * sizeof(SInt16));
	outputBuffer = (float*)malloc(maxFrames *sizeof(float));
	analysisBuffer = (float*)malloc(maxFrames *sizeof(float));
	
	// set the init stuff for fft based on number of frames
	
	fftLog2n = log2f(maxFrames);		// log base2 of max number of frames, eg., 10 for 1024
	fftN = 1 << fftLog2n;					// actual max number of frames, eg., 1024 - what a silly way to compute it

    
	fftNOver2 = maxFrames/2;                // half fft size
	fftBufferCapacity = maxFrames;          // yet another way of expressing fft size
	fftIndex = 0;                           // index for reading frame data in callback
	
	// split complex number buffer
	fftA.realp = (float *)malloc(fftNOver2 * sizeof(float));		// 
	fftA.imagp = (float *)malloc(fftNOver2 * sizeof(float));		// 
	
	
	// zero return indicates an error setting up internal buffers
	
	fftSetup = vDSP_create_fftsetup(fftLog2n, FFT_RADIX2);
    if( fftSetup == (FFTSetup) 0) {
        NSLog(@"Error - unable to allocate FFT setup buffers" );
	}
	
}




/////////////////////////////////////////
// Setup stuff for convolution testing 

- (void)convolutionSetup {
	

	int i;
    
    // just throwing this in here for testing 
    // these are the callback data conversion buffers
    
    conversionBufferLeft = (void *) malloc(2048 * sizeof(SInt16));
    conversionBufferRight = (void *) malloc(2048 * sizeof(SInt16));
	
	
	filterLength = 101;
    
    // signal length is actually 1024 but we're padding it 
    // with convolution the result length is signal + filter - 1
    
    signalLength = 2048;
    resultLength = 2048;
    
    filterBuffer = (void*)malloc(filterLength * sizeof(float));
    signalBuffer = (void*)malloc(signalLength * sizeof(float));
    resultBuffer = (void*)malloc(resultLength * sizeof(float));
//    paddingBuffer = (void*)malloc(paddingLength * sizeof(float));

    
// build a filter 
// 101 point windowed sinc lowpass filter from http://www.dspguide.com/
// table 16-1
    
// note - now the filter gets rebuilt on the fly according to UI value for cutoff frequency
//    
    

// calculate lowpass filter kernel    

    
    int m = 100;
    float fc = .14;
    
    for( i = 0; i < 101 ; i++ ) {
        if((i - m / 2) == 0 ) {
            filterBuffer[i] = 2 * M_PI * fc;
        }
        else {
            filterBuffer[i] = sin(2 * M_PI * fc * (i - m / 2)) / (i - m / 2);
        }
        filterBuffer[i] = filterBuffer[i] * (.54 - .46 * cos(2 * M_PI * i / m ));
    }
     
// normalize for unity gain at dc
    
    float sum = 0;
    for ( i = 0 ; i < 101 ; i++ ) {
        sum = sum + filterBuffer[i]; 
    }
    
    for ( i = 0 ; i < 101 ; i++ ) {
        filterBuffer[i] = filterBuffer[i] / sum;
    }
	
}


//////////////////
// read loop files

#pragma mark -
#pragma mark Read audio files into memory

- (void) readAudioFilesIntoMemory {

    for (int audioFile = 0; audioFile < NUM_FILES; ++audioFile)  {
    
        NSLog (@"readAudioFilesIntoMemory - file %i", audioFile);
        
        // Instantiate an extended audio file object.
        ExtAudioFileRef audioFileObject = 0;
        
        // Open an audio file and associate it with the extended audio file object.
        OSStatus result = ExtAudioFileOpenURL (sourceURLArray[audioFile], &audioFileObject);
        
        if (noErr != result || NULL == audioFileObject) {[self printErrorMessage: @"ExtAudioFileOpenURL" withStatus: result]; return;}

        // Get the audio file's length in frames.
        UInt64 totalFramesInFile = 0;
        UInt32 frameLengthPropertySize = sizeof (totalFramesInFile);
        
        result =    ExtAudioFileGetProperty (
                        audioFileObject,
                        kExtAudioFileProperty_FileLengthFrames,
                        &frameLengthPropertySize,
                        &totalFramesInFile
                    );

        if (noErr != result) {[self printErrorMessage: @"ExtAudioFileGetProperty (audio file length in frames)" withStatus: result]; return;}
        
        // Assign the frame count to the soundStructArray instance variable
        soundStructArray[audioFile].frameCount = totalFramesInFile;

        // Get the audio file's number of channels.
        AudioStreamBasicDescription fileAudioFormat = {0};
        UInt32 formatPropertySize = sizeof (fileAudioFormat);
        
        result =    ExtAudioFileGetProperty (
                        audioFileObject,
                        kExtAudioFileProperty_FileDataFormat,
                        &formatPropertySize,
                        &fileAudioFormat
                    );

        if (noErr != result) {[self printErrorMessage: @"ExtAudioFileGetProperty (file audio format)" withStatus: result]; return;}

        UInt32 channelCount = fileAudioFormat.mChannelsPerFrame;
        
        // Allocate memory in the soundStructArray instance variable to hold the left channel, 
        //    or mono, audio data
        soundStructArray[audioFile].audioDataLeft =
            (AudioUnitSampleType *) calloc (totalFramesInFile, sizeof (AudioUnitSampleType));

        AudioStreamBasicDescription importFormat = {0};
        if (2 == channelCount) {
        
            soundStructArray[audioFile].isStereo = YES;
            // Sound is stereo, so allocate memory in the soundStructArray instance variable to  
            //    hold the right channel audio data
            soundStructArray[audioFile].audioDataRight =
                (AudioUnitSampleType *) calloc (totalFramesInFile, sizeof (AudioUnitSampleType));
            importFormat = stereoStreamFormat;
            
        } else if (1 == channelCount) {
        
            soundStructArray[audioFile].isStereo = NO;
            importFormat = monoStreamFormat;
            
        } else {
        
            NSLog (@"*** WARNING: File format not supported - wrong number of channels");
            ExtAudioFileDispose (audioFileObject);
            return;
        }

        // Assign the appropriate mixer input bus stream data format to the extended audio 
        //        file object. This is the format used for the audio data placed into the audio 
        //        buffer in the SoundStruct data structure, which is in turn used in the 
        //        inputRenderCallback callback function.
        
        result =    ExtAudioFileSetProperty (
                        audioFileObject,
                        kExtAudioFileProperty_ClientDataFormat,
                        sizeof (importFormat),
                        &importFormat
                    );

        if (noErr != result) {[self printErrorMessage: @"ExtAudioFileSetProperty (client data format)" withStatus: result]; return;}
        
        // Set up an AudioBufferList struct, which has two roles:
        //
        //        1. It gives the ExtAudioFileRead function the configuration it 
        //            needs to correctly provide the data to the buffer.
        //
        //        2. It points to the soundStructArray[audioFile].audioDataLeft buffer, so 
        //            that audio data obtained from disk using the ExtAudioFileRead function
        //            goes to that buffer

        // Allocate memory for the buffer list struct according to the number of 
        //    channels it represents.
        AudioBufferList *bufferList;

        bufferList = (AudioBufferList *) malloc (
            sizeof (AudioBufferList) + sizeof (AudioBuffer) * (channelCount - 1)
        );

        if (NULL == bufferList) {NSLog (@"*** malloc failure for allocating bufferList memory"); return;}
        
        // initialize the mNumberBuffers member
        bufferList->mNumberBuffers = channelCount;
        
        // initialize the mBuffers member to 0
        AudioBuffer emptyBuffer = {0};
        size_t arrayIndex;
        for (arrayIndex = 0; arrayIndex < channelCount; arrayIndex++) {
            bufferList->mBuffers[arrayIndex] = emptyBuffer;
        }
        
        // set up the AudioBuffer structs in the buffer list
        bufferList->mBuffers[0].mNumberChannels  = 1;
        bufferList->mBuffers[0].mDataByteSize    = totalFramesInFile * sizeof (AudioUnitSampleType);
        bufferList->mBuffers[0].mData            = soundStructArray[audioFile].audioDataLeft;
    
        if (2 == channelCount) {
            bufferList->mBuffers[1].mNumberChannels  = 1;
            bufferList->mBuffers[1].mDataByteSize    = totalFramesInFile * sizeof (AudioUnitSampleType);
            bufferList->mBuffers[1].mData            = soundStructArray[audioFile].audioDataRight;
        }

        // Perform a synchronous, sequential read of the audio data out of the file and
        //    into the soundStructArray[audioFile].audioDataLeft and (if stereo) .audioDataRight members.
        UInt32 numberOfPacketsToRead = (UInt32) totalFramesInFile;
        
        result = ExtAudioFileRead (
                     audioFileObject,
                     &numberOfPacketsToRead,
                     bufferList
                 );

        free (bufferList);
        
        if (noErr != result) {
        
            [self printErrorMessage: @"ExtAudioFileRead failure - " withStatus: result];
            
            // If reading from the file failed, then free the memory for the sound buffer.
            free (soundStructArray[audioFile].audioDataLeft);
            soundStructArray[audioFile].audioDataLeft = 0;
            
            if (2 == channelCount) {
                free (soundStructArray[audioFile].audioDataRight);
                soundStructArray[audioFile].audioDataRight = 0;
            }
            
            ExtAudioFileDispose (audioFileObject);            
            return;
        }
        
        NSLog (@"Finished reading file %i into memory", audioFile);

        // Set the sample index to zero, so that playback starts at the 
        //    beginning of the sound.
        soundStructArray[audioFile].sampleNumber = 0;

        // Dispose of the extended audio file object, which also
        //    closes the associated file.
        ExtAudioFileDispose (audioFileObject);
    }
}

////////////////////////////////////////////////////////////////////////////////////////////
// create and setup audio processing graph by setting component descriptions and adding nodes

- (void) setupAudioProcessingGraph {
    
    OSStatus result = noErr;
    
 
    // Create a new audio processing graph.
    result = NewAUGraph (&processingGraph);
    
    if (noErr != result) {[self printErrorMessage: @"NewAUGraph" withStatus: result]; return;}
    
    
    //............................................................................
    // Specify the audio unit component descriptions for the audio units to be
    //    added to the graph.

    // remote I/O unit connects both to mic/lineIn and to speaker
    AudioComponentDescription iOUnitDescription;
    iOUnitDescription.componentType          = kAudioUnitType_Output;
    iOUnitDescription.componentSubType       = kAudioUnitSubType_RemoteIO;
    iOUnitDescription.componentManufacturer  = kAudioUnitManufacturer_Apple;
    iOUnitDescription.componentFlags         = 0;
    iOUnitDescription.componentFlagsMask     = 0;



    // Multichannel mixer unit
    AudioComponentDescription MixerUnitDescription;
    MixerUnitDescription.componentType          = kAudioUnitType_Mixer;
    MixerUnitDescription.componentSubType       = kAudioUnitSubType_MultiChannelMixer;
    MixerUnitDescription.componentManufacturer  = kAudioUnitManufacturer_Apple;
    MixerUnitDescription.componentFlags         = 0;
    MixerUnitDescription.componentFlagsMask     = 0;

    // au unit effect for mixer output - lowPass filter

    AudioComponentDescription auEffectUnitDescription; 
    auEffectUnitDescription.componentType = kAudioUnitType_Effect;
    auEffectUnitDescription.componentSubType = kAudioUnitSubType_LowPassFilter;
    auEffectUnitDescription.componentManufacturer = kAudioUnitManufacturer_Apple;

    //  au mixer input channel effect
    // 
    //
    //  high pass filter

    AudioComponentDescription auInputEffect1UnitDescription; 
    auInputEffect1UnitDescription.componentType = kAudioUnitType_Effect;
    auInputEffect1UnitDescription.componentSubType = kAudioUnitSubType_HighPassFilter;
    auInputEffect1UnitDescription.componentManufacturer = kAudioUnitManufacturer_Apple;

    // fileplayer

    AudioComponentDescription auFilePlayerUnitDescription; 
    auFilePlayerUnitDescription.componentType = kAudioUnitType_Generator;
    auFilePlayerUnitDescription.componentSubType = kAudioUnitSubType_AudioFilePlayer;
    auFilePlayerUnitDescription.componentManufacturer = kAudioUnitManufacturer_Apple;

    // sampler

    AudioComponentDescription auSamplerUnitDescription; 
    auSamplerUnitDescription.componentType = kAudioUnitType_MusicDevice ;
    auSamplerUnitDescription.componentSubType = kAudioUnitSubType_Sampler;
    auSamplerUnitDescription.componentManufacturer = kAudioUnitManufacturer_Apple;

    ///////////////////////////////////////////////	
    // Add the nodes to the audio processing graph
    ///////////////////////////////////////////////
    
  
    NSLog (@"Adding nodes to audio processing graph");
    

    
    // io unit 
    
    result =    AUGraphAddNode (
                                processingGraph,
                                &iOUnitDescription,
                                &iONode);
    
    if (noErr != result) {[self printErrorMessage: @"AUGraphNewNode failed for I/O unit" withStatus: result]; return;}
    
	
    
    // mixer unit
    
    result =    AUGraphAddNode (
                                processingGraph,
                                &MixerUnitDescription,
                                &mixerNode
                                );
    
    if (noErr != result) {[self printErrorMessage: @"AUGraphNewNode failed for Mixer unit" withStatus: result]; return;}
    
    
    
    
    
    
    // au effect unit 
    
    
	CheckError(AUGraphAddNode(
                              processingGraph,
                              &auEffectUnitDescription,
                              &auEffectNode), 
			   "AUGraphNode for auEffectUnit failed");
    
    
    
    
    // sampler unit
    
    CheckError(AUGraphAddNode(processingGraph, &auSamplerUnitDescription, &samplerNode), 
			   "AUGraphNode[kAudioUnitSubType_Sampler] failed");
    
    
    
    // fileplayer unit
    
	CheckError(AUGraphAddNode(processingGraph, &auFilePlayerUnitDescription, &filePlayerNode), 
			   "AUGraphNode[kAudioUnitSubType_AudioFilePlayer] failed");
    
    // input effect1 unit
    
	CheckError(AUGraphAddNode(processingGraph, &auInputEffect1UnitDescription, &inputEffect1Node), 
			   "AUGraphNode[kAudioUnitSubType_Reverb2P] failed");
    

}

- (void) connectAudioProcessingGraph {
    
     OSStatus result = noErr;
    
    //............................................................................
    // Connect the nodes of the audio processing graph
    
    // note: you only need to connect nodes which don't have assigned callbacks.
    // So for example, the mic/lineIn channel doesn't need to be connected.  
    
	
	NSLog (@"Connecting nodes in audio processing graph");
    
    
    /*	
     // this call should only be used if you don't need to process the mic input with a callback
     
     // Connect the output of the input bus of the I/O unit to the Multichannel Mixer unit input.
     result =	AUGraphConnectNodeInput (
     processingGraph,
     iONode,				// source node
     1,					// source node bus number
     mixerNode,			// destination node
     micBus					// destintaion node bus number
     );
     
     if (result) {[self printErrorMessage: @"AUGraphConnectNodeInput - I/O unit to Multichannel Mixer unit" withStatus: result]; return;}
     
     
     */
    
    
    NSLog (@"Connecting the sampler output to the mixer input node 4");
    
	// connect file player bus 0 (sampler output) to mixer bus 2 (third input)
	CheckError(AUGraphConnectNodeInput(processingGraph, samplerNode, 0, mixerNode, 4),
			   "AUGraphConnectNodeInput failed (sampler 0 to mixer 1)");
    
    
    
    NSLog (@"Connecting the filePlayer output to input effect 1");
    
	// connect file player bus 0  to input effect1
	CheckError(AUGraphConnectNodeInput(processingGraph, filePlayerNode, 0, inputEffect1Node, 0),
			   "AUGraphConnectNodeInput failed (fileplayer 0 to input effect1)");
    
    
    NSLog (@"Connecting the input effect 1 output to the mixer input node 5");
    
	// connect file player bus 0 (input effect 1 output) to mixer bus 5 
	CheckError(AUGraphConnectNodeInput(processingGraph, inputEffect1Node, 0, mixerNode, 5),
			   "AUGraphConnectNodeInput failed (input effect1 0 to mixer 5)");
    
    
    
    
    NSLog (@"Connecting the mixer output to the input of mixer effect element");
    
    
    
    result = AUGraphConnectNodeInput (
                                      processingGraph,
                                      mixerNode,         // source node
                                      0,                 // source node output bus number
                                      auEffectNode,            // destination node
                                      0                  // desintation node input bus number
                                      );
    
    if (noErr != result) {[self printErrorMessage: @"AUGraphConnectNodeInput" withStatus: result]; return;}
    
    
    
    
    NSLog (@"Connecting the effect output to the input of the I/O unit output element");
    
    
    
    result = AUGraphConnectNodeInput (
                                      processingGraph,
                                      auEffectNode,         // source node
                                      0,                 // source node output bus number
                                      iONode,            // destination node
                                      0                  // desintation node input bus number
                                      );
    
    if (noErr != result) {[self printErrorMessage: @"AUGraphConnectNodeInput" withStatus: result]; return;}
    

}

#pragma mark -
#pragma mark Audio processing graph setup

// This method does the audio processing graph:

    //  - Instantiate and open an audio processing graph
    //  - specify audio unit component descriptions for all units in the graph
    //  - add nodes to the graph
    //  - Open graph and get the audio unit nodes for the graph
    //  - configure the io input unit
    //  - Configure the Multichannel Mixer unit
    //     * specify the number of input buses
    //     * specify the output sample rate
    //     * specify the maximum frames-per-slice
    //      - configure each input channel of mixer
    //          - set callback structs
    //          - set asbd's
    //  - configure any other audio units (fx, sampler, fileplayer)
    //  - make connections
    //  - start the audio processing graph
    //  configure audio unit params
    //  setup midi and fileplayer 

- (void) configureAndInitializeAudioProcessingGraph {

    NSLog (@"Configuring and then initializing audio processing graph");
    OSStatus result = noErr;

    UInt16 busNumber;           // mixer input bus number (starts with 0)
    
    // instantiate and setup audio processing graph by setting component descriptions and adding nodes
    
    
    [self setupAudioProcessingGraph];
      
	

///////////////////////////////////////////////////////////////////    
//............................................................................
// Open the audio processing graph

    // Following this call, the audio units are instantiated but not initialized
    //    (no resource allocation occurs and the audio units are not in a state to
    //    process audio).
    result = AUGraphOpen (processingGraph);
    
    if (noErr != result) {[self printErrorMessage: @"AUGraphOpen" withStatus: result]; return;}
    

//
//  at this point we set all the audio units individually
//
//  - get unit instance from audio graph node
//  - set bus io params
//  - set other params
//  - set ASBD's
//
    
    
//............................................................................
// Obtain the I/O unit instance from the corresponding node.
	result =	AUGraphNodeInfo (
								 processingGraph,
								 iONode,
								 NULL,
								 &ioUnit
								 );
	
	if (result) {[self printErrorMessage: @"AUGraphNodeInfo - I/O unit" withStatus: result]; return;}
    

/////////////////////////////
// I/O Unit Setup (input bus)
	

	
    if(inputDeviceIsAvailable) {            // if no input device, skip this step
        AudioUnitElement ioUnitInputBus = 1;
	
        // Enable input for the I/O unit, which is disabled by default. (Output is
        //	enabled by default, so there's no need to explicitly enable it.)
        UInt32 enableInput = 1;
	
        AudioUnitSetProperty (
						  ioUnit,
						  kAudioOutputUnitProperty_EnableIO,
						  kAudioUnitScope_Input,
						  ioUnitInputBus,
						  &enableInput,
						  sizeof (enableInput)
						  );
	
	
        // Specify the stream format for output side of the I/O unit's 
        //	input bus (bus 1). For a description of these fields, see 
        //	AudioStreamBasicDescription in Core Audio Data Types Reference.
        //
        // Instead of explicitly setting the fields in the ASBD as is done 
        //	here, you can use the SetAUCanonical method from the Core Audio 
        //	"Examples" folder. Refer to:
        //		/Developer/Examples/CoreAudio/PublicUtility/CAStreamBasicDescription.h
	
        // The AudioUnitSampleType data type is the recommended type for sample data in audio
        //	units

 
        //  set the stream format for the callback that does processing
        // of the mic/line input samples
 
        // using 8.24 fixed point now because SInt doesn't work in stereo

        // Apply the stream format to the output scope of the I/O unit's input bus.

	
        // tz 11/28 stereo input!!
        //
        // we could set the asbd to stereo and then decide in the callback
        // whether to use the right channel or not, but for now I would like to not have
        // the extra rendering and copying step for an un-used channel - so we'll customize
        // the asbd selection here...
    
        //    Now checking for number of input channels to decide mono or stereo asbd   
        //    note, we're assuming mono for one channel, stereo for anything else
        //    if no input channels, then the program shouldn't have gotten this far.
   
        if( displayNumberOfInputChannels == 1) {
            NSLog (@"Setting kAudioUnitProperty_StreamFormat (monoStreamFormat) for the I/O unit input bus's output scope");
            result =	AudioUnitSetProperty (
                                            ioUnit,
                                          kAudioUnitProperty_StreamFormat,
                                          kAudioUnitScope_Output,
                                          ioUnitInputBus,
                                          &monoStreamFormat,
                                          sizeof (monoStreamFormat)
                                          );
        
            if (result) {[self printErrorMessage: @"AudioUnitSetProperty (set I/O unit input stream format output scope) monoStreamFormat" withStatus: result]; return;}
        }
        else {
            NSLog (@"Setting kAudioUnitProperty_StreamFormat (stereoStreamFormat) for the I/O unit input bus's output scope");
            result =	AudioUnitSetProperty (
                                          ioUnit,
                                          kAudioUnitProperty_StreamFormat,
                                          kAudioUnitScope_Output,
                                          ioUnitInputBus,
                                          &stereoStreamFormat,
                                          sizeof (stereoStreamFormat)
                                          );
        
            if (result) {[self printErrorMessage: @"AudioUnitSetProperty (set I/O unit input stream format output scope) stereoStreamFormat" withStatus: result]; return;}
        
        }
    }    
	
    // this completes setup for the RIO audio unit other than seting the callback which gets attached to the mixer input bus
    
     

	
	
//////////////////////////////////////////////////////////////
// Obtain the mixer unit instance from its corresponding node.

    result =    AUGraphNodeInfo (
                    processingGraph,
                    mixerNode,
                    NULL,
                    &mixerUnit
                );
    
    if (noErr != result) {[self printErrorMessage: @"AUGraphNodeInfo" withStatus: result]; return;}
    
/////////////////////////////////
// Multichannel Mixer unit Setup

    
    
    UInt32 busCount   = 6;    // bus count for mixer unit input
    UInt32 guitarBus  = 0;    // mixer unit bus 0 will be stereo and will take the guitar sound
    UInt32 beatsBus   = 1;    // mixer unit bus 1 will be mono and will take the beats sound
	UInt32 micBus	  = 2;    // mixer unit bus 2 will be mono and will take the microphone input
	UInt32 synthBus   = 3;    // mixer unit bus 2 will be mono and will take the microphone input
//    UInt32 samplerBus   = 4;     
//    UInt32 filePlayerBus = 5;
    
    NSLog (@"Setting mixer unit input bus count to: %lu", busCount);
    result = AudioUnitSetProperty (
                 mixerUnit,
                 kAudioUnitProperty_ElementCount,
                 kAudioUnitScope_Input,
                 0,
                 &busCount,
                 sizeof (busCount)
             );

    if (noErr != result) {[self printErrorMessage: @"AudioUnitSetProperty (set mixer unit bus count)" withStatus: result]; return;}


    NSLog (@"Setting kAudioUnitProperty_MaximumFramesPerSlice for mixer unit global scope");
    // Increase the maximum frames per slice allows the mixer unit to accommodate the
    //    larger slice size used when the screen is locked.
    UInt32 maximumFramesPerSlice = 4096;
    
    result = AudioUnitSetProperty (
                 mixerUnit,
                 kAudioUnitProperty_MaximumFramesPerSlice,
                 kAudioUnitScope_Global,
                 0,
                 &maximumFramesPerSlice,
                 sizeof (maximumFramesPerSlice)
             );

    if (noErr != result) {[self printErrorMessage: @"AudioUnitSetProperty (set mixer unit input stream format)" withStatus: result]; return;}

	UInt16 fileCount = 2;	// number of 'file' busses to init on the mixer
    // Attach the input render callback and context to each input bus
	// this is for the two file players
	// subtract 2 from bus count because we're not including mic & synth bus for now...  tz 
    for (UInt16 busNumber = 0; busNumber < fileCount; ++busNumber) {

        // Setup the structure that contains the input render callback 
        AURenderCallbackStruct inputCallbackStruct;
        inputCallbackStruct.inputProc        = &inputRenderCallback;
        inputCallbackStruct.inputProcRefCon  = soundStructArray;
        
        NSLog (@"Registering the render callback with mixer unit input bus %u", busNumber);
        // Set a callback for the specified node's specified input
        result = AUGraphSetNodeInputCallback (
                     processingGraph,
                     mixerNode,
                     busNumber,
                     &inputCallbackStruct
                 );

        if (noErr != result) {[self printErrorMessage: @"AUGraphSetNodeInputCallback" withStatus: result]; return;}
    }

/////////////////////////////////////////////////////////////////////
// now attach the separate render callback for the mic/lineIn channel 
	

	if(inputDeviceIsAvailable) {
	
        UInt16 busNumber = 2;		// mic channel on mixer
	
        // Setup the structure that contains the input render callback 
        AURenderCallbackStruct inputCallbackStruct;
	
        inputCallbackStruct.inputProc        = micLineInCallback;	// 8.24 version
        inputCallbackStruct.inputProcRefCon  = self;
	
	
        NSLog (@"Registering the render callback - mic/lineIn - with mixer unit input bus %u", busNumber);
        // Set a callback for the specified node's specified input
        result = AUGraphSetNodeInputCallback (
										  processingGraph,
										  mixerNode,
										  busNumber,
										  &inputCallbackStruct
										  );
	
        if (noErr != result) {[self printErrorMessage: @"AUGraphSetNodeInputCallback mic/lineIn" withStatus: result]; return;}
	
	
    }
	
	
	
	
	
	///////////////////////////////////////////////////////////////
	// now attach the separate render callback for the synth channel
	
	busNumber = 3;		// synth channel on mixer
		
    AURenderCallbackStruct synthCallbackStruct;     // Setup structure that contains the render callback function
	
	synthCallbackStruct.inputProc        = synthRenderCallback;	// for sound generation
    synthCallbackStruct.inputProcRefCon  = self;                // this pointer allows callback to access scope of this class

	
	
	NSLog (@"Registering the render callback - synth - with mixer unit input bus %u", busNumber);
	// Set a callback for the specified node's specified input
	result = AUGraphSetNodeInputCallback (
										  processingGraph,
										  mixerNode,
										  busNumber,
										  &synthCallbackStruct
										  );
	
	if (noErr != result) {[self printErrorMessage: @"AUGraphSetNodeInputCallback" withStatus: result]; return;}
	

	
    // Since there's no callback on the sampler, or fileplayer mixer input channels  
    // they just get hooked up in the au graph connection process
    // callbacks are only required when the application needs access to frame data
    //
    // conversely the buses which are assigned callbacks do not need to be connected in the
    // au graph connection process.
   
    
    ///////////////////////////////////////////////    
    // set all the ASBD's for the mixer input buses	
    //
    // each mixer input bus needs an asbd that matches the asbd of the output bus its pulling data from
    //
    // In the case of the synth bus, which generates its own data, the asbd can be anything reasonable that
    // works on the input bus.
    //
    // The asbd of the mixer input bus does not have to match the asbd of the mixer output bus. 
    // In that sense, the mixer acts as a format converter. But I don't know to what extent this will work.
    // It does sample format conversions, but I don't know that it can do sample rate conversions.
    

    NSLog (@"Setting stereo stream format for mixer unit 0 (stereo guitar loop) input bus");
    result = AudioUnitSetProperty (
                 mixerUnit,
                 kAudioUnitProperty_StreamFormat,
                 kAudioUnitScope_Input,
                 guitarBus,
                 &stereoStreamFormat,
                 sizeof (stereoStreamFormat)
             );

    if (noErr != result) {[self printErrorMessage: @"AudioUnitSetProperty (set mixer unit bus 0 (stereo guitar loop) stream format)" withStatus: result];return;}
    

    NSLog (@"Setting mono stream format for mixer unit bus 1 (mono beats loop)");
    result = AudioUnitSetProperty (
                 mixerUnit,
                 kAudioUnitProperty_StreamFormat,
                 kAudioUnitScope_Input,
                 beatsBus,
                 &monoStreamFormat,
                 sizeof (monoStreamFormat)
             );

    if (noErr != result) {[self printErrorMessage: @"AudioUnitSetProperty set mixer unit bus 1 (mono beats loop) stream format)" withStatus: result];return;}


	
	// set either mono or stereo 8.24 format (default) for mic/lineIn bus
    //
    // Note: you can also get mono mic/line input using SInt16 samples (see synth asbd)
    // But SInt16 gives asbd format errors with more than one channel
    
    if(displayNumberOfInputChannels == 1) {     // number of available channels determines mono/stereo choice
	
        NSLog (@"Setting monoStreamFormat for mixer unit bus 2 (mic/lineIn)");
        result = AudioUnitSetProperty (
								   mixerUnit,
								   kAudioUnitProperty_StreamFormat,
								   kAudioUnitScope_Input,
								   micBus,
								   &monoStreamFormat,
								   sizeof (monoStreamFormat)
								   );
	
        if (noErr != result) {[self printErrorMessage: @"AudioUnitSetProperty (set mixer unit bus 2 mic/line stream format mono)" withStatus: result];return;}
    }
    else if(displayNumberOfInputChannels > 1) {  // do the stereo asbd
        NSLog (@"Setting stereoStreamFormat for mixer unit bus 2 mic/lineIn input");
        result = AudioUnitSetProperty (
                                       mixerUnit,
                                       kAudioUnitProperty_StreamFormat,
                                       kAudioUnitScope_Input,
                                       micBus,
                                       &stereoStreamFormat,
                                       sizeof (stereoStreamFormat)
                                       );
        
        if (noErr != result) {[self printErrorMessage: @"AudioUnitSetProperty (set mixer unit bus 2 mic/line stream format stereo)" withStatus: result];return;} 
        
    }
	
	
	
	// set Sint16 mono asbd for synth bus 
	
    NSLog (@"Setting Sint16 stream format for mixer unit synth input bus 3");
    result = AudioUnitSetProperty (
								   mixerUnit,
								   kAudioUnitProperty_StreamFormat,
								   kAudioUnitScope_Input,
								   synthBus,
								   &SInt16StreamFormat,
								   sizeof (SInt16StreamFormat)
								   );
	
    if (noErr != result) {[self printErrorMessage: @"AudioUnitSetProperty (set mixer unit synth input bus 3 mono stream format)" withStatus: result];return;}
	
// ok this is where things change up a little because we're using audio unit effects. otherwise we
// would just set the sample rate on the mixer output scope and be done...
    
// we're going to need to set the mixer output scope to match
// the effects input scope asbd - but had to move it up in the code because
// the effects asbd not defined yet    
    
// we'll postpone any settings for the sampler, and fileplayer until we take care of setting up the
// effects unit asbd in the next step.
    
// but if you have no effects after the mixer, just uncomment the next little section here...  and life will be easier 
    
 

/*	

    NSLog (@"Setting sample rate for mixer unit output scope");
    // Set the mixer unit's output sample rate format. This is the only aspect of the output stream
    //    format that must be explicitly set.
    result = AudioUnitSetProperty (
                 mixerUnit,
                 kAudioUnitProperty_SampleRate,
                 kAudioUnitScope_Output,
                 0,
                 &graphSampleRate,
                 sizeof (graphSampleRate)
             );

    if (noErr != result) {[self printErrorMessage: @"AudioUnitSetProperty (set mixer unit output stream format)" withStatus: result]; return;}

 */
   
 //
    
    

    
    
 	
/////////////////////////////////////////////////////////////////////////	
//
// Obtain the au effect unit instance from its corresponding node.
 
    NSLog (@"Getting effect Node Info...");
    result =    AUGraphNodeInfo (
                                 processingGraph,
                                 auEffectNode,
                                 NULL,
                                 &auEffectUnit
                                 );
    
    if (noErr != result) {[self printErrorMessage: @"AUGraphNodeInfo - auEffect" withStatus: result]; return;}
   
// setup ASBD for effects unit 
    
// This section is very confusing but important to getting things working
// 
// The output of the mixer is now feeding the input of an effects au
//
// if the effects au wasn't there, you would just set the sample rate on the output scope of the mixer
// as is explained in the Apple docs, and be done with it. Essesntially letting the system handle the 
// rest of the asbd setup for the mixer output
//
// But... in our setup, since the mixer ouput is not at the end of the chain, we set the sample rate on the 
// effects output scope instead. 
    
// and for the effects unit input scope... we need to obtain the default asbd from the the effects unit - this is 
// where things are weird because the default turns out to be 32bit float packed 2 channel, non interleaved
//
// and we use the asbd we obtain (auEffectStreamFormat) to apply to the output scope of the mixer. and any
// other effects au's that we set up.
// 
// The critical thing here is that you need to 1)set the audio unit description for auEffectUnit, 2)add it to the audio graph, then
// 3) get the instance of the unit from its node in the audio graph (see just prior to this comment)
// at that point the asbd has been initialized to the proper default. If you try to do the AudioUnitGetProperty before
// that point you'll get an error -50
//
// As an alternative you could manually set the effects unit asbd to 32bit float, packed 2 channel, non interleaved - 
// ahead of time, like we did with the other asbd's. 
//
    

// get default asbd properties of au effect unit, 
// this sets up the auEffectStreamFormat asbd

	UInt32 asbdSize = sizeof (auEffectStreamFormat);
	memset (&auEffectStreamFormat, 0, sizeof (auEffectStreamFormat ));
	CheckError(AudioUnitGetProperty(auEffectUnit, kAudioUnitProperty_StreamFormat, kAudioUnitScope_Input, 0, &auEffectStreamFormat, &asbdSize), 
			   "Couldn't get aueffectunit ASBD");
	
    // debug print to find out what's actually in this asbd
    
    NSLog (@"The stream format for the effects unit:");
    [self printASBD: auEffectStreamFormat];
    
    auEffectStreamFormat.mSampleRate = graphSampleRate;      // make sure the sample rate is correct
    
    // now set this asbd to the effect unit input scope 
    // note: if the asbd sample rate is already equal to graphsamplerate then this next statement is not
    // necessary because we derived the asbd from what it was already set to.
    
  
	CheckError(AudioUnitSetProperty(auEffectUnit, kAudioUnitProperty_StreamFormat, kAudioUnitScope_Input, 0, &auEffectStreamFormat, sizeof(auEffectStreamFormat)), 
			   "Couldn't set ASBD on effect unit input");
    
  
    // set the sample rate on the effect unit output scope...
    //
    // Here
    // i'm just doing for the effect the same thing that worked for the 
    // mixer output when there was no effect
    //
      
    NSLog (@"Setting sample rate for au effect unit output scope");
    // Set the mixer unit's output sample rate format. This is the only aspect of the output stream
    //    format that must be explicitly set.
    result = AudioUnitSetProperty (
                                   auEffectUnit,
                                   kAudioUnitProperty_SampleRate,
                                   kAudioUnitScope_Output,
                                   0,
                                   &graphSampleRate,
                                   sizeof (graphSampleRate)
                                   );
    
    if (noErr != result) {[self printErrorMessage: @"AudioUnitSetProperty (set au effect unit output stream format)" withStatus: result]; return;}
   
    
    
    // and finally... set our new effect stream format on the output scope of the mixer.
    // app will blow up at runtime without this
    
   
    CheckError(AudioUnitSetProperty(mixerUnit, kAudioUnitProperty_StreamFormat, kAudioUnitScope_Output, 0, &auEffectStreamFormat, sizeof(auEffectStreamFormat)), 
			   "Couldn't set ASBD on mixer output"); 
  

    
    
    
////////// sampler unit ///////////////
    
    // Obtain the sampler unit instance from the corresponding node.
    
       NSLog (@"Getting sampler unit instance...");
    
	result =	AUGraphNodeInfo (
								 processingGraph,
								 samplerNode,
								 NULL,
								 &auSamplerUnit
								 );
	
	if (result) {[self printErrorMessage: @"AUGraphNodeInfo - sampler unit" withStatus: result]; return;}
    
    // - very important note...
    //
    // the aiff files for the sampler referneced in the .aupreset 
    // must be in an actual referenced folder in xcode (ie blue folder) not a yellow one
    //
    // or you'll get the error -43 when trying to load the preset plist
    //
    //
    
    // These next 2 property settings are not essential
    // but were included in the Apple sampler preset example code
    //
    // apparently the asbd doesn't need to be set on the sampler's output scope
    // or on the mixer channel input scope
    // It works ok with the default 

        
    // Set the Sampler unit's output sample rate.
    
      
    result =    AudioUnitSetProperty (
                                      auSamplerUnit,
                                      kAudioUnitProperty_SampleRate,
                                      kAudioUnitScope_Output,
                                      0,
                                      &graphSampleRate,
                                      sizeof (graphSampleRate)                                    
                                      );
    
    if(result != noErr) {
        NSLog(@"AudioUnitSetProperty (set Sampler unit output stream sample rate). Error code: %d '%.4s'", (int) result, (const char *)&result);
    }
    
    // Set the Sampler unit's maximum frames-per-slice. 
    
    result =    AudioUnitSetProperty (
                                      auSamplerUnit,
                                      kAudioUnitProperty_MaximumFramesPerSlice,
                                      kAudioUnitScope_Global,
                                      0,
                                      &maximumFramesPerSlice,
                                      sizeof (maximumFramesPerSlice)
                                     
                                      );
    
    if( result != noErr) {
        NSLog(@"AudioUnitSetProperty (set Sampler unit maximum frames per slice). Error code: %d '%.4s'", (int) result, (const char *)&result);
    }
    
///////////////////////////////////////////////////////////////
// file player unit 
//
// Note: we are using an au effect between the fileplayer and the mixer input bus
//
// The asbd's are set in the same manner as were set with the au effect on the mixer output bus (see above)
//
    // Obtain the file player unit instance from the corresponding node.
    
    NSLog (@"Getting fileplayer unit instance...");
    
	result =	AUGraphNodeInfo (
								 processingGraph,
								 filePlayerNode,
								 NULL,
								 &auFilePlayerUnit
								 );
	
	if (result) {[self printErrorMessage: @"AUGraphNodeInfo - file player unit" withStatus: result]; return;}
    
   NSLog (@"setting fileplayer asbd (output scope)");
    
    CheckError(AudioUnitSetProperty(auFilePlayerUnit, kAudioUnitProperty_StreamFormat, kAudioUnitScope_Output, 0, &auEffectStreamFormat, sizeof(auEffectStreamFormat)), 
			   "Couldn't set ASBD on fileplayer output (output scope");

    ////////////// input effect1 /////////////////////////
  
    
    // Obtain the input effect1 unit instance from the corresponding node.
    
    NSLog (@"Getting input effect1 unit instance...");
    
	result =	AUGraphNodeInfo (
								 processingGraph,
								 inputEffect1Node,
								 NULL,
								 &auInputEffect1
								 );
	
	if (result) {[self printErrorMessage: @"AUGraphNodeInfo - input effect1 unit" withStatus: result]; return;}

    
    
    
    // set stream format for input effect1 -     
    
    NSLog (@"setting input effect1 asbd input scope");
    
    CheckError(AudioUnitSetProperty(auInputEffect1, kAudioUnitProperty_StreamFormat, kAudioUnitScope_Input, 0, &auEffectStreamFormat, sizeof(auEffectStreamFormat)), 
			   "Couldn't set ASBD on input effect1 input scope");

    NSLog (@"setting input effect1 asbd scope output");
    
    CheckError(AudioUnitSetProperty(auInputEffect1, kAudioUnitProperty_StreamFormat, kAudioUnitScope_Output, 0, &auEffectStreamFormat, sizeof(auEffectStreamFormat)), 
			   "Couldn't set ASBD on input effect1 output");

    // note: Its not necessary to set the asbd on the mixer input bus - it just works
    
    
    // Connect the nodes of the audio processing graph
    
    
    [self connectAudioProcessingGraph];
        
       
       

    //............................................................................
    // Initialize audio processing graph

    // Diagnostic code
    // Call CAShow if you want to look at the state of the audio processing 
    //    graph.
    NSLog (@"Audio processing graph state immediately before initializing it:");
    CAShow (processingGraph);

    NSLog (@"Initializing the audio processing graph");
    // Initialize the audio processing graph, configure audio data stream formats for
    //    each input and output, and validate the connections between audio units.
    result = AUGraphInitialize (processingGraph);
    
    if (noErr != result) {[self printErrorMessage: @"AUGraphInitialize" withStatus: result]; return;}

    ////////////////////////////////////////////////
    // post-init configs
    //
    // set fx parameters, and various initial settings for things
    	
	
	Float32 lowPassCutoffFrequency = 800.0;
	CheckError(AudioUnitSetParameter(auEffectUnit,
									 kLowPassParam_CutoffFrequency,
									 kAudioUnitScope_Global,
									 0,
									 lowPassCutoffFrequency,
									 0),
			   "Coulnd't set kLowPassParam_CutoffFrequency");
    

/* 
 
    //  as of 11/2011 the reverb2 unit has the dreaded bypass bug... 
    //  bypass it once and it never comes back
 
    Float32 reverb2DryWetMix = 80.0;
	CheckError(AudioUnitSetParameter(auInputEffect1,
									 kReverb2Param_DryWetMix,
									 kAudioUnitScope_Global,
									 0,
									 reverb2DryWetMix,
									 0),
			   "Coulnd't set kReverb2Param_DryWetMix ");
    
*/
 
    // high pass filter bypass works ok
    
     Float32 highPassFilterCutoff = 1000.0;
	CheckError(AudioUnitSetParameter(auInputEffect1,
									 kHipassParam_CutoffFrequency,
									 kAudioUnitScope_Global,
									 0,
									 highPassFilterCutoff,
									 0),
			   "Coulnd't set kHipassParam_CutoffFrequency ");

    // turn off synthesizer and initialize some reasonable values for frequency and phasae
    
    synthNoteOn = NO;
 	sinFreq = 200.0;
	sinPhase = 0;

    

    // setup sampler midi and fileplayer 
    
    [self setUpAUFilePlayer];
	[self setUpAUSampler];
	[self setUpMIDI];


    // wow - this completes all the audiograph setup and initialization
}


#pragma mark -
#pragma mark Playback control

/////////////////
// Start playback
//
//  This is the master on/off switch that starts the processing graph
//
- (void) startAUGraph  {

    NSLog (@"Starting audio processing graph");
    OSStatus result = AUGraphStart (processingGraph);
    if (noErr != result) {[self printErrorMessage: @"AUGraphStart" withStatus: result]; return;}

    self.playing = YES;
}

////////////////
// Stop playback
- (void) stopAUGraph {

    NSLog (@"Stopping audio processing graph");
    Boolean isRunning = false;
    OSStatus result = AUGraphIsRunning (processingGraph, &isRunning);
    if (noErr != result) {[self printErrorMessage: @"AUGraphIsRunning" withStatus: result]; return;}
    
    if (isRunning) {
    
        result = AUGraphStop (processingGraph);
        if (noErr != result) {[self printErrorMessage: @"AUGraphStop" withStatus: result]; return;}
        self.playing = NO;
    }
}


#pragma mark -
#pragma mark Mixer unit control

////////////////////////
// mixer handler methods

////////////////////////////////////
// Enable or disable a specified bus
- (void) enableMixerInput: (UInt32) inputBus isOn: (AudioUnitParameterValue) isOnValue {

    NSLog (@"Bus %d now %@", (int) inputBus, isOnValue ? @"on" : @"off");
         
    OSStatus result = AudioUnitSetParameter (
                         mixerUnit,
                         kMultiChannelMixerParam_Enable,
                         kAudioUnitScope_Input,
                         inputBus,
                         isOnValue,
                         0
                      );

    if (noErr != result) {[self printErrorMessage: @"AudioUnitSetParameter (enable the mixer unit)" withStatus: result]; return;}
    

    // Ensure that the sound loops stay in sync when reenabling an input bus
    if (0 == inputBus && 1 == isOnValue) {
        soundStructArray[0].sampleNumber = soundStructArray[1].sampleNumber;
    }
    
    if (1 == inputBus && 1 == isOnValue) {
        soundStructArray[1].sampleNumber = soundStructArray[0].sampleNumber;
    }
    
    // We removed the UI switch for bus 1 (beats) and merged it with bus 0 (guitar)
    // So if bus 0 switch is pressed call this method again with '1'
    // A wimpy form of recursion.

    
    if( inputBus == 0 ) {
        inputBus = 1;
        [self enableMixerInput: inputBus isOn: isOnValue ];
    }
  
 

    
}

//////////////////////////////////////////////////////
// Set the mixer unit input volume for a specified bus
- (void) setMixerInput: (UInt32) inputBus gain: (AudioUnitParameterValue) newGain {

/*
    This method does *not* ensure that sound loops stay in sync if the user has 
    moved the volume of an input channel to zero. When a channel's input 
    level goes to zero, the corresponding input render callback is no longer 
    invoked. Consequently, the sample number for that channel remains constant 
    while the sample number for the other channel continues to increment. As a  
    workaround, the view controller Nib file specifies that the minimum input
    level is 0.01, not zero. (tz: changed this to .00001)
    
    The enableMixerInput:isOn: method in this class, however, does ensure that the 
    loops stay in sync when a user disables and then reenables an input bus.
*/
    OSStatus result = AudioUnitSetParameter (
                         mixerUnit,
                         kMultiChannelMixerParam_Volume,
                         kAudioUnitScope_Input,
                         inputBus,
                         newGain,
                         0
                      );

    if (noErr != result) {[self printErrorMessage: @"AudioUnitSetParameter (set mixer unit input volume)" withStatus: result]; return;}
    
}

///////////////////////////////////
// Set the mixer unit output volume
- (void) setMixerOutputGain: (AudioUnitParameterValue) newGain {

//   Float32 outputLevel;
    
    OSStatus result = AudioUnitSetParameter (
                         mixerUnit,
                         kMultiChannelMixerParam_Volume,
                         kAudioUnitScope_Output,
                         0,
                         newGain,
                         0
                      );

    if (noErr != result) {[self printErrorMessage: @"AudioUnitSetParameter (set mixer unit output volume)" withStatus: result]; return;}
    
    // for testing - get the current output level
    // This didn't work. I'd like to know how to do this without a callback
    
    //   outputLevel = [self getMixerOutputLevel];
    
}


/////////////////////////////////
// turn on or off master mixer fx
- (void) setMixerFx: (AudioUnitParameterValue) isOnValue {
	
	NSLog (@"mixerFxSwitch now %@", isOnValue ? @"on" : @"off");
	
    UInt32 bypassed = (BOOL) isOnValue ? NO : YES ;
	NSLog (@"setting bypassed to %ld", bypassed);
    
    // ok there's a bug in disortion and reverb - once you bypass it, you can't
    // turn it back on - it just leaves dead air
    //
    
    CheckError(AudioUnitSetProperty (auEffectUnit
                                     ,
									 kAudioUnitProperty_BypassEffect,
									 kAudioUnitScope_Global,
									 0,
									 &bypassed,
									 sizeof(bypassed)),
			   "Couldn't set bypassed status");
    
    
    
}

/////////////////////////////////////////
// turn on or off mixer input bus 5 au fx
- (void) setMixerBus5Fx: (AudioUnitParameterValue) isOnValue {
	
	NSLog (@"mixerBus5FxSwitch now %@", isOnValue ? @"on" : @"off");
	
    
    UInt32 bypassed = (BOOL) isOnValue ? NO : YES ;
	NSLog (@"setting bypassed to %ld", bypassed);
    
    // ok there's a bug in disortion & reverb - once you bypass it, you can't
    // turn it back on - it just leaves dead air
    //
    
    CheckError(AudioUnitSetProperty (auInputEffect1
                                     ,
									 kAudioUnitProperty_BypassEffect,
									 kAudioUnitScope_Global,
									 0,
									 &bypassed,
									 sizeof(bypassed)),
			   "Couldn't set bypassed status");
    
}





/////////////////////////////////////////
// Get the mxer unit output level (post)
- (Float32) getMixerOutputLevel {

// this does not work in any shape or form on any bus of the mixer
// input or output scope....
    
    Float32 outputLevel;
   
    
    CheckError( AudioUnitGetParameter (      mixerUnit,
                                             kMultiChannelMixerParam_PostAveragePower,
                                             kAudioUnitScope_Output,
                                             0,
                                             &outputLevel
                                             ) ,"AudioUnitGetParameter (get mixer unit level") ;
    
    
    
    
    // printf("mixer level is: %f\n", outputLevel);
    
    return outputLevel;
    
}


//////////////////////////////////
// play a note on the midi sampler
//
//
//

- (void) playSamplerNote {
    
    NSLog(@"play sampler note");
    
    UInt32 midiStatus;
    UInt32 note;
    UInt32 velocity; 
    
    midiStatus = 0x90;      // note on message
    note = 62;
    velocity = 80; 

        
    CheckError(MusicDeviceMIDIEvent (auSamplerUnit,
                                     midiStatus,
                                     note,
                                     velocity,
                                     0), 
               "Couldn't send MIDI event");

    
    
             
}


- (void) stopSamplerNote {
    
    NSLog(@"stop sampler note");
    
    UInt32 midiStatus;
    UInt32 note;
    UInt32 velocity; 
    
    midiStatus = 0x80;        // note off message
                              // a common alternative is to send a note-on with a velocity of 0  
    note = 64;
    velocity = 64; 
    
    
    CheckError(MusicDeviceMIDIEvent (auSamplerUnit,
                                     midiStatus,
                                     note,
                                     velocity,
                                     0), 
               "Couldn't send MIDI event");
    
    
    
    
}




//////////////////////////////////////////////////////////////
// play/stop a note on the synth (well its not really a synth)
//
// we mean the synth callback that demonstrates how to generate sounds
// at the sample level. Currently just a sine wave
//
// the actual note processing is handled by the AR envelope generator
// These methods just detect when the button is pressed or released


- (void) playSynthNote {
    
    NSLog( @"play synth note");
    
    synthNoteOn = YES;
    sinFreq = 391.0;    // G
    
  
}


- (void) stopSynthNote {
    
    NSLog(@"stop synth note");
    synthNoteOn = NO;  
    
}



//////////////////
// mic fx handlers
//
//  mic fx now handled by setting MixerHostAudio instance variables in
//  the view control UI object handlers
//
//  then the instance variables get evaluated inside the mic callback function
//




////////////////////////////////////////////
// delegates, utilities, other housekeeping
////////////////////////////////////////////

#pragma mark -
#pragma mark Audio Session Delegate Methods
// Respond to having been interrupted. This method sends a notification to the 
//    controller object, which in turn invokes the playOrStop: toggle method. The 
//    interruptedDuringPlayback flag lets the  endInterruptionWithFlags: method know 
//    whether playback was in progress at the time of the interruption.
- (void) beginInterruption {

    NSLog (@"Audio session was interrupted.");
    
    if (playing) {
    
        self.interruptedDuringPlayback = YES;
        
        NSString *MixerHostAudioObjectPlaybackStateDidChangeNotification = @"MixerHostAudioObjectPlaybackStateDidChangeNotification";
        [[NSNotificationCenter defaultCenter] postNotificationName: MixerHostAudioObjectPlaybackStateDidChangeNotification object: self]; 
    }
}


// Respond to the end of an interruption. This method gets invoked, for example, 
//    after the user dismisses a clock alarm. 
- (void) endInterruptionWithFlags: (NSUInteger) flags {

    // Test if the interruption that has just ended was one from which this app 
    //    should resume playback.
    if (flags & AVAudioSessionInterruptionFlags_ShouldResume) {

        NSError *endInterruptionError = nil;
        [[AVAudioSession sharedInstance] setActive: YES
                                             error: &endInterruptionError];
        if (endInterruptionError != nil) {
        
            NSLog (@"Unable to reactivate the audio session after the interruption ended.");
            return;
            
        } else {
        
            NSLog (@"Audio session reactivated after interruption.");
            
            if (interruptedDuringPlayback) {
            
                self.interruptedDuringPlayback = NO;

                // Resume playback by sending a notification to the controller object, which
                //    in turn invokes the playOrStop: toggle method.
                NSString *MixerHostAudioObjectPlaybackStateDidChangeNotification = @"MixerHostAudioObjectPlaybackStateDidChangeNotification";
                [[NSNotificationCenter defaultCenter] postNotificationName: MixerHostAudioObjectPlaybackStateDidChangeNotification object: self]; 

            }
        }
    }
}


#pragma mark -
#pragma mark Utility methods

// You can use this method during development and debugging to look at the
//    fields of an AudioStreamBasicDescription struct.
- (void) printASBD: (AudioStreamBasicDescription) asbd {

    char formatIDString[5];
    UInt32 formatID = CFSwapInt32HostToBig (asbd.mFormatID);
    bcopy (&formatID, formatIDString, 4);
    formatIDString[4] = '\0';
    
    NSLog (@"  Sample Rate:         %10.0f",  asbd.mSampleRate);
    NSLog (@"  Format ID:           %10s",    formatIDString);
    NSLog (@"  Format Flags:        %10lu",    asbd.mFormatFlags);
    NSLog (@"  Bytes per Packet:    %10lu",    asbd.mBytesPerPacket);
    NSLog (@"  Frames per Packet:   %10lu",    asbd.mFramesPerPacket);
    NSLog (@"  Bytes per Frame:     %10lu",    asbd.mBytesPerFrame);
    NSLog (@"  Channels per Frame:  %10lu",    asbd.mChannelsPerFrame);
    NSLog (@"  Bits per Channel:    %10lu",    asbd.mBitsPerChannel);
}


- (void) printErrorMessage: (NSString *) errorString withStatus: (OSStatus) result {

    
    char str[20];
	// see if it appears to be a 4-char-code
	*(UInt32 *)(str + 1) = CFSwapInt32HostToBig(result);
	if (isprint(str[1]) && isprint(str[2]) && isprint(str[3]) && isprint(str[4])) {
		str[0] = str[5] = '\'';
		str[6] = '\0';
	} else
		// no, format it as an integer
		sprintf(str, "%d", (int)result);
	
//	fprintf(stderr, "Error: %s (%s)\n", operation, str);
    
    
    NSLog (
        @"*** %@ error: %s\n",
                errorString,
                str
    );
}



#pragma mark -
#pragma mark Deallocate

- (void) dealloc {

    for (int audioFile = 0; audioFile < NUM_FILES; ++audioFile)  {    
    
        if (sourceURLArray[audioFile] != NULL) CFRelease (sourceURLArray[audioFile]);

        if (soundStructArray[audioFile].audioDataLeft != NULL) {
            free (soundStructArray[audioFile].audioDataLeft);
            soundStructArray[audioFile].audioDataLeft = 0;
        }
                
        if (soundStructArray[audioFile].audioDataRight != NULL) {
            free (soundStructArray[audioFile].audioDataRight);
            soundStructArray[audioFile].audioDataRight = 0;
        }
    }
    
    [super dealloc];
}

////////////////////
// file player setup

-(OSStatus) setUpAUFilePlayer {
    
    NSLog (@"Setting up filePlayer\n" );

    
	// open the file
	NSString *filePath = [[NSBundle mainBundle] pathForResource: FILE_PLAYER_FILE ofType:FILE_PLAYER_FILE_TYPE];
	CFURLRef audioURL = (__bridge CFURLRef) [NSURL fileURLWithPath:filePath];
	
	// open the input audio file
	CheckError(AudioFileOpenURL(audioURL, kAudioFileReadPermission, 0, &filePlayerFile),
			   "AudioFileOpenURL failed");
    
    
	// tell the file player unit to load the file we want to play
	CheckError(AudioUnitSetProperty(auFilePlayerUnit, kAudioUnitProperty_ScheduledFileIDs, 
									kAudioUnitScope_Global, 0, &filePlayerFile, sizeof(filePlayerFile)),
			   "AudioUnitSetProperty[kAudioUnitProperty_ScheduledFileIDs] failed");
	
	UInt64 nPackets;
	UInt32 propsize = sizeof(nPackets);
	CheckError(AudioFileGetProperty(filePlayerFile, kAudioFilePropertyAudioDataPacketCount,
									&propsize, &nPackets),
			   "AudioFileGetProperty[kAudioFilePropertyAudioDataPacketCount] failed");
	
	// get file's asbd
	AudioStreamBasicDescription fileASBD;
	UInt32 fileASBDPropSize = sizeof(fileASBD);
	CheckError(AudioFileGetProperty(filePlayerFile, kAudioFilePropertyDataFormat,
									&fileASBDPropSize, &fileASBD),
			   "couldn't get file's data format");
    
	// tell the file player AU to play the entire file
	ScheduledAudioFileRegion rgn;
	memset (&rgn.mTimeStamp, 0, sizeof(rgn.mTimeStamp));
	rgn.mTimeStamp.mFlags = kAudioTimeStampSampleTimeValid;
	rgn.mTimeStamp.mSampleTime = 0;
	rgn.mCompletionProc = NULL;
	rgn.mCompletionProcUserData = NULL;
	rgn.mAudioFile = filePlayerFile;
	rgn.mLoopCount = INT_MAX;
	rgn.mStartFrame = 0;
	rgn.mFramesToPlay = nPackets * fileASBD.mFramesPerPacket;
	
	CheckError(AudioUnitSetProperty(auFilePlayerUnit, kAudioUnitProperty_ScheduledFileRegion, 
									kAudioUnitScope_Global, 0,&rgn, sizeof(rgn)),
			   "AudioUnitSetProperty[kAudioUnitProperty_ScheduledFileRegion] failed");
	
	// prime the file player AU with default values
	UInt32 defaultVal = 0;
	CheckError(AudioUnitSetProperty(auFilePlayerUnit, kAudioUnitProperty_ScheduledFilePrime, 
									kAudioUnitScope_Global, 0, &defaultVal, sizeof(defaultVal)),
			   "AudioUnitSetProperty[kAudioUnitProperty_ScheduledFilePrime] failed");
	
	// tell the file player AU when to start playing (-1 sample time means next render cycle)
	AudioTimeStamp startTime;
	memset (&startTime, 0, sizeof(startTime));
	startTime.mFlags = kAudioTimeStampSampleTimeValid;
	startTime.mSampleTime = -1;
	CheckError(AudioUnitSetProperty(auFilePlayerUnit, kAudioUnitProperty_ScheduleStartTimeStamp, 
									kAudioUnitScope_Global, 0, &startTime, sizeof(startTime)),
			   "AudioUnitSetProperty[kAudioUnitProperty_ScheduleStartTimeStamp]");
	
	// file duration (unused)
    //	Float32 fileDuration = (nPackets * fileASBD.mFramesPerPacket) / fileASBD.mSampleRate;
	
	
	return noErr;
}








////////////////
// sampler setup


-(OSStatus) setUpAUSampler {
    
	NSString *filePath = [[NSBundle mainBundle] pathForResource: AU_SAMPLER_PRESET_FILE ofType:@"aupreset"];
	CFURLRef presetURL = (__bridge CFURLRef) [NSURL fileURLWithPath:filePath];
    
    
    if (presetURL) {
		NSLog(@"Attempting to load preset... \n");
        	}
	else {
		NSLog(@"COULD NOT GET PRESET PATH!");
	}

    
     
    
    // load preset file into a CFDataRef
	CFDataRef presetData = NULL;
	SInt32 errorCode = noErr;
	Boolean gotPresetData =
	CFURLCreateDataAndPropertiesFromResource(kCFAllocatorSystemDefault,
                                             presetURL,
                                             &presetData,
                                             NULL,
                                             NULL,
                                             &errorCode);
	CheckError(errorCode, "couldn't load .aupreset data");
	CheckError(!gotPresetData, "couldn't load .aupreset data");
    
	// convert this into a property list
	CFPropertyListFormat presetPlistFormat = {0};
	CFErrorRef presetPlistError = NULL;
	
	CFPropertyListRef presetPlist = CFPropertyListCreateWithData(kCFAllocatorSystemDefault,
																 presetData,
																 kCFPropertyListImmutable,
																 &presetPlistFormat, 
																 &presetPlistError);
	if (presetPlistError) {
		return -999;
	}
    
    NSLog(@"setting plist...\n");
    
	// set this plist as the kAudioUnitProperty_ClassInfo on _auSampler
	if (presetPlist) {
		CheckError(AudioUnitSetProperty(self.auSamplerUnit,
										kAudioUnitProperty_ClassInfo,
										kAudioUnitScope_Global,
										0,
										&presetPlist, 
										sizeof(presetPlist)),
				   "Couldn't set aupreset plist as sampler's class info");
	}
	
    
	return noErr;
	
}


//////////////
//  MIDI setup

#pragma mark - midi



-(OSStatus) setUpMIDI {
	
	MIDIClientRef client;
	void* callbackContext = (__bridge void*) self;
	CheckError (MIDIClientCreate(CFSTR("Core MIDI to System Sounds Demo"), MyMIDINotifyProc, callbackContext, &client),
				"Couldn't create MIDI client");
	
	MIDIPortRef inPort;
	CheckError (MIDIInputPortCreate(client, CFSTR("Input port"), MyMIDIReadProc, callbackContext, &inPort),
				"Couldn't create MIDI input port");
	
	unsigned long sourceCount = MIDIGetNumberOfSources();
	// printf ("%ld sources\n", sourceCount);
	for (int i = 0; i < sourceCount; ++i) {
		MIDIEndpointRef src = MIDIGetSource(i);
		CFStringRef endpointName = NULL;
		CheckError(MIDIObjectGetStringProperty(src, kMIDIPropertyName, &endpointName),
				   "Couldn't get endpoint name");
		char endpointNameC[255];
		CFStringGetCString(endpointName, endpointNameC, 255, kCFStringEncodingUTF8);
		// printf("  source %d: %s\n", i, endpointNameC);
		MIDIPortConnectSource(inPort, src, NULL);
	}
	return noErr;
	
}



@end

