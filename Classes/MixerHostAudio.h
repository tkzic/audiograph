/*
 
 Originally from Apple MixerHost app
 
 File: MixerHostAudio.h
 Abstract: Audio object: Handles all audio tasks for the application.
 Version: 1.0


*/


#import <AudioToolbox/AudioToolbox.h>
#import <AVFoundation/AVFoundation.h>
#import <Accelerate/Accelerate.h>			// for vdsp functions

//#import "TPCircularBuffer.h"              // for ring buffers 

#define NUM_FILES 2                         // number of audio files read in by old method

#define kDelayBufferLength 1024 * 100       // measured in slices - a couple seconds worth at 44.1k

// Data structure for mono or stereo sound, to pass to the application's render callback function, 
//    which gets invoked by a Mixer unit input bus when it needs more audio to play.
//
// Note: this is used by the callbacks for playing looped files (old way)
typedef struct {

    BOOL                 isStereo;           // set to true if there is data in the audioDataRight member
    UInt32               frameCount;         // the total number of frames in the audio data
    UInt32               sampleNumber;       // the next audio sample to play
    AudioUnitSampleType  *audioDataLeft;     // the complete left (or mono) channel of audio data read from an audio file
    AudioUnitSampleType  *audioDataRight;    // the complete right channel of audio data read from an audio file
	    
} soundStruct, *soundStructPtr;




@interface MixerHostAudio : NSObject <AVAudioSessionDelegate> {

    Float64                         graphSampleRate;                // audio graph sample rate
    CFURLRef                        sourceURLArray[NUM_FILES];      // for handling loop files
    soundStruct                     soundStructArray[NUM_FILES];    // scope reference for loop file callback
	


    // Before using an AudioStreamBasicDescription struct you must initialize it to 0. However, because these ASBDs
    // are declared in external storage, they are automatically initialized to 0. 
    
    AudioStreamBasicDescription     stereoStreamFormat;     // standard stereo 8.24 fixed point
    AudioStreamBasicDescription     monoStreamFormat;       // standard mono 8.24 fixed point
    AudioStreamBasicDescription     SInt16StreamFormat;		// signed 16 bit int sample format
	AudioStreamBasicDescription     floatStreamFormat;		// float sample format (for testing)
    AudioStreamBasicDescription     auEffectStreamFormat;		// audio unit Effect format 
    

    AUGraph                         processingGraph;        // the main audio graph
    BOOL                            playing;                // indicates audiograph is running
    BOOL                            interruptedDuringPlayback;  // indicates interruption happened while audiograph running

    // some of the audio units in this app
    
    
    AudioUnit                       ioUnit;                  // remote io unit
    AudioUnit                       mixerUnit;                  // multichannel mixer audio unit
    AudioUnit						auEffectUnit;           // this is the master effect on mixer output
	AudioUnit						auInputEffect1;         // this is a mixer input bus effect
    AudioUnit						auSampler;              // midi sampler
	AudioUnit						auFilePlayer;           // ios5 fileplayer
    
    
    // audio graph nodes
    
    AUNode      iONode;             // node for I/O unit speaker
    AUNode      mixerNode;          // node for Multichannel Mixer unit
    AUNode      auEffectNode;       // master mix effect node
    AUNode      samplerNode;         // sampler node
    AUNode      filePlayerNode;      // fileplayer node
    AUNode      inputEffect1Node;    // input channel fx node

    
	// fft
		
	FFTSetup fftSetup;			// fft predefined structure required by vdsp fft functions
	COMPLEX_SPLIT fftA;			// complex variable for fft
	int fftLog2n;               // base 2 log of fft size
    int fftN;                   // fft size
    int fftNOver2;              // half fft size
	size_t fftBufferCapacity;	// fft buffer size (in samples)
	size_t fftIndex;            // read index pointer in fft buffer 
    
    // working buffers for sample data
        
	void *dataBuffer;               //  input buffer from mic/line
	float *outputBuffer;            //  fft conversion buffer
	float *analysisBuffer;          //  fft analysis buffer
    SInt16 *conversionBufferLeft;   // for data conversion from fixed point to integer
    SInt16 *conversionBufferRight;   // for data conversion from fixed point to integer

    // convolution 
    
   	float *filterBuffer;        // impusle response buffer
    int filterLength;           // length of filterBuffer
    float *signalBuffer;        // signal buffer
    int signalLength;           // signal length
    float *resultBuffer;        // result buffer
    int resultLength;           // result length
	    
    
// new instance variables for UI display objects
	
    int displayInputFrequency;              // frequency determined by analysis 
    float displayInputLevelLeft;            // average input level for meter left channel
    float displayInputLevelRight;           // average input level for meter right channel
    int displayNumberOfInputChannels;       // number of input channels detected on startup
    
    
// for the synth callback - these are now instance variables so we can pass em back and forth to mic callback using self for inrefcon
    
    float sinFreq;        // frequency of sine wave to generate
    float sinPhase;       // current phase
    BOOL synthNoteOn;     // determines whether note is playing
    
// fileplayer     
    
    AudioFileID filePlayerFile;
    
// mic FX type selection
    
    int micFxType;  // enumerated fx types: 
                    // 0: ring mod
                    // 1: fft
                    // 2: pitch shift
                    // 3: echo (delay)
                    // 4: low pass filter (moving average)
                    // 5: low pass filter (convolution)
    
    BOOL micFxOn;       // toggle for using mic fx
    float micFxControl; // multipurpose mix fx control slider
    
   
    BOOL inputDeviceIsAvailable;    // indicates whether input device is available on ipod touch
    
    	
}

// property declarations - corresponding to instance variables declared above

@property (readwrite)           AudioStreamBasicDescription stereoStreamFormat;
@property (readwrite)           AudioStreamBasicDescription monoStreamFormat;
@property (readwrite)           AudioStreamBasicDescription SInt16StreamFormat;	
@property (readwrite)           AudioStreamBasicDescription floatStreamFormat;	
@property (readwrite)           AudioStreamBasicDescription auEffectStreamFormat;	

@property (readwrite)           Float64                     graphSampleRate;
@property (getter = isPlaying)  BOOL                        playing;
@property                       BOOL                        interruptedDuringPlayback;

@property                       AudioUnit                   mixerUnit;
@property                       AudioUnit                   ioUnit;
@property                       AudioUnit                   auEffectUnit;
@property                       AudioUnit                   auInputEffect1;
@property                       AudioUnit                   auSamplerUnit;
@property                       AudioUnit                   auFilePlayerUnit;

@property       AUNode      iONode;             
@property       AUNode      mixerNode;         
@property       AUNode      auEffectNode;       
@property       AUNode      samplerNode;         
@property       AUNode      filePlayerNode;      
@property       AUNode      inputEffect1Node;    

@property                       AudioFileID                 filePlayerFile;  


@property FFTSetup fftSetup;			
@property COMPLEX_SPLIT fftA;			
@property int fftLog2n;
@property int fftN;
@property int fftNOver2;		

@property void *dataBuffer;			
@property float *outputBuffer;		
@property float *analysisBuffer;	

@property SInt16 *conversionBufferLeft;	
@property SInt16 *conversionBufferRight;	

@property float *filterBuffer;      // filter buffer
@property int filterLength;         // filter length
@property float *signalBuffer;      // signal buffer
@property int signalLength;         // signal length
@property float *resultBuffer;      // signal buffer
@property int resultLength;         // signal length


@property size_t fftBufferCapacity;	
@property size_t fftIndex;	


@property (assign) int displayInputFrequency;
@property (assign) float displayInputLevelLeft;
@property (assign) float displayInputLevelRight;
@property (assign) int displayNumberOfInputChannels;


@property float sinFreq;
@property float sinPhase;
@property BOOL  synthNoteOn;

@property int   micFxType;
@property BOOL  micFxOn;
@property float micFxControl;


@property BOOL inputDeviceIsAvailable;




// function (method) declarations

- (void) obtainSoundFileURLs;
- (void) setupAudioSession;
- (void) setupStereoStreamFormat;
- (void) setupMonoStreamFormat;


- (void) setupSInt16StreamFormat;
- (void) setupFloatStreamFormat; // tz

- (void) readAudioFilesIntoMemory;

- (void) configureAndInitializeAudioProcessingGraph;

- (void) setupAudioProcessingGraph;
- (void) connectAudioProcessingGraph;

- (void) startAUGraph;
- (void) stopAUGraph;

- (void) enableMixerInput: (UInt32) inputBus isOn: (AudioUnitParameterValue) isONValue;
- (void) setMixerInput: (UInt32) inputBus gain: (AudioUnitParameterValue) inputGain;
- (void) setMixerOutputGain: (AudioUnitParameterValue) outputGain;
- (void) setMixerFx: (AudioUnitParameterValue) isOnValue;
- (void) setMixerBus5Fx: (AudioUnitParameterValue) isOnValue;

- (void) playSamplerNote;
- (void) stopSamplerNote;

- (void) playSynthNote;
- (void) stopSynthNote;

- (void) printASBD: (AudioStreamBasicDescription) asbd;
- (void) printErrorMessage: (NSString *) errorString withStatus: (OSStatus) result;

-(OSStatus) setUpAUSampler;
-(OSStatus) setUpAUFilePlayer;
-(OSStatus) setUpMIDI;


- (void) convolutionSetup;
- (void) FFTSetup;
- (void) initDelayBuffer;

- (Float32) getMixerOutputLevel;


@end

