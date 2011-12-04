/*
 
 Apple's original comments:
 
    File: MixerHostViewController.m
Abstract: View controller: Sets up the user interface and conveys UI actions 
to the MixerHostAudio object. Also responds to state-change notifications from
the MixerHostAudio object.
 Version: 1.0


*/


#import "MixerHostViewController.h"
#import "MixerHostAudio.h"

NSString *MixerHostAudioObjectPlaybackStateDidChangeNotification = @"MixerHostAudioObjectPlaybackStateDidChangeNotification";


@implementation MixerHostViewController

// To learn about properties, see "Declared Properties" in the "Cocoa Objects" chapter
//    in Cocoa Fundamentals Guide
@synthesize playButton;

@synthesize mixerBus0Switch;
@synthesize mixerBus0LevelFader;

// @synthesize mixerBus1Switch;
@synthesize mixerBus1LevelFader;

@synthesize mixerBus2Switch;
@synthesize mixerBus2LevelFader;
@synthesize micFxSelector;
@synthesize micFxFader;
@synthesize micFxSwitch;


@synthesize micFreqDisplay;
@synthesize micLevelDisplay;
@synthesize inputLevelDisplayLeft;
@synthesize inputLevelDisplayRight;

@synthesize numberOfInputChannelsDisplay;



@synthesize mixerBus3Switch;
@synthesize mixerBus3LevelFader;
@synthesize synthPlayButton;

@synthesize mixerBus4Switch;
@synthesize mixerBus4LevelFader;
@synthesize samplerPlayButton;

@synthesize mixerBus5Switch;
@synthesize mixerBus5LevelFader;
@synthesize mixerBus5FxSwitch;


@synthesize mixerOutputLevelFader;
@synthesize mixerFxSwitch;


@synthesize audioObject;





# pragma mark -
# pragma mark User interface methods
// Set the initial multichannel mixer unit parameter values according to the UI state
- (void) initializeMixerSettingsToUI {

    // Initialize mixer settings to UI
	
	// turn off some of the mixer input buses
	
	mixerBus0Switch.on = NO;
//	mixerBus1Switch.on = NO;
    mixerBus2Switch.on = YES;
	mixerBus3Switch.on = NO;
    mixerBus4Switch.on = NO;
	mixerBus5Switch.on = NO;    
    
    micFxSwitch.on = NO;
    mixerFxSwitch.on = NO;  
    mixerBus5FxSwitch.on = NO;
    
//  initialize all the MixerHostAudio methods which respond to UI objects
	
    
    [audioObject enableMixerInput: 0 isOn: mixerBus0Switch.isOn];
//    [audioObject enableMixerInput: 1 isOn: mixerBus1Switch.isOn];
	[audioObject enableMixerInput: 2 isOn: mixerBus2Switch.isOn];
	[audioObject enableMixerInput: 3 isOn: mixerBus3Switch.isOn];
    [audioObject enableMixerInput: 4 isOn: mixerBus3Switch.isOn];
    
    [audioObject enableMixerInput: 5 isOn: mixerBus3Switch.isOn];
    [audioObject setMixerBus5Fx: mixerBus5FxSwitch.isOn];
    
    [audioObject setMixerOutputGain: mixerOutputLevelFader.value];
    [audioObject setMixerFx: mixerFxSwitch.isOn];
    
    [audioObject setMixerInput: 0 gain: mixerBus0LevelFader.value];
    [audioObject setMixerInput: 1 gain: mixerBus1LevelFader.value];
    [audioObject setMixerInput: 2 gain: mixerBus2LevelFader.value];
	[audioObject setMixerInput: 3 gain: mixerBus3LevelFader.value];
    [audioObject setMixerInput: 4 gain: mixerBus4LevelFader.value];
	[audioObject setMixerInput: 5 gain: mixerBus5LevelFader.value];


	audioObject.micFxOn = NO;
    audioObject.micFxControl = .5;
    audioObject.micFxType = 0;
    
	
	micFreqDisplay.text = @"go";
	
	// this updated the pitch field at regular intervals
	
	[NSTimer scheduledTimerWithTimeInterval:0.1
									 target:self
								   selector:@selector(myMethod:)
								   userInfo:audioObject
									repeats: YES];
	
	
}

// Handle a change in the mixer output gain slider.
- (IBAction) mixerOutputGainChanged: (UISlider *) sender {

    [audioObject setMixerOutputGain: (AudioUnitParameterValue) sender.value];
}

// Handle a change in a mixer input gain slider. The "tag" value of the slider lets this 
//    method distinguish between the two channels.
- (IBAction) mixerInputGainChanged: (UISlider *) sender {

    UInt32 inputBus = sender.tag;
    [audioObject setMixerInput: (UInt32) inputBus gain: (AudioUnitParameterValue) sender.value];
}

/////////////////////////////////////////////
// Handle a change in mic fx control slider. 

- (IBAction) micFxFaderChanged: (UISlider *) sender {
	

    audioObject.micFxControl = (float) micFxFader.value;
    // printf("micFxControl: %f\n", audioObject.micFxControl);
}

/////////////////////////////////////
// Handle mic fx switch button press

- (IBAction) enableMicFx: (UISwitch *) sender {
	
	//    UInt32 inputBus = sender.tag;
	 audioObject.micFxOn = (BOOL) sender.isOn;
    NSLog(@"micFxOn: %s", audioObject.micFxOn ? "on" : "off" );
   
}

//////////////////////////////////////
// handle mic FX type selection

- (IBAction) micFxSelectorChanged: (UISegmentedControl *) sender {
    
// get currently selected index of segmented control which in this case is the 
// mic fx type and set instance variable (property) in MixerHostAudio
    
    
    audioObject.micFxType =  (int) self.micFxSelector.selectedSegmentIndex;
    NSLog(@"micFxType: %d", audioObject.micFxType);
    
}






////////////////////////////////////////
// Handle mixer fx switch button press

- (IBAction) enableMixerFx: (UISwitch *) sender {
	
// get current setting of the switch and send it to the handler method
    
    AudioUnitParameterValue isOn = (AudioUnitParameterValue) sender.isOn;
    [audioObject setMixerFx: isOn];
}
-(IBAction) enableMixerBus5Fx:	(UISwitch *) sender {

// get current setting of the switch and send it to the handler method

AudioUnitParameterValue isOn = (AudioUnitParameterValue) sender.isOn;
[audioObject setMixerBus5Fx: isOn];
    
}


- (IBAction) samplerPlayButtonPressed:                (UIButton *) sender {
    
    // get current setting of the button and send it to the handler method
    
//    AudioUnitParameterValue isOn = (AudioUnitParameterValue) sender.isOn;
    [audioObject playSamplerNote ];
    
}    

- (IBAction) samplerPlayButtonReleased:                (UIButton *) sender {
    
     [audioObject stopSamplerNote ];
}


- (IBAction) synthPlayButtonPressed:                (UIButton *) sender {
    
    // get current setting of the button and send it to the handler method
    
    //    AudioUnitParameterValue isOn = (AudioUnitParameterValue) sender.isOn;
    [audioObject playSynthNote ];
    
}    

- (IBAction) synthPlayButtonReleased:                (UIButton *) sender {
    
    [audioObject stopSynthNote ];
}




// This is the timer callback method
// sorry about the name
//
// it checks the value of instance variables in MixerHostAudio
// and displays them at regular intervals

// in the crazy convoluted world of objective-c
// userInfo conveniently points to AudioObject 
//
- (void) myMethod: (NSTimer *) timer {
	
//	float z = [[timer userInfo] frequency];
//    UInt32 y = [[timer userInfo] micLevel];
    int numChannels;
		
	micFreqDisplay.text = [NSString stringWithFormat:@"%d", 
                        [[timer userInfo] displayInputFrequency]];
    
    numChannels = [[timer userInfo] displayNumberOfInputChannels];
    numberOfInputChannelsDisplay.text = [NSString stringWithFormat:@"%d",numChannels];
    

        
    inputLevelDisplayLeft.progress =  [[timer userInfo] displayInputLevelLeft];
    
    if(numChannels == 1) {      // if mono duplicate left channel meter
        inputLevelDisplayRight.progress = [[timer userInfo] displayInputLevelLeft];
    }
    else if(numChannels == 2) {                      // otherwise use separate data for right channel
        inputLevelDisplayRight.progress = [[timer userInfo] displayInputLevelRight];
    }
	
    // note: we're asuuming that we'll only have 1 or 2 channels (mono or stereo)
    // If zero input channels the program would have exited from AVSession init
    // if more than two, we actually need to do some work to find out what they are and 
    // provide a UI to configure them
    
    
    
}



#pragma mark -
#pragma mark Audio processing graph control

// Handle a play/stop button tap
- (IBAction) playOrStop: (id) sender {

    if (audioObject.isPlaying) {
    
        [audioObject stopAUGraph];
        self.playButton.title = @"Play";
        
    } else {
    
        [audioObject startAUGraph];
        self.playButton.title = @"Stop";
    } 
}

// Handle a change in playback state that resulted from an audio session interruption or end of interruption
- (void) handlePlaybackStateChanged: (id) notification {

    [self playOrStop: nil];
}


#pragma mark -
#pragma mark Mixer unit control

// Handle a Mixer unit input on/off switch action. The "tag" value of the switch lets this
//    method distinguish between the two channels.
- (IBAction) enableMixerInput: (UISwitch *) sender {

    UInt32 inputBus = sender.tag;
    AudioUnitParameterValue isOn = (AudioUnitParameterValue) sender.isOn;
    
    [audioObject enableMixerInput: inputBus isOn: isOn];

}


#pragma mark -
#pragma mark Remote-control event handling
// Respond to remote control events
- (void) remoteControlReceivedWithEvent: (UIEvent *) receivedEvent {

    if (receivedEvent.type == UIEventTypeRemoteControl) {
    
        switch (receivedEvent.subtype) {
        
            case UIEventSubtypeRemoteControlTogglePlayPause:
                [self playOrStop: nil];
                break;
            
            default:
                break;
        }
    }
}


#pragma mark -
#pragma mark Notification registration
// If this app's audio session is interrupted when playing audio, it needs to update its user interface 
//    to reflect the fact that audio has stopped. The MixerHostAudio object conveys its change in state to
//    this object by way of a notification. To learn about notifications, see Notification Programming Topics.
- (void) registerForAudioObjectNotifications {

    NSNotificationCenter *notificationCenter = [NSNotificationCenter defaultCenter];
    
    [notificationCenter addObserver: self
                           selector: @selector (handlePlaybackStateChanged:)
                               name: MixerHostAudioObjectPlaybackStateDidChangeNotification
                             object: audioObject];
}


#pragma mark -
#pragma mark Application state management

- (void) viewDidLoad {

    [super viewDidLoad];

    MixerHostAudio *newAudioObject = [[MixerHostAudio alloc] init];
    self.audioObject = newAudioObject;
    [newAudioObject release];


    [self registerForAudioObjectNotifications];
    [self initializeMixerSettingsToUI];
    
      
}

+ (void) doAlert {
	UIAlertView *alert = [[UIAlertView alloc] initWithTitle:@"title" 
													message:@"message" 
												    delegate:self cancelButtonTitle:@"cancel" otherButtonTitles:nil];
	[alert autorelease];
	[alert show];
}
	

// If using a nonmixable audio session category, as this app does, you must activate reception of 
//    remote-control events to allow reactivation of the audio session when running in the background.
//    Also, to receive remote-control events, the app must be eligible to become the first responder.
- (void) viewDidAppear: (BOOL) animated {

    [super viewDidAppear: animated];
    [[UIApplication sharedApplication] beginReceivingRemoteControlEvents];
    [self becomeFirstResponder];
    
    // this alert needs to be here, becuase if its anywhere in the viewDidLoad sequence you'll
    // get the error: applications are expected to have a root view controller at en of app launch...
    
    if(audioObject.inputDeviceIsAvailable == NO) {
        UIAlertView *alert = [[UIAlertView alloc] initWithTitle:nil message:@"Mic is not available. Please terminate the app. Then connect an input device and restart. Or you can use the app now without a mic."  delegate:self cancelButtonTitle:@"OK" otherButtonTitles:nil];
        [alert show];
        [alert release];
       
    }
    

    
    
}

- (BOOL) canBecomeFirstResponder {

    return YES;
}


- (void) didReceiveMemoryWarning {
    // Releases the view if it doesn't have a superview.
    [super didReceiveMemoryWarning];
    
    // Release any cached data, images, etc that aren't in use.
}


- (void) viewWillDisppear: (BOOL) animated {

    [[UIApplication sharedApplication] endReceivingRemoteControlEvents];
    [self resignFirstResponder];
    
    [super viewWillDisappear: animated];
}


- (void) viewDidUnload {

    self.playButton             = nil;
    self.mixerBus0Switch        = nil;
//    self.mixerBus1Switch        = nil;
	self.mixerBus2Switch        = nil;
	self.mixerBus3Switch        = nil;
	self.mixerBus4Switch        = nil;
	self.mixerBus5Switch        = nil;
	
    
    
    self.mixerBus0LevelFader    = nil;
    self.mixerBus1LevelFader    = nil;
	self.mixerBus2LevelFader    = nil;
	self.mixerBus3LevelFader    = nil;
	self.mixerBus4LevelFader    = nil;
	self.mixerBus5LevelFader    = nil;
    
	self.micFxSwitch            = nil;
    self.micFxFader				= nil;
	
    self.mixerOutputLevelFader  = nil;
    self.mixerFxSwitch          = nil;

    [[NSNotificationCenter defaultCenter] removeObserver: self
                                                    name: MixerHostAudioObjectPlaybackStateDidChangeNotification
                                                  object: audioObject];

    self.audioObject            = nil;
    [super viewDidUnload];
}


- (void) dealloc {

    [playButton             release];
    
    [mixerBus0Switch        release];
//    [mixerBus1Switch        release];
    [mixerBus2Switch        release];
     [mixerBus3Switch        release];
     [mixerBus4Switch        release];
     [mixerBus5Switch        release];
    
    [mixerBus0LevelFader    release];
    [mixerBus1LevelFader    release];
      [mixerBus2LevelFader    release];
      [mixerBus3LevelFader    release];
      [mixerBus4LevelFader    release];
      [mixerBus5LevelFader    release];
    
    [mixerOutputLevelFader  release];
    [mixerFxSwitch          release];
    
    [micFxSwitch            release];
    [micFxFader             release];

    [[NSNotificationCenter defaultCenter] removeObserver: self
                                                    name: MixerHostAudioObjectPlaybackStateDidChangeNotification
                                                  object: audioObject];

    [audioObject            release];
    [[UIApplication sharedApplication] endReceivingRemoteControlEvents];
    [super dealloc];
}

@end
