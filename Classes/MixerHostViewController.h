/*
 
 Apple's original comments:
 
    File: MixerHostViewController.h
Abstract: View controller: Sets up the user interface and conveys UI actions 
to the MixerHostAudio object. Also responds to state-change notifications from
the MixerHostAudio object.
 Version: 1.0



*/


#import <UIKit/UIKit.h>

@class MixerHostAudio;

@interface MixerHostViewController : UIViewController {

    UIBarButtonItem *playButton;            // master play (on/off) button
    
// mixer bus switches    
    
    UISwitch        *mixerBus0Switch;       // guitar loop
//    UISwitch        *mixerBus1Switch;     // beats loop (now on guitar switch)

	
    UISwitch        *mixerBus2Switch;       // mic
    UISegmentedControl  *micFxSelector;     // mic FX type selector
    UISlider		*micFxFader;            // mic FX control fader
	UISwitch		*micFxSwitch;           // mic FX toggle

	UISwitch        *mixerBus3Switch;       // synth
    UIButton        *synthPlayButton;       // play synth notes
    
    
    UISwitch        *mixerBus4Switch;       // midi sampler
    UIButton        *samplerPlayButton;     // play midi notes
    
	UISwitch        *mixerBus5Switch;       // file player
    UISwitch        *mixerBus5FxSwitch;       // file player fx switch

// mixer bus faders    
    
    UISlider        *mixerBus0LevelFader;
    UISlider        *mixerBus1LevelFader;
	UISlider        *mixerBus2LevelFader;
	UISlider        *mixerBus3LevelFader;
	UISlider        *mixerBus4LevelFader;
	UISlider        *mixerBus5LevelFader;
	
// mixer master section    
    
	UISwitch		*mixerFxSwitch;             // main fx 
    UISlider        *mixerOutputLevelFader;     // main fader
	

    

	
// meters and indicators
    
    UILabel		*micFreqDisplay;
	UILabel		*micLevelDisplay;           // getting rid of this!!!!
    UIProgressView *inputLevelDisplayLeft;
    UIProgressView *inputLevelDisplayRight;    
    UILabel     *numberOfInputChannelsDisplay;

    
    
    MixerHostAudio  *audioObject;
}

@property (nonatomic, retain)    IBOutlet UIBarButtonItem    *playButton; 

@property (nonatomic, retain)    IBOutlet UISwitch           *mixerBus0Switch;
// @property (nonatomic, retain)    IBOutlet UISwitch           *mixerBus1Switch;

@property (nonatomic, retain)    IBOutlet UISwitch           *mixerBus2Switch;
@property (nonatomic, retain)    IBOutlet UISegmentedControl *micFxSelector;


@property (nonatomic, retain)    IBOutlet UILabel			*micFreqDisplay;
@property (nonatomic, retain)    IBOutlet UILabel			*micLevelDisplay;
@property (nonatomic, retain)    IBOutlet UIProgressView    *inputLevelDisplayLeft;
@property (nonatomic, retain)    IBOutlet UIProgressView    *inputLevelDisplayRight;

@property (nonatomic, retain)    IBOutlet UILabel           *numberOfInputChannelsDisplay; 

@property (nonatomic, retain)    IBOutlet UISwitch           *mixerBus3Switch;
@property (nonatomic, retain)    IBOutlet UIButton           *synthPlayButton;

@property (nonatomic, retain)    IBOutlet UISwitch           *mixerBus4Switch;
@property (nonatomic, retain)    IBOutlet UIButton           *samplerPlayButton;

@property (nonatomic, retain)    IBOutlet UISwitch           *mixerBus5Switch;
@property (nonatomic, retain)    IBOutlet UISwitch           *mixerBus5FxSwitch;

@property (nonatomic, retain)    IBOutlet UISlider           *mixerBus0LevelFader;
@property (nonatomic, retain)    IBOutlet UISlider           *mixerBus1LevelFader;
@property (nonatomic, retain)    IBOutlet UISlider           *mixerBus2LevelFader;
@property (nonatomic, retain)    IBOutlet UISlider           *mixerBus3LevelFader;
@property (nonatomic, retain)    IBOutlet UISlider           *mixerBus4LevelFader;
@property (nonatomic, retain)    IBOutlet UISlider           *mixerBus5LevelFader;

@property (nonatomic, retain)    IBOutlet UISlider           *mixerOutputLevelFader;
@property (nonatomic, retain)    IBOutlet UISwitch           *mixerFxSwitch;


@property (nonatomic, retain)    IBOutlet UISlider           *micFxFader;
@property (nonatomic, retain)    IBOutlet UISwitch           *micFxSwitch;


@property (nonatomic, retain)    MixerHostAudio              *audioObject;


- (IBAction) enableMixerInput:          (UISwitch *) sender;
- (IBAction) mixerInputGainChanged:     (UISlider *) sender;
- (IBAction) mixerOutputGainChanged:    (UISlider *) sender;
- (IBAction) playOrStop:                (id) sender;

- (IBAction) samplerPlayButtonPressed:                (UIButton *) sender;
- (IBAction) samplerPlayButtonReleased:                (UIButton *) sender;

- (IBAction) synthPlayButtonPressed:                (UIButton *) sender;
- (IBAction) synthPlayButtonReleased:                (UIButton *) sender;



-(IBAction) micFxFaderChanged:			(UISlider *) sender; 
-(IBAction) enableMicFx:				(UISwitch *) sender;
-(IBAction) micFxSelectorChanged:       (UISegmentedControl *) sender;

-(IBAction) enableMixerFx:				(UISwitch *) sender;
-(IBAction) enableMixerBus5Fx:				(UISwitch *) sender;

+ (void) doAlert;

- (void) handlePlaybackStateChanged: (id) notification;
- (void) initializeMixerSettingsToUI;
- (void) registerForAudioObjectNotifications;
// - (void) updateFreqDisplay;							// update text field with current pitch 

- (void) myMethod: (NSTimer *) timer;				// freq display timer method selector


@end
