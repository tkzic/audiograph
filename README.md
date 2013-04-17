audioGraph:
README.txt
March 3, 2013

[notes] updated to run in iOS 6 (version 6.1.2) and xCode 4.6

Had some trouble with git this afternoon - unfortunately have had to recommit all the source files in the entire repository even though there were a small number of changes. The compiler warnings and runtime issues should be resolved. please contact me at tkzic@megalink.net if you have problems getting this current build to run in xCode.

As of today, The app store version of audioGraph is still broken in iOS 6.
The version 1.1 update has been submitted is waiting for Apple's blessing.

----------------------------------------

iOS audio processing graph demonstration

"An audio processing graph is a Core Foundation opaque type, AUGraph, that you use to construct and manage an audio unit processing chain. A graph can leverage the capabilities of multiple audio units and multiple render callback functions, allowing you to create nearly any audio processing solution you can imagine." - From Apple's "Audio Unit Hosting Guide For iOS" 

AudioGraph is a superset of Apple's MixerHost application. 

Features include:

* Mono and stereo mic/line input
* Audio effects including:
	o Ring modulator
	o FFT passthrough using the Accelerate vDSP framework.
	o Real-time variable pitch shifting and detection using STFT
	o Simple variable speed delay using a ring buffer 
	o Recursive moving average filter with variable number of points
	o Convolution example with variable filter cutoff frequency
* Stereo level meter
* Synthesizer example - sine wave with an envelope generator
* iOS 5 features including:
	o MIDI sampler audio unit
	o file player audio unit
	o audio unit effects
* Runs on iPad, iPhone, and iPod-Touch
* Open source
* Available as free download from iTunes App Store
* Music by Van Lawton
* Plus everything from MixerHost 

Requirements:

* A device that runs iOS 5.x 
* Headphones.

Instructions:

Launch the app and press Play. 

Source code, documentation, support, downloads:

http://zerokidz.com/audiograph (website)
https://github.com/tkzic/audiograph (github)
audiograph@zerokidz.com (support/questions)
http://itunes.apple.com/app/audiograph/id486193487 (app)

Credits:

This project is derived from work by:

Chris Adamson
Stephan M. Bernsee
Michael Tyson
Stephen Smith
Apple Core-Audio mailing list
stackoverflow.com
Apple's IOS developer program

Thank you.

- Tom Zicarelli 12/12/2011

Here is the original README file for MixerHost
----------------------------------------------

===========================================================================
DESCRIPTION:

MixerHost demonstrates how to use the Multichannel Mixer audio unit in an iOS application. It also demonstrates how to use a render callback function to provide audio to an audio unit input bus. In this sample, the audio delivered by the callback comes from two short loops read from disk. You could use a similar callback, however, to synthesize sounds to feed into a mixer unit. 

This sample is described in Audio Unit Hosting Guide for iOS.

The code in MixerHost instantiates two system-supplied audio units--the Multichannel Mixer unit (of subtype kAudioUnitSubType_MultichannelMixer) and the Remote I/O unit (of subtype kAudioUnitSubType_RemoteIO)--and connects them together using an audio processing graph (an AUGraph opaque type). The app functions as an audio mixer, letting a user control the playback levels of two sound loops.

The sample provides a user interface for controlling the following Multichannel Mixer unit parameters:

    * input bus enable
    * input bus gain
    * output bus gain

This sample shows how to:

    * Write an input render callback function
    * Locate system audio units at runtime and then load, instantiate, configure, 
        and connect them
    * Correctly use audio stream formats in the context of an audio processing
       graph
    * Instantiate, open, initialize, and start an audio processing graph
    * Control a Multichannel Mixer unit through a user interface

This sample also shows how to:

    * Configure an audio application for playing in the background by adding the 
        "app plays audio" key to the info.plist file
    * Use the AVAudioSession class to configure audio behavior, set hardware
        sample rate, and handle interruptions
    * Make the app eligible for its audio session to be reactivated while in the 
        background
    * Respond to remote-control events as described in Event Handling Guide for 
        iOS
    * Allocate memory for an AudioBufferList struct so that it can handle more
        than one channel of audio
   * Use the C interface from Audio Session Services to handle audio hardware 
        route changes
    * Use Cocoa notifications to communicate state changes from the audio object 
        back to the controller object

To test how this app can reactivate its audio session while in the background in iOS 4.0 or later:

    1. Launch the app and start playback.
    2. Press the Home button. MixerHost continues to play in the background.
    3. Launch the Clock app and set a one-minute countdown timer. Leave the 
        Clock app running in the foreground.
    4. When the timer expires, an alarm sounds, which interrupts MixerHost and 
        stops its audio.
    5. Tap OK to dismiss the Timer Done alert. The MixerHost audio resumes 
        playback while the app remains in the background.

To test how this app responds to remote-control events:

    1. Launch the app and start playback.
    2. Press the Home button. MixerHost continues to play in the background.
    3. Double-press the Home button to display the running apps.
    4. Swipe right to expose the audio transport controls.
    5. Notice the MixerHost icon at the bottom-right of the screen. This 
        indicates that MixerHost is the current target of remote-control
        events.
    6. Tap the play/pause toggle button; MixerHost stops. Tap it again;
        MixerHost resumes playback. Tap the MixerHost icon; MixerHost
        comes to the foreground.


===========================================================================
RELATED INFORMATION:

Audio Unit Hosting Guide for iOS, May 2010
Audio Session Programming Guide, April 2010


===========================================================================
BUILD REQUIREMENTS:

Mac OS X v10.6.4, Xcode 3.2, iOS 4.0


===========================================================================
RUNTIME REQUIREMENTS:

Simulator: Mac OS X v10.6.4
Devices:   iOS 4.0


===========================================================================
PACKAGING LIST:

MixerHostAppDelegate.h
MixerHostAppDelegate.m

The MixerHostAppDelegate class defines the application delegate object, responsible for instantiating the controller object (defined in the MixerHostViewController class) and adding the application's view to the application window.

MixerHostViewController.h
MixerHostViewController.m

The MixerHostViewController class defines the controller object for the application. The object helps set up the user interface, responds to and manages user interaction, responds to notifications from the MixerHostAudio object to handle audio interruptions and audio route changes, and handles various housekeeping duties.

MixerHostAudio.h
MixerHostAudio.m

The MixerHostAudio class encapsulates all of the audio capabilities for the application. It handles audio session configuration, use of the ExtAudioFileRef opaque type for reading audio files from disk into memory, and construction and management of the audio processing graph. It detects interruptions and audio route changes and uses notifications to communicate audio state changes back to the MixerHostViewController object.


===========================================================================
CHANGES FROM PREVIOUS VERSIONS:

Version 1.0. New sample application that demonstrates how to host a Multichannel Mixer unit.
 
================================================================================
Copyright (C) 2010 Apple Inc. All rights reserved.
