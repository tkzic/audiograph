//
//  SNFCoreAudioUtils.h
//  VTMAUGraphDemo
//
//  Created by Chris Adamson on 11/8/11.
//  Copyright (c) 2011 Subsequently and Furthermore, Inc. All rights reserved.
//

#ifndef VTMAUGraphDemo_SNFCoreAudioUtils_h
#define VTMAUGraphDemo_SNFCoreAudioUtils_h

#import <AudioToolbox/AudioToolbox.h>

void CheckError(OSStatus error, const char *operation);

typedef struct {
	AudioUnit mixerUnit;
	UInt32 inputBus;
} SNFAUMixerInput;


#endif
