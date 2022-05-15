/*
 * video_recorder.h
 *
 *  Created on: Feb 11, 2022
 *      Author: jasper
 */

#ifndef INC_VIDEO_RECORDER_H_
#define INC_VIDEO_RECORDER_H_

#ifdef __cplusplus
extern "C" {
#endif

void VR_Power_On(void);
void VR_Power_Off(void);
void VR_Start_Rec(void);
void VR_Stop_Rec(void);

#ifdef __cplusplus
}
#endif

#endif /* INC_VIDEO_RECORDER_H_ */
