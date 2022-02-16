#ifndef _QUAD_ENCODER_H
#define _QUAD_ENCODER_H

// for arduino leonardo
#define QUAD_ENC1_PIN_A		0
#define QUAD_ENC1_PIN_B		1
#define QUAD_ENC2_PIN_A		2
#define QUAD_ENC2_PIN_B		3

#define	ROTATION_DEG	900	
#define	CPR				3960	// 4*990
#define MAX_ENCODER_ROTATION 3000
#define MAX_ENCODER_TRANSLATION 3000
int ROTATION_MAX = MAX_ENCODER_ROTATION; 
#define	ROTATION_MID (ROTATION_MAX>>1) 
int TRANSLATION_MAX = MAX_ENCODER_TRANSLATION;
#define	TRANSLATION_MID (TRANSLATION_MAX>>1)

//-----------------------------------------------------------------------------------------------

class cQuadEncoder
{
public:
	void Init (int32_t positionX, int32_t positionY ,uint8_t pullups = false);
	int32_t ReadX ();
	int32_t ReadY ();
	void WriteX (int32_t pos);
	void WriteY (int32_t pos);
	void UpdateX ();
	void UpdateY ();

private:
// 	volatile b8 mIndexFound;
// 	volatile u8 mLastState;
// 	volatile s32 mPosition;
};

extern cQuadEncoder gQuadEncoder;

#endif // _QUAD_ENCODER_H
