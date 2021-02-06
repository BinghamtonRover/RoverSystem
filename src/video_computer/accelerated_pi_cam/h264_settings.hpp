#ifndef H264_SETTINGS_H
#define H264_SETTINGS_H

class H264Settings {
public:
	OMX_U32 qp_i = 0;
	OMX_U32 qp_p = 0;
	OMX_BOOL qp = OMX_FALSE;
	OMX_U32 idr_period = 0;
	OMX_BOOL sei = OMX_FALSE;
	OMX_U32 eede_loss_rate = 0;
	OMX_BOOL eede = OMX_FALSE;
	OMX_BOOL inline_headers = OMX_FALSE;
};

#endif
