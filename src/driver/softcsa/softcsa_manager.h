#ifndef __SOFTCSA_MANAGER_H__
#define __SOFTCSA_MANAGER_H__

#include <cstdint>
#include <map>
#include <zapit/client/zapitclient.h>
#include <mutex>
#include <utility>
#include <vector>

#include <zapit/channel.h>
#include "softcsa_session.h"

#define CW_ALGO_CSA_ALT 3 // Not in ca_descr_algo enum (0-2); from OSCam globals.h

class CSoftCSAManager
{
public:
	static CSoftCSAManager *getInstance();

	// Called by CDvbApiClient reader thread
	void onDescrMode(uint32_t demux_index, uint32_t algo, uint32_t cipher_mode);
	void onCW(uint32_t demux_index, uint32_t parity, const uint8_t *cw);
	void addReaderPid(uint32_t demux_index, unsigned short pid);

	// Called by zapit
	void registerDemux(uint32_t demux_index, t_channel_id channel_id,
	                   SoftCSASessionType type, int adapter, int demux_unit, int frontend_num);
	void addPid(uint32_t demux_index, unsigned short pid);
	void setDecoderPids(uint32_t demux_index, unsigned short vpid, unsigned short apid, unsigned short pcrpid);
	void setDecoderTypes(uint32_t demux_index, int video_type, int audio_type);
	void stopSession(t_channel_id channel_id, SoftCSASessionType type);
	void stopAll();

	bool isActive(t_channel_id channel_id);

	// Recording support: start a RECORD session with the file descriptor
	// If session exists and CSA-ALT is active, starts immediately.
	// If session doesn't exist yet, stores fd for deferred start.
	bool startRecordSession(t_channel_id channel_id, int fd);

private:
	CSoftCSAManager();
	~CSoftCSAManager();

	struct DemuxState {
		t_channel_id channel_id;
		SoftCSASessionType type;
		int adapter;
		int demux_unit;
		int frontend_num;
		bool csa_alt_active;   // CA_SET_DESCR_MODE with algo==3 received
		uint8_t ecm_mode;      // stored per demux for setKey()
		CSoftCSASession *session;
		std::vector<unsigned short> pids; // stored before session creation
		std::vector<unsigned short> pending_reader_pids; // ECM PIDs queued before session start
		unsigned short video_pid;
		unsigned short audio_pid;
		unsigned short pcr_pid;
		int video_type;        // VIDEO_FORMAT stream type
		int audio_type;        // audio channel type for AUDIO_SET_BYPASS_MODE
		int record_fd;         // deferred fd for RECORD sessions (-1 = none)
	};

	// Primary: demux_index -> state (for CW routing from CDvbApiClient)
	std::map<uint32_t, DemuxState> demux_states;

	// Secondary: (channel_id, type) -> demux_index (for lifecycle from zapit)
	std::map<std::pair<t_channel_id, SoftCSASessionType>, uint32_t> channel_to_demux;

	std::mutex mtx;


};

#endif
