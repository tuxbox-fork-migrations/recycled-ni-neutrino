#ifndef __SOFTCSA_SESSION_H__
#define __SOFTCSA_SESSION_H__

#include <cstdint>
#include <vector>
#include <thread>
#include <atomic>

class CSoftCSAEngine;
class cDemux;

enum SoftCSASessionType {
	SOFTCSA_SESSION_LIVE,
	SOFTCSA_SESSION_PIP,
	SOFTCSA_SESSION_RECORD
};

class CSoftCSASession
{
public:
	CSoftCSASession(SoftCSASessionType type, int adapter, int demux_unit, int frontend_num);
	~CSoftCSASession();

	CSoftCSAEngine *getEngine() { return engine; }

	// Configure PIDs for the reader filter (before start)
	void addPid(unsigned short pid);

	// Set decoder PIDs (must be called before start for LIVE/PIP)
	void setDecoderPids(unsigned short vpid, unsigned short apid, unsigned short pcrpid);

	// Dynamically add a PID to the reader filter (after start, for ECM PIDs)
	bool addReaderPid(unsigned short pid);

	// Start/stop the loopback or recording
	bool start();             // LIVE/PIP: start DVR loopback thread
	bool startRecord(int fd); // RECORD: start descramble-to-file thread
	void stop();
	bool setupDecoderOnly();  // set up decode demux (demux7 DVR) without starting thread

	bool isRunning() const { return running.load(); }
	SoftCSASessionType getType() const { return session_type; }
	int getDecodeDemuxUnit() const { return decode_demux_unit; }
	int getAdapterNum() const { return adapter_num; }

private:
	void loopbackThread();
	void recordThread();
	bool setupDecoder();

	SoftCSASessionType session_type;
	CSoftCSAEngine *engine;
	cDemux *demux;           // HAL demux for TS reading (LIVE and RECORD)

	int dvr_fd;              // write fd on dvr{DECODE_DEMUX} (LIVE only)

	int record_fd;           // not owned by this session; caller manages lifetime
	int adapter_num;
	int demux_unit;          // zapit's decoder demux unit (typically 0)
	int frontend_num;

	unsigned short dec_vpid;
	unsigned short dec_apid;
	unsigned short dec_pcrpid;
	int decode_demux_unit;   // dynamically found decode demux

	uint8_t *buffer;
	static const int BUFFER_SIZE = 128 * 1024;

	std::vector<unsigned short> pids;

	std::thread worker;
	std::atomic<bool> running;
};

#endif
