#include "softcsa_manager.h"
#include "softcsa_engine.h"
#include <cstdio>
#include <unistd.h>

CSoftCSAManager *CSoftCSAManager::getInstance()
{
	static CSoftCSAManager instance;
	return &instance;
}

CSoftCSAManager::CSoftCSAManager() {}
CSoftCSAManager::~CSoftCSAManager() { stopAll(); }

void CSoftCSAManager::registerDemux(uint32_t demux_index, t_channel_id channel_id,
                                     SoftCSASessionType type, int adapter, int demux_unit, int frontend_num)
{
	CSoftCSASession *old_session = NULL;

	{
		std::lock_guard<std::mutex> lock(mtx);

		/* Clean up any existing state for this demux (e.g. PMT update) */
		auto existing = demux_states.find(demux_index);
		if (existing != demux_states.end())
		{
			old_session = existing->second.session;
			existing->second.session = NULL;
			auto old_key = std::make_pair(existing->second.channel_id, existing->second.type);
			channel_to_demux.erase(old_key);
		}

		DemuxState state;
		state.channel_id = channel_id;
		state.type = type;
		state.adapter = adapter;
		state.demux_unit = demux_unit;
		state.frontend_num = frontend_num;
		state.csa_alt_active = false;
		state.ecm_mode = 0;
		state.session = NULL;
		state.video_pid = 0;
		state.audio_pid = 0;
		state.pcr_pid = 0;
		state.video_type = 0;
		state.audio_type = 0;
		state.record_fd = -1;

		demux_states[demux_index] = state;
		channel_to_demux[std::make_pair(channel_id, type)] = demux_index;

		printf("[softcsa] registerDemux: demux %u for channel %llx type %d\n",
		       demux_index, (unsigned long long)channel_id, type);
	}

	if (old_session) {
		old_session->stop();
		delete old_session;
	}
}

void CSoftCSAManager::addPid(uint32_t demux_index, unsigned short pid)
{
	std::lock_guard<std::mutex> lock(mtx);

	auto it = demux_states.find(demux_index);
	if (it == demux_states.end())
		return;

	it->second.pids.push_back(pid);

	if (it->second.session)
		it->second.session->addPid(pid);
}

void CSoftCSAManager::setDecoderPids(uint32_t demux_index, unsigned short vpid, unsigned short apid, unsigned short pcrpid)
{
	std::lock_guard<std::mutex> lock(mtx);
	auto it = demux_states.find(demux_index);
	if (it == demux_states.end())
		return;
	it->second.video_pid = vpid;
	it->second.audio_pid = apid;
	it->second.pcr_pid = pcrpid;
}

void CSoftCSAManager::setDecoderTypes(uint32_t demux_index, int video_type, int audio_type)
{
	std::lock_guard<std::mutex> lock(mtx);
	auto it = demux_states.find(demux_index);
	if (it == demux_states.end())
		return;
	it->second.video_type = video_type;
	it->second.audio_type = audio_type;
}

void CSoftCSAManager::onDescrMode(uint32_t demux_index, uint32_t algo, uint32_t cipher_mode)
{
	bool needs_session = false;
	bool is_live = false;
	t_channel_id expected_channel = 0;

	/* Phase 1: Determine if session creation is needed */
	{
		std::lock_guard<std::mutex> lock(mtx);
		auto it = demux_states.find(demux_index);
		if (it != demux_states.end() && algo == CW_ALGO_CSA_ALT && !it->second.session) {
			needs_session = true;
			is_live = (it->second.type == SOFTCSA_SESSION_LIVE || it->second.type == SOFTCSA_SESSION_PIP);
			expected_channel = it->second.channel_id;
		}
	}

	/* Fast path: no session needed, just update metadata */
	if (!needs_session) {
		std::lock_guard<std::mutex> lock(mtx);
		auto it = demux_states.find(demux_index);
		if (it == demux_states.end())
			return;
		printf("[softcsa] onDescrMode: demux %u algo %u cipher_mode %u (update only)\n",
		       demux_index, algo, cipher_mode);
		if (algo == CW_ALGO_CSA_ALT) {
			it->second.csa_alt_active = true;
			it->second.ecm_mode = (uint8_t)cipher_mode;
		}
		return;
	}

	/* Phase 2: Create session and set up decode demux (demux7, DVR source).
	 * This only touches demux7, not demux0 — safe to do before decoder stop. */
	int reader_fd = -1;
	bool session_started = false;
	{
		std::lock_guard<std::mutex> lock(mtx);
		auto it = demux_states.find(demux_index);

		if (it == demux_states.end() || it->second.channel_id != expected_channel) {
			if (it != demux_states.end())
				printf("[softcsa] onDescrMode: channel changed during setup, aborting\n");
			return;
		}
		if (it->second.session) {
			printf("[softcsa] onDescrMode: session already exists for demux %u\n", demux_index);
			return;
		}

		printf("[softcsa] onDescrMode: demux %u algo %u cipher_mode %u\n",
		       demux_index, algo, cipher_mode);

		DemuxState &ds = it->second;
		ds.csa_alt_active = true;
		ds.ecm_mode = (uint8_t)cipher_mode;

		ds.session = new CSoftCSASession(ds.type, ds.adapter, ds.demux_unit, ds.frontend_num);

		printf("[softcsa] createSession: demux %u type %d, %zu stored PIDs\n",
		       demux_index, ds.type, ds.pids.size());

		for (auto pid : ds.pids)
			ds.session->addPid(pid);

		ds.session->setDecoderPids(ds.video_pid, ds.audio_pid, ds.pcr_pid);

		if (!is_live) {
			/* RECORD: no decoder stop needed, start immediately if fd available */
			if (ds.type == SOFTCSA_SESSION_RECORD && ds.record_fd >= 0) {
				printf("[softcsa] createSession: auto-starting record for demux %u fd %d\n",
				       demux_index, ds.record_fd);
				if (!ds.session->startRecord(ds.record_fd))
					printf("[softcsa] createSession: failed to start record\n");
				ds.record_fd = -1;
			}
			return;
		}

		/* LIVE: set up decode demux (demux7 DVR) BEFORE the IPC call.
		 * This only touches demux7, not demux0. */
		if (!ds.session->setupDecoderOnly()) {
			printf("[softcsa] createSession: setupDecoder failed for demux %u\n", demux_index);
			delete ds.session;
			ds.session = NULL;
			return;
		}
	}

	/* Phase 3: Atomic stop decoder + open reader (IPC to zapit thread).
	 * Builds PID list and sends to zapit which does the Enigma2-identical
	 * sequence: close decoder fds → open reader fd on demux0 → return fd. */
	if (is_live) {
		/* Build PID list for the reader */
		static const unsigned short psi_pids[] = {
			0x0000, /* PAT */
			0x0001, /* CAT */
			0x0010, /* NIT */
			0x0011, /* SDT */
			0x0012, /* EIT */
		};
		std::vector<unsigned short> all_pids;
		for (size_t i = 0; i < sizeof(psi_pids) / sizeof(psi_pids[0]); i++)
			all_pids.push_back(psi_pids[i]);

		{
			std::lock_guard<std::mutex> lock(mtx);
			auto it = demux_states.find(demux_index);
			if (it != demux_states.end() && it->second.session) {
				for (auto pid : it->second.pids)
					all_pids.push_back(pid);
			}
		}

		int adapter = 0, demux_unit = 0;
		{
			std::lock_guard<std::mutex> lock(mtx);
			auto it = demux_states.find(demux_index);
			if (it != demux_states.end()) {
				adapter = it->second.adapter;
				demux_unit = it->second.demux_unit;
			}
		}

		CZapitClient zapit;
		reader_fd = zapit.stopSoftCSADecoder(adapter, demux_unit,
			all_pids.data(), (int)all_pids.size());

		printf("[softcsa] stopSoftCSADecoder returned reader_fd=%d\n", reader_fd);
	}

	/* Phase 4: Start loopback thread with the reader fd.
	 * Ownership rule: if start() is called (even if it fails), the session
	 * owns reader_fd (stored at top of start(), closed by destructor).
	 * If start() is never called (session gone), we must close reader_fd. */
	if (is_live && reader_fd >= 0) {
		bool start_called = false;
		{
			std::lock_guard<std::mutex> lock(mtx);
			auto it = demux_states.find(demux_index);
			if (it != demux_states.end() && it->second.session) {
				start_called = true;
				if (it->second.session->start(reader_fd)) {
					session_started = true;
					for (auto pid : it->second.pending_reader_pids)
						it->second.session->addReaderPid(pid);
					it->second.pending_reader_pids.clear();
				} else {
					printf("[softcsa] createSession: start() failed\n");
				}
			} else {
				printf("[softcsa] createSession: session gone during IPC\n");
			}
		}
		if (!start_called) {
			printf("[softcsa] closing orphaned reader_fd %d\n", reader_fd);
			::close(reader_fd);
		}
	} else if (is_live) {
		printf("[softcsa] createSession: no reader fd, loopback not started\n");
	}

	/* Clean up partial session if loopback was not started */
	if (is_live && !session_started) {
		CSoftCSASession *to_delete = NULL;
		{
			std::lock_guard<std::mutex> lock(mtx);
			auto it = demux_states.find(demux_index);
			if (it != demux_states.end() && it->second.session) {
				printf("[softcsa] cleaning up partial session for demux %u\n", demux_index);
				to_delete = it->second.session;
				it->second.session = NULL;
			}
		}
		if (to_delete) {
			to_delete->stop();
			delete to_delete;
		}
	}

	/* Phase 5: Start decoder on decode demux */
	if (is_live && session_started) {
		CZapitClient zapit;
		int ddmx = -1, dadapter = 0;
		unsigned short dvpid = 0, dapid = 0, dpcrpid = 0;
		int vtype = 0, atype = 0;
		{
			std::lock_guard<std::mutex> lock(mtx);
			auto it = demux_states.find(demux_index);
			if (it != demux_states.end() && it->second.session) {
				ddmx = it->second.session->getDecodeDemuxUnit();
				dadapter = it->second.session->getAdapterNum();
				dvpid = it->second.video_pid;
				dapid = it->second.audio_pid;
				dpcrpid = it->second.pcr_pid;
				vtype = it->second.video_type;
				atype = it->second.audio_type;
			}
		}
		zapit.startSoftCSADecoder(ddmx, dadapter, dvpid, dapid, dpcrpid, vtype, atype);
	}
}

void CSoftCSAManager::addReaderPid(uint32_t demux_index, unsigned short pid)
{
	std::lock_guard<std::mutex> lock(mtx);

	auto it = demux_states.find(demux_index);
	if (it == demux_states.end())
		return;

	DemuxState &ds = it->second;
	if (ds.session && ds.session->isRunning())
		ds.session->addReaderPid(pid);
	else
		ds.pending_reader_pids.push_back(pid);
}

void CSoftCSAManager::onCW(uint32_t demux_index, uint32_t parity, const uint8_t *cw)
{
	std::lock_guard<std::mutex> lock(mtx);

	auto it = demux_states.find(demux_index);
	if (it == demux_states.end())
		return;

	DemuxState &ds = it->second;
	if (!ds.csa_alt_active || !ds.session)
		return;

	printf("[softcsa] onCW: demux %u parity %u ecm_mode %u -> setKey\n",
	       demux_index, parity, ds.ecm_mode);
	ds.session->getEngine()->setKey(parity, ds.ecm_mode, cw);
}

void CSoftCSAManager::stopSession(t_channel_id channel_id, SoftCSASessionType type)
{
	CSoftCSASession *session_to_stop = NULL;

	/* Extract session pointer under lock, then stop/delete outside lock.
	 * session->stop() calls worker.join() which blocks — holding the mutex
	 * during join would deadlock if the loopback thread or CW handler
	 * tries to acquire it (addReaderPid, onCW). */
	{
		std::lock_guard<std::mutex> lock(mtx);

		auto key = std::make_pair(channel_id, type);
		auto ch_it = channel_to_demux.find(key);
		if (ch_it == channel_to_demux.end())
			return;

		uint32_t demux_index = ch_it->second;
		auto dm_it = demux_states.find(demux_index);
		if (dm_it != demux_states.end())
		{
			/* Close deferred record fd if it was never used */
			if (dm_it->second.record_fd >= 0) {
				::close(dm_it->second.record_fd);
				dm_it->second.record_fd = -1;
			}
			if (dm_it->second.session) {
				session_to_stop = dm_it->second.session;
				dm_it->second.session = NULL;
			}
			demux_states.erase(dm_it);
			channel_to_demux.erase(ch_it);
		}
	}

	if (session_to_stop) {
		session_to_stop->stop();
		delete session_to_stop;
	}
}

void CSoftCSAManager::stopAll()
{
	std::vector<CSoftCSASession *> sessions_to_stop;

	{
		std::lock_guard<std::mutex> lock(mtx);
		for (auto &pair : demux_states)
		{
			if (pair.second.session)
			{
				sessions_to_stop.push_back(pair.second.session);
				pair.second.session = NULL;
			}
		}
		demux_states.clear();
		channel_to_demux.clear();
	}

	for (auto *s : sessions_to_stop) {
		s->stop();
		delete s;
	}
}

bool CSoftCSAManager::startRecordSession(t_channel_id channel_id, int fd)
{
	std::lock_guard<std::mutex> lock(mtx);

	auto key = std::make_pair(channel_id, SOFTCSA_SESSION_RECORD);
	auto ch_it = channel_to_demux.find(key);
	if (ch_it == channel_to_demux.end()) {
		printf("[softcsa] startRecordSession: no RECORD demux registered for channel %llx\n",
		       (unsigned long long)channel_id);
		return false;
	}

	uint32_t demux_index = ch_it->second;
	auto dm_it = demux_states.find(demux_index);
	if (dm_it == demux_states.end())
		return false;

	DemuxState &ds = dm_it->second;

	if (ds.session && ds.csa_alt_active) {
		printf("[softcsa] startRecordSession: starting record for demux %u fd %d\n",
		       demux_index, fd);
		return ds.session->startRecord(fd);
	}

	printf("[softcsa] startRecordSession: deferring record start for demux %u fd %d\n",
	       demux_index, fd);
	ds.record_fd = fd;
	return true;
}

bool CSoftCSAManager::isActive(t_channel_id channel_id)
{
	std::lock_guard<std::mutex> lock(mtx);

	for (int t = SOFTCSA_SESSION_LIVE; t <= SOFTCSA_SESSION_RECORD; t++)
	{
		auto key = std::make_pair(channel_id, (SoftCSASessionType)t);
		auto it = channel_to_demux.find(key);
		if (it != channel_to_demux.end())
		{
			auto dm = demux_states.find(it->second);
			if (dm != demux_states.end() && dm->second.csa_alt_active)
				return true;
		}
	}
	return false;
}
