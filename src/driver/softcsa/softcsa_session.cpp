#include "softcsa_session.h"
#include "softcsa_engine.h"
#include <dmx_hal.h>
#include <system/set_threadname.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cerrno>
#include <fcntl.h>
#include <unistd.h>
#include <poll.h>
#include <sys/ioctl.h>

CSoftCSASession::CSoftCSASession(SoftCSASessionType type, int adapter, int demux_unit_num, int frontend_num_val)
	: session_type(type)
	, engine(new CSoftCSAEngine())
	, demux(NULL)
	, dvr_fd(-1)
	, record_fd(-1)
	, adapter_num(adapter)
	, demux_unit(demux_unit_num)
	, frontend_num(frontend_num_val)
	, dec_vpid(0)
	, dec_apid(0)
	, dec_pcrpid(0)
	, decode_demux_unit(-1)
	, buffer(NULL)
	, running(false)
{
	if (posix_memalign((void **)&buffer, 16, BUFFER_SIZE) != 0) {
		buffer = NULL;
	}

	/* Use cDemux for TS reading — same mechanism as recording.
	 * The HAL opens with O_RDWR|O_NONBLOCK and manages the fd internally.
	 * This works for both LIVE (DVR loopback) and RECORD (file write). */
	demux = new cDemux(demux_unit);
	demux->Open(DMX_TP_CHANNEL, NULL, BUFFER_SIZE);
}

CSoftCSASession::~CSoftCSASession()
{
	if (running.load())
		stop();
	delete engine;
	engine = NULL;
	delete demux;
	demux = NULL;
	free(buffer);
	buffer = NULL;
}

void CSoftCSASession::setDecoderPids(unsigned short vpid, unsigned short apid, unsigned short pcrpid)
{
	dec_vpid = vpid;
	dec_apid = apid;
	dec_pcrpid = pcrpid;
}

void CSoftCSASession::addPid(unsigned short pid)
{
	pids.push_back(pid);

	/* Forward PIDs to cDemux immediately (LIVE and RECORD) */
	if (demux) {
		if (pids.size() == 1)
			demux->pesFilter(pid);
		else
			demux->addPid(pid);
	}
}

bool CSoftCSASession::addReaderPid(unsigned short pid)
{
	if (!demux)
		return false;

	if (!demux->addPid(pid)) {
		printf("[softcsa] addReaderPid %04x failed\n", pid);
		return false;
	}
	printf("[softcsa] addReaderPid %04x ok\n", pid);
	return true;
}

bool CSoftCSASession::setupDecoder()
{
	/*
	 * Find a decode demux by probing from the last demux backward
	 * Probe demuxes from highest to lowest to find one that accepts DVR source.
	 * Skip demux_unit (reader/live). No PES filters are set here — they are
	 * deferred to CMD_SOFTCSA_START_DECODER to avoid PID-stealing from demux0.
	 */
	decode_demux_unit = -1;

	for (int d = MAX_DMX_UNITS - 1; d >= 0; d--) {
		if (d == demux_unit)
			continue; /* skip reader/live demux */

		char probe_path[64];
		snprintf(probe_path, sizeof(probe_path), "/dev/dvb/adapter%d/demux%d", adapter_num, d);

		/* Two-step source initialization: FRONT0 first, then DVR.
		 * The bcm7251s kernel requires this transition to properly
		 * configure the demux for DVR input. */
		cDemux::SetSource(d, (int)DMX_SOURCE_FRONT0);
		if (!cDemux::SetSource(d, (int)DMX_SOURCE_DVR0 + d)) {
			printf("[softcsa] probe demux%d: SetSource(DVR%d) failed\n", d, d);
			cDemux::SetSource(d, (int)DMX_SOURCE_FRONT0);
			continue;
		}

		/* SetSource succeeded — this demux is usable */
		decode_demux_unit = d;
		printf("[softcsa] found decode demux%d\n", d);
		break;
	}

	if (decode_demux_unit < 0) {
		printf("[softcsa] no usable decode demux found\n");
		return false;
	}

	/* Open DVR device for writing (before decoder filters are started) */
	char dvr_path[64];
	snprintf(dvr_path, sizeof(dvr_path), "/dev/dvb/adapter%d/dvr%d",
		adapter_num, decode_demux_unit);
	dvr_fd = ::open(dvr_path, O_WRONLY | O_CLOEXEC);
	if (dvr_fd < 0) {
		printf("[softcsa] failed to open %s: %s\n", dvr_path, strerror(errno));
		cDemux::SetSource(decode_demux_unit, (int)DMX_SOURCE_FRONT0);
		return false;
	}

	printf("[softcsa] decode demux%d: DVR%d opened\n",
		decode_demux_unit, decode_demux_unit);

	return true;
}

bool CSoftCSASession::start()
{
	if (running.load())
		return false;

	if (!buffer || !demux) {
		printf("[softcsa] start: no buffer or demux\n");
		return false;
	}

	if (!dec_vpid) {
		printf("[softcsa] start: no video PID set\n");
		return false;
	}

	if (decode_demux_unit < 0) {
		printf("[softcsa] start: no decode demux (setupDecoderOnly not called?)\n");
		return false;
	}

	/* Start the demux filter — identical to cRecord::Start() path.
	 * The HAL cDemux was already configured by addPid() → pesFilter()/addPid(). */
	demux->Start();

	printf("[softcsa] start: demux%d (HAL cDemux, DMX_TP_CHANNEL), decode demux%d\n",
		demux_unit, decode_demux_unit);

	running = true;
	worker = std::thread(&CSoftCSASession::loopbackThread, this);

	return true;
}

bool CSoftCSASession::startRecord(int fd)
{
	if (running.load())
		return false;

	if (!buffer || !demux)
		return false;

	record_fd = fd;
	demux->Start();
	running = true;
	worker = std::thread(&CSoftCSASession::recordThread, this);
	return true;
}

void CSoftCSASession::stop()
{
	running = false;

	if (session_type == SOFTCSA_SESSION_LIVE || session_type == SOFTCSA_SESSION_PIP) {
		/* Close DVR and decoder fds BEFORE joining the thread.
		 * The loopback thread may be blocked in write(dvr_fd) because
		 * the hardware decoder was stopped and the DVR buffer is full.
		 * Closing dvr_fd causes write() to fail with EBADF, unblocking
		 * the thread so it can exit. */
		if (dvr_fd >= 0) {
			::close(dvr_fd);
			dvr_fd = -1;
		}
	}

	if (worker.joinable())
		worker.join();

	if (session_type == SOFTCSA_SESSION_LIVE || session_type == SOFTCSA_SESSION_PIP) {
		/* Restore decode demux source to default */
		if (decode_demux_unit >= 0)
			cDemux::SetSource(decode_demux_unit, (int)DMX_SOURCE_FRONT0);
	}

	/* Stop cDemux (LIVE and RECORD) — closes the HAL fd */
	if (demux)
		demux->Stop();
}

bool CSoftCSASession::setupDecoderOnly()
{
	return setupDecoder();
}

void CSoftCSASession::loopbackThread()
{
	set_threadname("n:softcsa_lb");
	printf("[softcsa] loopback thread started (dvr_fd=%d)\n", dvr_fd);

	int read_count = 0;
	long long total_bytes = 0;
	bool first_data = true;

	while (running) {
		/* Read via HAL cDemux — identical to recordThread().
		 * cDemux::Read() does poll+read internally with timeout. */
		int len = demux->Read(buffer, BUFFER_SIZE, 100); /* 100ms timeout */
		if (len <= 0)
			continue;

		if (first_data) {
			printf("[softcsa] first data: %d bytes, PIDs:\n", len);
			for (int off = 0; off + 188 <= len && off < 10 * 188; off += 188) {
				if (buffer[off] == 0x47) {
					uint16_t p = ((buffer[off + 1] & 0x1F) << 8) | buffer[off + 2];
					int tsc = (buffer[off + 3] >> 6) & 3;
					printf("[softcsa]   PID=0x%04x TSC=%d %s\n", p, tsc,
						tsc ? "SCRAMBLED" : "clear");
				}
			}
			first_data = false;
		}
		read_count++;
		total_bytes += len;

		/* Count BEFORE descramble to see original TSC state */
		if (read_count <= 3) {
			int vpid_count = 0, clear_count = 0, scrambled_count = 0, null_count = 0;
			for (int off = 0; off + 188 <= len; off += 188) {
				if (buffer[off] == 0x47) {
					uint16_t p = ((buffer[off + 1] & 0x1F) << 8) | buffer[off + 2];
					int tsc = (buffer[off + 3] >> 6) & 3;
					if (p == dec_vpid || p == dec_apid) vpid_count++;
					if (p == 0x1FFF) null_count++;
					if (tsc == 0) clear_count++; else scrambled_count++;
				}
			}
			int descrambled = engine->descramble(buffer, len);
			printf("[softcsa] read %d: %d bytes, %d descrambled, av=%d clear=%d scrambled=%d null=%d\n",
				read_count, len, descrambled, vpid_count, clear_count, scrambled_count, null_count);
		} else {
			engine->descramble(buffer, len);
		}

		int written = 0;
		while (written < len && running) {
			int w = ::write(dvr_fd, buffer + written, len - written);
			if (w < 0) {
				if (errno == EINTR)
					continue;
				if (errno == EAGAIN || errno == EWOULDBLOCK) {
					usleep(1000);
					continue;
				}
				if (!running || errno == EBADF) break;
				printf("[softcsa] loopback write error: %s\n", strerror(errno));
				break;
			}
			written += w;
		}
	}

	printf("[softcsa] loopback thread stopped (%d reads, %lld bytes)\n",
		read_count, total_bytes);
}

void CSoftCSASession::recordThread()
{
	set_threadname("n:softcsa_rec");
	while (running) {
		int len = demux->Read(buffer, BUFFER_SIZE, 100); // 100ms timeout
		if (len > 0) {
			engine->descramble(buffer, len);
			int written = 0;
			while (written < len && running) {
				int ret = ::write(record_fd, buffer + written, len - written);
				if (ret < 0) {
					if (errno == EINTR)
						continue;
					printf("[softcsa] record write error: %s\n", strerror(errno));
					break;
				}
				written += ret;
			}
		}
	}
}
