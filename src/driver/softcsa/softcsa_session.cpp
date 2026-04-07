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
	, reader_fd(-1)
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

	if (type == SOFTCSA_SESSION_RECORD) {
		/* Recording: use cDemux on the recording unit (no DVR loopback needed) */
		demux = new cDemux(demux_unit);
		demux->Open(DMX_TP_CHANNEL, NULL, BUFFER_SIZE);
	}
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
	if (reader_fd >= 0) {
		::close(reader_fd);
		reader_fd = -1;
	}
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

	/* Recording mode: forward PIDs to cDemux immediately */
	if (demux) {
		if (pids.size() == 1)
			demux->pesFilter(pid);
		else
			demux->addPid(pid);
	}
}

bool CSoftCSASession::addReaderPid(unsigned short pid)
{
	if (reader_fd < 0)
		return false;

	uint16_t p = pid;
	if (ioctl(reader_fd, DMX_ADD_PID, &p) < 0) {
		printf("[softcsa] addReaderPid %04x failed: %s\n", pid, strerror(errno));
		return false;
	}
	printf("[softcsa] addReaderPid %04x ok\n", pid);
	return true;
}

bool CSoftCSASession::setupReader(int fd)
{
	if (fd < 0) {
		printf("[softcsa] setupReader: invalid fd %d\n", fd);
		return false;
	}
	reader_fd = fd;
	printf("[softcsa] reader fd=%d on demux%d (from CMD_STOP_DECODER)\n",
		reader_fd, demux_unit);
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

bool CSoftCSASession::start(int reader_fd_param)
{
	if (running.load())
		return false;

	/* Store reader fd immediately so the destructor always closes it,
	 * even if we bail out early. Prevents fd leak on error paths. */
	reader_fd = reader_fd_param;

	if (!buffer) {
		printf("[softcsa] start: no buffer allocated\n");
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

	if (reader_fd < 0) {
		printf("[softcsa] start: invalid reader fd %d\n", reader_fd);
		return false;
	}

	printf("[softcsa] start: reader fd=%d on demux%d, decode demux%d\n",
		reader_fd, demux_unit, decode_demux_unit);

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
		/* Close reader demux fd (after thread has exited) */
		if (reader_fd >= 0) {
			::close(reader_fd);
			reader_fd = -1;
		}
		/* Restore demux sources to default */
		if (decode_demux_unit >= 0)
			cDemux::SetSource(decode_demux_unit, (int)DMX_SOURCE_FRONT0);
	}

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
	printf("[softcsa] loopback thread started (reader_fd=%d, dvr_fd=%d)\n",
		reader_fd, dvr_fd);

	int poll_count = 0;
	int read_count = 0;
	long long total_bytes = 0;
	bool first_data = true;

	while (running) {
		struct pollfd pfd;
		pfd.fd = reader_fd;
		pfd.events = POLLIN;
		int ret = ::poll(&pfd, 1, 100); /* 100ms timeout */
		if (ret < 0) {
			if (errno == EINTR)
				continue;
			if (!running) break; /* fd closed during stop() */
			printf("[softcsa] poll error: %s\n", strerror(errno));
			break;
		}
		poll_count++;
		if (ret == 0) {
			if (poll_count % 50 == 0) /* log every 5 seconds */
				printf("[softcsa] diag: %d polls, %d reads, %lld bytes total\n",
					poll_count, read_count, total_bytes);
			continue; /* timeout, check running flag */
		}
		if (!(pfd.revents & POLLIN)) {
			if (pfd.revents)
				printf("[softcsa] poll revents: 0x%x\n", pfd.revents);
			continue;
		}

		int len = ::read(reader_fd, buffer, BUFFER_SIZE);
		if (len <= 0) {
			if (len < 0 && errno != EAGAIN) {
				if (!running) break; /* fd closed during stop() */
				printf("[softcsa] read error: %s\n", strerror(errno));
			}
			continue;
		}

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
