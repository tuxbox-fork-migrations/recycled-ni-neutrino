#ifndef __SOFTCSA_ENGINE_H__
#define __SOFTCSA_ENGINE_H__

#include <cstdint>
#include <atomic>

extern "C" {
#include <dvbcsa/dvbcsa.h>
}

class CSoftCSAEngine
{
public:
	CSoftCSAEngine();
	~CSoftCSAEngine();

	// Set control word (called from CW handler thread)
	// parity: 0=even, 1=odd
	void setKey(int parity, uint8_t ecm_mode, const uint8_t *cw);

	// Descramble TS packets in buffer (called from loopback thread)
	// Returns number of packets descrambled
	int descramble(uint8_t *data, int len);

private:
	struct dvbcsa_bs_key_s *key_even[2];
	struct dvbcsa_bs_key_s *key_odd[2];
	std::atomic<int> key_even_idx;
	std::atomic<int> key_odd_idx;
	std::atomic<bool> key_even_set{false};
	std::atomic<bool> key_odd_set{false};
	unsigned int batch_size;
	struct dvbcsa_bs_batch_s *even_batch;
	struct dvbcsa_bs_batch_s *odd_batch;
};

#endif
