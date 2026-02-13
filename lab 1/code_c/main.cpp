#include <cstdio>
#include <cstdint>
#include <string>
#include <vector>

extern "C" {
#include "waves.h"
}

enum ParseState {
	EXPECT_CMD,
	EXPECT_LEN,
	EXPECT_DATA
};

struct TransactionState {
	ParseState state = EXPECT_CMD;
	bool is_write = false;
	bool is_stream = false;
	uint8_t address = 0;
	uint8_t remaining = 0;
	std::vector<uint8_t> data;
};

static std::string hex2(uint32_t v) {
	char buf[3];
	std::snprintf(buf, sizeof(buf), "%02x", static_cast<unsigned int>(v & 0xff));
	return std::string(buf);
}

static void emit_transaction(const TransactionState& tx) {
	const char* op = tx.is_write ? "WR" : "RD";
	std::string addr = hex2(tx.address);
	if (!tx.is_stream) {
		if (tx.data.empty()) {
			return;
		}
		std::string val = hex2(tx.data[0]);
		std::printf("%s %s %s\n", op, addr.c_str(), val.c_str());
		return;
	}

	std::printf("%s STREAM %s", op, addr.c_str());
	for (size_t i = 0; i < tx.data.size(); i++) {
		std::string val = hex2(tx.data[i]);
		std::printf(" %s", val.c_str());
	}
	std::printf("\n");
}

static void reset_transaction(TransactionState& tx) {
	tx.state = EXPECT_CMD;
	tx.is_write = false;
	tx.is_stream = false;
	tx.address = 0;
	tx.remaining = 0;
	tx.data.clear();
}

static void handle_exchange(TransactionState& tx, uint8_t mosi, uint8_t miso) {
	switch (tx.state) {
	case EXPECT_CMD: {
		tx.address = static_cast<uint8_t>((mosi >> 2) & 0x3f);
		tx.is_write = ((mosi >> 1) & 0x1) != 0;
		tx.is_stream = (mosi & 0x1) != 0;
		tx.data.clear();
		if (tx.is_stream) {
			tx.state = EXPECT_LEN;
		} else {
			tx.remaining = 1;
			tx.state = EXPECT_DATA;
		}
		break;
	}
	case EXPECT_LEN:
		tx.remaining = mosi;
		tx.data.clear();
		if (tx.remaining == 0) {
			emit_transaction(tx);
			reset_transaction(tx);
		} else {
			tx.state = EXPECT_DATA;
		}
		break;
	case EXPECT_DATA: {
		uint8_t val = tx.is_write ? mosi : miso;
		tx.data.push_back(val);
		if (tx.remaining > 0) {
			tx.remaining--;
		}
		if (tx.remaining == 0) {
			emit_transaction(tx);
			reset_transaction(tx);
		}
		break;
	}
	default:
		break;
	}
}

int main(int argc, char** argv) {
	UNUSED(argc);
	UNUSED(argv);

	waves* w = parse_file(stdin);
	if (w == NULL) {
		return 1;
	}

	int idx_cpol = signal2index(w, (char*)"cpol");
	int idx_cpha = signal2index(w, (char*)"cpha");
	int idx_sclk = signal2index(w, (char*)"sclk");
	int idx_ss = signal2index(w, (char*)"ss");
	int idx_mosi = signal2index(w, (char*)"mosi");
	int idx_miso = signal2index(w, (char*)"miso");

	if (idx_cpol < 0 || idx_cpha < 0 || idx_sclk < 0 || idx_ss < 0 || idx_mosi < 0 || idx_miso < 0) {
		panic("missing required signals\n");
	}

	uint32_t cpol = signal_at_idx(w, idx_cpol, 0) & 0x1;
	uint32_t cpha = signal_at_idx(w, idx_cpha, 0) & 0x1;
	bool sample_posedge = (cpol == 0) ? (cpha == 0) : (cpha == 1);

	uint32_t prev_sclk = signal_at_idx(w, idx_sclk, 0) & 0x1;
	uint32_t prev_ss = signal_at_idx(w, idx_ss, 0) & 0x1;
	bool ss_active = (prev_ss == 0);

	uint8_t cur_mosi = 0;
	uint8_t cur_miso = 0;
	int bit_count = 0;
	TransactionState tx;

	for (uint32_t i = 1; i < w->nsamples; i++) {
		uint32_t cur_sclk = signal_at_idx(w, idx_sclk, static_cast<int>(i)) & 0x1;
		uint32_t cur_ss = signal_at_idx(w, idx_ss, static_cast<int>(i)) & 0x1;

		if (cur_ss != prev_ss) {
			ss_active = (cur_ss == 0);
			cur_mosi = 0;
			cur_miso = 0;
			bit_count = 0;
			reset_transaction(tx);
		}

		if (ss_active && cur_sclk != prev_sclk) {
			bool posedge = (prev_sclk == 0 && cur_sclk == 1);
			bool negedge = (prev_sclk == 1 && cur_sclk == 0);
			bool sample_edge = (sample_posedge && posedge) || (!sample_posedge && negedge);
			if (sample_edge) {
				uint32_t mosi = signal_at_idx(w, idx_mosi, static_cast<int>(i)) & 0x1;
				uint32_t miso = signal_at_idx(w, idx_miso, static_cast<int>(i)) & 0x1;
				cur_mosi = static_cast<uint8_t>((cur_mosi << 1) | mosi);
				cur_miso = static_cast<uint8_t>((cur_miso << 1) | miso);
				bit_count++;
				if (bit_count == 8) {
					handle_exchange(tx, cur_mosi, cur_miso);
					cur_mosi = 0;
					cur_miso = 0;
					bit_count = 0;
				}
			}
		}

		prev_sclk = cur_sclk;
		prev_ss = cur_ss;
	}

	free_waves(w);
	return 0;
}
